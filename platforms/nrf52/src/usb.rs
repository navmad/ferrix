//! USB CDC-ACM Serial Interface and Dual-Mode CLI / 1200-baud DFU Auto-Reset
//!
//! Provides:
//! 1. Dual-Mode Serial Handler:
//!    - **Repeater Mode (`meshcore-cli -s <port> -i -r`)**: Text-based CLI commands
//!      (`get name`, `ver`, `board`, `get radio`, `stats-radio`, `advert`, `time`, `reboot`, etc.).
//!    - **Companion Mode (`meshcore-cli -s <port>`)**: Binary frame protocol
//!      (`< [len_lo] [len_hi] [payload]` and `> [len_lo] [len_hi] [payload]`).
//! 2. 1200-baud touch detection: automatically reboots the nRF52840 into the
//!    DFU bootloader (`GPREGRET = 0x57`) when flashing over serial, eliminating
//!    physical reset button presses.

use defmt::{debug, info};
use embassy_nrf::peripherals::USBD;
use embassy_nrf::usb::Driver;

use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use core::sync::atomic::Ordering;

use ferrix_core::command::COMPANION_PUSH_CHANNEL;
use ferrix_core::companion::cli::CliHost;
use ferrix_core::companion::framing::{
    encode_usb_frame, UsbFrameReader, USB_INBOUND_DELIMITER,
};
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_core::storage::get_callsign;

use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;

pub type UsbDriver = Driver<'static, USBD, &'static SoftwareVbusDetect>;

static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<State> = StaticCell::new();

/// Reboot into the Adafruit DFU bootloader.
fn reboot_to_bootloader() -> ! {
    let sd_active =
        ferrix_core::state::BLE_STATUS.load(Ordering::Relaxed) >= 3;
    crate::reset::reboot_to_bootloader(sd_active)
}

/// Helper: Write an entire byte slice across USB CDC-ACM packet boundaries
/// Send one companion response over the USB binary framing.
async fn send_usb_companion_frame(sender: &mut Sender<'static, UsbDriver>, resp: &[u8]) {
    let mut out_frame = [0u8; 300];
    if let Ok(flen) = encode_usb_frame(resp, &mut out_frame) {
        let _ = write_all(sender, &out_frame[..flen]).await;
    }
}

/// Stream `GET_CONTACTS` over USB with the same framing as the other responses.
async fn stream_contacts_usb(sender: &mut Sender<'static, UsbDriver>) {
    use ferrix_core::companion::protocol;

    let count = protocol::contacts_count();
    send_usb_companion_frame(sender, &protocol::encode_contacts_start(count)).await;

    let mut max_lastmod = 0u32;
    for idx in 0..count as usize {
        let Some(c) = protocol::contact_at(idx) else {
            continue;
        };
        if c.last_seen_epoch > max_lastmod {
            max_lastmod = c.last_seen_epoch;
        }
        let resp = ferrix_core::companion::contacts::serialize_contact(&c);
        send_usb_companion_frame(sender, &resp).await;
    }

    send_usb_companion_frame(sender, &protocol::encode_contacts_end(max_lastmod)).await;
}

async fn write_all(sender: &mut Sender<'static, UsbDriver>, mut data: &[u8]) -> Result<(), ()> {
    while !data.is_empty() {
        let chunk = data.len().min(64);
        sender.write_packet(&data[..chunk]).await.map_err(|_| ())?;
        data = &data[chunk..];
    }
    Ok(())
}

/// Output sink for the text CLI, writing 64-byte CDC chunks.
pub struct UsbCliSink<'a>(pub &'a mut Sender<'static, UsbDriver>);

impl ferrix_core::companion::cli::TextSink for UsbCliSink<'_> {
    async fn send(&mut self, mut data: &[u8]) -> Result<(), ()> {
        while !data.is_empty() {
            let chunk = data.len().min(64);
            self.0
                .write_packet(&data[..chunk])
                .await
                .map_err(|_| ())?;
            data = &data[chunk..];
        }
        Ok(())
    }
}

#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, UsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
pub async fn cdc_acm_task(
    class: CdcAcmClass<'static, UsbDriver>,
    public_key: [u8; 32],
    private_key: [u8; 64],
    hw_model: &'static str,
    capabilities: u32,
    host: &'static dyn CliHost,
) -> ! {
    let (mut sender, mut receiver, control) = class.split_with_control();
    let callsign = get_callsign();
    let mut node = CompanionNode::new(
        callsign.as_str(),
        hw_model,
        capabilities,
        public_key,
        private_key,
        host,
    );

    let mut frame_reader = UsbFrameReader::new();
    let mut text_line_buf = [0u8; 256];
    let mut text_line_len = 0usize;

    loop {
        receiver.wait_connection().await;
        info!("USB CDC-ACM Connected");
        // Drop pushes left over from a previous session so a new app never
        // receives stale events (matches the BLE disconnect drain).
        while COMPANION_PUSH_CHANNEL
            .try_receive()
            .is_ok()
        {}

        if receiver.line_coding().data_rate() == 1200 {
            embassy_time::Timer::after_millis(150).await;
            reboot_to_bootloader();
        }

        // Pushes are only written once the host has used the framed companion
        // protocol. A text CLI session must not receive binary frames.
        let mut binary_session = false;
        frame_reader.reset();
        let mut in_binary_mode = false;

        loop {
            let changed_fut = control.control_changed();
            let mut rx_buf = [0u8; 64];
            let read_fut = receiver.read_packet(&mut rx_buf);
            let push_fut = async {
                if binary_session {
                    Some(
                        COMPANION_PUSH_CHANNEL
                            .receive()
                            .await,
                    )
                } else {
                    core::future::pending::<Option<heapless::Vec<u8, 256>>>().await
                }
            };

            match embassy_futures::select::select3(changed_fut, read_fut, push_fut).await {
                embassy_futures::select::Either3::First(_) => {
                    let rate = receiver.line_coding().data_rate();
                    if rate == 1200 {
                        embassy_time::Timer::after_millis(150).await;
                        reboot_to_bootloader();
                    }
                }
                embassy_futures::select::Either3::Third(Some(msg)) => {
                    let mut out_frame = [0u8; 300];
                    if let Ok(flen) = encode_usb_frame(&msg, &mut out_frame) {
                        let _ = write_all(&mut sender, &out_frame[..flen]).await;
                    }
                }
                embassy_futures::select::Either3::Third(None) => {}
                embassy_futures::select::Either3::Second(read_res) => {
                    match read_res {
                        Ok(n) => {
                            let rate = receiver.line_coding().data_rate();
                            if rate == 1200 {
                                embassy_time::Timer::after_millis(150).await;
                                reboot_to_bootloader();
                            }
                            debug!("USB CDC received {} bytes", n);

                            // Process received bytes
                            for &b in &rx_buf[..n] {
                                // 1. Binary Frame Mode (`< [len_lo] [len_hi] [payload]`)
                                if in_binary_mode || b == USB_INBOUND_DELIMITER {
                                    in_binary_mode = true;
                                    binary_session = true;
                                    if let Some(payload) = frame_reader.push_byte(b) {
                                        in_binary_mode = false;
                                        if payload.first() == Some(&ferrix_core::companion::protocol::CMD_GET_CONTACTS) {
                                            stream_contacts_usb(&mut sender).await;
                                        } else {
                                            let mut responses = heapless::Vec::<heapless::Vec<u8, 200>, 10>::new();
                                            ferrix_core::companion::protocol::handle_command(&mut node, payload, host, |resp| {
                                                let mut v = heapless::Vec::new();
                                                let _ = v.extend_from_slice(resp);
                                                let _ = responses.push(v);
                                            })
                                            .await;
                                            for resp in responses {
                                                let mut out_frame = [0u8; 300];
                                                if let Ok(flen) = encode_usb_frame(&resp, &mut out_frame) {
                                                    let _ = write_all(&mut sender, &out_frame[..flen]).await;
                                                }
                                            }
                                        }
                                    }
                                } else {
                                    // 2. Repeater Text CLI Mode (`meshcore-cli -r`)
                                    // Any text byte switches this connection out of
                                    // companion mode, so pushes stop immediately.
                                    binary_session = false;
                                    if b == b'\r' || b == b'\n' {
                                        if text_line_len > 0 {
                                            let cmd_str = core::str::from_utf8(&text_line_buf[..text_line_len]).unwrap_or("");
                                            ferrix_core::companion::cli::handle_text_cli(
                                                &mut UsbCliSink(&mut sender),
                                                cmd_str,
                                                &mut node,
                                                host,
                                                true,
                                            )
                                            .await;
                                            text_line_len = 0;
                                        }
                                    } else if b == 0x08 || b == 0x7F {
                                        // Backspace / Delete
                                        if text_line_len > 0 {
                                            text_line_len -= 1;
                                        }
                                    } else if text_line_len < text_line_buf.len() && b >= 0x20 && b <= 0x7E {
                                        text_line_buf[text_line_len] = b;
                                        text_line_len += 1;
                                    }
                                }
                            }
                        }
                        Err(_) => break, // Host disconnected
                    }
                }
            }
        }
    }
}

/// Initialize USB CDC-ACM and return (UsbDevice, CdcAcmClass) ready to spawn
pub fn init_usb(
    driver: UsbDriver,
    vid: u16,
    pid: u16,
    manufacturer: &'static str,
    product: &'static str,
) -> (
    UsbDevice<'static, UsbDriver>,
    CdcAcmClass<'static, UsbDriver>,
) {
    // 0x239A is Adafruit VID, 0x8029 is Feather nRF52840 / ThinkNode M1
    let mut config = Config::new(vid, pid);
    config.manufacturer = Some(manufacturer);
    config.product = Some(product);
    // Omit serial number so macOS uses USB location ID (/dev/cu.usbmodem1101),
    // matching the bootloader port name for seamless 1200-baud touch flashing!
    config.serial_number = None;
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let config_desc = CONFIG_DESC.init([0; 256]);
    let bos_desc = BOS_DESC.init([0; 256]);
    let msos_desc = MSOS_DESC.init([0; 256]);
    let control_buf = CONTROL_BUF.init([0; 64]);
    let cdc_state = CDC_STATE.init(State::new());

    let mut builder = Builder::new(
        driver,
        config,
        config_desc,
        bos_desc,
        msos_desc,
        control_buf,
    );

    let class = CdcAcmClass::new(&mut builder, cdc_state, 64);
    let usb = builder.build();

    (usb, class)
}
