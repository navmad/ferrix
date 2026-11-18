//! USB CDC-ACM Serial Interface and Dual-Mode CLI / 1200-baud DFU Auto-Reset
//!
//! Provides:
//! 1. Dual-Mode Serial Handler:
//!    - **Repeater Mode (`meshcore-cli -s <port> -i -r`)**: Text-based CLI commands.
//!    - **Companion Mode (`meshcore-cli -s <port>`)**: Binary frame protocol.
//! 2. 1200-baud touch detection for DFU flashing.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, info};
use embassy_nrf::peripherals::USBD;
use embassy_nrf::usb::Driver;
use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State};
use heapless::Vec;
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use ferrix_core::command::PushSink;
use ferrix_core::companion::cli::CliHost;
use ferrix_core::companion::framing::{encode_usb_frame, MAX_USB_FRAME_SIZE};
use ferrix_core::companion::session::{CompanionLink, CompanionSession};
use ferrix_core::storage::get_callsign;

pub type UsbDriver = Driver<'static, USBD, &'static SoftwareVbusDetect>;

/// Outbound USB pushes. Direct command responses do not use this queue.
pub static USB_PUSH_CHANNEL: Channel<CriticalSectionRawMutex, Vec<u8, 256>, 8> = Channel::new();

static USB_CONNECTED: AtomicBool = AtomicBool::new(false);
/// True only while the companion session is in binary mode; text CLI does not
/// consume pushes.
static USB_PUSH_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Push sink registered with the core router.
pub struct UsbPushSink;

pub static USB_PUSH_SINK: UsbPushSink = UsbPushSink;

impl PushSink for UsbPushSink {
    fn is_connected(&self) -> bool {
        USB_CONNECTED.load(Ordering::Relaxed) && USB_PUSH_ACTIVE.load(Ordering::Relaxed)
    }

    fn try_push(&self, frame: &[u8]) -> bool {
        if frame.len() > 256 {
            return false;
        }
        let mut vec = Vec::<u8, 256>::new();
        if vec.extend_from_slice(frame).is_err() {
            return false;
        }
        USB_PUSH_CHANNEL.try_send(vec).is_ok()
    }
}

embassy_nrf::bind_interrupts!(pub struct UsbIrqs {
    USBD => embassy_nrf::usb::InterruptHandler<USBD>;
});

static VBUS_DETECT: StaticCell<SoftwareVbusDetect> = StaticCell::new();

/// Build the USB driver with the platform's interrupt binding and VBUS
/// detector, so each board does not repeat the same static wiring.
pub fn driver(usbd: USBD) -> UsbDriver {
    let vbus = &*VBUS_DETECT.init(SoftwareVbusDetect::new(true, true));
    Driver::new(usbd, UsbIrqs, vbus)
}

static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<State> = StaticCell::new();

/// Reboot into the Adafruit DFU bootloader.
fn reboot_to_bootloader() -> ! {
    let sd_active = ferrix_core::state::BLE_STATUS.load(Ordering::Relaxed) >= 3;
    crate::reset::reboot_to_bootloader(sd_active)
}

async fn write_all(sender: &mut Sender<'static, UsbDriver>, mut data: &[u8]) -> Result<(), ()> {
    while !data.is_empty() {
        let chunk = data.len().min(64);
        sender.write_packet(&data[..chunk]).await.map_err(|_| ())?;
        data = &data[chunk..];
    }
    Ok(())
}

/// Companion link over the USB CDC endpoint.
struct CdcLink<'a>(&'a mut Sender<'static, UsbDriver>);

impl CompanionLink for CdcLink<'_> {
    async fn write_frame(&mut self, frame: &[u8]) {
        let mut out = [0u8; MAX_USB_FRAME_SIZE];
        if let Ok(len) = encode_usb_frame(frame, &mut out) {
            let _ = write_all(self.0, &out[..len]).await;
        }
    }

    async fn write_text(&mut self, text: &[u8]) {
        let _ = write_all(self.0, text).await;
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
    let mut session = CompanionSession::new(
        callsign.as_str(),
        hw_model,
        capabilities,
        public_key,
        private_key,
        host,
    );

    loop {
        receiver.wait_connection().await;
        USB_CONNECTED.store(true, Ordering::Relaxed);
        info!("USB CDC-ACM Connected");
        // Drop pushes left over from a previous session so a new app never
        // receives stale events (matches the BLE disconnect drain).
        while USB_PUSH_CHANNEL.try_receive().is_ok() {}

        if receiver.line_coding().data_rate() == 1200 {
            embassy_time::Timer::after_millis(150).await;
            reboot_to_bootloader();
        }

        session.reset();

        loop {
            let changed_fut = control.control_changed();
            let mut rx_buf = [0u8; 64];
            let read_fut = receiver.read_packet(&mut rx_buf);
            let push_enabled = session.binary_session();
            USB_PUSH_ACTIVE.store(push_enabled, Ordering::Relaxed);
            let push_fut = async {
                if push_enabled {
                    Some(USB_PUSH_CHANNEL.receive().await)
                } else {
                    core::future::pending::<Option<heapless::Vec<u8, 256>>>().await
                }
            };

            match embassy_futures::select::select3(changed_fut, read_fut, push_fut).await {
                embassy_futures::select::Either3::First(_) => {
                    if receiver.line_coding().data_rate() == 1200 {
                        embassy_time::Timer::after_millis(150).await;
                        reboot_to_bootloader();
                    }
                }
                embassy_futures::select::Either3::Third(Some(msg)) => {
                    CdcLink(&mut sender).write_frame(&msg).await;
                }
                embassy_futures::select::Either3::Third(None) => {}
                embassy_futures::select::Either3::Second(read_res) => match read_res {
                    Ok(n) => {
                        if receiver.line_coding().data_rate() == 1200 {
                            embassy_time::Timer::after_millis(150).await;
                            reboot_to_bootloader();
                        }
                        debug!("USB CDC received {} bytes", n);
                        for &b in &rx_buf[..n] {
                            session.feed_byte(b, &mut CdcLink(&mut sender)).await;
                        }
                    }
                    Err(_) => {
                        USB_CONNECTED.store(false, Ordering::Relaxed);
                        USB_PUSH_ACTIVE.store(false, Ordering::Relaxed);
                        break;
                    }
                },
            }
        }
        USB_CONNECTED.store(false, Ordering::Relaxed);
        USB_PUSH_ACTIVE.store(false, Ordering::Relaxed);
        while USB_PUSH_CHANNEL.try_receive().is_ok() {}
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
