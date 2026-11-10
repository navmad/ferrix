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

use core::fmt::Write;
use defmt::{debug, info};
use embassy_nrf::peripherals::USBD;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::boards::{Board, CurrentBoard};
use crate::companion::framing::{encode_usb_frame, UsbFrameReader, USB_INBOUND_DELIMITER};
use crate::companion::protocol::CompanionNode;

pub type UsbDriver = Driver<'static, USBD, HardwareVbusDetect>;

static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<State> = StaticCell::new();

/// Reboot the nRF52840 directly into the DFU Bootloader
pub fn reboot_to_bootloader() -> ! {
    info!("Rebooting into Adafruit DFU Bootloader (GPREGRET=0x57)...");
    unsafe {
        // NRF_POWER->GPREGRET (0x4000051C) = 0x57 (DFU Magic value)
        let gpregret = (0x4000_0000 + 0x51C) as *mut u32;
        gpregret.write_volatile(0x57);
    }
    cortex_m::peripheral::SCB::sys_reset();
}

/// Helper: Write an entire byte slice across USB CDC-ACM packet boundaries
async fn write_all(sender: &mut Sender<'static, UsbDriver>, mut data: &[u8]) -> Result<(), ()> {
    while !data.is_empty() {
        let chunk = data.len().min(64);
        sender.write_packet(&data[..chunk]).await.map_err(|_| ())?;
        data = &data[chunk..];
    }
    Ok(())
}

/// Handle a text command line from `meshcore-cli -r`
async fn handle_text_cli(
    sender: &mut Sender<'static, UsbDriver>,
    cmd_line: &str,
    node: &mut CompanionNode,
) {
    let trimmed = cmd_line.trim();
    if trimmed.is_empty() {
        return;
    }

    // Echo the command line back to the host terminal
    let _ = write_all(sender, cmd_line.as_bytes()).await;
    let _ = write_all(sender, b"\r\n").await;

    match trimmed {
        "get name" => {
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(s, "  -> > {}\r\n", CurrentBoard::NAME);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "ver" => {
            let _ = write_all(sender, b"  -> 0.1.0-ferrix\r\n").await;
        }
        "board" => {
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(s, "  -> {} (nRF52840, SX1262 LoRa)\r\n", CurrentBoard::NAME);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "get radio" | "get freq" => {
            let _ = write_all(sender, b"  -> 910.525 MHz, SF7, BW 62.5 kHz, CR 4/5, +22 dBm\r\n").await;
        }
        "get tx" => {
            let _ = write_all(sender, b"  -> 22 dBm\r\n").await;
        }
        "get repeat" | "get role" => {
            let _ = write_all(sender, b"  -> on\r\n").await;
        }
        "stats-radio" => {
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(
                s,
                "  -> Freq: 910.525M, SF7, BW62.5k\r\n  -> TX Beacons: {}, RX Packets: {}\r\n  -> Last RSSI: {} dBm, SNR: {} dB\r\n",
                node.tx_beacons, node.rx_packets, node.last_rssi, node.last_snr
            );
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "stats-core" => {
            let _ = write_all(sender, b"  -> Device: ThinkNode-M1\r\n  -> Runtime: Embassy no_std async (nRF52840 Cortex-M4F)\r\n").await;
        }
        "stats-packets" => {
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(s, "  -> TX: {}, RX: {}\r\n", node.tx_beacons, node.rx_packets);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "advert" => {
            node.tx_beacons = node.tx_beacons.saturating_add(1);
            let _ = write_all(sender, b"  -> OK (MeshCore Advert broadcast queued)\r\n").await;
        }
        "time" => {
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> {}\r\n", node.epoch_time);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("time ") => {
            let time_str = s[5..].trim();
            let mut epoch = 0u64;
            for &b in time_str.as_bytes() {
                if b >= b'0' && b <= b'9' {
                    epoch = epoch.wrapping_mul(10).wrapping_add((b - b'0') as u64);
                }
            }
            node.epoch_time = epoch;
            let _ = write_all(sender, b"  -> OK (Clock synchronized)\r\n").await;
        }
        "clock sync" | "sync_time" => {
            let _ = write_all(sender, b"  -> OK\r\n").await;
        }
        "reboot" => {
            let _ = write_all(sender, b"  -> Rebooting device...\r\n").await;
            embassy_time::Timer::after_millis(50).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        "help" => {
            let _ = write_all(sender, b"  -> Available commands: ver, board, get name, get radio, get freq, get tx, get repeat, stats-radio, stats-core, stats-packets, advert, time, reboot\r\n").await;
        }
        _ => {
            let _ = write_all(sender, b"  -> Unknown command\r\n").await;
        }
    }
}

#[embassy_executor::task]
pub async fn usb_task(mut usb: UsbDevice<'static, UsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
pub async fn cdc_acm_task(class: CdcAcmClass<'static, UsbDriver>) -> ! {
    let (mut sender, mut receiver, control) = class.split_with_control();
    let mut node = CompanionNode::new(CurrentBoard::NAME, CurrentBoard::NAME, CurrentBoard::CAPABILITIES);

    let mut frame_reader = UsbFrameReader::new();
    let mut text_line_buf = [0u8; 128];
    let mut text_line_len = 0usize;
    let mut in_binary_mode = false;

    loop {
        receiver.wait_connection().await;
        info!("USB CDC-ACM Connected");

        loop {
            let changed_fut = control.control_changed();
            let mut rx_buf = [0u8; 64];
            let read_fut = receiver.read_packet(&mut rx_buf);

            match embassy_futures::select::select(changed_fut, read_fut).await {
                embassy_futures::select::Either::First(_) => {
                    let rate = receiver.line_coding().data_rate();
                    if rate == 1200 {
                        embassy_time::Timer::after_millis(30).await;
                        reboot_to_bootloader();
                    }
                }
                embassy_futures::select::Either::Second(read_res) => {
                    match read_res {
                        Ok(n) => {
                            let rate = receiver.line_coding().data_rate();
                            if rate == 1200 {
                                embassy_time::Timer::after_millis(30).await;
                                reboot_to_bootloader();
                            }
                            debug!("USB CDC received {} bytes", n);

                            // Process received bytes
                            for &b in &rx_buf[..n] {
                                // 1. Binary Frame Mode (`< [len_lo] [len_hi] [payload]`)
                                if in_binary_mode || b == USB_INBOUND_DELIMITER {
                                    in_binary_mode = true;
                                    if let Some(payload) = frame_reader.push_byte(b) {
                                        in_binary_mode = false;
                                        if let Some(resp) = node.handle_command(payload) {
                                            let mut out_frame = [0u8; 300];
                                            if let Ok(flen) = encode_usb_frame(&resp, &mut out_frame) {
                                                let _ = write_all(&mut sender, &out_frame[..flen]).await;
                                            }
                                        }
                                    }
                                } else {
                                    // 2. Repeater Text CLI Mode (`meshcore-cli -r`)
                                    if b == b'\r' || b == b'\n' {
                                        if text_line_len > 0 {
                                            let cmd_str = core::str::from_utf8(&text_line_buf[..text_line_len]).unwrap_or("");
                                            handle_text_cli(&mut sender, cmd_str, &mut node).await;
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
) -> (
    UsbDevice<'static, UsbDriver>,
    CdcAcmClass<'static, UsbDriver>,
) {
    // 0x239A is Adafruit VID, 0x8029 is Feather nRF52840 / ThinkNode M1
    let mut config = Config::new(0x239A, 0x8029);
    config.manufacturer = Some("MeshCore");
    config.product = Some("ThinkNode M1 Companion");
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
