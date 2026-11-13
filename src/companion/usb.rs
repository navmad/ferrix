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

use core::cell::RefCell;
use core::fmt::Write;
use defmt::{debug, info};
use embassy_nrf::peripherals::USBD;
use embassy_nrf::usb::Driver;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;

/// Shared raw mutex type for cross-module synchronization.
///
/// The BLE GATT callback can run in a SoftDevice event context, so shared
/// state must be protected against preemption. `CriticalSectionRawMutex` uses
/// the nrf-softdevice critical-section implementation, which is nested-safe and
/// only disables non-reserved interrupts for the short critical section. Keep
/// cryptographic work outside these locks.
type SharedRawMutex = CriticalSectionRawMutex;
use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use crate::boards::{Board, CurrentBoard};
use crate::companion::framing::{encode_usb_frame, UsbFrameReader, USB_INBOUND_DELIMITER};
use crate::companion::protocol::CompanionNode;
use crate::meshcore::mesh::{DiscoveredContact, ReceivedMessage};

#[derive(Clone, Copy, Debug)]
pub struct RadioSettings {
    pub frequency_hz: u32,
    pub tx_power_dbm: i8,
    pub sf: u8,
    pub bw_khz: u32,
    pub cr: u8,
    pub is_repeater: bool,
    pub path_hash_mode: u8,
}

pub static RADIO_SETTINGS: Mutex<SharedRawMutex, RefCell<RadioSettings>> = Mutex::new(RefCell::new(RadioSettings {
    frequency_hz: 910_525_000,
    tx_power_dbm: 22,
    sf: 7,
    bw_khz: 62,
    cr: 5,
    is_repeater: false,
    path_hash_mode: 0,
}));

pub static DISCOVERED_CONTACTS: Mutex<SharedRawMutex, RefCell<heapless::Vec<DiscoveredContact, 32>>> =
    Mutex::new(RefCell::new(heapless::Vec::new()));

pub static MESSAGE_LOG: Mutex<SharedRawMutex, RefCell<heapless::Vec<ReceivedMessage, 16>>> =
    Mutex::new(RefCell::new(heapless::Vec::new()));

#[derive(Clone, Copy, Debug)]
pub struct ChannelData {
    pub name: [u8; 32],
    pub secret: [u8; 16],
}

pub const DEFAULT_PUBLIC_SECRET: [u8; 16] = [
    0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a,
    0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72,
];

const fn default_channels() -> [ChannelData; 8] {
    let mut chs = [ChannelData {
        name: [0u8; 32],
        secret: [0u8; 16],
    }; 8];
    chs[0].name[0] = b'P';
    chs[0].name[1] = b'u';
    chs[0].name[2] = b'b';
    chs[0].name[3] = b'l';
    chs[0].name[4] = b'i';
    chs[0].name[5] = b'c';
    chs[0].secret = DEFAULT_PUBLIC_SECRET;
    chs
}

pub static CHANNELS: Mutex<SharedRawMutex, RefCell<[ChannelData; 8]>> =
    Mutex::new(RefCell::new(default_channels()));

#[derive(Clone, Copy, Debug)]
pub struct DeviceClock {
    pub base_epoch: u32,
    pub base_instant_secs: u64,
}

pub static DEVICE_CLOCK: Mutex<SharedRawMutex, RefCell<DeviceClock>> =
    Mutex::new(RefCell::new(DeviceClock {
        base_epoch: 0,
        base_instant_secs: 0,
    }));

pub fn get_device_time() -> u32 {
    DEVICE_CLOCK.lock(|c| {
        let clock = c.borrow();
        if clock.base_epoch == 0 {
            0
        } else {
            let elapsed = embassy_time::Instant::now().as_secs().saturating_sub(clock.base_instant_secs);
            clock.base_epoch.saturating_add(elapsed as u32)
        }
    })
}

pub fn set_device_time(epoch: u32) {
    DEVICE_CLOCK.lock(|c| {
        *c.borrow_mut() = DeviceClock {
            base_epoch: epoch,
            base_instant_secs: embassy_time::Instant::now().as_secs(),
        };
    });
}

#[derive(Clone, Copy, Debug)]
pub struct RadioTelemetry {
    pub tx_beacons: u32,
    pub rx_packets: u32,
    pub last_rssi: i16,
    pub last_snr: i8,
    pub has_rx: bool,
}

pub static RADIO_TELEMETRY: Mutex<SharedRawMutex, RefCell<RadioTelemetry>> = Mutex::new(RefCell::new(RadioTelemetry {
    tx_beacons: 0,
    rx_packets: 0,
    last_rssi: 0,
    last_snr: 0,
    has_rx: false,
}));

#[derive(Clone, Debug)]
pub enum UsbCommand {
    SetName(heapless::String<32>),
    SetPrivateKey([u8; 64]),
    SetTime(u64),
    TriggerAdvert { flood: bool },
    SetAdvertLatLon { lat: i32, lon: i32 },
    SetAutoAddConfig { config: u8, max_hops: u8 },
    SendGroupText {
        channel_idx: u8,
        timestamp: u32,
        text: heapless::String<128>,
    },
    SendDirectText {
        /// First six bytes of the destination public key.
        dest_prefix: [u8; 6],
        timestamp: u32,
        txt_type: u8,
        attempt: u8,
        text: heapless::String<160>,
    },
    SendControlData {
        data: heapless::Vec<u8, 183>,
    },
    SendRawData {
        path_len: u8,
        path: [u8; 64],
        payload: heapless::Vec<u8, 184>,
    },
    SendRawPacket {
        raw: heapless::Vec<u8, 255>,
    },
    SendTracePath {
        tag: u32,
        auth: u32,
        flags: u8,
        path_len: u8,
        path: [u8; 64],
    },
    SetFrequency(u32),
    SetTxPower(i8),
    SetRepeat(bool),
    SetSpreadingFactor(u8),
    SetBandwidth(u32),
    SetCodingRate(u8),
    SetPathHashMode(u8),
}

#[inline]
fn hex_to_nibble(b: u8) -> u8 {
    match b {
        b'0'..=b'9' => b - b'0',
        b'a'..=b'f' => b - b'a' + 10,
        b'A'..=b'F' => b - b'A' + 10,
        _ => 0,
    }
}

#[inline]
fn hex_to_nibble_opt(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

use embassy_nrf::usb::vbus_detect::SoftwareVbusDetect;

pub static USB_COMMAND_CHANNEL: Channel<SharedRawMutex, UsbCommand, 4> = Channel::new();

pub type UsbDriver = Driver<'static, USBD, &'static SoftwareVbusDetect>;

static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
static CDC_STATE: StaticCell<State> = StaticCell::new();

/// Reboot the nRF52840 directly into the DFU Bootloader
pub fn reboot_to_bootloader() -> ! {
    info!("Rebooting into Adafruit DFU Bootloader (GPREGRET=0x57)...");
    let sd_active = crate::companion::ble::BLE_STATUS.load(core::sync::atomic::Ordering::Relaxed) >= 3;
    if sd_active {
        unsafe {
            let _ = nrf_softdevice::raw::sd_power_gpregret_clr(0, 0xFF);
            let _ = nrf_softdevice::raw::sd_power_gpregret_set(0, 0x57);
        }
    } else {
        unsafe {
            let gpregret = (0x4000_0000 + 0x51C) as *mut u32;
            gpregret.write_volatile(0x57);
        }
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
        "get name" | "name" => {
            let mut s: heapless::String<64> = heapless::String::new();
            let callsign = crate::storage::get_callsign();
            let _ = write!(s, "  -> > {}\r\n", callsign.as_str());
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "get public.key" | "get public_key" | "public.key" | "get pubkey" => {
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(s, "  -> ");
            for b in &node.public_key {
                let _ = write!(s, "{:02x}", b);
            }
            let _ = write!(s, "\r\n");
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "get prv.key" | "get private_key" | "prv.key" => {
            let mut s: heapless::String<256> = heapless::String::new();
            let _ = write!(s, "  -> ");
            for b in &node.private_key {
                let _ = write!(s, "{:02x}", b);
            }
            let _ = write!(s, "\r\n");
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set prv.key ") => {
            let hex_str = s[12..].trim();
            let valid_hex = hex_str.bytes().all(|b| b.is_ascii_hexdigit());
            if valid_hex && hex_str.len() == 128 {
                // MeshCore's 64-byte expanded key (`scalar ‖ nonce_prefix`).
                let mut key = [0u8; 64];
                for i in 0..64 {
                    let hi = hex_to_nibble(hex_str.as_bytes()[i * 2]);
                    let lo = hex_to_nibble(hex_str.as_bytes()[i * 2 + 1]);
                    key[i] = (hi << 4) | lo;
                }
                match crate::meshcore::identity::LocalIdentity::from_private(
                    node.callsign.as_str(),
                    &key,
                ) {
                    Some(id) => {
                        node.private_key = key;
                        node.public_key = *id.pub_key();
                        crate::storage::set_custom_priv_key(key);
                        info!("Node expanded private & public key updated via CLI");
                        let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetPrivateKey(key));
                        let _ = write_all(sender, b"  -> OK\r\n").await;
                    }
                    None => {
                        let _ = write_all(sender, b"  -> Error, reserved public key\r\n").await;
                    }
                }
            } else if valid_hex && hex_str.len() == 64 {
                // Convenience: accept a 32-byte seed and expand it for storage.
                let mut seed = [0u8; 32];
                for i in 0..32 {
                    let hi = hex_to_nibble(hex_str.as_bytes()[i * 2]);
                    let lo = hex_to_nibble(hex_str.as_bytes()[i * 2 + 1]);
                    seed[i] = (hi << 4) | lo;
                }
                let id = crate::meshcore::identity::LocalIdentity::from_seed(
                    node.callsign.as_str(),
                    &seed,
                );
                let key = id.private_key_bytes();
                node.private_key = key;
                node.public_key = *id.pub_key();
                crate::storage::set_custom_priv_key(key);
                info!("Node seed imported and expanded; public key updated via CLI");
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetPrivateKey(key));
                let _ = write_all(sender, b"  -> OK\r\n").await;
            } else {
                let _ = write_all(sender, b"  -> Error, expected 64 or 128 hex chars\r\n").await;
            }
        }
        "set name" => {
            let _ = write_all(sender, b"  -> Error, bad chars\r\n").await;
        }
        s if s.starts_with("set name ") || s.starts_with("name ") => {
            let new_name = if let Some(rest) = s.strip_prefix("set name ") {
                rest.trim()
            } else if let Some(rest) = s.strip_prefix("name ") {
                rest.trim()
            } else {
                ""
            };

            let is_valid = !new_name.is_empty()
                && new_name.len() <= 31
                && !new_name.bytes().any(|b| {
                    b < 0x20
                        || b > 0x7E
                        || b == b'['
                        || b == b']'
                        || b == b'\\'
                        || b == b':'
                        || b == b','
                        || b == b'?'
                        || b == b'*'
                });

            if is_valid {
                crate::storage::set_callsign(new_name);
                node.callsign.clear();
                let _ = node.callsign.push_str(new_name);
                info!("Node callsign updated to: {}", new_name);
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetName(node.callsign.clone()));
                let _ = write_all(sender, b"  -> OK\r\n").await;
            } else {
                let _ = write_all(sender, b"  -> Error, bad chars\r\n").await;
            }
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
            let (f, sf, bw, cr, tx, mode) = RADIO_SETTINGS.lock(|r| {
                let cfg = r.borrow();
                (cfg.frequency_hz, cfg.sf, cfg.bw_khz, cfg.cr, cfg.tx_power_dbm, cfg.path_hash_mode)
            });
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(s, "  -> {:.3} MHz, SF{}, BW {} kHz, CR 4/{}, {} dBm, hash-size {}B\r\n", f as f32 / 1_000_000.0, sf, bw, cr, tx, mode + 1);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set freq ") => {
            let val = s[9..].trim();
            // Accepts e.g. "910525000" or "910.525"
            let freq_hz: Option<u32> = if val.contains('.') {
                // parse as MHz float
                let parts: heapless::Vec<&str, 2> = val.split('.').collect();
                if parts.len() == 2 {
                    let mhz: u32 = parts[0].parse().unwrap_or(0);
                    let mut khz_str = heapless::String::<6>::new();
                    let _ = khz_str.push_str(parts[1]);
                    while khz_str.len() < 6 {
                        let _ = khz_str.push('0');
                    }
                    let frac: u32 = khz_str.parse().unwrap_or(0);
                    Some(mhz * 1_000_000 + frac)
                } else {
                    None
                }
            } else {
                val.parse().ok()
            };

            if let Some(f) = freq_hz {
                if f >= 150_000_000 && f <= 960_000_000 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().frequency_hz = f;
                    });
                    crate::storage::request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetFrequency(f));
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(s, "  -> OK (Freq set to {:.3} MHz)\r\n", f as f32 / 1_000_000.0);
                    let _ = write_all(sender, s.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"  -> Error: frequency out of range (150-960 MHz)\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"  -> Error: invalid frequency format\r\n").await;
            }
        }
        "get tx" => {
            let tx = RADIO_SETTINGS.lock(|r| r.borrow().tx_power_dbm);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> {} dBm\r\n", tx);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set tx ") => {
            if let Ok(tx) = s[7..].trim().parse::<i8>() {
                if tx >= -9 && tx <= 22 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().tx_power_dbm = tx;
                    });
                    crate::storage::request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetTxPower(tx));
                    let mut s: heapless::String<32> = heapless::String::new();
                    let _ = write!(s, "  -> OK (TX set to {} dBm)\r\n", tx);
                    let _ = write_all(sender, s.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"  -> Error: TX power must be between -9 and 22 dBm\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"  -> Error: invalid TX power number\r\n").await;
            }
        }
        "get sf" => {
            let sf = RADIO_SETTINGS.lock(|r| r.borrow().sf);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> SF{}\r\n", sf);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set sf ") => {
            let val = s[7..].trim().trim_start_matches("sf").trim_start_matches("SF");
            if let Ok(sf) = val.parse::<u8>() {
                if sf >= 5 && sf <= 12 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().sf = sf;
                    });
                    crate::storage::request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetSpreadingFactor(sf));
                    let mut s: heapless::String<32> = heapless::String::new();
                    let _ = write!(s, "  -> OK (SF set to {})\r\n", sf);
                    let _ = write_all(sender, s.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"  -> Error: SF must be between 5 and 12\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"  -> Error: invalid SF\r\n").await;
            }
        }
        "get bw" => {
            let bw = RADIO_SETTINGS.lock(|r| r.borrow().bw_khz);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> {} kHz\r\n", bw);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set bw ") => {
            let val = s[7..].trim().trim_end_matches("k").trim_end_matches("kHz");
            let bw_khz: Option<u32> = if val == "62.5" || val == "62" {
                Some(62)
            } else if val == "125" {
                Some(125)
            } else if val == "250" {
                Some(250)
            } else if val == "500" {
                Some(500)
            } else {
                None
            };
            if let Some(bw) = bw_khz {
                RADIO_SETTINGS.lock(|r| {
                    r.borrow_mut().bw_khz = bw;
                });
                crate::storage::request_save_config();
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetBandwidth(bw));
                let mut s: heapless::String<32> = heapless::String::new();
                let _ = write!(s, "  -> OK (BW set to {} kHz)\r\n", bw);
                let _ = write_all(sender, s.as_bytes()).await;
            } else {
                let _ = write_all(sender, b"  -> Error: supported BW values are 62.5, 125, 250, 500\r\n").await;
            }
        }
        "get cr" => {
            let cr = RADIO_SETTINGS.lock(|r| r.borrow().cr);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> 4/{}\r\n", cr);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set cr ") => {
            let val = s[7..].trim();
            let cr_denom = if val == "5" || val == "4/5" {
                Some(5)
            } else if val == "6" || val == "4/6" {
                Some(6)
            } else if val == "7" || val == "4/7" {
                Some(7)
            } else if val == "8" || val == "4/8" {
                Some(8)
            } else {
                None
            };
            if let Some(cr) = cr_denom {
                RADIO_SETTINGS.lock(|r| {
                    r.borrow_mut().cr = cr;
                });
                crate::storage::request_save_config();
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetCodingRate(cr));
                let mut s: heapless::String<32> = heapless::String::new();
                let _ = write!(s, "  -> OK (CR set to 4/{})\r\n", cr);
                let _ = write_all(sender, s.as_bytes()).await;
            } else {
                let _ = write_all(sender, b"  -> Error: supported CR values are 4/5, 4/6, 4/7, 4/8\r\n").await;
            }
        }
        "get path.hash.mode" | "get hash-mode" | "get hash-size" => {
            let mode = RADIO_SETTINGS.lock(|r| r.borrow().path_hash_mode);
            let mut resp = heapless::String::<64>::new();
            let _ = write!(resp, "  -> path.hash.mode = {} ({} bytes/hop)\r\n", mode, mode + 1);
            let _ = write_all(sender, resp.as_bytes()).await;
        }
        s if s.starts_with("set path.hash.mode ") || s.starts_with("set hash-mode ") => {
            let val_str = s.split_whitespace().last().unwrap_or("");
            if let Ok(mode) = val_str.parse::<u8>() {
                if mode <= 2 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().path_hash_mode = mode;
                    });
                    crate::storage::request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetPathHashMode(mode));
                    let mut resp = heapless::String::<64>::new();
                    let _ = write!(resp, "  -> OK (path.hash.mode set to {}, {} bytes/hop)\r\n", mode, mode + 1);
                    let _ = write_all(sender, resp.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"  -> Error: path.hash.mode must be 0 (1B), 1 (2B), or 2 (3B)\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"  -> Error: invalid number\r\n").await;
            }
        }
        s if s.starts_with("set hash-size ") => {
            let val_str = s.split_whitespace().last().unwrap_or("");
            if let Ok(size) = val_str.parse::<u8>() {
                if (1..=3).contains(&size) {
                    let mode = size - 1;
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().path_hash_mode = mode;
                    });
                    crate::storage::request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetPathHashMode(mode));
                    let mut resp = heapless::String::<64>::new();
                    let _ = write!(resp, "  -> OK (hash-size set to {} bytes, mode {})\r\n", size, mode);
                    let _ = write_all(sender, resp.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"  -> Error: hash-size must be 1, 2, or 3 bytes\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"  -> Error: invalid number\r\n").await;
            }
        }
        "get repeat" | "get role" => {
            let is_rep = RADIO_SETTINGS.lock(|r| r.borrow().is_repeater);
            if is_rep {
                let _ = write_all(sender, b"  -> on (repeater)\r\n").await;
            } else {
                let _ = write_all(sender, b"  -> off (client)\r\n").await;
            }
        }
        s if s.starts_with("set repeat ") || s.starts_with("set role ") => {
            let val = if s.starts_with("set repeat ") { &s[11..] } else { &s[9..] }.trim();
            let is_rep = val == "on" || val == "1" || val == "repeater" || val == "true";
            RADIO_SETTINGS.lock(|r| {
                r.borrow_mut().is_repeater = is_rep;
            });
            crate::storage::request_save_config();
            let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetRepeat(is_rep));
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "  -> OK (Repeater: {})\r\n", if is_rep { "on" } else { "off" });
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("send ") => {
            let msg = s[5..].trim();
            if msg.is_empty() {
                let _ = write_all(sender, b"  -> Error: message cannot be empty\r\n").await;
            } else {
                let mut s_msg = heapless::String::<128>::new();
                let _ = s_msg.push_str(msg);
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SendGroupText {
                    channel_idx: 0,
                    timestamp: get_device_time(),
                    text: s_msg,
                });
                RADIO_TELEMETRY.lock(|t| {
                    let mut tel = t.borrow_mut();
                    tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                });
                let _ = write_all(sender, b"  -> OK (Group message broadcast queued)\r\n").await;
            }
        }
        s if s.starts_with("msg ") => {
            let rest = s[4..].trim();
            if let Some(space_idx) = rest.find(' ') {
                let hex_str = &rest[..space_idx];
                let msg = rest[space_idx + 1..].trim();
                if hex_str.len() == 8 {
                    let mut dest = [0u8; 4];
                    let mut valid = true;
                    for i in 0..4 {
                        if let (Some(hi), Some(lo)) = (
                            hex_to_nibble_opt(hex_str.as_bytes()[i * 2]),
                            hex_to_nibble_opt(hex_str.as_bytes()[i * 2 + 1]),
                        ) {
                            dest[i] = (hi << 4) | lo;
                        } else {
                            valid = false;
                            break;
                        }
                    }
                    if valid && !msg.is_empty() {
                        let prefix = DISCOVERED_CONTACTS.lock(|c| {
                            c.borrow()
                                .iter()
                                .find(|ct| &ct.pub_key[..4] == &dest[..])
                                .map(|ct| {
                                    let mut p = [0u8; 6];
                                    p.copy_from_slice(&ct.pub_key[..6]);
                                    p
                                })
                        });
                        match prefix {
                            Some(dest_prefix) => {
                                let mut s_msg = heapless::String::<160>::new();
                                if s_msg.push_str(msg).is_ok() {
                                    let _ = USB_COMMAND_CHANNEL.try_send(
                                        UsbCommand::SendDirectText {
                                            dest_prefix,
                                            timestamp: get_device_time(),
                                            txt_type: 0,
                                            attempt: 0,
                                            text: s_msg,
                                        },
                                    );
                                    RADIO_TELEMETRY.lock(|t| {
                                        let mut tel = t.borrow_mut();
                                        tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                                    });
                                    let mut out_str = heapless::String::<72>::new();
                                    let _ = write!(
                                        out_str,
                                        "  -> OK (Direct message queued to {})\r\n",
                                        hex_str
                                    );
                                    let _ = write_all(sender, out_str.as_bytes()).await;
                                } else {
                                    let _ = write_all(
                                        sender,
                                        b"  -> Error: message exceeds 160 bytes\r\n",
                                    )
                                    .await;
                                }
                            }
                            None => {
                                let _ = write_all(
                                    sender,
                                    b"  -> Error: no contact with that 4-byte hash\r\n",
                                )
                                .await;
                            }
                        }
                    } else {
                        let _ = write_all(sender, b"  -> Error: invalid hex ID or empty text\r\n").await;
                    }
                } else {
                    let _ = write_all(sender, b"  -> Error: target ID must be 8 hex characters (4 bytes)\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"  -> Usage: msg <8-hex-id> <message>\r\n").await;
            }
        }
        "contacts" | "get contacts" => {
            let contacts_snapshot: heapless::Vec<DiscoveredContact, 32> = DISCOVERED_CONTACTS.lock(|c| c.borrow().clone());
            if contacts_snapshot.is_empty() {
                let _ = write_all(sender, b"  -> No contacts discovered yet (listening for adverts...)\r\n").await;
            } else {
                let mut hdr = heapless::String::<64>::new();
                let _ = write!(hdr, "  -> Discovered Contacts ({}):\r\n", contacts_snapshot.len());
                let _ = write_all(sender, hdr.as_bytes()).await;
                let _ = write_all(sender, b"  -> ID        CALLSIGN        RSSI     SNR   BAT   SEEN\r\n").await;
                for c in contacts_snapshot.iter() {
                    let mut s = heapless::String::<128>::new();
                    let _ = write!(
                        s,
                        "  -> {:02x}{:02x}{:02x}{:02x}  {:<14}  {:>4}dBm  {:>3}dB  {:>3}% {:>3}%  {}s\r\n",
                        c.node_id[0], c.node_id[1], c.node_id[2], c.node_id[3],
                        c.callsign.as_str(),
                        c.rssi, c.snr, c.latitude, c.longitude, c.last_seen_epoch
                    );
                    let _ = write_all(sender, s.as_bytes()).await;
                }
            }
        }
        "clear contacts" => {
            DISCOVERED_CONTACTS.lock(|c| {
                c.borrow_mut().clear();
            });
            let _ = write_all(sender, b"  -> OK (Contacts cleared)\r\n").await;
        }
        "messages" | "inbox" | "get messages" => {
            let msg_snapshot: heapless::Vec<ReceivedMessage, 16> = MESSAGE_LOG.lock(|m| m.borrow().clone());
            if msg_snapshot.is_empty() {
                let _ = write_all(sender, b"  -> No messages received yet\r\n").await;
            } else {
                let _ = write_all(sender, b"  -> === Recent Messages ===\r\n").await;
                for msg in msg_snapshot.iter() {
                    let mut s = heapless::String::<256>::new();
                    let mode = if msg.is_direct { "[DM]" } else { "[GRP]" };
                    let _ = write!(
                        s,
                        "  -> {} <{}> {}: {} (RSSI: {}dBm, SNR: {}dB)\r\n",
                        mode, msg.sender.as_str(), msg.timestamp, msg.text.as_str(), msg.rssi, msg.snr
                    );
                    let _ = write_all(sender, s.as_bytes()).await;
                }
            }
        }
        "clear messages" => {
            MESSAGE_LOG.lock(|m| {
                m.borrow_mut().clear();
            });
            let _ = write_all(sender, b"  -> OK (Messages cleared)\r\n").await;
        }
        "stats-radio" => {
            let (tx, rx, rssi, snr, _has_rx) = RADIO_TELEMETRY.lock(|t| {
                let tel = t.borrow();
                (tel.tx_beacons, tel.rx_packets, tel.last_rssi, tel.last_snr, tel.has_rx)
            });
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"noise_floor\":-118,\"last_rssi\":{},\"last_snr\":{:.2},\"tx_air_secs\":{},\"rx_air_secs\":{}}}\r\n",
                rssi, snr as f32, tx, rx
            );
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "stats-ble" | "get ble" => {
            let st = crate::companion::ble::BLE_STATUS.load(core::sync::atomic::Ordering::Relaxed);
            let mut s: heapless::String<128> = heapless::String::new();
            match st {
                0 => { let _ = write!(s, "  -> BLE: Not started\r\n"); }
                1 => { let _ = write!(s, "  -> BLE: Probing SoftDevice...\r\n"); }
                2 => { let _ = write!(s, "  -> BLE: SoftDevice enabled, setting vector table...\r\n"); }
                3 => { let _ = write!(s, "  -> BLE: Active & Advertising (MeshCore-M1 NUS)\r\n"); }
                code if code >= 0x1000 => {
                    let w0 = unsafe { *(0x1000 as *const u32) };
                    let w1 = unsafe { *(0x1004 as *const u32) };
                    let _ = write!(s, "  -> BLE: sd_softdevice_enable failed (code 0x{:04x}, flash@0x1000: 0x{:08x} 0x{:08x})\r\n", code & 0xFFF, w0, w1);
                }
                code => {
                    let _ = write!(s, "  -> BLE: Error state {}\r\n", code);
                }
            }
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "stats-core" => {
            let uptime = embassy_time::Instant::now().as_secs();
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(s, "  -> {{\"battery_mv\":4120,\"uptime_secs\":{},\"errors\":0,\"queue_len\":0}}\r\n", uptime);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "stats-packets" => {
            let (tx, rx) = RADIO_TELEMETRY.lock(|t| {
                let tel = t.borrow();
                (tel.tx_beacons, tel.rx_packets)
            });
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(
                s,
                "  -> {{\"recv\":{},\"sent\":{},\"flood_tx\":{},\"direct_tx\":0,\"flood_rx\":{},\"direct_rx\":0,\"recv_errors\":0}}\r\n",
                rx, tx, tx, rx
            );
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "clear stats" => {
            RADIO_TELEMETRY.lock(|t| {
                let mut tel = t.borrow_mut();
                tel.tx_beacons = 0;
                tel.rx_packets = 0;
                tel.has_rx = false;
            });
            let _ = write_all(sender, b"  -> OK\r\n").await;
        }
        "advert" => {
            RADIO_TELEMETRY.lock(|t| {
                let mut tel = t.borrow_mut();
                tel.tx_beacons = tel.tx_beacons.saturating_add(1);
            });
            let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::TriggerAdvert { flood: true });
            let _ = write_all(sender, b"  -> OK (MeshCore Advert broadcast queued)\r\n").await;
        }
        "time" | "get time" => {
            let mut s: heapless::String<32> = heapless::String::new();
            let epoch = get_device_time();
            let _ = write!(s, "  -> {}\r\n", epoch);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set time ") || s.starts_with("time ") => {
            let time_str = if s.starts_with("set time ") { &s[9..] } else { &s[5..] }.trim();
            let mut epoch = 0u64;
            for &b in time_str.as_bytes() {
                if b >= b'0' && b <= b'9' {
                    epoch = epoch.wrapping_mul(10).wrapping_add((b - b'0') as u64);
                }
            }
            node.epoch_time = epoch;
            set_device_time(epoch as u32);
            let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetTime(epoch));
            let _ = write_all(sender, b"  -> OK (Clock synchronized)\r\n").await;
        }
        "channels" | "channel" => {
            let mut buf: heapless::String<256> = heapless::String::new();
            let _ = write!(buf, "Channels:\r\n");
            CHANNELS.lock(|c| {
                let list = c.borrow();
                for (i, ch) in list.iter().enumerate() {
                    let name_len = ch.name.iter().position(|&b| b == 0).unwrap_or(32);
                    if name_len > 0 {
                        let name_str = core::str::from_utf8(&ch.name[..name_len]).unwrap_or("<invalid>");
                        let _ = write!(buf, "  [{}] {}\r\n", i, name_str);
                    }
                }
            });
            let _ = write_all(sender, buf.as_bytes()).await;
        }
        "clock sync" | "sync_time" => {
            let _ = write_all(sender, b"  -> OK\r\n").await;
        }
        "dfu" | "bootloader" => {
            let _ = write_all(sender, b"  -> Entering DFU bootloader...\r\n").await;
            embassy_time::Timer::after_millis(50).await;
            reboot_to_bootloader();
        }
        "reboot" => {
            let _ = write_all(sender, b"  -> Rebooting device...\r\n").await;
            embassy_time::Timer::after_millis(50).await;
            cortex_m::peripheral::SCB::sys_reset();
        }
        "save" => {
            crate::storage::request_save_config();
            let _ = write_all(sender, b"  -> OK (Config save queued to flash)\r\n").await;
        }
        "help" => {
            let _ = write_all(sender, b"  -> Commands:\r\n  ->   Identity: get/set name, get/set prv.key, get public.key\r\n  ->   Radio:    get/set freq, get/set tx, get/set sf, get/set bw, get/set cr, get/set repeat, get/set hash-mode, stats-radio\r\n  ->   Mesh:     send <text>, msg <hex_id> <text>, messages, contacts, advert, time, channels\r\n  ->   System:   save, ver, board, stats-core, stats-packets, clear stats/contacts/messages, reboot\r\n").await;
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
pub async fn cdc_acm_task(
    class: CdcAcmClass<'static, UsbDriver>,
    public_key: [u8; 32],
    private_key: [u8; 64],
) -> ! {
    let (mut sender, mut receiver, control) = class.split_with_control();
    let callsign = crate::storage::get_callsign();
    let mut node = CompanionNode::new(
        callsign.as_str(),
        CurrentBoard::NAME,
        CurrentBoard::CAPABILITIES,
        public_key,
        private_key,
    );

    let mut frame_reader = UsbFrameReader::new();
    let mut text_line_buf = [0u8; 128];
    let mut text_line_len = 0usize;
    let mut in_binary_mode = false;

    loop {
        receiver.wait_connection().await;
        info!("USB CDC-ACM Connected");

        if receiver.line_coding().data_rate() == 1200 {
            embassy_time::Timer::after_millis(150).await;
            reboot_to_bootloader();
        }

        loop {
            let changed_fut = control.control_changed();
            let mut rx_buf = [0u8; 64];
            let read_fut = receiver.read_packet(&mut rx_buf);

            match embassy_futures::select::select(changed_fut, read_fut).await {
                embassy_futures::select::Either::First(_) => {
                    let rate = receiver.line_coding().data_rate();
                    if rate == 1200 {
                        embassy_time::Timer::after_millis(150).await;
                        reboot_to_bootloader();
                    }
                }
                embassy_futures::select::Either::Second(read_res) => {
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
                                    if let Some(payload) = frame_reader.push_byte(b) {
                                        in_binary_mode = false;
                                        let mut responses = heapless::Vec::<heapless::Vec<u8, 200>, 10>::new();
                                        node.handle_command(payload, |resp| {
                                            let mut v = heapless::Vec::new();
                                            let _ = v.extend_from_slice(resp);
                                            let _ = responses.push(v);
                                        });
                                        for resp in responses {
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
