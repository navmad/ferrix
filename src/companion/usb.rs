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
    /// Off-grid "client repeat" mode (`client_repeat`). Distinct from
    /// the repeater node role: it enables forwarding only on an allowed
    /// off-grid frequency.
    pub client_repeat: bool,
    /// Frequency to return to when off-grid mode is turned off.
    pub offgrid_home_freq: u32,
    pub path_hash_mode: u8,
}

pub static RADIO_SETTINGS: Mutex<SharedRawMutex, RefCell<RadioSettings>> = Mutex::new(RefCell::new(RadioSettings {
    frequency_hz: 910_525_000,
    tx_power_dbm: 22,
    sf: 7,
    bw_khz: 62,
    cr: 5,
    is_repeater: false,
    client_repeat: false,
    offgrid_home_freq: 0,
    path_hash_mode: 0,
}));

/// Contact auto-add policy, shared by every companion transport and the main
/// loop. `manual_add_contacts` bit 0 selects the policy: 0 = add every advert,
/// 1 = add only the types enabled in `config`.
#[derive(Clone, Copy, Debug)]
pub struct AutoAddSettings {
    pub manual_add_contacts: u8,
    pub config: u8,
    pub max_hops: u8,
}

pub static AUTOADD_SETTINGS: Mutex<SharedRawMutex, RefCell<AutoAddSettings>> =
    Mutex::new(RefCell::new(AutoAddSettings {
        // MeshCore's fresh-node default: auto-add all advert types. The type
        // mask starts empty because it is only consulted when manual mode is
        // selected.
        manual_add_contacts: 0,
        config: 0x00,
        max_hops: 0,
    }));

/// Companion defaults on nRF52840: 350 contacts, 40 channels.
pub const MAX_CONTACTS: usize = 350;
/// Companion default group-channel count.
pub const MAX_CHANNELS: usize = 40;

pub static DISCOVERED_CONTACTS: Mutex<SharedRawMutex, RefCell<heapless::Vec<DiscoveredContact, MAX_CONTACTS>>> =
    Mutex::new(RefCell::new(heapless::Vec::new()));

/// Number of retained messages (matched by the persistent snapshot size).
pub const MAX_MESSAGES: usize = 16;

pub static MESSAGE_LOG: Mutex<SharedRawMutex, RefCell<heapless::Vec<ReceivedMessage, MAX_MESSAGES>>> =
    Mutex::new(RefCell::new(heapless::Vec::new()));

#[derive(Clone, Copy, Debug)]
pub struct ChannelData {
    pub name: [u8; 32],
    pub secret: [u8; 16],
}

/// Convert the internal integer kHz bandwidth to the precise Hz value used by
/// the companion protocol and the SX1262 (62 -> 62.5 kHz, 7 -> 7.8 kHz, ...).
pub fn bw_khz_to_hz(bw_khz: u32) -> u32 {
    match bw_khz {
        7 => 7_800,
        10 => 10_400,
        15 => 15_600,
        20 => 20_800,
        31 => 31_250,
        41 => 41_670,
        62 => 62_500,
        other => other * 1000,
    }
}

/// Parse a frequency from the USB CLI. Accepts:
/// - `910.525` (MHz, decimal)
/// - `910525` (kHz, integer)
/// - `910525000` (Hz)
/// - `910` (MHz)
pub(crate) fn parse_freq_hz(val: &str) -> Option<u32> {
    if val.contains('.') {
        let (int_part, frac_part) = val.split_once('.')?;
        let mhz: u32 = int_part.parse().ok()?;
        let mut frac = heapless::String::<6>::new();
        for c in frac_part.chars() {
            if !c.is_ascii_digit() || frac.len() == 6 {
                return None;
            }
            let _ = frac.push(c);
        }
        while frac.len() < 6 {
            let _ = frac.push('0');
        }
        let frac_num: u32 = frac.parse().ok()?;
        mhz.checked_mul(1_000_000)?.checked_add(frac_num)
    } else {
        let n: u32 = val.parse().ok()?;
        if n >= 100_000_000 {
            Some(n) // Hz
        } else if n >= 100_000 {
            Some(n.checked_mul(1000)?) // kHz
        } else if n >= 100 {
            Some(n.checked_mul(1_000_000)?) // MHz
        } else {
            None
        }
    }
}

/// Parse a bandwidth from the USB CLI. Accepts `62.5`, `62`, or `62500`.
fn parse_bw_khz(val: &str) -> Option<u32> {
    let val = val.trim_end_matches("kHz").trim_end_matches("k").trim();
    let exact = match val {
        "7.8" => Some(7),
        "10.4" => Some(10),
        "15.6" => Some(15),
        "20.8" => Some(20),
        "31.25" => Some(31),
        "41.67" => Some(41),
        "62.5" => Some(62),
        _ => None,
    };
    if exact.is_some() {
        return exact;
    }
    let n: u32 = val.parse().ok()?;
    // >= 7000 is Hz (62500 -> 62); smaller values are already kHz.
    let bw = if n >= 7_000 { n / 1000 } else { n };
    matches!(bw, 7 | 10 | 15 | 20 | 31 | 41 | 62 | 125 | 250 | 500).then_some(bw)
}

/// Parse a coding rate denominator: `5`, `4/5`, `6`, `4/6`, ...
fn parse_cr(val: &str) -> Option<u8> {
    match val.trim() {
        "5" | "4/5" => Some(5),
        "6" | "4/6" => Some(6),
        "7" | "4/7" => Some(7),
        "8" | "4/8" => Some(8),
        _ => None,
    }
}

/// Render a bandwidth with trailing zeros stripped: 62.5, 125, 250, 500, 31.25...
fn format_bw(bw_khz: u32) -> heapless::String<16> {
    let hz = bw_khz_to_hz(bw_khz);
    let mut s = heapless::String::<16>::new();
    if hz % 1000 == 0 {
        let _ = write!(s, "{}", hz / 1000);
    } else if hz % 100 == 0 {
        let _ = write!(s, "{}.{}", hz / 1000, (hz % 1000) / 100);
    } else if hz % 10 == 0 {
        let _ = write!(s, "{}.{:02}", hz / 1000, (hz % 1000) / 10);
    } else {
        let _ = write!(s, "{}.{:03}", hz / 1000, hz % 1000);
    }
    s
}

/// Allowed client-repeat (off-grid) frequencies (kHz).
pub const OFFGRID_FREQ_RANGES: [(u32, u32); 3] = [
    (433_000, 433_000),
    (869_495, 869_495),
    (918_000, 918_000),
];

pub fn is_offgrid_freq_khz(khz: u32) -> bool {
    OFFGRID_FREQ_RANGES
        .iter()
        .any(|&(lo, hi)| khz >= lo && khz <= hi)
}

pub fn is_offgrid_freq_hz(hz: u32) -> bool {
    is_offgrid_freq_khz(hz / 1000)
}

/// Off-grid active: client repeat set AND on an allowed frequency.
pub fn offgrid_active() -> bool {
    RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        s.client_repeat && is_offgrid_freq_hz(s.frequency_hz)
    })
}

/// Effective forwarding flag: a dedicated repeater role, or off-grid mode.
pub fn forwarding_enabled() -> bool {
    RADIO_SETTINGS.lock(|r| r.borrow().is_repeater) || offgrid_active()
}

/// Nearest allowed off-grid frequency to `freq_hz`, or None above 960 MHz.
pub fn nearest_offgrid_freq_hz(freq_hz: u32) -> Option<u32> {
    let khz = freq_hz / 1000;
    if khz > 960_000 {
        return None;
    }
    let mut best = None;
    let mut best_dist = u32::MAX;
    for (lo, hi) in OFFGRID_FREQ_RANGES {
        let c = khz.clamp(lo, hi);
        let dist = c.abs_diff(khz);
        if dist < best_dist {
            best_dist = dist;
            best = Some(c * 1000);
        }
    }
    best
}

pub struct OffgridChange {
    pub changed: bool,
    pub freq_changed: bool,
    pub no_band: bool,
}

/// Toggle client-repeat/off-grid mode. Turning it on
/// retunes to the nearest allowed band and remembers the previous frequency;
/// turning it off restores that frequency.
pub fn offgrid_set(on: bool) -> OffgridChange {
    RADIO_SETTINGS.lock(|r| {
        let mut s = r.borrow_mut();
        if on {
            if s.client_repeat && is_offgrid_freq_hz(s.frequency_hz) {
                return OffgridChange { changed: false, freq_changed: false, no_band: false };
            }
            if is_offgrid_freq_hz(s.frequency_hz) {
                s.client_repeat = true;
                s.offgrid_home_freq = 0;
                return OffgridChange { changed: true, freq_changed: false, no_band: false };
            }
            if let Some(hz) = nearest_offgrid_freq_hz(s.frequency_hz) {
                s.offgrid_home_freq = s.frequency_hz;
                s.frequency_hz = hz;
                s.client_repeat = true;
                return OffgridChange { changed: true, freq_changed: true, no_band: false };
            }
            OffgridChange { changed: false, freq_changed: false, no_band: true }
        } else {
            let was_set = s.client_repeat;
            let home = if s.client_repeat && is_offgrid_freq_hz(s.frequency_hz) {
                s.offgrid_home_freq
            } else {
                0
            };
            s.client_repeat = false;
            s.offgrid_home_freq = 0;
            if home != 0 {
                s.frequency_hz = home;
                OffgridChange { changed: true, freq_changed: true, no_band: false }
            } else {
                OffgridChange { changed: was_set, freq_changed: false, no_band: false }
            }
        }
    })
}

pub const DEFAULT_PUBLIC_SECRET: [u8; 16] = [
    0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a,
    0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72,
];

pub(crate) const fn default_channels() -> [ChannelData; MAX_CHANNELS] {
    let mut chs = [ChannelData {
        name: [0u8; 32],
        secret: [0u8; 16],
    }; MAX_CHANNELS];
    chs[0].name[0] = b'P';
    chs[0].name[1] = b'u';
    chs[0].name[2] = b'b';
    chs[0].name[3] = b'l';
    chs[0].name[4] = b'i';
    chs[0].name[5] = b'c';
    chs[0].secret = DEFAULT_PUBLIC_SECRET;
    chs
}

pub static CHANNELS: Mutex<SharedRawMutex, RefCell<[ChannelData; MAX_CHANNELS]>> =
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
        text: heapless::String<160>,
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
    SetRadio {
        frequency_hz: u32,
        bw_khz: u32,
        sf: u8,
        cr: u8,
    },
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
/// Send one companion response over the USB binary framing.
async fn send_usb_companion_frame(sender: &mut Sender<'static, UsbDriver>, resp: &[u8]) {
    let mut out_frame = [0u8; 300];
    if let Ok(flen) = encode_usb_frame(resp, &mut out_frame) {
        let _ = write_all(sender, &out_frame[..flen]).await;
    }
}

/// Stream `GET_CONTACTS` over USB with the same framing as the other responses.
async fn stream_contacts_usb(sender: &mut Sender<'static, UsbDriver>) {
    use crate::companion::protocol;

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
        let resp = protocol::serialize_contact(&c);
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

/// Partial private-key import. Some terminals split long lines, so the CLI
/// accepts a 128-character key in several chunks.
struct KeyImport {
    buf: [u8; 128],
    len: usize,
}

static KEY_IMPORT: Mutex<SharedRawMutex, RefCell<KeyImport>> =
    Mutex::new(RefCell::new(KeyImport {
        buf: [0u8; 128],
        len: 0,
    }));

fn normalize_hex(input: &str) -> Option<heapless::String<256>> {
    let mut out = heapless::String::<256>::new();
    let mut s = input.trim();
    if s.len() >= 2 && (s.starts_with("0x") || s.starts_with("0X")) {
        s = &s[2..];
    }
    for c in s.chars() {
        if c.is_ascii_hexdigit() {
            out.push(c).ok()?;
        } else if c.is_ascii_whitespace() || matches!(c, ':' | '-' | '_' | '\'' | '"' | ',') {
            continue;
        } else {
            return None;
        }
    }
    Some(out)
}

/// Append a normalized hex chunk. Returns `(length, Some(key))` once 128
/// characters have been collected, or `Err(())` if the total would overflow.
fn key_import_append(hex: &str) -> Result<(usize, Option<[u8; 64]>), ()> {
    KEY_IMPORT.lock(|cell| {
        let mut state = cell.borrow_mut();

        // If a complete key is pasted again while a partial one is buffered,
        // restart with the complete key instead of overflowing.
        if hex.len() == 128 {
            state.len = 0;
        } else if state.len + hex.len() > 128 {
            state.len = 0;
            return Err(());
        }
        for &b in hex.as_bytes() {
            let idx = state.len;
            state.buf[idx] = b;
            state.len = idx + 1;
        }
        if state.len == 128 {
            let mut key = [0u8; 64];
            for i in 0..64 {
                key[i] = (hex_to_nibble(state.buf[i * 2]) << 4)
                    | hex_to_nibble(state.buf[i * 2 + 1]);
            }
            state.len = 0;
            Ok((128, Some(key)))
        } else {
            Ok((state.len, None))
        }
    })
}

async fn apply_private_key(
    sender: &mut Sender<'static, UsbDriver>,
    node: &mut CompanionNode,
    key: [u8; 64],
) {
    match crate::meshcore::identity::LocalIdentity::from_private(node.callsign.as_str(), &key) {
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

    // If a private key was split across lines, keep consuming hex chunks.
    // Any non-hex line cancels the pending import and is handled normally.
    if KEY_IMPORT.lock(|k| k.borrow().len) > 0 {
        if let Some(hex) = normalize_hex(trimmed) {
            match key_import_append(hex.as_str()) {
                Ok((_, Some(key))) => {
                    apply_private_key(sender, node, key).await;
                    return;
                }
                Ok((len, None)) => {
                    let mut s: heapless::String<96> = heapless::String::new();
                    let _ = write!(
                        s,
                        "  -> Key incomplete ({}/128), paste the remaining {} hex chars\r\n",
                        len,
                        128 - len
                    );
                    let _ = write_all(sender, s.as_bytes()).await;
                    return;
                }
                Err(()) => {
                    let _ = write_all(sender, b"  -> Error, key too long\r\n").await;
                    return;
                }
            }
        }
        KEY_IMPORT.lock(|k| k.borrow_mut().len = 0);
    }

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
        s if s == "set prv.key" || s.starts_with("set prv.key ") => {
            let arg = s.strip_prefix("set prv.key").unwrap_or("").trim();
            let normalized = if arg.is_empty() {
                Some(heapless::String::<256>::new())
            } else {
                normalize_hex(arg)
            };

            match normalized {
                Some(hex) if hex.len() == 128 => {
                    let mut key = [0u8; 64];
                    for i in 0..64 {
                        key[i] = (hex_to_nibble(hex.as_bytes()[i * 2]) << 4)
                            | hex_to_nibble(hex.as_bytes()[i * 2 + 1]);
                    }
                    apply_private_key(sender, node, key).await;
                }
                Some(hex) if hex.len() == 64 => {
                    // Convenience: accept a 32-byte seed and expand it.
                    let mut seed = [0u8; 32];
                    for i in 0..32 {
                        seed[i] = (hex_to_nibble(hex.as_bytes()[i * 2]) << 4)
                            | hex_to_nibble(hex.as_bytes()[i * 2 + 1]);
                    }
                    let id = crate::meshcore::identity::LocalIdentity::from_seed(
                        node.callsign.as_str(),
                        &seed,
                    );
                    let key = id.private_key_bytes();
                    apply_private_key(sender, node, key).await;
                }
                Some(hex) if hex.len() < 128 => {
                    match key_import_append(hex.as_str()) {
                        Ok((_, Some(key))) => apply_private_key(sender, node, key).await,
                        Ok((len, None)) => {
                            let mut out: heapless::String<96> = heapless::String::new();
                            let _ = write!(
                                out,
                                "  -> Key incomplete ({}/128), paste the remaining {} hex chars\r\n",
                                len,
                                128 - len
                            );
                            let _ = write_all(sender, out.as_bytes()).await;
                        }
                        Err(()) => {
                            let _ = write_all(sender, b"  -> Error, key too long\r\n").await;
                        }
                    }
                }
                Some(hex) => {
                    let mut out: heapless::String<64> = heapless::String::new();
                    let _ = write!(
                        out,
                        "  -> Error, expected 64 or 128 hex chars (got {})\r\n",
                        hex.len()
                    );
                    let _ = write_all(sender, out.as_bytes()).await;
                }
                None => {
                    let _ = write_all(sender, b"  -> Error, invalid hex characters\r\n").await;
                }
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
        "get freq" => {
            let f = RADIO_SETTINGS.lock(|r| r.borrow().frequency_hz);
            let mut s: heapless::String<48> = heapless::String::new();
            let _ = write!(s, "> {:.3}\r\n", f as f32 / 1_000_000.0);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "get radio" | "radio" => {
            let (f, sf, bw, cr) = RADIO_SETTINGS.lock(|r| {
                let cfg = r.borrow();
                (cfg.frequency_hz, cfg.sf, cfg.bw_khz, cfg.cr)
            });
            let mut s: heapless::String<64> = heapless::String::new();
            let bw_str = format_bw(bw);
            let _ = write!(s, "> {:.3},{},{},{}\r\n", f as f32 / 1_000_000.0, bw_str.as_str(), sf, cr);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s == "set radio" || s.starts_with("set radio ") => {
            let arg = s["set radio".len()..].trim();
            let parsed = if arg.eq_ignore_ascii_case("default") {
                Some((910_525_000u32, 62u32, 7u8, 5u8))
            } else if arg.is_empty() {
                None
            } else {
                let mut parts = arg
                    .split(|c: char| c == ',' || c.is_whitespace())
                    .filter(|p| !p.is_empty());
                match (parts.next(), parts.next(), parts.next(), parts.next(), parts.next()) {
                    (Some(f), Some(b), Some(sf), Some(cr), None) => {
                        match (parse_freq_hz(f), parse_bw_khz(b), sf.parse::<u8>().ok(), parse_cr(cr)) {
                            (Some(freq), Some(bw), Some(sf), Some(cr))
                                if (5..=12).contains(&sf) =>
                            {
                                Some((freq, bw, sf, cr))
                            }
                            _ => None,
                        }
                    }
                    _ => None,
                }
            };

            match parsed {
                Some((freq, bw, sf, cr)) if (150_000_000..=960_000_000).contains(&freq) => {
                    RADIO_SETTINGS.lock(|r| {
                        let mut st = r.borrow_mut();
                        st.frequency_hz = freq;
                        st.bw_khz = bw;
                        st.sf = sf;
                        st.cr = cr;
                        // The caller owns the new frequency, so the remembered
                        // off-grid home is no longer valid. Leaving an allowed
                        // band also clears the client-repeat flag.
                        st.offgrid_home_freq = 0;
                        if !is_offgrid_freq_hz(freq) {
                            st.client_repeat = false;
                        }
                    });
                    crate::storage::request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetRadio {
                        frequency_hz: freq,
                        bw_khz: bw,
                        sf,
                        cr,
                    });
                    let bw_str = format_bw(bw);
                    let mut resp: heapless::String<64> = heapless::String::new();
                    let _ = write!(
                        resp,
                        "OK - {:.3},{},{},{}\r\n",
                        freq as f32 / 1_000_000.0,
                        bw_str.as_str(),
                        sf,
                        cr
                    );
                    let _ = write_all(sender, resp.as_bytes()).await;
                }
                _ => {
                    let _ = write_all(
                        sender,
                        b"Error: freq 150-960, bw 7-500, sf 5-12, cr 5-8, or default\r\n",
                    )
                    .await;
                }
            }
        }
        s if s.starts_with("set freq ") => {
            let val = s[9..].trim();
            // Accepts "910.525" (MHz), "910525" (kHz), or "910525000" (Hz).
            if let Some(f) = parse_freq_hz(val) {
                if f >= 150_000_000 && f <= 960_000_000 {
                    RADIO_SETTINGS.lock(|r| {
                        r.borrow_mut().frequency_hz = f;
                    });
                    crate::storage::request_save_config();
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetFrequency(f));
                    let mut s: heapless::String<64> = heapless::String::new();
                    let _ = write!(s, "OK - {:.3}\r\n", f as f32 / 1_000_000.0);
                    let _ = write_all(sender, s.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"Error: frequency out of range (150-960 MHz)\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"Error: invalid frequency format\r\n").await;
            }
        }
        "get tx" => {
            let tx = RADIO_SETTINGS.lock(|r| r.borrow().tx_power_dbm);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", tx);
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
                    let _ = write!(s, "OK - tx power={} dBm\r\n", tx);
                    let _ = write_all(sender, s.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"Error: TX power must be between -9 and 22 dBm\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"Error: invalid TX power number\r\n").await;
            }
        }
        "get sf" => {
            let sf = RADIO_SETTINGS.lock(|r| r.borrow().sf);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", sf);
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
                    let _ = write!(s, "OK - {}\r\n", sf);
                    let _ = write_all(sender, s.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"Error: SF must be between 5 and 12\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"Error: invalid SF\r\n").await;
            }
        }
        "get bw" => {
            let bw = RADIO_SETTINGS.lock(|r| r.borrow().bw_khz);
            let mut s: heapless::String<32> = heapless::String::new();
            let bw_str = format_bw(bw);
            let _ = write!(s, "> {}\r\n", bw_str.as_str());
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set bw ") => {
            let val = s[7..].trim();
            if let Some(bw) = parse_bw_khz(val) {
                RADIO_SETTINGS.lock(|r| {
                    r.borrow_mut().bw_khz = bw;
                });
                crate::storage::request_save_config();
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetBandwidth(bw));
                let mut s: heapless::String<32> = heapless::String::new();
                let bw_str = format_bw(bw);
                let _ = write!(s, "OK - {}\r\n", bw_str.as_str());
                let _ = write_all(sender, s.as_bytes()).await;
            } else {
                let _ = write_all(sender, b"Error: supported BW are 7.8, 10.4, 15.6, 20.8, 31.25, 41.67, 62.5, 125, 250, 500 kHz (or Hz)\r\n").await;
            }
        }
        "get cr" => {
            let cr = RADIO_SETTINGS.lock(|r| r.borrow().cr);
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", cr);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set cr ") => {
            let val = s[7..].trim();
            if let Some(cr) = parse_cr(val) {
                RADIO_SETTINGS.lock(|r| {
                    r.borrow_mut().cr = cr;
                });
                crate::storage::request_save_config();
                let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetCodingRate(cr));
                let mut s: heapless::String<32> = heapless::String::new();
                let _ = write!(s, "OK - 4/{}\r\n", cr);
                let _ = write_all(sender, s.as_bytes()).await;
            } else {
                let _ = write_all(sender, b"Error: supported CR values are 4/5, 4/6, 4/7, 4/8\r\n").await;
            }
        }
        "get path.hash.mode" | "get hash-mode" | "get hash-size" => {
            let mode = RADIO_SETTINGS.lock(|r| r.borrow().path_hash_mode);
            let mut resp = heapless::String::<64>::new();
            let _ = write!(resp, "> {}\r\n", mode);
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
                    let _ = write!(resp, "OK\r\n");
                    let _ = write_all(sender, resp.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"Error: path.hash.mode must be 0, 1 or 2\r\n").await;
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
                    let _ = write!(resp, "OK\r\n");
                    let _ = write_all(sender, resp.as_bytes()).await;
                } else {
                    let _ = write_all(sender, b"Error: hash-size must be 1, 2 or 3 bytes\r\n").await;
                }
            } else {
                let _ = write_all(sender, b"  -> Error: invalid number\r\n").await;
            }
        }
        "get repeat" => {
            // For a companion this reflects off-grid mode.
            let on = crate::companion::usb::offgrid_active();
            let mut s: heapless::String<16> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", if on { "on" } else { "off" });
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "get role" => {
            let is_rep = RADIO_SETTINGS.lock(|r| r.borrow().is_repeater);
            let mut s: heapless::String<16> = heapless::String::new();
            let _ = write!(s, "> {}\r\n", if is_rep { "on" } else { "off" });
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("set repeat ") => {
            let val = s[11..].trim();
            let on = matches!(val, "on" | "1" | "true");
            let off = matches!(val, "off" | "0" | "false");
            if !on && !off {
                let _ = write_all(sender, b"Error: must be on or off\r\n").await;
            } else {
                let change = crate::companion::usb::offgrid_set(on);
                if change.no_band {
                    let _ = write_all(
                        sender,
                        b"Error: no off-grid frequency in this band\r\n",
                    )
                    .await;
                } else {
                    if change.changed {
                        crate::storage::request_save_config();
                    }
                    if change.freq_changed {
                        let _ = crate::radio::lora_phy::request_radio_config();
                    }
                    let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetRepeat(
                        crate::companion::usb::forwarding_enabled(),
                    ));
                    let msg: &[u8] = if on {
                        b"OK - repeat is now ON\r\n"
                    } else {
                        b"OK - repeat is now OFF\r\n"
                    };
                    let _ = write_all(sender, msg).await;
                }
            }
        }
        s if s.starts_with("set role ") => {
            let val = s[9..].trim();
            let is_rep = matches!(val, "on" | "1" | "repeater" | "true");
            RADIO_SETTINGS.lock(|r| {
                r.borrow_mut().is_repeater = is_rep;
            });
            crate::storage::request_save_config();
            let _ = USB_COMMAND_CHANNEL.try_send(UsbCommand::SetRepeat(
                crate::companion::usb::forwarding_enabled(),
            ));
            let mut s: heapless::String<32> = heapless::String::new();
            let _ = write!(s, "OK - role={}\r\n", if is_rep { "repeater" } else { "client" });
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s.starts_with("send ") => {
            let msg = s[5..].trim();
            if msg.is_empty() {
                let _ = write_all(sender, b"  -> Error: message cannot be empty\r\n").await;
            } else {
                let mut s_msg = heapless::String::<160>::new();
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
            let count = DISCOVERED_CONTACTS.lock(|c| c.borrow().len());
            if count == 0 {
                let _ = write_all(sender, b"  -> No contacts discovered yet (listening for adverts...)\r\n").await;
            } else {
                let mut hdr = heapless::String::<64>::new();
                let _ = write!(hdr, "  -> Discovered Contacts ({}):\r\n", count);
                let _ = write_all(sender, hdr.as_bytes()).await;
                let _ = write_all(sender, b"  -> ID        TYPE  CALLSIGN        RSSI     SNR   LAT        LON        SEEN\r\n").await;
                for idx in 0..count {
                    let Some(c) = DISCOVERED_CONTACTS.lock(|c| c.borrow().get(idx).cloned()) else {
                        continue;
                    };
                    let ty = match c.contact_type {
                        2 => "RPT",
                        3 => "ROOM",
                        4 => "SENS",
                        _ => "CHAT",
                    };
                    let mut s = heapless::String::<128>::new();
                    let _ = write!(
                        s,
                        "  -> {:02x}{:02x}{:02x}{:02x}  {:<4}  {:<14}  {:>4}dBm  {:>3}dB  {:>10} {:>10}  {}s\r\n",
                        c.node_id[0], c.node_id[1], c.node_id[2], c.node_id[3],
                        ty,
                        c.callsign.as_str(),
                        c.rssi, c.snr, c.latitude, c.longitude, c.last_seen_epoch
                    );
                    let _ = write_all(sender, s.as_bytes()).await;
                }
            }
        }
        "get autoadd" | "get auto-add" => {
            let a = AUTOADD_SETTINGS.lock(|x| *x.borrow());
            let enabled = (a.manual_add_contacts & 1) == 0 || a.config != 0;
            let mode = if (a.manual_add_contacts & 1) == 0 { "all" } else { "types" };
            let mut s: heapless::String<96> = heapless::String::new();
            let _ = write!(
                s,
                "  -> auto-add: {} (mode={}, config=0x{:02X}, max_hops={})\r\n",
                if enabled { "on" } else { "off" },
                mode,
                a.config,
                a.max_hops
            );
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "get companion" => {
            use core::sync::atomic::Ordering;
            let dq = crate::companion::protocol::DEVICE_QUERY_COUNT.load(Ordering::Relaxed);
            let ga = crate::companion::protocol::GET_AUTOADD_COUNT.load(Ordering::Relaxed);
            let sa = crate::companion::protocol::SET_AUTOADD_COUNT.load(Ordering::Relaxed);
            let lc = crate::companion::protocol::LAST_CMD.load(Ordering::Relaxed);
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(
                s,
                "  -> device_query={} get_autoadd={} set_autoadd={} last_cmd=0x{:02X}\r\n",
                dq, ga, sa, lc
            );
            let _ = write_all(sender, s.as_bytes()).await;
        }
        s if s == "set autoadd" || s.starts_with("set autoadd ") => {
            let arg = s["set autoadd".len()..].trim();
            let (manual, config, max_hops, label) = if arg.eq_ignore_ascii_case("on") {
                (0u8, 0x00u8, 0u8, 1u8)
            } else if arg.eq_ignore_ascii_case("off") {
                (1u8, 0x00u8, 0u8, 2u8)
            } else {
                let mut parts = arg
                    .split(|c: char| c == ',' || c.is_whitespace())
                    .filter(|p| !p.is_empty());
                match (parts.next(), parts.next(), parts.next()) {
                    (Some(cfg), Some(hops), None) => {
                        let cfg_val = cfg
                            .trim_start_matches("0x")
                            .trim_start_matches("0X");
                        let config = u8::from_str_radix(cfg_val, 16)
                            .ok()
                            .filter(|v| *v <= 0x1F);
                        let hops = hops.parse::<u8>().ok().filter(|v| *v <= 64);
                        match (config, hops) {
                            (Some(config), Some(max_hops)) => (1, config, max_hops, 3),
                            _ => (0, 0, 0, 0),
                        }
                    }
                    _ => (0, 0, 0, 0),
                }
            };

            if label != 0 {
                AUTOADD_SETTINGS.lock(|a| {
                    *a.borrow_mut() = AutoAddSettings {
                        manual_add_contacts: manual,
                        config,
                        max_hops,
                    };
                });
                node.manual_add_contacts = manual;
                node.autoadd_config = config;
                node.autoadd_max_hops = max_hops;
                crate::storage::request_save_config();

                let mut s: heapless::String<96> = heapless::String::new();
                if label == 1 {
                    let _ = write!(s, "  -> OK (auto-add on)\r\n");
                } else if label == 2 {
                    let _ = write!(s, "  -> OK (auto-add off)\r\n");
                } else {
                    let _ = write!(
                        s,
                        "  -> OK (config=0x{:02X}, max_hops={})\r\n",
                        config, max_hops
                    );
                }
                let _ = write_all(sender, s.as_bytes()).await;
            } else {
                let _ = write_all(
                    sender,
                    b"Error: use set autoadd on|off or set autoadd <config>,<max_hops>\r\n",
                )
                .await;
            }
        }
        "clear contacts" => {
            DISCOVERED_CONTACTS.lock(|c| {
                c.borrow_mut().clear();
            });
            crate::storage::request_contacts();
            let _ = write_all(sender, b"  -> OK (Contacts cleared)\r\n").await;
        }
        "messages" | "inbox" | "get messages" => {
            let msg_snapshot: heapless::Vec<ReceivedMessage, MAX_MESSAGES> = MESSAGE_LOG.lock(|m| m.borrow().clone());
            if msg_snapshot.is_empty() {
                let _ = write_all(sender, b"  -> No messages received yet\r\n").await;
            } else {
                let _ = write_all(sender, b"  -> === Recent Messages ===\r\n").await;
                for msg in msg_snapshot.iter() {
                    let mut s = heapless::String::<320>::new();
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
            crate::storage::request_messages();
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
            let mv = crate::telemetry::battery_millivolts();
            let pct = crate::telemetry::battery_percentage();
            let mut s: heapless::String<128> = heapless::String::new();
            let _ = write!(s, "  -> {{\"battery_mv\":{},\"battery_pct\":{},\"uptime_secs\":{},\"errors\":0,\"queue_len\":0}}\r\n", mv, pct, uptime);
            let _ = write_all(sender, s.as_bytes()).await;
        }
        "battery" | "get battery" | "batt" | "get batt" => {
            let mv = crate::telemetry::battery_millivolts();
            let pct = crate::telemetry::battery_percentage();
            let mut s: heapless::String<64> = heapless::String::new();
            let _ = write!(s, "  -> {} mV ({}%)\r\n", mv, pct);
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
            let _ = write_all(sender, b"  -> Commands:\r\n  ->   Identity: get/set name, get/set prv.key, get public.key\r\n  ->   Radio:    get radio, set radio <freq>,<bw>,<sf>,<cr>, get/set freq/tx/sf/bw/cr, get/set repeat, get/set hash-mode, stats-radio\r\n  ->   Mesh:     send <text>, msg <hex_id> <text>, messages, contacts, get/set autoadd, advert, time, channels\r\n  ->   System:   save, ver, board, stats-core, stats-packets, clear stats/contacts/messages, reboot\r\n").await;
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
    let mut text_line_buf = [0u8; 256];
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
                                        if payload.first() == Some(&crate::companion::protocol::CMD_GET_CONTACTS) {
                                            stream_contacts_usb(&mut sender).await;
                                        } else {
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
