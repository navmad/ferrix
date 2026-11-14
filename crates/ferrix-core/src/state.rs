//! Shared application state.
//!
//! Still global for now; the runtime will later own a single `State` value and
//! pass it explicitly. Keeping the types and statics here removes the
//! board/application coupling without changing behavior.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use heapless::Vec;

use crate::types::{
    AutoAddSettings, ChannelData, DiscoveredContact, InboxItem, RadioSettings, RadioTelemetry,
    MAX_CHANNELS, MAX_CONTACTS, MAX_MESSAGES,
};

type SharedRawMutex = CriticalSectionRawMutex;

pub static RADIO_SETTINGS: Mutex<SharedRawMutex, RefCell<RadioSettings>> =
    Mutex::new(RefCell::new(RadioSettings {
        frequency_hz: 910_525_000,
        tx_power_dbm: 22,
        sf: 7,
        bw_khz: 62,
        cr: 5,
        is_repeater: false,
        client_repeat: false,
        offgrid_home_freq: 0,
        path_hash_mode: 0,
        rx_duty_cycle: false,
        airtime_factor_tenths: 90,
    }));

/// Frequencies where off-grid client repeat is permitted.
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

/// Toggle client-repeat/off-grid mode. Turning it on retunes to the nearest
/// allowed band and remembers the previous frequency; turning it off restores
/// that frequency.
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

pub static AUTOADD_SETTINGS: Mutex<SharedRawMutex, RefCell<AutoAddSettings>> =
    Mutex::new(RefCell::new(AutoAddSettings {
        manual_add_contacts: 0,
        config: 0x00,
        max_hops: 0,
    }));

/// Current local identity, shared by every companion transport.
pub static NODE_PUBLIC_KEY: Mutex<SharedRawMutex, RefCell<[u8; 32]>> =
    Mutex::new(RefCell::new([0u8; 32]));
pub static NODE_PRIVATE_KEY: Mutex<SharedRawMutex, RefCell<[u8; 64]>> =
    Mutex::new(RefCell::new([0u8; 64]));

pub fn set_identity(public_key: [u8; 32], private_key: [u8; 64]) {
    NODE_PUBLIC_KEY.lock(|k| *k.borrow_mut() = public_key);
    NODE_PRIVATE_KEY.lock(|k| *k.borrow_mut() = private_key);
}

pub fn current_public_key() -> [u8; 32] {
    NODE_PUBLIC_KEY.lock(|k| *k.borrow())
}

pub fn current_private_key() -> [u8; 64] {
    NODE_PRIVATE_KEY.lock(|k| *k.borrow())
}

/// Wall-clock base for companion timestamps: the epoch at the moment the
/// instant counter was sampled.
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

/// Current device epoch, or `0` when the clock has never been set.
pub fn get_device_time() -> u32 {
    DEVICE_CLOCK.lock(|c| {
        let clock = c.borrow();
        if clock.base_epoch == 0 {
            0
        } else {
            let elapsed = embassy_time::Instant::now()
                .as_secs()
                .saturating_sub(clock.base_instant_secs);
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

/// BLE companion transport status (0 = off, 1..=5 = init stages, high bit set
/// on SoftDevice init failure).
pub static BLE_STATUS: AtomicU32 = AtomicU32::new(0);
/// True while a companion app is connected over BLE (for the dashboard).
pub static BLE_CONNECTED: AtomicBool = AtomicBool::new(false);
/// Pairing PIN reported to the app and used by the security handler.
pub static BLE_PIN: AtomicU32 = AtomicU32::new(123456);

/// Blue status LED disabled by the user (persisted in the config page).
pub static LEDS_DISABLED: AtomicBool = AtomicBool::new(false);

pub fn leds_disabled() -> bool {
    LEDS_DISABLED.load(Ordering::Relaxed)
}

pub fn init_leds_disabled(disabled: bool) {
    LEDS_DISABLED.store(disabled, Ordering::Relaxed);
}

pub const DEFAULT_PUBLIC_SECRET: [u8; 16] = [
    0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a, 0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72,
];

pub const fn default_channels() -> [ChannelData; MAX_CHANNELS] {
    let mut channels = [ChannelData {
        name: [0u8; 32],
        secret: [0u8; 16],
    }; MAX_CHANNELS];
    channels[0].name[0] = b'P';
    channels[0].name[1] = b'u';
    channels[0].name[2] = b'b';
    channels[0].name[3] = b'l';
    channels[0].name[4] = b'i';
    channels[0].name[5] = b'c';
    channels[0].secret = DEFAULT_PUBLIC_SECRET;
    channels
}

pub static CHANNELS: Mutex<SharedRawMutex, RefCell<[ChannelData; MAX_CHANNELS]>> =
    Mutex::new(RefCell::new(default_channels()));

/// Maximum concurrent companion logins tracked by the node.
pub const MAX_CONNECTIONS: usize = 4;

/// One authenticated companion connection (identified by its public key).
#[derive(Clone, Copy, Debug)]
pub struct Connection {
    pub pub_key: [u8; 32],
    pub last_activity_ms: u64,
    pub next_keepalive_ms: u64,
    pub expected_ack: u32,
    pub keep_alive_secs: u16,
}

pub static CONNECTIONS: Mutex<SharedRawMutex, RefCell<Vec<Connection, MAX_CONNECTIONS>>> =
    Mutex::new(RefCell::new(Vec::new()));

/// Login currently awaiting its room/server response, with an expiry instant.
pub static PENDING_LOGIN: Mutex<SharedRawMutex, RefCell<Option<([u8; 32], u64)>>> =
    Mutex::new(RefCell::new(None));

/// Packet scoping for outgoing floods.
pub struct ScopeState {
    pub send_key: Option<[u8; 16]>,
    pub force_unscoped: bool,
}
pub static SCOPE: Mutex<SharedRawMutex, RefCell<ScopeState>> =
    Mutex::new(RefCell::new(ScopeState {
        send_key: None,
        force_unscoped: false,
    }));

/// Preferences managed by `SET_OTHER_PARAMS` / default flood scope and
/// persisted in the config page.
#[derive(Clone, Copy)]
pub struct CompanionPrefs {
    pub advert_loc_policy: u8,
    pub telemetry_mode_base: u8,
    pub telemetry_mode_loc: u8,
    pub telemetry_mode_env: u8,
    pub multi_acks: u8,
    pub scope_name: [u8; 31],
    pub scope_key: [u8; 16],
}

pub const DEFAULT_PREFS: CompanionPrefs = CompanionPrefs {
    advert_loc_policy: 0,
    telemetry_mode_base: 0,
    telemetry_mode_loc: 0,
    telemetry_mode_env: 0,
    multi_acks: 0,
    scope_name: [0; 31],
    scope_key: [0; 16],
};

static PREFS: Mutex<SharedRawMutex, RefCell<CompanionPrefs>> =
    Mutex::new(RefCell::new(DEFAULT_PREFS));

pub fn prefs() -> CompanionPrefs {
    PREFS.lock(|p| *p.borrow())
}

pub fn update_prefs(update: impl FnOnce(&mut CompanionPrefs)) {
    PREFS.lock(|p| update(&mut p.borrow_mut()));
}

fn non_zero_key(key: &[u8; 16]) -> Option<[u8; 16]> {
    if key.iter().any(|b| *b != 0) {
        Some(*key)
    } else {
        None
    }
}

/// Scope for ordinary outgoing floods: explicit override, else the default.
/// `force_unscoped` disables both.
pub fn flood_scope_send() -> Option<[u8; 16]> {
    SCOPE.lock(|s| {
        let scope = s.borrow();
        if scope.force_unscoped {
            None
        } else if let Some(key) = scope.send_key {
            non_zero_key(&key)
        } else {
            non_zero_key(&prefs().scope_key)
        }
    })
}

/// `(override set, force_unscoped)` for diagnostics.
pub fn scope_override() -> (bool, bool) {
    SCOPE.lock(|s| {
        let scope = s.borrow();
        (scope.send_key.is_some() || scope.force_unscoped, scope.force_unscoped)
    })
}

/// Scope used for self adverts: always the persisted default.
pub fn flood_scope_default() -> Option<[u8; 16]> {
    non_zero_key(&prefs().scope_key)
}

/// Advert position after applying the location policy.
pub fn advert_location(lat: f32, lon: f32) -> (f32, f32) {
    if prefs().advert_loc_policy == 0 {
        (0.0, 0.0)
    } else {
        (lat, lon)
    }
}

pub static SCOPED_FLOODS: AtomicU32 = AtomicU32::new(0);

fn scope_packet(pkt: &mut meshcore_wire::Packet, key: [u8; 16]) {
    let code = meshcore_wire::crypto::transport_code(
        &key,
        pkt.payload_type(),
        &pkt.payload[..pkt.payload_len],
    );
    pkt.header = (pkt.header & !0x03) | meshcore_wire::packet::ROUTE_TYPE_TRANSPORT_FLOOD;
    pkt.transport_codes = [code, 0];
    SCOPED_FLOODS.fetch_add(1, Ordering::Relaxed);
}

/// Attach the current send scope (override or default) to a flood packet.
pub fn apply_flood_scope(pkt: &mut meshcore_wire::Packet) {
    if pkt.route_type() == meshcore_wire::packet::ROUTE_TYPE_FLOOD {
        if let Some(key) = flood_scope_send() {
            scope_packet(pkt, key);
        }
    }
}

/// Attach the persisted default scope (used for self adverts).
pub fn apply_flood_scope_default(pkt: &mut meshcore_wire::Packet) {
    if pkt.route_type() == meshcore_wire::packet::ROUTE_TYPE_FLOOD {
        if let Some(key) = flood_scope_default() {
            scope_packet(pkt, key);
        }
    }
}

/// Active node callsign, mirrored from the persisted config.
pub static NODE_CALLSIGN: Mutex<SharedRawMutex, RefCell<heapless::String<32>>> =
    Mutex::new(RefCell::new(heapless::String::new()));

/// Imported 64-byte expanded private key, or `None` to derive from the seed.
pub static CUSTOM_PRIV_KEY: Mutex<SharedRawMutex, RefCell<Option<[u8; 64]>>> =
    Mutex::new(RefCell::new(None));

pub static DISCOVERED_CONTACTS: Mutex<
    SharedRawMutex,
    RefCell<Vec<DiscoveredContact, MAX_CONTACTS>>,
> = Mutex::new(RefCell::new(Vec::new()));

pub static MESSAGE_LOG: Mutex<SharedRawMutex, RefCell<Vec<InboxItem, MAX_MESSAGES>>> =
    Mutex::new(RefCell::new(Vec::new()));

pub static RADIO_TELEMETRY: Mutex<SharedRawMutex, RefCell<RadioTelemetry>> =
    Mutex::new(RefCell::new(RadioTelemetry {
        tx_beacons: 0,
        rx_packets: 0,
        last_rssi: 0,
        last_snr: 0,
        has_rx: false,
        tx_air_ms: 0,
        rx_air_ms: 0,
        tx_wait_max_ms: 0,
        tx_control: 0,
        tx_flood: 0,
        tx_background: 0,
        tx_stale: 0,
        tx_cancelled: 0,
        tx_retries: 0,
        tx_dropped: 0,
    }));

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_channel_zero_is_public() {
        let channels = default_channels();
        assert_eq!(&channels[0].name[..6], b"Public");
        assert_eq!(channels[0].secret, DEFAULT_PUBLIC_SECRET);
        assert!(channels[1..].iter().all(|ch| ch.name == [0u8; 32]));
        assert!(channels[1..].iter().all(|ch| ch.secret == [0u8; 16]));
    }

    #[test]
    fn companion_limits_match_protocol() {
        assert_eq!(MAX_MESSAGES, 256);
        assert_eq!(MAX_CHANNELS, 40);
        assert_eq!(MAX_CONTACTS, 350);
        assert_eq!(MAX_CONNECTIONS, 4);
    }
}
