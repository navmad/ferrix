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
