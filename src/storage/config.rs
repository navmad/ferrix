//! MeshCore Persistent Flash Storage & Configuration Management
//!
//! Stores node identity (callsign), cryptographic keys, radio settings,
//! and group channel table permanently in nRF52840 internal flash.
//!
//! Preserved across firmware updates, serial DFU flashing, and power cycles.

use core::cell::RefCell;
use defmt::{error, info};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;

/// Shared raw mutex type — see usb.rs for documentation.
type SharedRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embedded_storage_async::nor_flash::NorFlash;
use heapless::String;

use crate::boards::Board;
use crate::companion::usb::ChannelData;

/// Address of the reserved 4 KB flash configuration page
/// (Top page of application flash, immediately preceding bootloader at 0x000ED000)
pub const CONFIG_FLASH_ADDR: u32 = 0x000EC000;
pub const CONFIG_MAGIC: u32 = 0x4D434647; // 'MCFG' (MeshCore Config)
pub const CONFIG_VERSION: u32 = 10;

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PersistentConfig {
    pub magic: u32,
    pub version: u32,
    pub callsign: [u8; 32],
    /// MeshCore's 64-byte expanded private key (`scalar ‖ nonce_prefix`).
    pub custom_priv_key: [u8; 64],
    pub has_custom_key: u8,
    pub is_repeater: u8,
    /// Off-grid client-repeat mode (MeshCore `client_repeat`).
    pub client_repeat: u8,
    /// Frequency to restore when off-grid mode is disabled.
    pub offgrid_home_freq: u32,
    pub radio_tx_power: i8,
    pub radio_sf: u8,
    pub radio_cr: u8,
    pub path_hash_mode: u8,
    pub autoadd_config: u8,     // bitmask: overwrite_oldest, chat, repeater, room_server, sensor
    pub autoadd_max_hops: u8,   // 0 = no limit, N = up to N-1 hops
    pub manual_add_contacts: u8, // bit 0: 0 = auto-add all, 1 = use autoadd_config
    pub leds_disabled: u8,      // 1 = all user LEDs off
    pub rx_duty_cycle: u8,      // 1 = duty-cycle/sniff RX
    pub airtime_factor_tenths: u8, // 0..90; 90 = 9.0 -> 10% duty
    pub flood_scope_name: [u8; 31],
    pub flood_scope_key: [u8; 16],
    pub advert_loc_policy: u8,
    pub telemetry_mode_base: u8,
    pub telemetry_mode_loc: u8,
    pub telemetry_mode_env: u8,
    pub multi_acks: u8,
    pub radio_freq_hz: u32,
    pub radio_bw_khz: u32,
    pub channels: [ChannelData; crate::companion::usb::MAX_CHANNELS],
    pub checksum: u32,
}

/// v9 layout: no flood-scope or advert/telemetry preferences.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistentConfigV9 {
    magic: u32,
    version: u32,
    callsign: [u8; 32],
    custom_priv_key: [u8; 64],
    has_custom_key: u8,
    is_repeater: u8,
    client_repeat: u8,
    offgrid_home_freq: u32,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    manual_add_contacts: u8,
    leds_disabled: u8,
    rx_duty_cycle: u8,
    airtime_factor_tenths: u8,
    flood_scope_name: [u8; 31],
    flood_scope_key: [u8; 16],
    advert_loc_policy: u8,
    telemetry_mode_base: u8,
    telemetry_mode_loc: u8,
    telemetry_mode_env: u8,
    multi_acks: u8,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    channels: [ChannelData; crate::companion::usb::MAX_CHANNELS],
    checksum: u32,
}

/// v6 layout: no `leds_disabled`. Read only for migration.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistentConfigV6 {
    magic: u32,
    version: u32,
    callsign: [u8; 32],
    custom_priv_key: [u8; 64],
    has_custom_key: u8,
    is_repeater: u8,
    client_repeat: u8,
    offgrid_home_freq: u32,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    manual_add_contacts: u8,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    channels: [ChannelData; crate::companion::usb::MAX_CHANNELS],
    checksum: u32,
}

/// v7 layout: no `rx_duty_cycle`. Read only for migration.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistentConfigV7 {
    magic: u32,
    version: u32,
    callsign: [u8; 32],
    custom_priv_key: [u8; 64],
    has_custom_key: u8,
    is_repeater: u8,
    client_repeat: u8,
    offgrid_home_freq: u32,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    manual_add_contacts: u8,
    leds_disabled: u8,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    channels: [ChannelData; crate::companion::usb::MAX_CHANNELS],
    checksum: u32,
}

/// v8 layout: no `airtime_factor`. Read only for migration.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistentConfigV8 {
    magic: u32,
    version: u32,
    callsign: [u8; 32],
    custom_priv_key: [u8; 64],
    has_custom_key: u8,
    is_repeater: u8,
    client_repeat: u8,
    offgrid_home_freq: u32,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    manual_add_contacts: u8,
    leds_disabled: u8,
    rx_duty_cycle: u8,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    channels: [ChannelData; crate::companion::usb::MAX_CHANNELS],
    checksum: u32,
}

/// v4 layout: no `manual_add_contacts`. Read only for migration.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistentConfigV4 {
    magic: u32,
    version: u32,
    callsign: [u8; 32],
    custom_priv_key: [u8; 64],
    has_custom_key: u8,
    is_repeater: u8,
    client_repeat: u8,
    offgrid_home_freq: u32,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    channels: [ChannelData; crate::companion::usb::MAX_CHANNELS],
    checksum: u32,
}

/// v2 layout: 8 channels. Read only for migration.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistentConfigV2 {
    magic: u32,
    version: u32,
    callsign: [u8; 32],
    custom_priv_key: [u8; 64],
    has_custom_key: u8,
    is_repeater: u8,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    channels: [ChannelData; 8],
    checksum: u32,
}

/// v3 layout: 40 channels, no client-repeat field. Read only for migration.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistentConfigV3 {
    magic: u32,
    version: u32,
    callsign: [u8; 32],
    custom_priv_key: [u8; 64],
    has_custom_key: u8,
    is_repeater: u8,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    channels: [ChannelData; crate::companion::usb::MAX_CHANNELS],
    checksum: u32,
}

/// v1 layout: the private key was a 32-byte seed. Read only for migration.
#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistentConfigV1 {
    magic: u32,
    version: u32,
    callsign: [u8; 32],
    custom_priv_key: [u8; 32],
    has_custom_key: u8,
    is_repeater: u8,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    channels: [ChannelData; 8],
    checksum: u32,
}

pub static NODE_CALLSIGN: Mutex<SharedRawMutex, RefCell<String<32>>> =
    Mutex::new(RefCell::new(String::new()));

pub static CUSTOM_PRIV_KEY: Mutex<SharedRawMutex, RefCell<Option<[u8; 64]>>> =
    Mutex::new(RefCell::new(None));

pub(crate) static SAVE_REQUEST_CHANNEL: Channel<SharedRawMutex, (), 2> = Channel::new();

/// Compute a 32-bit checksum covering a configuration structure up to its
/// checksum field.
fn compute_checksum<T>(cfg: &T) -> u32 {
    let words = unsafe {
        core::slice::from_raw_parts(
            cfg as *const T as *const u32,
            (core::mem::size_of::<T>() - 4) / 4,
        )
    };
    let mut sum = 0u32;
    for &w in words {
        sum = sum.wrapping_add(w);
    }
    sum
}

fn expand_channels(src: [ChannelData; 8]) -> [ChannelData; crate::companion::usb::MAX_CHANNELS] {
    let mut dst = [ChannelData {
        name: [0u8; 32],
        secret: [0u8; 16],
    }; crate::companion::usb::MAX_CHANNELS];
    dst[..8].copy_from_slice(&src);
    dst
}

fn migrate_v1(v1: PersistentConfigV1) -> PersistentConfig {
    let custom_priv_key = if v1.has_custom_key != 0 {
        // Use the seed path so a reserved public key is regenerated exactly as
        // a fresh identity would be.
        crate::meshcore::identity::LocalIdentity::from_seed("", &v1.custom_priv_key)
            .private_key_bytes()
    } else {
        [0u8; 64]
    };
    let mut cfg = PersistentConfig {
        magic: CONFIG_MAGIC,
        version: CONFIG_VERSION,
        callsign: v1.callsign,
        custom_priv_key,
        has_custom_key: v1.has_custom_key,
        is_repeater: v1.is_repeater,
        client_repeat: 0,
        offgrid_home_freq: 0,
        radio_tx_power: v1.radio_tx_power,
        radio_sf: v1.radio_sf,
        radio_cr: v1.radio_cr,
        path_hash_mode: v1.path_hash_mode,
        autoadd_config: 0,
        autoadd_max_hops: v1.autoadd_max_hops,
        manual_add_contacts: 0,
        leds_disabled: 0,
        rx_duty_cycle: 0,
        airtime_factor_tenths: 90,
        flood_scope_name: [0; 31],
        flood_scope_key: [0; 16],
        advert_loc_policy: 0,
        telemetry_mode_base: 0,
        telemetry_mode_loc: 0,
        telemetry_mode_env: 0,
        multi_acks: 0,
        radio_freq_hz: v1.radio_freq_hz,
        radio_bw_khz: v1.radio_bw_khz,
        channels: expand_channels(v1.channels),
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

fn migrate_v2(v2: PersistentConfigV2) -> PersistentConfig {
    let mut cfg = PersistentConfig {
        magic: CONFIG_MAGIC,
        version: CONFIG_VERSION,
        callsign: v2.callsign,
        custom_priv_key: v2.custom_priv_key,
        has_custom_key: v2.has_custom_key,
        is_repeater: v2.is_repeater,
        client_repeat: 0,
        offgrid_home_freq: 0,
        radio_tx_power: v2.radio_tx_power,
        radio_sf: v2.radio_sf,
        radio_cr: v2.radio_cr,
        path_hash_mode: v2.path_hash_mode,
        autoadd_config: 0,
        autoadd_max_hops: v2.autoadd_max_hops,
        manual_add_contacts: 0,
        leds_disabled: 0,
        rx_duty_cycle: 0,
        airtime_factor_tenths: 90,
        flood_scope_name: [0; 31],
        flood_scope_key: [0; 16],
        advert_loc_policy: 0,
        telemetry_mode_base: 0,
        telemetry_mode_loc: 0,
        telemetry_mode_env: 0,
        multi_acks: 0,
        radio_freq_hz: v2.radio_freq_hz,
        radio_bw_khz: v2.radio_bw_khz,
        channels: expand_channels(v2.channels),
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

fn migrate_v3(v3: PersistentConfigV3) -> PersistentConfig {
    let mut cfg = PersistentConfig {
        magic: CONFIG_MAGIC,
        version: CONFIG_VERSION,
        callsign: v3.callsign,
        custom_priv_key: v3.custom_priv_key,
        has_custom_key: v3.has_custom_key,
        is_repeater: v3.is_repeater,
        client_repeat: 0,
        offgrid_home_freq: 0,
        radio_tx_power: v3.radio_tx_power,
        radio_sf: v3.radio_sf,
        radio_cr: v3.radio_cr,
        path_hash_mode: v3.path_hash_mode,
        autoadd_config: 0,
        autoadd_max_hops: v3.autoadd_max_hops,
        manual_add_contacts: 0,
        leds_disabled: 0,
        rx_duty_cycle: 0,
        airtime_factor_tenths: 90,
        flood_scope_name: [0; 31],
        flood_scope_key: [0; 16],
        advert_loc_policy: 0,
        telemetry_mode_base: 0,
        telemetry_mode_loc: 0,
        telemetry_mode_env: 0,
        multi_acks: 0,
        radio_freq_hz: v3.radio_freq_hz,
        radio_bw_khz: v3.radio_bw_khz,
        channels: v3.channels,
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

fn migrate_v4(v4: PersistentConfigV4) -> PersistentConfig {
    let mut cfg = PersistentConfig {
        magic: CONFIG_MAGIC,
        version: CONFIG_VERSION,
        callsign: v4.callsign,
        custom_priv_key: v4.custom_priv_key,
        has_custom_key: v4.has_custom_key,
        is_repeater: v4.is_repeater,
        client_repeat: v4.client_repeat,
        offgrid_home_freq: v4.offgrid_home_freq,
        radio_tx_power: v4.radio_tx_power,
        radio_sf: v4.radio_sf,
        radio_cr: v4.radio_cr,
        path_hash_mode: v4.path_hash_mode,
        autoadd_config: 0,
        autoadd_max_hops: v4.autoadd_max_hops,
        manual_add_contacts: 0,
        leds_disabled: 0,
        rx_duty_cycle: 0,
        airtime_factor_tenths: 90,
        flood_scope_name: [0; 31],
        flood_scope_key: [0; 16],
        advert_loc_policy: 0,
        telemetry_mode_base: 0,
        telemetry_mode_loc: 0,
        telemetry_mode_env: 0,
        multi_acks: 0,
        radio_freq_hz: v4.radio_freq_hz,
        radio_bw_khz: v4.radio_bw_khz,
        channels: v4.channels,
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

fn migrate_v6(v6: PersistentConfigV6) -> PersistentConfig {
    let mut cfg = PersistentConfig {
        magic: v6.magic,
        version: CONFIG_VERSION,
        callsign: v6.callsign,
        custom_priv_key: v6.custom_priv_key,
        has_custom_key: v6.has_custom_key,
        is_repeater: v6.is_repeater,
        client_repeat: v6.client_repeat,
        offgrid_home_freq: v6.offgrid_home_freq,
        radio_tx_power: v6.radio_tx_power,
        radio_sf: v6.radio_sf,
        radio_cr: v6.radio_cr,
        path_hash_mode: v6.path_hash_mode,
        autoadd_config: v6.autoadd_config,
        autoadd_max_hops: v6.autoadd_max_hops,
        manual_add_contacts: v6.manual_add_contacts,
        leds_disabled: 0,
        rx_duty_cycle: 0,
        airtime_factor_tenths: 90,
        flood_scope_name: [0; 31],
        flood_scope_key: [0; 16],
        advert_loc_policy: 0,
        telemetry_mode_base: 0,
        telemetry_mode_loc: 0,
        telemetry_mode_env: 0,
        multi_acks: 0,
        radio_freq_hz: v6.radio_freq_hz,
        radio_bw_khz: v6.radio_bw_khz,
        channels: v6.channels,
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

fn migrate_v7(v7: PersistentConfigV7) -> PersistentConfig {
    let mut cfg = PersistentConfig {
        magic: v7.magic,
        version: CONFIG_VERSION,
        callsign: v7.callsign,
        custom_priv_key: v7.custom_priv_key,
        has_custom_key: v7.has_custom_key,
        is_repeater: v7.is_repeater,
        client_repeat: v7.client_repeat,
        offgrid_home_freq: v7.offgrid_home_freq,
        radio_tx_power: v7.radio_tx_power,
        radio_sf: v7.radio_sf,
        radio_cr: v7.radio_cr,
        path_hash_mode: v7.path_hash_mode,
        autoadd_config: v7.autoadd_config,
        autoadd_max_hops: v7.autoadd_max_hops,
        manual_add_contacts: v7.manual_add_contacts,
        leds_disabled: v7.leds_disabled,
        rx_duty_cycle: 0,
        airtime_factor_tenths: 90,
        flood_scope_name: [0; 31],
        flood_scope_key: [0; 16],
        advert_loc_policy: 0,
        telemetry_mode_base: 0,
        telemetry_mode_loc: 0,
        telemetry_mode_env: 0,
        multi_acks: 0,
        radio_freq_hz: v7.radio_freq_hz,
        radio_bw_khz: v7.radio_bw_khz,
        channels: v7.channels,
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

fn migrate_v9(v9: PersistentConfigV9) -> PersistentConfig {
    let defaults = crate::companion::protocol::DEFAULT_PREFS;
    let mut cfg = PersistentConfig {
        magic: v9.magic,
        version: CONFIG_VERSION,
        callsign: v9.callsign,
        custom_priv_key: v9.custom_priv_key,
        has_custom_key: v9.has_custom_key,
        is_repeater: v9.is_repeater,
        client_repeat: v9.client_repeat,
        offgrid_home_freq: v9.offgrid_home_freq,
        radio_tx_power: v9.radio_tx_power,
        radio_sf: v9.radio_sf,
        radio_cr: v9.radio_cr,
        path_hash_mode: v9.path_hash_mode,
        autoadd_config: v9.autoadd_config,
        autoadd_max_hops: v9.autoadd_max_hops,
        manual_add_contacts: v9.manual_add_contacts,
        leds_disabled: v9.leds_disabled,
        rx_duty_cycle: v9.rx_duty_cycle,
        airtime_factor_tenths: v9.airtime_factor_tenths,
        flood_scope_name: defaults.scope_name,
        flood_scope_key: defaults.scope_key,
        advert_loc_policy: defaults.advert_loc_policy,
        telemetry_mode_base: defaults.telemetry_mode_base,
        telemetry_mode_loc: defaults.telemetry_mode_loc,
        telemetry_mode_env: defaults.telemetry_mode_env,
        multi_acks: defaults.multi_acks,
        radio_freq_hz: v9.radio_freq_hz,
        radio_bw_khz: v9.radio_bw_khz,
        channels: v9.channels,
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

fn migrate_v8(v8: PersistentConfigV8) -> PersistentConfig {
    let mut cfg = PersistentConfig {
        magic: v8.magic,
        version: CONFIG_VERSION,
        callsign: v8.callsign,
        custom_priv_key: v8.custom_priv_key,
        has_custom_key: v8.has_custom_key,
        is_repeater: v8.is_repeater,
        client_repeat: v8.client_repeat,
        offgrid_home_freq: v8.offgrid_home_freq,
        radio_tx_power: v8.radio_tx_power,
        radio_sf: v8.radio_sf,
        radio_cr: v8.radio_cr,
        path_hash_mode: v8.path_hash_mode,
        autoadd_config: v8.autoadd_config,
        autoadd_max_hops: v8.autoadd_max_hops,
        manual_add_contacts: v8.manual_add_contacts,
        leds_disabled: v8.leds_disabled,
        rx_duty_cycle: v8.rx_duty_cycle,
        airtime_factor_tenths: 90,
        flood_scope_name: [0; 31],
        flood_scope_key: [0; 16],
        advert_loc_policy: 0,
        telemetry_mode_base: 0,
        telemetry_mode_loc: 0,
        telemetry_mode_env: 0,
        multi_acks: 0,
        radio_freq_hz: v8.radio_freq_hz,
        radio_bw_khz: v8.radio_bw_khz,
        channels: v8.channels,
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

/// v5 had no layout change, but its auto-add defaults were wrong for the
/// companion app's manual mode (`0x1E` meant "manual" still matched every
/// type). Resetting to MeshCore's fresh-node defaults makes the app's
/// manual/off switch behave as expected.
fn migrate_v5(v5: PersistentConfigV6) -> PersistentConfig {
    let mut cfg = migrate_v6(v5);
    cfg.autoadd_config = 0;
    cfg.manual_add_contacts = 0;
    cfg.checksum = 0;
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

/// Synchronously read and validate persistent configuration from internal
/// flash. Returns `(config, migrated_from_v1)`.
pub fn read_config() -> Option<(PersistentConfig, bool)> {
    let cfg = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfig).read_volatile() };
    if cfg.magic != CONFIG_MAGIC {
        return None;
    }
    if cfg.version == CONFIG_VERSION && compute_checksum(&cfg) == cfg.checksum {
        return Some((cfg, false));
    }
    if cfg.version == 9 {
        let v9 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV9).read_volatile() };
        if compute_checksum(&v9) == v9.checksum {
            return Some((migrate_v9(v9), true));
        }
    }
    if cfg.version == 8 {
        let v8 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV8).read_volatile() };
        if compute_checksum(&v8) == v8.checksum {
            return Some((migrate_v8(v8), true));
        }
    }
    if cfg.version == 7 {
        let v7 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV7).read_volatile() };
        if compute_checksum(&v7) == v7.checksum {
            return Some((migrate_v7(v7), true));
        }
    }
    if cfg.version == 6 {
        let v6 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV6).read_volatile() };
        if compute_checksum(&v6) == v6.checksum {
            return Some((migrate_v6(v6), true));
        }
    }
    if cfg.version == 5 {
        let v5 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV6).read_volatile() };
        if compute_checksum(&v5) == v5.checksum {
            return Some((migrate_v5(v5), true));
        }
    }
    if cfg.version == 4 {
        let v4 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV4).read_volatile() };
        if compute_checksum(&v4) == v4.checksum {
            return Some((migrate_v4(v4), true));
        }
    }
    if cfg.version == 3 {
        let v3 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV3).read_volatile() };
        if compute_checksum(&v3) == v3.checksum {
            return Some((migrate_v3(v3), true));
        }
    }
    if cfg.version == 2 {
        let v2 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV2).read_volatile() };
        if compute_checksum(&v2) == v2.checksum {
            return Some((migrate_v2(v2), true));
        }
    }
    if cfg.version == 1 {
        let v1 = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfigV1).read_volatile() };
        if compute_checksum(&v1) == v1.checksum {
            return Some((migrate_v1(v1), true));
        }
    }
    None
}

/// Retrieve the active node callsign
pub fn get_callsign() -> String<32> {
    NODE_CALLSIGN.lock(|c| c.borrow().clone())
}

/// Retrieve the active path hash mode (0: 1B, 1: 2B, 2: 3B)
pub fn get_path_hash_mode() -> u8 {
    crate::companion::usb::RADIO_SETTINGS.lock(|r| r.borrow().path_hash_mode)
}

/// Set a new path hash mode and queue a persistent flash save
pub fn set_path_hash_mode(mode: u8) {
    if mode <= 2 {
        crate::companion::usb::RADIO_SETTINGS.lock(|r| {
            r.borrow_mut().path_hash_mode = mode;
        });
        request_save_config();
    }
}

/// Set a new node callsign and queue a persistent flash save
pub fn set_callsign(new_name: &str) {
    let mut changed = false;
    NODE_CALLSIGN.lock(|c| {
        let mut s = c.borrow_mut();
        if s.as_str() != new_name {
            s.clear();
            let _ = s.push_str(new_name);
            changed = true;
        }
    });
    if changed {
        request_save_config();
    }
}

/// Set the 64-byte expanded private key and queue a persistent flash save
pub fn set_custom_priv_key(key: [u8; 64]) {
    CUSTOM_PRIV_KEY.lock(|k| {
        *k.borrow_mut() = Some(key);
    });
    request_save_config();
}

/// Signal the storage task to write the current settings to flash
pub fn request_save_config() {
    let _ = SAVE_REQUEST_CHANNEL.try_send(());
}

/// Initialize configuration from flash (or defaults if flash is uninitialized)
pub fn init_config() -> (String<32>, Option<[u8; 64]>) {
    if let Some((cfg, migrated)) = read_config() {
        info!("Storage: Found valid persistent config in flash at 0x{:08x}!", CONFIG_FLASH_ADDR);

        // Load callsign
        let name_len = cfg.callsign.iter().position(|&b| b == 0).unwrap_or(32);
        let mut name = String::<32>::new();
        if name_len > 0 {
            if let Ok(s) = core::str::from_utf8(&cfg.callsign[..name_len]) {
                let _ = name.push_str(s);
            }
        }
        if name.is_empty() {
            let _ = name.push_str(crate::boards::CurrentBoard::NAME);
        }
        NODE_CALLSIGN.lock(|c| *c.borrow_mut() = name.clone());

        // Load channels
        crate::companion::usb::CHANNELS.lock(|c| {
            *c.borrow_mut() = cfg.channels;
        });

        // Load radio settings
        crate::companion::usb::RADIO_SETTINGS.lock(|r| {
            let mut s = r.borrow_mut();
            s.frequency_hz = cfg.radio_freq_hz;
            s.tx_power_dbm = cfg.radio_tx_power;
            s.sf = cfg.radio_sf;
            s.bw_khz = cfg.radio_bw_khz;
            s.cr = cfg.radio_cr;
            s.is_repeater = cfg.is_repeater != 0;
            s.client_repeat = cfg.client_repeat != 0;
            s.offgrid_home_freq = cfg.offgrid_home_freq;
            s.path_hash_mode = if cfg.path_hash_mode <= 2 { cfg.path_hash_mode } else { 0 };
            s.rx_duty_cycle = cfg.rx_duty_cycle != 0;
            s.airtime_factor_tenths = cfg.airtime_factor_tenths.min(90);
        });

        // Load the contact auto-add policy.
        crate::companion::usb::AUTOADD_SETTINGS.lock(|a| {
            *a.borrow_mut() = crate::companion::usb::AutoAddSettings {
                manual_add_contacts: if cfg.manual_add_contacts != 0 { 1 } else { 0 },
                config: cfg.autoadd_config,
                max_hops: cfg.autoadd_max_hops.min(64),
            };
        });

        crate::companion::usb::init_leds_disabled(cfg.leds_disabled != 0);

        crate::companion::protocol::update_prefs(|p| {
            p.scope_name = cfg.flood_scope_name;
            p.scope_key = cfg.flood_scope_key;
            p.advert_loc_policy = if cfg.advert_loc_policy <= 2 {
                cfg.advert_loc_policy
            } else {
                0
            };
            p.telemetry_mode_base = cfg.telemetry_mode_base.min(2);
            p.telemetry_mode_loc = cfg.telemetry_mode_loc.min(2);
            p.telemetry_mode_env = cfg.telemetry_mode_env.min(2);
            p.multi_acks = cfg.multi_acks;
        });

        let priv_key = if cfg.has_custom_key != 0 {
            CUSTOM_PRIV_KEY.lock(|k| *k.borrow_mut() = Some(cfg.custom_priv_key));
            Some(cfg.custom_priv_key)
        } else {
            None
        };

        if migrated {
            info!("Storage: Migrated config to v10 (up to {} contacts / {} channels)", crate::companion::usb::MAX_CONTACTS, crate::companion::usb::MAX_CHANNELS);
            request_save_config();
        }

        info!("Storage: Loaded callsign '{}', channels, and radio parameters", name.as_str());
        (name, priv_key)
    } else {
        info!("Storage: No persistent config found (first boot or unformatted). Initializing defaults.");
        let mut name = String::<32>::new();
        let _ = name.push_str(crate::boards::CurrentBoard::NAME);
        NODE_CALLSIGN.lock(|c| *c.borrow_mut() = name.clone());
        (name, None)
    }
}

/// Gather the active state from all subsystems into a PersistentConfig snapshot
pub fn assemble_current_config() -> PersistentConfig {
    let mut callsign_bytes = [0u8; 32];
    NODE_CALLSIGN.lock(|c| {
        let s = c.borrow();
        let len = s.len().min(31);
        callsign_bytes[..len].copy_from_slice(&s.as_bytes()[..len]);
    });

    let (freq, tx, sf, bw, cr, is_rep, client_repeat, offgrid_home_freq, path_mode, rx_duty, af) =
        crate::companion::usb::RADIO_SETTINGS.lock(|r| {
            let s = r.borrow();
            (
                s.frequency_hz,
                s.tx_power_dbm,
                s.sf,
                s.bw_khz,
                s.cr,
                s.is_repeater,
                s.client_repeat,
                s.offgrid_home_freq,
                s.path_hash_mode,
                s.rx_duty_cycle,
                s.airtime_factor_tenths,
            )
        });

    let channels = crate::companion::usb::CHANNELS.lock(|c| *c.borrow());

    let (has_custom_key, custom_priv_key) = CUSTOM_PRIV_KEY.lock(|k| {
        if let Some(key) = *k.borrow() {
            (1u8, key)
        } else {
            (0u8, [0u8; 64])
        }
    });

    let auto = crate::companion::usb::AUTOADD_SETTINGS.lock(|a| *a.borrow());
    let prefs = crate::companion::protocol::prefs();

    let mut cfg = PersistentConfig {
        magic: CONFIG_MAGIC,
        version: CONFIG_VERSION,
        callsign: callsign_bytes,
        custom_priv_key,
        has_custom_key,
        is_repeater: if is_rep { 1 } else { 0 },
        client_repeat: if client_repeat { 1 } else { 0 },
        offgrid_home_freq,
        radio_tx_power: tx,
        radio_sf: sf,
        radio_cr: cr,
        path_hash_mode: path_mode,
        autoadd_config: auto.config,
        autoadd_max_hops: auto.max_hops,
        manual_add_contacts: auto.manual_add_contacts,
        leds_disabled: if crate::companion::usb::leds_disabled() { 1 } else { 0 },
        rx_duty_cycle: if rx_duty { 1 } else { 0 },
        airtime_factor_tenths: af,
        flood_scope_name: prefs.scope_name,
        flood_scope_key: prefs.scope_key,
        advert_loc_policy: prefs.advert_loc_policy,
        telemetry_mode_base: prefs.telemetry_mode_base,
        telemetry_mode_loc: prefs.telemetry_mode_loc,
        telemetry_mode_env: prefs.telemetry_mode_env,
        multi_acks: prefs.multi_acks,
        radio_freq_hz: freq,
        radio_bw_khz: bw,
        channels,
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

/// Commit the current configuration snapshot to the config page.
pub(crate) async fn save_config(flash: &mut nrf_softdevice::Flash) {
    let config = assemble_current_config();
    let config_bytes = unsafe {
        core::slice::from_raw_parts(
            &config as *const _ as *const u8,
            core::mem::size_of::<PersistentConfig>(),
        )
    };

    info!(
        "Storage: Committing persistent configuration to flash at 0x{:08x}...",
        CONFIG_FLASH_ADDR
    );
    if let Err(e) = flash.erase(CONFIG_FLASH_ADDR, CONFIG_FLASH_ADDR + 4096).await {
        error!("Storage: Flash page erase error: {:?}", e);
        return;
    }
    if let Err(e) = flash.write(CONFIG_FLASH_ADDR, config_bytes).await {
        error!("Storage: Flash write error: {:?}", e);
        return;
    }
    info!("Storage: Configuration successfully saved to flash!");
}
