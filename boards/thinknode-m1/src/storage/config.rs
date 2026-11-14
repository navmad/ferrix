//! MeshCore Persistent Flash Storage & Configuration Management
//!
//! Stores node identity (callsign), cryptographic keys, radio settings,
//! and group channel table permanently in nRF52840 internal flash.
//!
//! Preserved across firmware updates, serial DFU flashing, and power cycles.

use defmt::{error, info};
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
pub const CONFIG_VERSION: u32 = 1;

/// Completed config-page writes and erases since boot.
pub static CONFIG_SAVES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static CONFIG_ERASES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

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

pub use ferrix_core::state::{CUSTOM_PRIV_KEY, NODE_CALLSIGN};

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

/// Read and validate the persisted configuration.
///
/// The layout starts fresh at version 1: a page written by any older firmware
/// (or with a different layout) is ignored and defaults are used.
pub fn read_config() -> Option<PersistentConfig> {
    let cfg = unsafe { (CONFIG_FLASH_ADDR as *const PersistentConfig).read_volatile() };
    if cfg.magic == CONFIG_MAGIC
        && cfg.version == CONFIG_VERSION
        && compute_checksum(&cfg) == cfg.checksum
    {
        Some(cfg)
    } else {
        None
    }
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
        let changed = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
            let mut s = r.borrow_mut();
            let changed = s.path_hash_mode != mode;
            s.path_hash_mode = mode;
            changed
        });
        if changed {
            request_save_config();
        }
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
    if let Some(cfg) = read_config() {
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
    CONFIG_ERASES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    if let Err(e) = flash.write(CONFIG_FLASH_ADDR, config_bytes).await {
        error!("Storage: Flash write error: {:?}", e);
        return;
    }
    CONFIG_SAVES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    info!("Storage: Configuration successfully saved to flash!");
}
