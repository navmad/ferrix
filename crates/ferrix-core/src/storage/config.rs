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

use crate::types::{ChannelData, MAX_CHANNELS};

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
    pub channels: [ChannelData; MAX_CHANNELS],
    pub checksum: u32,
}

pub use crate::state::{CUSTOM_PRIV_KEY, NODE_CALLSIGN};

static SAVE_REQUEST_CHANNEL: Channel<SharedRawMutex, (), 2> = Channel::new();

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
pub fn read_config(page: &[u8]) -> Option<PersistentConfig> {
    if page.len() < core::mem::size_of::<PersistentConfig>() {
        return None;
    }
    let cfg = unsafe { core::ptr::read_unaligned(page.as_ptr() as *const PersistentConfig) };
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
    crate::state::RADIO_SETTINGS.lock(|r| r.borrow().path_hash_mode)
}

/// Set a new path hash mode and queue a persistent flash save
pub fn set_path_hash_mode(mode: u8) {
    if mode <= 2 {
        let changed = crate::state::RADIO_SETTINGS.lock(|r| {
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

/// Wait for a coalesced configuration save request.
pub async fn wait_save_request() {
    SAVE_REQUEST_CHANNEL.receive().await;
}

/// Drop queued save requests after a coalescing delay.
pub fn drain_save_requests() {
    while SAVE_REQUEST_CHANNEL.try_receive().is_ok() {}
}

/// Signal the storage task to write the current settings to flash
pub fn request_save_config() {
    let _ = SAVE_REQUEST_CHANNEL.try_send(());
}

/// Initialize configuration from flash (or defaults if flash is uninitialized)
pub fn init_config(page: &[u8], default_name: &str) -> (String<32>, Option<[u8; 64]>) {
    if let Some(cfg) = read_config(page) {
        info!("Storage: Found valid persistent config page");

        // Load callsign
        let name_len = cfg.callsign.iter().position(|&b| b == 0).unwrap_or(32);
        let mut name = String::<32>::new();
        if name_len > 0 {
            if let Ok(s) = core::str::from_utf8(&cfg.callsign[..name_len]) {
                let _ = name.push_str(s);
            }
        }
        if name.is_empty() {
            let _ = name.push_str(default_name);
        }
        NODE_CALLSIGN.lock(|c| *c.borrow_mut() = name.clone());

        // Load channels
        crate::state::CHANNELS.lock(|c| {
            *c.borrow_mut() = cfg.channels;
        });

        // Load radio settings
        crate::state::RADIO_SETTINGS.lock(|r| {
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
        crate::state::AUTOADD_SETTINGS.lock(|a| {
            *a.borrow_mut() = crate::types::AutoAddSettings {
                manual_add_contacts: if cfg.manual_add_contacts != 0 { 1 } else { 0 },
                config: cfg.autoadd_config,
                max_hops: cfg.autoadd_max_hops.min(64),
            };
        });

        crate::state::init_leds_disabled(cfg.leds_disabled != 0);

        crate::state::update_prefs(|p| {
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
        let _ = name.push_str(default_name);
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
        crate::state::RADIO_SETTINGS.lock(|r| {
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

    let channels = crate::state::CHANNELS.lock(|c| *c.borrow());

    let (has_custom_key, custom_priv_key) = CUSTOM_PRIV_KEY.lock(|k| {
        if let Some(key) = *k.borrow() {
            (1u8, key)
        } else {
            (0u8, [0u8; 64])
        }
    });

    let auto = crate::state::AUTOADD_SETTINGS.lock(|a| *a.borrow());
    let prefs = crate::state::prefs();

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
        leds_disabled: if crate::state::leds_disabled() { 1 } else { 0 },
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
pub async fn save_config<F>(flash: &mut F, region: &core::ops::Range<u32>)
where
    F: NorFlash,
    F::Error: defmt::Format,
{
    let config = assemble_current_config();
    let config_bytes = unsafe {
        core::slice::from_raw_parts(
            &config as *const _ as *const u8,
            core::mem::size_of::<PersistentConfig>(),
        )
    };

    info!(
        "Storage: Committing persistent configuration to flash at 0x{:08x}...",
        region.start
    );
    if let Err(e) = flash.erase(region.start, region.end).await {
        error!("Storage: Flash page erase error: {:?}", e);
        return;
    }
    CONFIG_ERASES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    if let Err(e) = flash.write(region.start, config_bytes).await {
        error!("Storage: Flash write error: {:?}", e);
        return;
    }
    CONFIG_SAVES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    info!("Storage: Configuration successfully saved to flash!");
}


#[cfg(test)]
mod tests {
    use super::*;

    fn config_bytes() -> [u8; core::mem::size_of::<PersistentConfig>()] {
        let cfg = PersistentConfig {
            magic: CONFIG_MAGIC,
            version: CONFIG_VERSION,
            callsign: [0u8; 32],
            custom_priv_key: [3u8; 64],
            has_custom_key: 1,
            is_repeater: 0,
            client_repeat: 0,
            offgrid_home_freq: 0,
            radio_tx_power: 22,
            radio_sf: 7,
            radio_cr: 5,
            path_hash_mode: 1,
            autoadd_config: 0,
            autoadd_max_hops: 0,
            manual_add_contacts: 0,
            leds_disabled: 0,
            rx_duty_cycle: 0,
            airtime_factor_tenths: 90,
            flood_scope_name: [0u8; 31],
            flood_scope_key: [0u8; 16],
            advert_loc_policy: 0,
            telemetry_mode_base: 0,
            telemetry_mode_loc: 0,
            telemetry_mode_env: 0,
            multi_acks: 0,
            radio_freq_hz: 910_525_000,
            radio_bw_khz: 62,
            channels: [ChannelData {
                name: [0u8; 32],
                secret: [0u8; 16],
            }; MAX_CHANNELS],
            checksum: 0,
        };
        let mut cfg = cfg;
        cfg.checksum = compute_checksum(&cfg);
        let mut bytes = [0u8; core::mem::size_of::<PersistentConfig>()];
        unsafe {
            core::ptr::copy_nonoverlapping(
                &cfg as *const PersistentConfig as *const u8,
                bytes.as_mut_ptr(),
                bytes.len(),
            );
        }
        bytes
    }

    #[test]
    fn parses_a_valid_page() {
        let bytes = config_bytes();
        let cfg = read_config(&bytes).expect("valid config");
        assert_eq!(cfg.magic, CONFIG_MAGIC);
        assert_eq!(cfg.radio_freq_hz, 910_525_000);
        assert_eq!(cfg.custom_priv_key, [3u8; 64]);
    }

    #[test]
    fn rejects_a_corrupt_page() {
        let mut bytes = config_bytes();
        bytes[12] ^= 0xFF;
        assert!(read_config(&bytes).is_none());
    }
}
