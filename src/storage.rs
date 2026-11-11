//! MeshCore Persistent Flash Storage & Configuration Management
//!
//! Stores node identity (callsign), cryptographic keys, radio settings,
//! and group channel table permanently in nRF52840 internal flash.
//!
//! Preserved across firmware updates, serial DFU flashing, and power cycles.

use core::cell::RefCell;
use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embedded_storage_async::nor_flash::NorFlash;
use heapless::String;

use crate::boards::Board;
use crate::companion::usb::ChannelData;

/// Address of the reserved 4 KB flash configuration page
/// (Top page of application flash, immediately preceding bootloader at 0x000ED000)
pub const CONFIG_FLASH_ADDR: u32 = 0x000EC000;
pub const CONFIG_MAGIC: u32 = 0x4D434647; // 'MCFG' (MeshCore Config)
pub const CONFIG_VERSION: u32 = 1;

#[repr(C, align(4))]
#[derive(Clone, Copy)]
pub struct PersistentConfig {
    pub magic: u32,
    pub version: u32,
    pub callsign: [u8; 32],
    pub custom_priv_key: [u8; 32],
    pub has_custom_key: u8,
    pub is_repeater: u8,
    pub radio_tx_power: i8,
    pub radio_sf: u8,
    pub radio_cr: u8,
    pub path_hash_mode: u8,
    pub _reserved: [u8; 2],
    pub radio_freq_hz: u32,
    pub radio_bw_khz: u32,
    pub channels: [ChannelData; 8],
    pub checksum: u32,
}

pub static NODE_CALLSIGN: Mutex<CriticalSectionRawMutex, RefCell<String<32>>> =
    Mutex::new(RefCell::new(String::new()));

pub static CUSTOM_PRIV_KEY: Mutex<CriticalSectionRawMutex, RefCell<Option<[u8; 32]>>> =
    Mutex::new(RefCell::new(None));

pub static SAVE_REQUEST_CHANNEL: Channel<CriticalSectionRawMutex, (), 2> = Channel::new();

/// Compute a 32-bit checksum covering the configuration structure up to the checksum field
pub fn compute_checksum(cfg: &PersistentConfig) -> u32 {
    let words = unsafe {
        core::slice::from_raw_parts(
            cfg as *const _ as *const u32,
            (core::mem::size_of::<PersistentConfig>() - 4) / 4,
        )
    };
    let mut sum = 0u32;
    for &w in words {
        sum = sum.wrapping_add(w);
    }
    sum
}

/// Synchronously read and validate persistent configuration from internal flash memory
pub fn read_config() -> Option<PersistentConfig> {
    let ptr = CONFIG_FLASH_ADDR as *const PersistentConfig;
    let cfg = unsafe { ptr.read_volatile() };
    if cfg.magic == CONFIG_MAGIC && cfg.version == CONFIG_VERSION {
        let expected = compute_checksum(&cfg);
        if expected == cfg.checksum {
            return Some(cfg);
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

/// Set custom private key and queue a persistent flash save
pub fn set_custom_priv_key(key: [u8; 32]) {
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
pub fn init_config() -> (String<32>, Option<[u8; 32]>) {
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
            s.path_hash_mode = if cfg.path_hash_mode <= 2 { cfg.path_hash_mode } else { 0 };
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

    let (freq, tx, sf, bw, cr, is_rep, path_mode) = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        (s.frequency_hz, s.tx_power_dbm, s.sf, s.bw_khz, s.cr, s.is_repeater, s.path_hash_mode)
    });

    let channels = crate::companion::usb::CHANNELS.lock(|c| *c.borrow());

    let (has_custom_key, custom_priv_key) = CUSTOM_PRIV_KEY.lock(|k| {
        if let Some(key) = *k.borrow() {
            (1u8, key)
        } else {
            (0u8, [0u8; 32])
        }
    });

    let mut cfg = PersistentConfig {
        magic: CONFIG_MAGIC,
        version: CONFIG_VERSION,
        callsign: callsign_bytes,
        custom_priv_key,
        has_custom_key,
        is_repeater: if is_rep { 1 } else { 0 },
        radio_tx_power: tx,
        radio_sf: sf,
        radio_cr: cr,
        path_hash_mode: path_mode,
        _reserved: [0; 2],
        radio_freq_hz: freq,
        radio_bw_khz: bw,
        channels,
        checksum: 0,
    };
    cfg.checksum = compute_checksum(&cfg);
    cfg
}

/// Background storage task that listens for save requests and commits to flash via SoftDevice
#[embassy_executor::task]
pub async fn storage_task(sd: &'static nrf_softdevice::Softdevice) -> ! {
    let mut flash = nrf_softdevice::Flash::take(sd);
    info!("Storage: Flash persistence task started (target page 0x{:08x})", CONFIG_FLASH_ADDR);

    loop {
        SAVE_REQUEST_CHANNEL.receive().await;
        // Debounce slightly in case multiple settings are saved in quick succession
        embassy_time::Timer::after_millis(200).await;
        while SAVE_REQUEST_CHANNEL.try_receive().is_ok() {}

        let config = assemble_current_config();
        let config_bytes = unsafe {
            core::slice::from_raw_parts(
                &config as *const _ as *const u8,
                core::mem::size_of::<PersistentConfig>(),
            )
        };

        info!("Storage: Committing persistent configuration to flash at 0x{:08x}...", CONFIG_FLASH_ADDR);
        if let Err(e) = flash.erase(CONFIG_FLASH_ADDR, CONFIG_FLASH_ADDR + 4096).await {
            error!("Storage: Flash page erase error: {:?}", e);
            continue;
        }
        if let Err(e) = flash.write(CONFIG_FLASH_ADDR, config_bytes).await {
            error!("Storage: Flash write error: {:?}", e);
            continue;
        }
        info!("Storage: Configuration successfully saved to flash!");
    }
}
