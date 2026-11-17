//! MeshCore persistent configuration.
//!
//! Settings and per-channel records live in a `sequential-storage` map in a
//! 64 KB internal-flash journal. Each save appends one settings record and any
//! dirty channel records, so frequent setting changes wear-level across the
//! whole journal instead of erasing a single page.

use core::ops::Range;

use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use heapless::String;
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};

use super::counted_flash::CountedFlash;
use super::util::{self, fixed_str, str_from_fixed, AlignedBuf, MappedAsyncFlash};
use super::MappedFlash;
use crate::types::{ChannelData, MAX_CHANNELS};

type SharedRawMutex = CriticalSectionRawMutex;

pub const CONFIG_MAGIC: u32 = 0x4643_4647; // "FCFG"
pub const CONFIG_VERSION: u16 = 1;
const DEFAULT_BLE_PIN: u32 = 123_456;
const SETTINGS_KEY: u16 = 0x0001;
const CHANNEL_KEY_BASE: u16 = 0x0100;
const SCRATCH_SIZE: usize = 256;

/// Completed config writes and page erases since boot.
pub static CONFIG_SAVES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static CONFIG_ERASES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct SettingsRecord {
    magic: u32,
    version: u16,
    _pad: u16,
    callsign: [u8; 32],
    custom_priv_key: [u8; 64],
    has_custom_key: u8,
    is_repeater: u8,
    client_repeat: u8,
    /// Explicit padding: `offgrid_home_freq` must be 4-byte aligned.
    _pad_before_offgrid: u8,
    offgrid_home_freq: u32,
    radio_tx_power: i8,
    radio_sf: u8,
    radio_cr: u8,
    path_hash_mode: u8,
    autoadd_config: u8,
    autoadd_max_hops: u8,
    manual_add_contacts: u8,
    /// Bit 0: LEDs disabled. Bit 1: sound/buzzer disabled.
    ui_disabled_flags: u8,
    rx_duty_cycle: u8,
    airtime_factor_tenths: u8,
    flood_scope_name: [u8; 31],
    flood_scope_key: [u8; 16],
    advert_loc_policy: u8,
    telemetry_mode_base: u8,
    telemetry_mode_loc: u8,
    telemetry_mode_env: u8,
    multi_acks: u8,
    /// Explicit padding: `ble_pin` must be 4-byte aligned.
    _pad_before_ble_pin: [u8; 2],
    ble_pin: u32,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
}

#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct ChannelRecord {
    present: u8,
    name: [u8; 32],
    secret: [u8; 16],
    _pad: [u8; 3],
}

const _: () = assert!(core::mem::size_of::<SettingsRecord>() <= SCRATCH_SIZE);
const _: () = assert!(core::mem::offset_of!(SettingsRecord, offgrid_home_freq) == 108);
const _: () = assert!(core::mem::offset_of!(SettingsRecord, ble_pin) == 176);
const _: () = assert!(core::mem::size_of::<SettingsRecord>() == 188);
const _: () = assert!(core::mem::size_of::<ChannelRecord>() <= SCRATCH_SIZE);

pub use crate::state::{BLE_PIN, CUSTOM_PRIV_KEY, NODE_CALLSIGN};

static SAVE_REQUEST_CHANNEL: Channel<SharedRawMutex, u32, 2> = Channel::new();
/// Highest configuration-save sequence number that has reached flash.
static SAVE_ACKED_SEQ: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
/// Next sequence number for a synchronous save request.
#[cfg(not(test))]
static NEXT_SAVE_SEQ: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(1);

/// Channels whose records must be written on the next save.
static DIRTY_CHANNELS: Mutex<SharedRawMutex, core::cell::RefCell<[bool; MAX_CHANNELS]>> =
    Mutex::new(core::cell::RefCell::new([false; MAX_CHANNELS]));

type ConfigCache = Cache<Uncached, Uncached, Uncached, u16>;
type ConfigMap<'a, F> = MapStorage<u16, CountedFlash<'a, F>, ConfigCache>;

fn open<'a, F>(flash: &'a mut F, region: &Range<u32>) -> ConfigMap<'a, F>
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    MapStorage::new(
        CountedFlash::new(flash, &CONFIG_ERASES),
        MapConfig::new(region.clone()),
        Cache::new_uncached(),
    )
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

/// Mark one channel slot dirty so its record is rewritten on the next save.
pub fn mark_channel_dirty(index: usize) {
    if index < MAX_CHANNELS {
        DIRTY_CHANNELS.lock(|d| d.borrow_mut()[index] = true);
    }
}

/// Wait for a coalesced configuration save request.
///
/// Returns the request's sequence number: zero for a fire-and-forget save, or
/// a non-zero value that must be acknowledged after the write completes.
pub async fn wait_save_request() -> u32 {
    SAVE_REQUEST_CHANNEL.receive().await
}

/// Drop queued save requests after a coalescing delay and return the highest
/// sequence number among them.
pub fn drain_save_requests() -> u32 {
    let mut highest = 0;
    while let Ok(seq) = SAVE_REQUEST_CHANNEL.try_receive() {
        highest = highest.max(seq);
    }
    highest
}

/// Signal the storage task to write the current settings to flash.
pub fn request_save_config() {
    let _ = SAVE_REQUEST_CHANNEL.try_send(0);
}

/// True when a config save request is queued.
pub fn has_pending_save() -> bool {
    !SAVE_REQUEST_CHANNEL.is_empty()
}

#[cfg(not(test))]
fn next_save_seq() -> u32 {
    loop {
        let seq = NEXT_SAVE_SEQ.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        if seq != 0 {
            return seq;
        }
    }
}

/// Request a configuration save and wait until it has reached flash.
///
/// Used when the caller cannot tolerate a later reboot losing the write, such
/// as a BLE PIN change that the app applies by restarting the node.
pub async fn request_save_config_and_wait() -> bool {
    #[cfg(test)]
    {
        true
    }
    #[cfg(not(test))]
    {
        use core::sync::atomic::Ordering;

        let seq = next_save_seq();
        let mut queued = false;
        for _ in 0..40 {
            if SAVE_REQUEST_CHANNEL.try_send(seq).is_ok() {
                queued = true;
                break;
            }
            embassy_time::Timer::after_millis(5).await;
        }
        if !queued {
            return false;
        }

        // Requests are FIFO and the storage loop acknowledges the highest
        // sequence it drained, so any acknowledgement at or above `seq`
        // proves this request has been written.
        for _ in 0..300 {
            if SAVE_ACKED_SEQ.load(Ordering::Acquire) >= seq {
                return true;
            }
            embassy_time::Timer::after_millis(5).await;
        }
        false
    }
}

/// Record that saves up to `seq` have completed.
pub fn notify_save_complete(seq: u32) {
    if seq != 0 {
        SAVE_ACKED_SEQ.fetch_max(seq, core::sync::atomic::Ordering::AcqRel);
    }
}

/// Initialize configuration from the journal, or defaults if it is empty.
pub fn init_config(
    reader: &'static dyn MappedFlash,
    region: Range<u32>,
    default_name: &str,
) -> (String<32>, Option<[u8; 64]>) {
    let mut flash = MappedAsyncFlash::new(reader, region.clone());
    let mut sbuf = AlignedBuf::<SCRATCH_SIZE>::new();
    let mut cbuf = AlignedBuf::<SCRATCH_SIZE>::new();

    let (settings, channels, found) = util::block_on(async {
        let mut map = open(&mut flash, &region);
        let settings = match map.fetch_item::<&[u8]>(&mut sbuf.0, &SETTINGS_KEY).await {
            Ok(Some(value)) if value.len() >= core::mem::size_of::<SettingsRecord>() => {
                let rec =
                    unsafe { core::ptr::read_unaligned(value.as_ptr() as *const SettingsRecord) };
                if rec.magic == CONFIG_MAGIC && rec.version == CONFIG_VERSION {
                    Some(rec)
                } else {
                    None
                }
            }
            _ => None,
        };

        let mut channels = [ChannelData {
            name: [0u8; 32],
            secret: [0u8; 16],
        }; MAX_CHANNELS];
        let mut found = [false; MAX_CHANNELS];
        for i in 0..MAX_CHANNELS {
            let key = CHANNEL_KEY_BASE + i as u16;
            if let Ok(Some(value)) = map.fetch_item::<&[u8]>(&mut cbuf.0, &key).await {
                if value.len() >= core::mem::size_of::<ChannelRecord>() {
                    let rec = unsafe {
                        core::ptr::read_unaligned(value.as_ptr() as *const ChannelRecord)
                    };
                    channels[i] = ChannelData {
                        name: rec.name,
                        secret: rec.secret,
                    };
                    found[i] = rec.present != 0;
                }
            }
        }
        (settings, channels, found)
    });

    if let Some(cfg) = settings {
        info!("Storage: Loaded persistent configuration");

        let mut name = String::<32>::new();
        let _ = name.push_str(str_from_fixed(&cfg.callsign));
        if name.is_empty() {
            let _ = name.push_str(default_name);
        }
        NODE_CALLSIGN.lock(|c| *c.borrow_mut() = name.clone());

        crate::state::CHANNELS.lock(|c| *c.borrow_mut() = channels);
        let missing = DIRTY_CHANNELS.lock(|d| {
            let mut dirty = d.borrow_mut();
            let mut missing = false;
            for i in 0..MAX_CHANNELS {
                dirty[i] = !found[i];
                missing |= !found[i];
            }
            missing
        });
        if missing {
            request_save_config();
        }

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
            s.path_hash_mode = if cfg.path_hash_mode <= 2 {
                cfg.path_hash_mode
            } else {
                0
            };
            s.rx_duty_cycle = cfg.rx_duty_cycle != 0;
            s.airtime_factor_tenths = cfg.airtime_factor_tenths.min(90);
        });

        crate::state::AUTOADD_SETTINGS.lock(|a| {
            *a.borrow_mut() = crate::types::AutoAddSettings {
                manual_add_contacts: if cfg.manual_add_contacts != 0 { 1 } else { 0 },
                config: cfg.autoadd_config,
                max_hops: cfg.autoadd_max_hops.min(64),
            };
        });

        crate::state::init_leds_disabled(cfg.ui_disabled_flags & 1 != 0);
        crate::state::init_buzzer_enabled(cfg.ui_disabled_flags & 2 == 0);

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
            p.multi_acks = cfg.multi_acks.min(1);
        });

        BLE_PIN.store(
            if (100_000..=999_999).contains(&cfg.ble_pin) {
                cfg.ble_pin
            } else {
                DEFAULT_BLE_PIN
            },
            core::sync::atomic::Ordering::Relaxed,
        );

        let priv_key = if cfg.has_custom_key != 0 {
            CUSTOM_PRIV_KEY.lock(|k| *k.borrow_mut() = Some(cfg.custom_priv_key));
            Some(cfg.custom_priv_key)
        } else {
            None
        };

        info!("Storage: callsign '{}', channels and radio restored", name.as_str());
        (name, priv_key)
    } else {
        info!("Storage: No persistent config found; initializing defaults");
        BLE_PIN.store(DEFAULT_BLE_PIN, core::sync::atomic::Ordering::Relaxed);
        let mut name = String::<32>::new();
        let _ = name.push_str(default_name);
        NODE_CALLSIGN.lock(|c| *c.borrow_mut() = name.clone());
        DIRTY_CHANNELS.lock(|d| d.borrow_mut().iter_mut().for_each(|x| *x = true));
        // Persist the defaults (including all channel slots) on first boot.
        request_save_config();
        (name, None)
    }
}

fn assemble_settings() -> SettingsRecord {
    let callsign = NODE_CALLSIGN.lock(|c| fixed_str::<32>(c.borrow().as_str()));

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

    let (has_custom_key, custom_priv_key) = CUSTOM_PRIV_KEY.lock(|k| {
        if let Some(key) = *k.borrow() {
            (1u8, key)
        } else {
            (0u8, [0u8; 64])
        }
    });

    let auto = crate::state::AUTOADD_SETTINGS.lock(|a| *a.borrow());
    let prefs = crate::state::prefs();

    SettingsRecord {
        magic: CONFIG_MAGIC,
        version: CONFIG_VERSION,
        _pad: 0,
        callsign,
        custom_priv_key,
        has_custom_key,
        is_repeater: if is_rep { 1 } else { 0 },
        client_repeat: if client_repeat { 1 } else { 0 },
        _pad_before_offgrid: 0,
        offgrid_home_freq,
        radio_tx_power: tx,
        radio_sf: sf,
        radio_cr: cr,
        path_hash_mode: path_mode,
        autoadd_config: auto.config,
        autoadd_max_hops: auto.max_hops,
        manual_add_contacts: auto.manual_add_contacts,
        ui_disabled_flags: (if crate::state::leds_disabled() { 1 } else { 0 })
            | (if crate::state::buzzer_enabled() { 0 } else { 2 }),
        rx_duty_cycle: if rx_duty { 1 } else { 0 },
        airtime_factor_tenths: af,
        flood_scope_name: prefs.scope_name,
        flood_scope_key: prefs.scope_key,
        advert_loc_policy: prefs.advert_loc_policy,
        telemetry_mode_base: prefs.telemetry_mode_base,
        telemetry_mode_loc: prefs.telemetry_mode_loc,
        telemetry_mode_env: prefs.telemetry_mode_env,
        multi_acks: prefs.multi_acks,
        _pad_before_ble_pin: [0; 2],
        ble_pin: BLE_PIN.load(core::sync::atomic::Ordering::Relaxed),
        radio_freq_hz: freq,
        radio_bw_khz: bw,
    }
}

/// Append the current settings and any dirty channel records to the journal.
pub async fn save_config<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let settings = assemble_settings();
    let settings_bytes = unsafe {
        core::slice::from_raw_parts(
            &settings as *const SettingsRecord as *const u8,
            core::mem::size_of::<SettingsRecord>(),
        )
    };

    let mut buf = AlignedBuf::<SCRATCH_SIZE>::new();
    let mut map = open(flash, region);

    if let Err(e) = map
        .store_item(&mut buf.0, &SETTINGS_KEY, &settings_bytes)
        .await
    {
        error!("Storage: config settings store failed: {:?}", e);
        return;
    }

    let dirty = {
        let d = DIRTY_CHANNELS.lock(|c| *c.borrow());
        d
    };

    if dirty.iter().any(|d| *d) {
        let channels = crate::state::CHANNELS.lock(|c| *c.borrow());
        for i in 0..MAX_CHANNELS {
            if !dirty[i] {
                continue;
            }
            let ch = channels[i];
            let present = if ch.name[0] != 0 || ch.secret != [0u8; 16] {
                1
            } else {
                0
            };
            let rec = ChannelRecord {
                present,
                name: ch.name,
                secret: ch.secret,
                _pad: [0; 3],
            };
            let bytes = unsafe {
                core::slice::from_raw_parts(
                    &rec as *const ChannelRecord as *const u8,
                    core::mem::size_of::<ChannelRecord>(),
                )
            };
            let key = CHANNEL_KEY_BASE + i as u16;
            if let Err(e) = map.store_item(&mut buf.0, &key, &bytes).await {
                error!("Storage: config channel store failed: {:?}", e);
                return;
            }
        }
    }

    DIRTY_CHANNELS.lock(|c| c.borrow_mut().iter_mut().for_each(|d| *d = false));
    CONFIG_SAVES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    info!("Storage: Configuration saved");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records_fit_the_scratch_buffer() {
        assert!(core::mem::size_of::<SettingsRecord>() <= SCRATCH_SIZE);
        assert!(core::mem::size_of::<ChannelRecord>() <= SCRATCH_SIZE);
        assert_eq!(core::mem::size_of::<ChannelRecord>(), 52);
    }
}
