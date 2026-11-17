//! Board bindings for the companion protocol, text CLI and routing engine.
//!
//! The protocol and CLI live in `ferrix-core`; this module maps their host
//! operations onto ThinkNode storage, radio, telemetry and reset.

use core::sync::atomic::Ordering;

use ferrix_core::companion::cli::{CliHost, PowerClockStats, RadioStats, StorageStats};
use ferrix_core::companion::protocol::CompanionHost;
use ferrix_core::engine::EngineContext;
use ferrix_core::types::{ChannelData, MAX_CHANNELS};
use meshcore_wire::Packet;

/// Persist the user's LED preference.
pub fn set_leds_disabled(disabled: bool) {
    if ferrix_core::state::LEDS_DISABLED.swap(disabled, Ordering::Relaxed) != disabled {
        crate::storage::request_save_config();
    }
}

/// Board implementation of the protocol's persistence/telemetry interface.
pub struct BoardCompanionHost;

pub static BOARD_HOST: BoardCompanionHost = BoardCompanionHost;

impl CompanionHost for BoardCompanionHost {
    fn battery_millivolts(&self) -> u16 {
        crate::battery_millivolts()
    }

    fn set_callsign(&self, name: &str) {
        crate::storage::set_callsign(name);
    }

    fn set_custom_priv_key(&self, key: [u8; 64]) {
        crate::storage::set_custom_priv_key(key);
    }

    fn set_path_hash_mode(&self, mode: u8) {
        crate::storage::set_path_hash_mode(mode);
    }

    fn request_save_config(&self) {
        crate::storage::request_save_config();
    }

    fn request_radio_config(&self) {
        crate::radio::request_radio_config();
    }

    fn request_factory_reset(&self) {
        crate::storage::request_factory_reset();
    }

    fn queue_consume_now(&self) {
        crate::storage::messages::queue_consume_now();
    }

    fn advert_remember(&self, pub_key: &[u8; 32], payload: &[u8]) {
        crate::storage::adverts::remember(pub_key, payload);
    }

    fn reboot(&self) -> ! {
        ferrix_nrf52::reset::system_reset()
    }
}

impl CliHost for BoardCompanionHost {
    fn leds_disabled(&self) -> bool {
        ferrix_core::state::leds_disabled()
    }

    fn set_leds_disabled(&self, disabled: bool) {
        set_leds_disabled(disabled);
    }

    fn storage_stats(&self) -> StorageStats {
        StorageStats {
            contact_saves: crate::storage::contacts::SAVES.load(Ordering::Relaxed),
            contact_erases: crate::storage::contacts::ERASES.load(Ordering::Relaxed),
            message_records: crate::storage::messages::RECORD_SAVES.load(Ordering::Relaxed),
            message_page_erases: crate::storage::messages::PAGE_ERASES.load(Ordering::Relaxed),
            message_queued: crate::storage::messages::QUEUED.load(Ordering::Relaxed),
            advert_records: crate::storage::adverts::RECORD_SAVES.load(Ordering::Relaxed),
            advert_erases: crate::storage::adverts::ERASES.load(Ordering::Relaxed),
            config_saves: crate::storage::config::CONFIG_SAVES.load(Ordering::Relaxed),
            config_erases: crate::storage::config::CONFIG_ERASES.load(Ordering::Relaxed),
            external_profile: cfg!(not(feature = "internal-storage")),
            external_unavailable: ferrix_core::storage::external_unavailable(),
            external_drops: ferrix_core::storage::external_drops(),
        }
    }

    fn radio_stats(&self) -> RadioStats {
        RadioStats {
            override_ms: ferrix_core::state::TX_SLOT_MS_OVERRIDE.load(Ordering::Relaxed),
            slot_ms: ferrix_core::state::TX_SLOT_MS_LAST.load(Ordering::Relaxed),
            used: ferrix_core::state::TX_SLOT_USED.load(Ordering::Relaxed),
            forward_scheduled: ferrix_core::state::FORWARD_SCHEDULED.load(Ordering::Relaxed),
            forward_sent: ferrix_core::state::FORWARD_SENT.load(Ordering::Relaxed),
            duplicate_deltas: ferrix_core::state::duplicate_deltas(),
        }
    }

    fn power_clock_stats(&self) -> Option<PowerClockStats> {
        use ferrix_nrf52::startup::{Lfclk, LfclkAccuracy};
        let hw = ferrix_nrf52::startup::hardware_power_clock_status();
        let lfclk_config = match hw.configured_lfclk {
            Lfclk::InternalRc => "rc(500ppm)",
            Lfclk::ExternalXtal(LfclkAccuracy::Ppm20) => "xtal(20ppm)",
            Lfclk::ExternalXtal(LfclkAccuracy::Ppm50) => "xtal(50ppm)",
            Lfclk::ExternalXtal(LfclkAccuracy::Ppm100) => "xtal(100ppm)",
            Lfclk::ExternalXtal(LfclkAccuracy::Ppm250) => "xtal(250ppm)",
        };
        let lfclk_src = match hw.lfclk_src {
            0 => "rc",
            1 => "xtal",
            2 => "synth",
            _ => "reserved",
        };
        Some(PowerClockStats {
            lfclk_config,
            lfclk_running: hw.lfclk_running,
            lfclk_src,
            lfclkstat_raw: hw.lfclkstat_raw,
            dcdc_reg1_configured: hw.configured_dcdc_reg1,
            dcdc_reg1_active: hw.dcdc_reg1_active,
            dcdc_reg0_active: hw.dcdc_reg0_active,
            sd_dcdc_result: hw.sd_dcdc_result,
        })
    }

    fn set_tx_slot_override(&self, ms: u32) {
        ferrix_core::state::TX_SLOT_MS_OVERRIDE.store(ms, Ordering::Relaxed);
    }

    fn clear_radio_stats(&self) {
        ferrix_core::state::FORWARD_SCHEDULED.store(0, Ordering::Relaxed);
        ferrix_core::state::FORWARD_SENT.store(0, Ordering::Relaxed);
        ferrix_core::state::clear_duplicate_deltas();
    }

    fn advert_stats(&self) -> (usize, usize, bool) {
        crate::storage::adverts::stats()
    }

    fn advert_slots(&self) -> usize {
        crate::storage::adverts::slots()
    }

    fn gps_enabled(&self) -> bool {
        crate::gps::gps_enabled()
    }

    fn set_gps_enabled(&self, on: bool) {
        crate::gps::set_gps_enabled(on);
    }

    fn reboot_to_bootloader(&self) -> ! {
        let sd_active = ferrix_core::state::BLE_STATUS.load(Ordering::Relaxed) >= 3;
        ferrix_nrf52::reset::reboot_to_bootloader(sd_active)
    }

    fn board_line(&self) -> &'static str {
        "ThinkNode M1 (nRF52840, SX1262 LoRa)"
    }
}

/// Routing engine host implementation backed by the board's shared state.
pub struct BoardEngineContext;

pub static BOARD_ENGINE_CONTEXT: BoardEngineContext = BoardEngineContext;

impl EngineContext for BoardEngineContext {
    fn path_hash_mode(&self) -> u8 {
        crate::storage::get_path_hash_mode()
    }

    fn extra_ack_count(&self) -> u8 {
        ferrix_core::state::prefs().multi_acks.min(1)
    }

    fn peers_by_hash(&self, hash: u8) -> heapless::Vec<[u8; 32], 32> {
        let mut peers = heapless::Vec::new();
        ferrix_core::state::DISCOVERED_CONTACTS.lock(|c| {
            for ct in c.borrow().iter() {
                if ct.pub_key[0] == hash {
                    let _ = peers.push(ct.pub_key);
                }
            }
        });
        peers
    }

    fn channels(&self) -> [ChannelData; MAX_CHANNELS] {
        ferrix_core::state::CHANNELS.lock(|chs| *chs.borrow())
    }

    fn peer_path(&self, pub_key: &[u8; 32]) -> (u8, [u8; 64]) {
        ferrix_core::state::DISCOVERED_CONTACTS.lock(|c| {
            if let Some(ct) = c.borrow().iter().find(|ct| ct.pub_key == *pub_key) {
                (ct.out_path_len, ct.out_path)
            } else {
                (0xFF, [0u8; 64])
            }
        })
    }

    fn apply_flood_scope(&self, pkt: &mut Packet) {
        ferrix_core::state::apply_flood_scope(pkt);
    }

    fn apply_flood_scope_default(&self, pkt: &mut Packet) {
        ferrix_core::state::apply_flood_scope_default(pkt);
    }

    fn note_forward_scheduled(&self) {
        ferrix_core::state::FORWARD_SCHEDULED.fetch_add(1, Ordering::Relaxed);
    }

    fn note_duplicate_delta(&self, ms: u64) {
        ferrix_core::state::record_duplicate_delta(ms);
    }

    fn cancel_flood(&self, hash: [u8; 8]) {
        crate::radio::cancel_flood(hash);
    }
}
