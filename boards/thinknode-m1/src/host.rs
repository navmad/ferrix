//! Board host implementation for the companion protocol and text CLI.
//!
//! The protocol and CLI live in `ferrix-core`; this module maps their host
//! operations onto ThinkNode storage, radio, telemetry and reset.

use ferrix_core::companion::cli::{CliHost, RadioStats, StorageStats};
use ferrix_core::companion::protocol::CompanionHost;
use heapless::Vec;

/// Board implementation of the protocol's persistence/telemetry interface.
pub struct BoardCompanionHost;

pub static BOARD_HOST: BoardCompanionHost = BoardCompanionHost;

impl CompanionHost for BoardCompanionHost {
    fn battery_millivolts(&self) -> u16 {
        crate::telemetry::battery_millivolts()
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
        crate::radio::sx1262::request_radio_config();
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

    fn advert_payload(&self, pub_key: &[u8; 32]) -> Option<Vec<u8, 132>> {
        crate::storage::adverts::payload(pub_key)
    }

    fn advert_export(&self, pub_key: &[u8; 32]) -> Option<Vec<u8, 134>> {
        crate::storage::adverts::export(pub_key)
    }

    fn reboot(&self) -> ! {
        ferrix_nrf52::reset::system_reset()
    }
}

impl CliHost for BoardCompanionHost {
    fn leds_disabled(&self) -> bool {
        crate::companion::usb::leds_disabled()
    }

    fn set_leds_disabled(&self, disabled: bool) {
        crate::companion::usb::set_leds_disabled(disabled);
    }

    fn storage_stats(&self) -> StorageStats {
        use core::sync::atomic::Ordering::Relaxed;
        StorageStats {
            contact_saves: crate::storage::contacts::SAVES.load(Relaxed),
            contact_erases: crate::storage::contacts::ERASES.load(Relaxed),
            message_records: crate::storage::messages::RECORD_SAVES.load(Relaxed),
            message_page_erases: crate::storage::messages::PAGE_ERASES.load(Relaxed),
            message_queued: crate::storage::messages::QUEUED.load(Relaxed),
            advert_records: crate::storage::adverts::RECORD_SAVES.load(Relaxed),
            advert_erases: crate::storage::adverts::ERASES.load(Relaxed),
            config_saves: crate::storage::config::CONFIG_SAVES.load(Relaxed),
            config_erases: crate::storage::config::CONFIG_ERASES.load(Relaxed),
        }
    }

    fn radio_stats(&self) -> RadioStats {
        use core::sync::atomic::Ordering::Relaxed;
        RadioStats {
            override_ms: crate::radio::TX_SLOT_MS_OVERRIDE.load(Relaxed),
            slot_ms: crate::radio::TX_SLOT_MS_LAST.load(Relaxed),
            used: crate::radio::TX_SLOT_USED.load(Relaxed),
            forward_scheduled: crate::radio::FORWARD_SCHEDULED.load(Relaxed),
            forward_sent: crate::radio::FORWARD_SENT.load(Relaxed),
            duplicate_deltas: crate::radio::duplicate_deltas(),
        }
    }

    fn set_tx_slot_override(&self, ms: u32) {
        crate::radio::TX_SLOT_MS_OVERRIDE
            .store(ms, core::sync::atomic::Ordering::Relaxed);
    }

    fn clear_radio_stats(&self) {
        use core::sync::atomic::Ordering::Relaxed;
        crate::radio::FORWARD_SCHEDULED.store(0, Relaxed);
        crate::radio::FORWARD_SENT.store(0, Relaxed);
        crate::radio::clear_duplicate_deltas();
    }

    fn advert_stats(&self) -> (usize, usize, bool) {
        crate::storage::adverts::stats()
    }

    fn advert_slots(&self) -> usize {
        crate::storage::adverts::slots()
    }

    fn reboot_to_bootloader(&self) -> ! {
        let sd_active =
            ferrix_core::state::BLE_STATUS.load(core::sync::atomic::Ordering::Relaxed) >= 3;
        ferrix_nrf52::reset::reboot_to_bootloader(sd_active)
    }

    fn board_line(&self) -> &'static str {
        "ThinkNode M1 (nRF52840, SX1262 LoRa)"
    }
}
