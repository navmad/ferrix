//! Board-agnostic implementations of the companion protocol, text CLI and
//! routing-engine host traits for nRF52 boards.
//!
//! Everything here maps host operations onto `ferrix-core` state, the shared
//! storage stats, the nRF52 clock/reset helpers and the GNSS/battery modules.
//! A board only supplies its identity line and storage profile through
//! [`NrfCompanionHost::new`].

use core::sync::atomic::Ordering;

use ferrix_core::companion::cli::{CliHost, PowerClockStats, RadioStats, StorageStats};
use ferrix_core::companion::protocol::CompanionHost;
use ferrix_core::engine::EngineContext;
use ferrix_core::types::{ChannelData, MAX_CHANNELS};
use meshcore_wire::Packet;

/// Persist the user's LED preference.
pub fn set_leds_disabled(disabled: bool) {
    if ferrix_core::state::LEDS_DISABLED.swap(disabled, Ordering::Relaxed) != disabled {
        ferrix_core::storage::request_save_config();
    }
}

fn softdevice_active() -> bool {
    ferrix_core::state::BLE_STATUS.load(Ordering::Relaxed) >= 3
}

/// Protocol/CLI host shared by nRF52 boards.
pub struct NrfCompanionHost {
    board_line: &'static str,
    external_profile: bool,
}

impl NrfCompanionHost {
    /// `board_line` is the CLI's board description; `external_profile` is
    /// `true` when contacts and messages live on external flash.
    pub const fn new(board_line: &'static str, external_profile: bool) -> Self {
        Self {
            board_line,
            external_profile,
        }
    }
}

impl CompanionHost for NrfCompanionHost {
    fn battery_millivolts(&self) -> u16 {
        crate::battery::battery_millivolts()
    }

    fn mcu_temperature_tenths_c(&self) -> Option<i16> {
        crate::startup::mcu_temperature_tenths_c(softdevice_active())
    }

    fn set_callsign(&self, name: &str) {
        ferrix_core::storage::set_callsign(name);
    }

    fn set_custom_priv_key(&self, key: [u8; 64]) {
        ferrix_core::storage::set_custom_priv_key(key);
    }

    fn set_path_hash_mode(&self, mode: u8) {
        ferrix_core::storage::set_path_hash_mode(mode);
    }

    fn request_save_config(&self) {
        ferrix_core::storage::request_save_config();
    }

    fn request_radio_config(&self) {
        ferrix_runtime::radio::request_radio_config();
    }

    fn request_factory_reset(&self) {
        ferrix_core::storage::request_factory_reset();
    }

    fn queue_consume_now(&self) {
        ferrix_core::storage::messages::queue_consume_now();
    }

    fn advert_remember(&self, pub_key: &[u8; 32], payload: &[u8]) {
        ferrix_core::storage::adverts::remember(pub_key, payload);
    }

    fn reboot(&self) -> ! {
        crate::reset::system_reset()
    }
}

impl CliHost for NrfCompanionHost {
    fn leds_disabled(&self) -> bool {
        ferrix_core::state::leds_disabled()
    }

    fn set_leds_disabled(&self, disabled: bool) {
        set_leds_disabled(disabled);
    }

    fn storage_stats(&self) -> StorageStats {
        use ferrix_core::storage::{adverts, config, contacts, messages};
        StorageStats {
            contact_saves: contacts::SAVES.load(Ordering::Relaxed),
            contact_erases: contacts::ERASES.load(Ordering::Relaxed),
            message_records: messages::RECORD_SAVES.load(Ordering::Relaxed),
            message_page_erases: messages::PAGE_ERASES.load(Ordering::Relaxed),
            message_queued: messages::QUEUED.load(Ordering::Relaxed),
            advert_records: adverts::RECORD_SAVES.load(Ordering::Relaxed),
            advert_erases: adverts::ERASES.load(Ordering::Relaxed),
            config_saves: config::CONFIG_SAVES.load(Ordering::Relaxed),
            config_erases: config::CONFIG_ERASES.load(Ordering::Relaxed),
            external_profile: self.external_profile,
            external_unavailable: ferrix_core::storage::external_unavailable(),
            external_drops: ferrix_core::storage::external_drops(),
        }
    }

    fn radio_stats(&self) -> RadioStats {
        RadioStats {
            override_ms: ferrix_core::state::TX_SLOT_MS_OVERRIDE.load(Ordering::Relaxed),
            slot_ms: ferrix_core::state::TX_SLOT_MS_LAST.load(Ordering::Relaxed),
            used: ferrix_core::state::TX_SLOT_USED.load(Ordering::Relaxed),
            ack_stats: [
                ferrix_core::state::ACK_RX_MATCHED.load(Ordering::Relaxed),
                ferrix_core::state::ACK_RX_UNMATCHED.load(Ordering::Relaxed),
                ferrix_core::state::PENDING_ACK_EVICTED.load(Ordering::Relaxed),
            ],
            queue_full: core::array::from_fn(|i| {
                ferrix_core::state::TX_QUEUE_FULL[i].load(Ordering::Relaxed)
            }),
        }
    }

    fn power_clock_stats(&self) -> Option<PowerClockStats> {
        use crate::startup::{Lfclk, LfclkAccuracy};
        let hw = crate::startup::hardware_power_clock_status();
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

    fn softdevice_vectors(&self) -> Option<(u32, u32)> {
        crate::flash::softdevice_vectors()
    }

    fn set_tx_slot_override(&self, ms: u32) {
        ferrix_core::state::TX_SLOT_MS_OVERRIDE.store(ms, Ordering::Relaxed);
    }

    fn clear_radio_stats(&self) {
        ferrix_core::state::ACK_RX_MATCHED.store(0, Ordering::Relaxed);
        ferrix_core::state::ACK_RX_UNMATCHED.store(0, Ordering::Relaxed);
        ferrix_core::state::PENDING_ACK_EVICTED.store(0, Ordering::Relaxed);
        for c in &ferrix_core::state::TX_QUEUE_FULL {
            c.store(0, Ordering::Relaxed);
        }
    }

    fn advert_slots(&self) -> usize {
        ferrix_core::storage::adverts::slots()
    }

    fn gps_enabled(&self) -> bool {
        crate::gnss::gps_enabled()
    }

    fn set_gps_enabled(&self, on: bool) {
        crate::gnss::set_gps_enabled(on);
    }

    fn reboot_to_bootloader(&self) -> ! {
        crate::reset::reboot_to_bootloader(softdevice_active())
    }

    fn reboot_to_ota_dfu(&self) -> ! {
        crate::reset::reboot_to_bootloader_mode(
            softdevice_active(),
            crate::reset::BootloaderMode::BleOta,
        )
    }

    fn board_line(&self) -> &'static str {
        self.board_line
    }
}

/// Routing engine host backed by the shared state.
pub struct NrfEngineContext;

impl EngineContext for NrfEngineContext {
    fn path_hash_mode(&self) -> u8 {
        ferrix_core::storage::get_path_hash_mode()
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

    fn anon_peer_key(&self, hash: u8) -> Option<[u8; 32]> {
        ferrix_core::companion::protocol::pending_anon_peer(hash)
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

    fn shared_secret(&self, prv: &[u8; 64], peer: &[u8; 32]) -> Option<[u8; 32]> {
        ferrix_core::ecdh::shared_secret(prv, peer)
    }

    fn on_private_key_changed(&self) {
        ferrix_core::ecdh::invalidate();
    }

    fn apply_flood_scope(&self, pkt: &mut Packet) {
        ferrix_core::state::apply_flood_scope(pkt);
    }

    fn apply_flood_scope_default(&self, pkt: &mut Packet) {
        ferrix_core::state::apply_flood_scope_default(pkt);
    }

    fn is_pending_ack(&self, crc: u32) -> bool {
        ferrix_runtime::app::is_pending_ack(crc)
    }

    fn note_tx_queue_full(&self, lane: meshcore_wire::tx::TxLane) {
        ferrix_core::state::TX_QUEUE_FULL[lane as usize].fetch_add(1, Ordering::Relaxed);
    }

    fn cancel_flood(&self, hash: [u8; 8]) {
        ferrix_runtime::radio::cancel_flood(hash);
    }
}
