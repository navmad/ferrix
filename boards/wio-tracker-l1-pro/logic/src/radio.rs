//! Mapping from the shared radio settings to the SX1262 LoRa configuration.

use ferrix_core::scheduler::preamble_for;
use ferrix_core::types::RadioSettings;
use ferrix_hal::radio::LoraConfig;

pub fn lora_config_from_settings(s: &RadioSettings) -> LoraConfig {
    LoraConfig {
        frequency_hz: s.frequency_hz,
        tx_power_dbm: s.tx_power_dbm,
        spreading_factor: s.sf,
        bandwidth_khz: s.bw_khz,
        coding_rate: s.cr,
        preamble_length: preamble_for(s.sf),
        rx_duty_cycle: s.rx_duty_cycle,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn settings(sf: u8) -> RadioSettings {
        RadioSettings {
            frequency_hz: 869_618_000,
            tx_power_dbm: 14,
            sf,
            bw_khz: 125,
            cr: 6,
            is_repeater: false,
            client_repeat: false,
            offgrid_home_freq: 0,
            path_hash_mode: 0,
            rx_duty_cycle: true,
            airtime_factor_tenths: 90,
        }
    }

    #[test]
    fn fields_map_one_to_one() {
        let c = lora_config_from_settings(&settings(10));
        assert_eq!(c.frequency_hz, 869_618_000);
        assert_eq!(c.tx_power_dbm, 14);
        assert_eq!(c.spreading_factor, 10);
        assert_eq!(c.bandwidth_khz, 125);
        assert_eq!(c.coding_rate, 6);
        assert!(c.rx_duty_cycle);
    }

    #[test]
    fn preamble_follows_spreading_factor() {
        assert_eq!(lora_config_from_settings(&settings(7)).preamble_length, 32);
        assert_eq!(lora_config_from_settings(&settings(8)).preamble_length, 32);
        assert_eq!(lora_config_from_settings(&settings(9)).preamble_length, 16);
        assert_eq!(lora_config_from_settings(&settings(12)).preamble_length, 16);
    }

    #[test]
    fn default_settings_match_the_default_lora_config_frequency() {
        let s = ferrix_core::state::RADIO_SETTINGS.lock(|r| *r.borrow());
        let c = lora_config_from_settings(&s);
        assert_eq!(c.frequency_hz, 910_525_000);
        assert_eq!(c.spreading_factor, 7);
    }
}
