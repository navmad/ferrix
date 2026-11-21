//! Board identity and fixed hardware constants.

use ferrix_l76k::L76kConfig;

/// Hardware model reported to MeshCore companion apps.
pub const NAME: &str = "ThinkNode M1";

/// USB (vendor, product) IDs the board enumerates with.
pub const USB_IDS: (u16, u16) = (0x239A, 0x8029);

/// ThinkNode M1 battery ADC: AIN2 (P0.04), 150K+150K divider (2:1), SAADC
/// gain 1/6 with the internal 0.6 V reference.
///
/// `7236 / 4096` matches the board's `vbat-mv-multiplier` (2:1 divider,
/// 3.6 V full scale, +0.5% SAADC gain trim).
pub const BATTERY_ADC_SCALE: u32 = 7236;

/// Fatal-error LED blink counts (blue LED P0.13).
pub const PANIC_BLINKS: u32 = 3;
pub const HARDFAULT_BLINKS: u32 = 6;

/// L76K wiring on ThinkNode M1: active-high standby (P1.02) and active-low
/// reset (P1.05) with a 120 ms reset pulse.
pub const L76K_CONFIG: L76kConfig = L76kConfig {
    standby_active_high: true,
    reset_active_low: true,
    reset_pulse_ms: 120,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn l76k_config_matches_driver_default() {
        let def = L76kConfig::default();
        assert_eq!(L76K_CONFIG.standby_active_high, def.standby_active_high);
        assert_eq!(L76K_CONFIG.reset_active_low, def.reset_active_low);
        assert_eq!(L76K_CONFIG.reset_pulse_ms, def.reset_pulse_ms);
    }

    #[test]
    fn fault_blink_counts_are_distinct_and_nonzero() {
        assert!(PANIC_BLINKS > 0);
        assert!(HARDFAULT_BLINKS > 0);
        assert_ne!(PANIC_BLINKS, HARDFAULT_BLINKS);
    }

    #[test]
    fn battery_scale_is_within_expected_divider_range() {
        // 2:1 divider with 3.6 V SAADC full scale is nominally 7200, plus +0.5% trim.
        assert_eq!(BATTERY_ADC_SCALE, 7236);
    }
}
