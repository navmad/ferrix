//! Battery Monitoring and Telemetry Subsystem for ThinkNode M1

use core::sync::atomic::{AtomicU16, Ordering};

/// Latest measured battery voltage. 0 means "not sampled / no battery".
static BATTERY_MV: AtomicU16 = AtomicU16::new(0);

/// A single Li-ion cell is protection-cut well above 2 V. Lower readings are
/// ADC noise, a failed sample, or a board without a battery, and must not
/// replace the last good value (which would briefly show 0%).
pub const MIN_PLAUSIBLE_BATTERY_MV: u16 = 2000;

pub fn set_battery_millivolts(mv: u16) {
    if mv >= MIN_PLAUSIBLE_BATTERY_MV {
        BATTERY_MV.store(mv, Ordering::Relaxed);
    }
}

pub fn battery_millivolts() -> u16 {
    BATTERY_MV.load(Ordering::Relaxed)
}

pub fn battery_percentage() -> u8 {
    millivolts_to_percentage(battery_millivolts())
}

/// ThinkNode M1 battery ADC: AIN2 (P0.04), 150K+150K divider (2:1), SAADC
/// gain 1/6 with the internal 0.6 V reference.
///
/// `7236 / 4096` matches the board's `vbat-mv-multiplier` (2:1 divider,
/// 3.6 V full scale, +0.5% SAADC gain trim).
pub fn adc_raw_to_battery_millivolts(raw: u16) -> u16 {
    let mv = (raw as u32 * 7236) / 4096;
    mv.min(u16::MAX as u32) as u16
}

/// Convert raw battery ADC millivolts to an estimated percentage (3.2V to 4.2V LiPo curve)
pub fn millivolts_to_percentage(mv: u16) -> u8 {
    if mv >= 4200 {
        100
    } else if mv <= 3300 {
        0
    } else {
        // Linear interpolation between 3.3V (0%) and 4.2V (100%)
        (((mv - 3300) as u32 * 100) / (4200 - 3300)) as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_battery_percentage() {
        assert_eq!(millivolts_to_percentage(4250), 100);
        assert_eq!(millivolts_to_percentage(4200), 100);
        assert_eq!(millivolts_to_percentage(3750), 50);
        assert_eq!(millivolts_to_percentage(3200), 0);
    }

    #[test]
    fn test_adc_raw_to_battery_millivolts() {
        // Full-scale 3.6 V at the pin, 2:1 divider -> ~7.2 V battery.
        assert_eq!(adc_raw_to_battery_millivolts(4095), 7234);
        assert_eq!(adc_raw_to_battery_millivolts(0), 0);
        // ~4.2 V battery -> ~2377 counts.
        assert_eq!(adc_raw_to_battery_millivolts(2377), 4199);
        // 3.7 V battery.
        assert_eq!(adc_raw_to_battery_millivolts(2094), 3699);
    }
}

