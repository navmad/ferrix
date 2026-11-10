//! Battery Monitoring and Telemetry Subsystem for ThinkNode M1

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
}

