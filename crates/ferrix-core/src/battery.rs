//! Battery percentage helpers shared by the companion protocol.

/// A single Li-ion cell is protection-cut well above 2 V. Lower readings are
/// ADC noise, a failed sample, or a board without a battery.
pub const MIN_PLAUSIBLE_BATTERY_MV: u16 = 2000;

/// Convert millivolts to an estimated percentage (3.3 V to 4.2 V LiPo curve).
pub fn millivolts_to_percentage(mv: u16) -> u8 {
    if mv >= 4200 {
        100
    } else if mv <= 3300 {
        0
    } else {
        (((mv - 3300) as u32 * 100) / (4200 - 3300)) as u8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn percentage_curve() {
        assert_eq!(millivolts_to_percentage(4250), 100);
        assert_eq!(millivolts_to_percentage(4200), 100);
        assert_eq!(millivolts_to_percentage(3750), 50);
        assert_eq!(millivolts_to_percentage(3200), 0);
    }
}
