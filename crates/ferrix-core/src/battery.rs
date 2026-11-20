//! Battery percentage, OCV curve, and smoothing helpers.

/// A single Li-ion cell is protection-cut well above 2 V. Lower readings are
/// ADC noise, a failed sample, or a board without a battery.
pub const MIN_PLAUSIBLE_BATTERY_MV: u16 = 2000;

/// Number of SAADC counts at full scale (12-bit).
pub const ADC_FULL_SCALE_COUNTS: u32 = 4096;

/// Convert a 12-bit SAADC reading to battery millivolts.
///
/// `scale_num` is `divider_ratio * full_scale_mv` rounded for any gain trim:
/// the result is `raw * scale_num / 4096`. With the internal 0.6 V reference
/// and gain 1/6 the full scale is 3600 mV, so a 2:1 divider gives 7200.
pub fn adc_raw_to_millivolts(raw: u16, scale_num: u32) -> u16 {
    let mv = (raw as u32 * scale_num) / ADC_FULL_SCALE_COUNTS;
    mv.min(u16::MAX as u32) as u16
}

/// Single-cell LiPo open-circuit voltage table in 10% steps from 100% down to
/// 0% (generic 3.7 V LiPo discharge curve, 4200..3300 mV).
const OCV_TABLE_MV: [u16; 11] = [
    4200, // 100%
    4110, //  90%
    4020, //  80%
    3950, //  70%
    3870, //  60%
    3840, //  50%
    3800, //  40%
    3750, //  30%
    3700, //  20%
    3600, //  10%
    3300, //   0%
];

/// Convert millivolts to an estimated state-of-charge percentage using the
/// 11-point LiPo OCV discharge curve.
pub fn millivolts_to_percentage(mv: u16) -> u8 {
    if mv == 0 || mv <= OCV_TABLE_MV[OCV_TABLE_MV.len() - 1] {
        return 0;
    }
    if mv >= OCV_TABLE_MV[0] {
        return 100;
    }

    let seg_pct = 10u32;
    for i in 1..OCV_TABLE_MV.len() {
        if mv >= OCV_TABLE_MV[i] {
            let pct_low = 100u32 - (i as u32) * seg_pct;
            let num = (mv - OCV_TABLE_MV[i]) as u32 * seg_pct;
            let den = (OCV_TABLE_MV[i - 1] - OCV_TABLE_MV[i]) as u32;
            return (pct_low + num / den) as u8;
        }
    }

    0
}

/// Apply `>= 2%` hysteresis to displayed battery percentage while allowing
/// exact `0%` / `100%` endpoints to latch.
pub fn apply_percentage_hysteresis(current_pct: u8, new_pct: u8) -> u8 {
    if new_pct == 0 || new_pct == 100 || new_pct.abs_diff(current_pct) >= 2 {
        new_pct
    } else {
        current_pct
    }
}

/// Exponential moving average (`alpha = 1/4`) over battery millivolt samples
/// to damp transient load sags and charger ripple.
#[derive(Clone, Copy, Debug, Default)]
pub struct BatteryMvFilter {
    filtered_mv: Option<u16>,
}

impl BatteryMvFilter {
    pub const fn new() -> Self {
        Self { filtered_mv: None }
    }

    pub fn update(&mut self, sample_mv: u16) -> u16 {
        let next = match self.filtered_mv {
            None => sample_mv,
            Some(prev) => ((prev as u32 * 3 + sample_mv as u32 + 2) / 4) as u16,
        };
        self.filtered_mv = Some(next);
        next
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adc_raw_to_millivolts_thinknode_scale() {
        // ThinkNode M1: 2:1 divider, 3.6 V full scale, +0.5% gain trim.
        assert_eq!(adc_raw_to_millivolts(4095, 7236), 7234);
        assert_eq!(adc_raw_to_millivolts(0, 7236), 0);
        assert_eq!(adc_raw_to_millivolts(2377, 7236), 4199);
        assert_eq!(adc_raw_to_millivolts(2094, 7236), 3699);
    }

    #[test]
    fn adc_raw_to_millivolts_wio_scale() {
        // Wio Tracker L1: 2:1 divider, 3.6 V full scale, no trim.
        assert_eq!(adc_raw_to_millivolts(2389, 7200), 4199);
        assert_eq!(adc_raw_to_millivolts(u16::MAX, 7200), u16::MAX);
    }

    #[test]
    fn percentage_curve() {
        assert_eq!(millivolts_to_percentage(0), 0);
        assert_eq!(millivolts_to_percentage(4250), 100);
        assert_eq!(millivolts_to_percentage(4200), 100);
        assert_eq!(millivolts_to_percentage(4000), 77);
        assert_eq!(millivolts_to_percentage(3840), 50);
        assert_eq!(millivolts_to_percentage(3300), 0);
        assert_eq!(millivolts_to_percentage(3200), 0);
    }

    #[test]
    fn smoother_damps_ripple_without_bouncing() {
        let mut filter = BatteryMvFilter::new();
        let mv0 = filter.update(4169);
        let mut pct = millivolts_to_percentage(mv0);
        assert_eq!(pct, 96);

        // Observed 18 mV ripple on USB charger must not oscillate percentage.
        for sample in [4151, 4158, 4169, 4151, 4153, 4169] {
            let mv = filter.update(sample);
            pct = apply_percentage_hysteresis(pct, millivolts_to_percentage(mv));
            assert_eq!(pct, 96);
        }

        // Sustained discharge updates cleanly once it clears hysteresis.
        for _ in 0..10 {
            let mv = filter.update(3950);
            pct = apply_percentage_hysteresis(pct, millivolts_to_percentage(mv));
        }
        assert!(pct <= 73);
    }
}
