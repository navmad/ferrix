//! Battery voltage state and SAADC sampling shared by nRF52 boards.
//!
//! Boards own the SAADC peripheral and pin; this module averages conversions
//! and publishes the latest value. The raw-to-millivolt arithmetic lives in
//! `ferrix_core::battery` so it is host-tested.

use core::sync::atomic::{AtomicU16, Ordering};

use embassy_futures::select::{select, Either};
use embassy_nrf::saadc::Saadc;
use embassy_time::Timer;

pub use ferrix_core::battery::MIN_PLAUSIBLE_BATTERY_MV;

/// Conversions averaged per sample (within 100 ms each) to suppress noise.
pub const SAMPLES: usize = 8;

/// Latest measured battery voltage. 0 means "not sampled / no battery".
static BATTERY_MV: AtomicU16 = AtomicU16::new(0);

pub fn set_battery_millivolts(mv: u16) {
    if mv >= MIN_PLAUSIBLE_BATTERY_MV {
        BATTERY_MV.store(mv, Ordering::Relaxed);
    }
}

pub fn battery_millivolts() -> u16 {
    BATTERY_MV.load(Ordering::Relaxed)
}

pub fn battery_percentage() -> u8 {
    ferrix_core::battery::millivolts_to_percentage(battery_millivolts())
}

/// Average [`SAMPLES`] conversions and scale to millivolts.
///
/// `scale_num` is the board's `raw * scale_num / 4096` factor (see
/// `ferrix_core::battery::adc_raw_to_millivolts`). Returns `None` on a timeout
/// or an implausible reading.
pub async fn sample_adc_mv(adc: &mut Saadc<'static, 1>, scale_num: u32) -> Option<u16> {
    let mut buf = [0i16; 1];
    let mut sum: u32 = 0;
    for _ in 0..SAMPLES {
        match select(adc.sample(&mut buf), Timer::after_millis(100)).await {
            Either::First(_) => sum += buf[0].max(0) as u32,
            Either::Second(_) => return None,
        }
    }
    let avg_raw = (sum / SAMPLES as u32) as u16;
    let mv = ferrix_core::battery::adc_raw_to_millivolts(avg_raw, scale_num);
    (mv >= MIN_PLAUSIBLE_BATTERY_MV).then_some(mv)
}
