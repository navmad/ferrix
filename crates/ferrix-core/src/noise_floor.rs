//! Live idle noise-floor RSSI estimator and deaf-aware SX1262 AGC watchdog.
//!
//! Hardware sampling produces instantaneous RSSI readings in half-dB units
//! (`-(raw as i16)`, e.g. `-236` for `-118.0 dBm`, `-237` for `-118.5 dBm`).
//! Taking the minimum across a short idle burst rejects transient RF peaks and
//! mid-packet energy before feeding the guarded EMA.

/// Fallback noise floor reported before the first valid RSSI sample.
pub const DEFAULT_NOISE_FLOOR_DBM: i16 = -118;

/// Sampling cadence in milliseconds while listening in continuous RX.
pub const NOISE_SAMPLE_INTERVAL_MS: u64 = 5_000;

/// Number of instantaneous RSSI reads taken per idle sampling burst.
pub const RSSI_BURST_SAMPLES: usize = 8;

/// Spacing between reads inside one RSSI burst, in microseconds.
pub const RSSI_BURST_SPACING_US: u64 = 200;

/// Minimum plausible half-dB instantaneous RSSI reading (`-140.0 dBm`).
const MIN_VALID_HALF_DB: i16 = -280;

/// Maximum plausible half-dB instantaneous RSSI reading (`-10.0 dBm`).
const MAX_VALID_HALF_DB: i16 = -20;

/// Upper clamp for the estimated noise floor in Q4 (`1/16 dB`) units (`-50 dBm`).
const CEILING_Q4: i32 = -50 * 16;

/// Number of initial ticks using fast warmup convergence without the upward guard.
const WARMUP_TICKS: u8 = 4;

/// Upward jump above the current floor (in Q4 = `1/16 dB`) treated as transient
/// interference (`10 dB`).
const UPWARD_GUARD_Q4: i32 = 10 * 16;

/// Allow one unguarded upward sample after this many consecutive guarded skips
/// so genuine environmental noise increases can still be tracked.
const UNGUARDED_INTERVAL: u8 = 16;

/// Minimum continuous RX silence (no valid packets, headers, or CRC errors)
/// before the AGC watchdog considers the receiver potentially deaf (10 minutes).
pub const AGC_SILENCE_MS: u64 = 600_000;

/// Number of consecutive bit-identical half-dB idle RSSI burst readings needed
/// to corroborate a frozen front end alongside RX silence.
pub const AGC_FROZEN_SAMPLES: u8 = 8;

/// Return the minimum valid half-dB reading from an instantaneous RSSI burst,
/// rejecting out-of-range samples and transient upward spikes.
pub fn burst_min_half_db(samples: &[i16]) -> Option<i16> {
    samples
        .iter()
        .copied()
        .filter(|&v| (MIN_VALID_HALF_DB..=MAX_VALID_HALF_DB).contains(&v))
        .min()
}

/// Guarded EMA estimator for the channel idle noise floor.
#[derive(Clone, Copy, Debug)]
pub struct NoiseFloorEstimator {
    /// Current estimate in Q4 (`1/16 dB`) fixed-point units.
    ema_q4: i32,
    /// Valid ticks accepted since boot or reset (`0` = unseeded).
    samples_seen: u8,
    /// Consecutive ticks rejected by the upward-jump guard.
    upward_guard_skips: u8,
}

impl NoiseFloorEstimator {
    pub const fn new() -> Self {
        Self {
            ema_q4: (DEFAULT_NOISE_FLOOR_DBM as i32) * 16,
            samples_seen: 0,
            upward_guard_skips: 0,
        }
    }

    /// Re-enter warmup after a radio frequency or bandwidth reconfiguration.
    pub fn reset(&mut self) {
        self.samples_seen = 0;
        self.upward_guard_skips = 0;
    }

    /// True once at least one valid idle RSSI burst has seeded the estimator.
    pub fn is_seeded(&self) -> bool {
        self.samples_seen > 0
    }

    /// Current noise floor rounded to the nearest integer dBm.
    pub fn noise_floor_dbm(&self) -> i16 {
        if self.samples_seen == 0 {
            return DEFAULT_NOISE_FLOOR_DBM;
        }
        let rounded = if self.ema_q4 >= 0 {
            (self.ema_q4 + 8) / 16
        } else {
            (self.ema_q4 - 8) / 16
        };
        rounded as i16
    }

    /// Feed one burst-minimum RSSI sample in half-dB units (`-236` = `-118.0 dBm`).
    ///
    /// Returns `Some(noise_floor_dbm)` if the sample was valid (whether or not
    /// the upward-jump guard skipped the EMA update), or `None` if out of range.
    pub fn update(&mut self, burst_min_half_db: i16) -> Option<i16> {
        if !(MIN_VALID_HALF_DB..=MAX_VALID_HALF_DB).contains(&burst_min_half_db) {
            return None;
        }
        let sample_q4 = ((burst_min_half_db as i32) * 8).min(CEILING_Q4);

        if self.samples_seen == 0 {
            self.ema_q4 = sample_q4;
            self.samples_seen = 1;
            self.upward_guard_skips = 0;
            return Some(self.noise_floor_dbm());
        }

        if self.samples_seen < WARMUP_TICKS {
            // Fast warmup (`alpha = 1/2`) without upward guard so the floor
            // locks quickly onto the real RF environment after boot/reconfig.
            let delta = sample_q4 - self.ema_q4;
            self.ema_q4 = (self.ema_q4 + div_round(delta, 2)).min(CEILING_Q4);
            self.samples_seen = self.samples_seen.saturating_add(1);
            self.upward_guard_skips = 0;
            return Some(self.noise_floor_dbm());
        }

        if sample_q4 > self.ema_q4 + UPWARD_GUARD_Q4 {
            self.upward_guard_skips = self.upward_guard_skips.saturating_add(1);
            if self.upward_guard_skips < UNGUARDED_INTERVAL {
                return Some(self.noise_floor_dbm());
            }
            // Periodic unguarded pass: environment stayed elevated long enough
            // to be a real baseline shift rather than transient interference.
            self.upward_guard_skips = 0;
        } else {
            self.upward_guard_skips = 0;
        }

        // Steady-state EMA (`alpha = 1/8`).
        let delta = sample_q4 - self.ema_q4;
        self.ema_q4 = (self.ema_q4 + div_round(delta, 8)).min(CEILING_Q4);
        Some(self.noise_floor_dbm())
    }
}

impl Default for NoiseFloorEstimator {
    fn default() -> Self {
        Self::new()
    }
}

#[inline]
fn div_round(num: i32, den: i32) -> i32 {
    if num >= 0 {
        (num + den / 2) / den
    } else {
        (num - den / 2) / den
    }
}

/// Corroborated deaf-receiver watchdog for SX1262 stuck-AGC recovery.
///
/// Triggers only when BOTH conditions hold simultaneously:
/// 1. At least [`AGC_SILENCE_MS`] (10 minutes) have elapsed without any RX
///    demodulator proof of life (valid packet, valid header, or CRC error).
/// 2. At least [`AGC_FROZEN_SAMPLES`] (8) consecutive idle RSSI burst readings
///    were bit-identical at half-dB resolution (indicating a frozen front end
///    rather than live thermal noise).
#[derive(Clone, Copy, Debug)]
pub struct AgcWatchdog {
    last_rx_proof_ms: u64,
    last_sample_half_db: Option<i16>,
    frozen_streak: u8,
}

impl AgcWatchdog {
    pub const fn new(now_ms: u64) -> Self {
        Self {
            last_rx_proof_ms: now_ms,
            last_sample_half_db: None,
            frozen_streak: 0,
        }
    }

    /// Record demodulator proof of life (valid packet, header, or CRC error).
    pub fn on_rx_proof(&mut self, now_ms: u64) {
        self.last_rx_proof_ms = now_ms;
        self.last_sample_half_db = None;
        self.frozen_streak = 0;
    }

    /// Restart the observation window after radio reconfiguration.
    pub fn on_reconfigure(&mut self, now_ms: u64) {
        self.on_rx_proof(now_ms);
    }

    /// Restart the observation window after an AGC reset completes.
    pub fn on_agc_reset(&mut self, now_ms: u64) {
        self.on_rx_proof(now_ms);
    }

    /// Record one valid burst-minimum idle RSSI reading in half-dB units.
    pub fn on_rssi_sample(&mut self, burst_min_half_db: i16) {
        match self.last_sample_half_db {
            Some(prev) if prev == burst_min_half_db => {
                self.frozen_streak = self.frozen_streak.saturating_add(1);
            }
            _ => {
                self.last_sample_half_db = Some(burst_min_half_db);
                self.frozen_streak = 1;
            }
        }
    }

    /// Consecutive identical idle RSSI readings observed so far.
    pub fn frozen_streak(&self) -> u8 {
        self.frozen_streak
    }

    /// True when both RX silence and frozen RSSI corroborate a stuck AGC.
    pub fn should_reset(&self, now_ms: u64) -> bool {
        now_ms.saturating_sub(self.last_rx_proof_ms) >= AGC_SILENCE_MS
            && self.frozen_streak >= AGC_FROZEN_SAMPLES
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn burst_min_rejects_transient_peaks_and_invalid_samples() {
        // -118.0, -90.0 (packet spike), -118.5, -117.0, 0 (invalid), -300 (invalid)
        let samples = [-236, -180, -237, -234, 0, -300, -160, -235];
        assert_eq!(burst_min_half_db(&samples), Some(-237));
        assert_eq!(burst_min_half_db(&[0, -10, -300]), None);
    }

    #[test]
    fn warmup_converges_quickly_without_upward_guard() {
        let mut est = NoiseFloorEstimator::new();
        assert!(!est.is_seeded());
        assert_eq!(est.noise_floor_dbm(), DEFAULT_NOISE_FLOOR_DBM);

        // Even if the real environment is -96 dBm (+22 dB above default -118),
        // warmup seeds directly without being blocked by the upward guard.
        assert_eq!(est.update(-192), Some(-96));
        assert!(est.is_seeded());
        assert_eq!(est.update(-196), Some(-97));
        assert_eq!(est.update(-196), Some(-98));
        assert_eq!(est.update(-196), Some(-98));
    }

    #[test]
    fn steady_state_ema_tracks_gradual_drift() {
        let mut est = NoiseFloorEstimator::new();
        // Seed and complete warmup at -118.0 dBm.
        for _ in 0..WARMUP_TICKS {
            est.update(-236);
        }
        assert_eq!(est.noise_floor_dbm(), -118);

        // Gradual +4 dB upward shift (-114.0 dBm, within 10 dB guard) converges.
        for _ in 0..24 {
            est.update(-228);
        }
        assert_eq!(est.noise_floor_dbm(), -114);

        // Downward shift (-122.0 dBm) has no guard and converges cleanly.
        for _ in 0..24 {
            est.update(-244);
        }
        assert_eq!(est.noise_floor_dbm(), -122);
    }

    #[test]
    fn upward_jump_guard_rejects_transient_spikes() {
        let mut est = NoiseFloorEstimator::new();
        for _ in 0..WARMUP_TICKS {
            est.update(-236); // -118.0 dBm
        }
        assert_eq!(est.noise_floor_dbm(), -118);

        // Several +28 dB interference bursts (-90 dBm) must not move the floor.
        for _ in 0..5 {
            assert_eq!(est.update(-180), Some(-118));
        }
        // Normal reading resets the skip counter.
        assert_eq!(est.update(-236), Some(-118));
        for _ in 0..5 {
            assert_eq!(est.update(-180), Some(-118));
        }
    }

    #[test]
    fn upward_jump_guard_allows_periodic_unguarded_pass() {
        let mut est = NoiseFloorEstimator::new();
        for _ in 0..WARMUP_TICKS {
            est.update(-236); // -118.0 dBm
        }
        // Sustained +20 dB shift (-98.0 dBm): first UNGUARDED_INTERVAL - 1
        // samples are skipped, then the unguarded pass nudges the EMA upward.
        for _ in 0..(UNGUARDED_INTERVAL - 1) {
            assert_eq!(est.update(-196), Some(-118));
        }
        let after_pass = est.update(-196).unwrap();
        assert!(
            after_pass > -118,
            "periodic unguarded pass must move floor upward, got {after_pass}"
        );
    }

    #[test]
    fn rejects_out_of_range_and_clamps_ceiling() {
        let mut est = NoiseFloorEstimator::new();
        assert_eq!(est.update(0), None);
        assert_eq!(est.update(-10), None);
        assert_eq!(est.update(-300), None);
        assert!(!est.is_seeded());

        // Valid -20.0 dBm reading is clamped to -50 dBm ceiling.
        assert_eq!(est.update(-40), Some(-50));
    }

    #[test]
    fn reset_reenters_warmup() {
        let mut est = NoiseFloorEstimator::new();
        for _ in 0..WARMUP_TICKS {
            est.update(-236);
        }
        assert_eq!(est.noise_floor_dbm(), -118);

        est.reset();
        assert!(!est.is_seeded());
        // New band at -104 dBm locks immediately instead of hitting upward guard.
        assert_eq!(est.update(-208), Some(-104));
    }

    #[test]
    fn agc_watchdog_requires_both_silence_and_frozen_rssi() {
        let mut wd = AgcWatchdog::new(1_000);

        // 8 frozen samples before 10 min silence -> must NOT trigger.
        for _ in 0..AGC_FROZEN_SAMPLES {
            wd.on_rssi_sample(-236);
        }
        assert_eq!(wd.frozen_streak(), AGC_FROZEN_SAMPLES);
        assert!(!wd.should_reset(1_000 + AGC_SILENCE_MS - 1));

        // Once 10 min silence also elapses -> triggers.
        assert!(wd.should_reset(1_000 + AGC_SILENCE_MS));
    }

    #[test]
    fn agc_watchdog_clears_on_rx_proof_or_half_db_dither() {
        let mut wd = AgcWatchdog::new(0);
        for _ in 0..(AGC_FROZEN_SAMPLES - 1) {
            wd.on_rssi_sample(-236);
        }
        // A 0.5 dB thermal dither (-118.0 -> -118.5) proves the front end is alive.
        wd.on_rssi_sample(-237);
        assert_eq!(wd.frozen_streak(), 1);
        assert!(!wd.should_reset(AGC_SILENCE_MS + 10_000));

        // Even if frozen resumes to 8, an RX packet or CRC error resets both timers.
        for _ in 0..AGC_FROZEN_SAMPLES {
            wd.on_rssi_sample(-237);
        }
        assert!(wd.should_reset(AGC_SILENCE_MS + 10_000));
        wd.on_rx_proof(AGC_SILENCE_MS + 10_000);
        assert_eq!(wd.frozen_streak(), 0);
        assert!(!wd.should_reset(AGC_SILENCE_MS + 10_000));
    }

    #[test]
    fn agc_watchdog_resets_window_after_recovery() {
        let mut wd = AgcWatchdog::new(0);
        for _ in 0..AGC_FROZEN_SAMPLES {
            wd.on_rssi_sample(-210);
        }
        assert!(wd.should_reset(AGC_SILENCE_MS));

        wd.on_agc_reset(AGC_SILENCE_MS);
        assert_eq!(wd.frozen_streak(), 0);
        assert!(!wd.should_reset(AGC_SILENCE_MS + AGC_SILENCE_MS - 1));
    }
}
