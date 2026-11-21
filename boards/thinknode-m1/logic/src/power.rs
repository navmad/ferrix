//! E-paper frontlight and panel sleep state machine: idle timeout, wake and
//! periodic refresh.
//!
//! Unlike an OLED, the ThinkNode M1 e-paper retains its image with the panel
//! asleep and the frontlight off. The frontlight starts off after boot, turns
//! on for [`BACKLIGHT_IDLE_SECS`] on user input, and puts the panel back to
//! deep sleep when the idle window expires.

use embassy_time::{Duration, Instant};

/// Seconds the display frontlight stays on after the last button press.
pub const BACKLIGHT_IDLE_SECS: u64 = 15;

#[derive(Debug)]
pub struct DisplayPower {
    backlight_on: bool,
    deadline: Instant,
    last_refresh: Instant,
}

impl DisplayPower {
    /// Frontlight starts off; the boot splash already put the panel to sleep.
    pub fn new(now: Instant) -> Self {
        Self {
            backlight_on: false,
            deadline: now + Duration::from_secs(BACKLIGHT_IDLE_SECS),
            last_refresh: now,
        }
    }

    pub fn is_backlight_on(&self) -> bool {
        self.backlight_on
    }

    /// Whether a background flush should immediately put the EPD back to deep
    /// sleep once painting finishes.
    pub fn sleep_after_flush(&self) -> bool {
        !self.backlight_on
    }

    /// User input: turn the frontlight on and re-arm the idle deadline.
    pub fn wake(&mut self, now: Instant) {
        self.backlight_on = true;
        self.deadline = now + Duration::from_secs(BACKLIGHT_IDLE_SECS);
    }

    /// `true` if the frontlight is on and the idle deadline has passed, so the
    /// frontlight should be switched off and the panel put to sleep.
    pub fn idle_expired(&self, now: Instant) -> bool {
        self.backlight_on && now >= self.deadline
    }

    /// Record that the frontlight is off (after idle expiry or before OTA DFU).
    pub fn sleep(&mut self) {
        self.backlight_on = false;
    }

    /// `true` if a periodic refresh is due. `interval_secs == 0` disables it.
    pub fn periodic_due(&self, now: Instant, interval_secs: u16) -> bool {
        interval_secs > 0
            && now.saturating_duration_since(self.last_refresh)
                >= Duration::from_secs(interval_secs as u64)
    }

    /// Record that a frame was handed to the display task.
    pub fn mark_refreshed(&mut self, now: Instant) {
        self.last_refresh = now;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embassy_time::MockDriver;

    fn secs(n: u64) -> Instant {
        Instant::from_secs(n)
    }

    #[test]
    fn starts_with_backlight_off_and_sleep_after_flush() {
        let p = DisplayPower::new(secs(0));
        assert!(!p.is_backlight_on());
        assert!(p.sleep_after_flush());
        assert!(!p.idle_expired(secs(0)));
        assert!(!p.idle_expired(secs(100)));
    }

    #[test]
    fn wake_enables_backlight_until_idle_deadline() {
        let mut p = DisplayPower::new(secs(0));
        p.wake(secs(10));
        assert!(p.is_backlight_on());
        assert!(!p.sleep_after_flush());
        assert!(!p.idle_expired(secs(24)));
        assert!(p.idle_expired(secs(25)));
        assert!(p.idle_expired(secs(500)));
    }

    #[test]
    fn wake_rearms_the_deadline() {
        let mut p = DisplayPower::new(secs(0));
        p.wake(secs(0));
        p.wake(secs(10));
        assert!(!p.idle_expired(secs(24)));
        assert!(p.idle_expired(secs(25)));
    }

    #[test]
    fn sleep_disarms_expiry_and_restores_sleep_after_flush() {
        let mut p = DisplayPower::new(secs(0));
        p.wake(secs(0));
        assert!(p.idle_expired(secs(15)));
        p.sleep();
        assert!(!p.is_backlight_on());
        assert!(p.sleep_after_flush());
        assert!(!p.idle_expired(secs(16)), "off backlight never re-expires");
    }

    #[test]
    fn periodic_refresh_respects_the_interval() {
        let mut p = DisplayPower::new(secs(0));
        assert!(!p.periodic_due(secs(1_000), 0), "0 disables refresh");
        assert!(!p.periodic_due(secs(59), 60));
        assert!(p.periodic_due(secs(60), 60));
        p.mark_refreshed(secs(60));
        assert!(!p.periodic_due(secs(119), 60));
        assert!(p.periodic_due(secs(120), 60));
    }

    #[test]
    fn works_with_the_mock_clock() {
        let driver = MockDriver::get();
        driver.reset();
        let mut p = DisplayPower::new(Instant::now());
        p.wake(Instant::now());
        driver.advance(Duration::from_secs(14));
        assert!(!p.idle_expired(Instant::now()));
        driver.advance(Duration::from_secs(1));
        assert!(p.idle_expired(Instant::now()));
        p.wake(Instant::now());
        assert!(!p.idle_expired(Instant::now()));
    }
}
