//! OLED power state machine: idle timeout, wake and periodic refresh.
//!
//! The board owns the channel sends; this type only decides *when* a command
//! is needed and records the outcome once the send succeeded, so a full
//! queue is retried on the next tick.

use embassy_time::{Duration, Instant};

/// Seconds the OLED stays on after the last button press.
pub const DISPLAY_IDLE_SECS: u64 = 30;

#[derive(Debug)]
pub struct DisplayPower {
    on: bool,
    deadline: Instant,
    last_refresh: Instant,
}

impl DisplayPower {
    /// Panel starts on with a fresh idle deadline.
    pub fn new(now: Instant) -> Self {
        Self {
            on: true,
            deadline: now + Duration::from_secs(DISPLAY_IDLE_SECS),
            last_refresh: now,
        }
    }

    pub fn is_on(&self) -> bool {
        self.on
    }

    /// User input: re-arm the idle deadline. Returns `true` if the panel is
    /// off and a `SetOn(true)` must be sent (call [`commit`](Self::commit)
    /// with `true` once it was accepted).
    pub fn wake(&mut self, now: Instant) -> bool {
        self.deadline = now + Duration::from_secs(DISPLAY_IDLE_SECS);
        !self.on
    }

    /// `true` if the panel is on and the idle deadline has passed, so a
    /// `SetOn(false)` must be sent.
    pub fn idle_expired(&self, now: Instant) -> bool {
        self.on && now >= self.deadline
    }

    /// Record that a power command was accepted.
    pub fn commit(&mut self, on: bool) {
        self.on = on;
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
    fn stays_on_until_the_idle_deadline() {
        let p = DisplayPower::new(secs(0));
        assert!(p.is_on());
        assert!(!p.idle_expired(secs(29)));
        assert!(p.idle_expired(secs(30)));
        assert!(p.idle_expired(secs(500)));
    }

    #[test]
    fn wake_rearms_the_deadline() {
        let mut p = DisplayPower::new(secs(0));
        assert!(!p.wake(secs(20)), "already on: no SetOn needed");
        assert!(!p.idle_expired(secs(49)));
        assert!(p.idle_expired(secs(50)));
    }

    #[test]
    fn wake_while_off_requests_power_on_until_committed() {
        let mut p = DisplayPower::new(secs(0));
        p.commit(false);
        assert!(!p.is_on());
        assert!(p.wake(secs(100)), "off: SetOn(true) required");
        // Channel was full: nothing committed, next input asks again.
        assert!(p.wake(secs(101)));
        p.commit(true);
        assert!(!p.wake(secs(102)));
    }

    #[test]
    fn failed_power_off_is_retried_every_tick() {
        let mut p = DisplayPower::new(secs(0));
        assert!(p.idle_expired(secs(31)));
        // Send failed -> no commit -> still expired next tick.
        assert!(p.idle_expired(secs(32)));
        p.commit(false);
        assert!(!p.idle_expired(secs(33)), "off panel never re-expires");
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
        driver.advance(Duration::from_secs(29));
        assert!(!p.idle_expired(Instant::now()));
        driver.advance(Duration::from_secs(1));
        assert!(p.idle_expired(Instant::now()));
        p.wake(Instant::now());
        assert!(!p.idle_expired(Instant::now()));
    }
}
