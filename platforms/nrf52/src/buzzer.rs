//! Passive buzzer support for nRF PWM outputs.
//!
//! The ThinkNode M1 uses a passive buzzer on P0.06, so it needs a square wave
//! rather than a static GPIO level. This wrapper keeps the output low when
//! idle and provides the short tones the UI needs.

use embassy_nrf::pwm::{Instance, SimplePwm};
use embassy_time::Timer;

/// A passive buzzer on one PWM channel.
pub struct PassiveBuzzer<'d, T: Instance> {
    pwm: SimplePwm<'d, T>,
}

impl<'d, T: Instance> PassiveBuzzer<'d, T> {
    /// Wrap an already-created PWM channel.
    pub fn new(pwm: SimplePwm<'d, T>) -> Self {
        let mut this = Self { pwm };
        this.stop();
        this
    }

    /// Drive one tone for `duration_ms`, then silence the output.
    pub async fn beep(&mut self, frequency_hz: u32, duration_ms: u64) {
        let frequency_hz = frequency_hz.clamp(200, 10_000);
        self.pwm.set_period(frequency_hz);
        let duty = (self.pwm.max_duty() / 2).max(1);
        self.pwm.set_duty(0, duty);
        self.pwm.enable();
        Timer::after_millis(duration_ms).await;
        self.stop();
    }

    /// Play a short sequence with a gap between notes.
    pub async fn melody(&mut self, notes: &[(u32, u64)], gap_ms: u64) {
        for &(frequency_hz, duration_ms) in notes {
            self.beep(frequency_hz, duration_ms).await;
            if gap_ms > 0 {
                Timer::after_millis(gap_ms).await;
            }
        }
    }

    /// Silence the buzzer and leave the PWM output low.
    pub fn stop(&mut self) {
        self.pwm.set_duty(0, 0);
        self.pwm.disable();
    }
}
