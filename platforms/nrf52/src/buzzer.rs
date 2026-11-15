//! Passive buzzer support.
//!
//! The ThinkNode M1 has a passive buzzer on P0.06. It is driven as a GPIO
//! square wave here rather than through a PWM peripheral: the tone frequency
//! is low enough that software toggling is audible, and it avoids sharing the
//! nRF PWM peripherals with future LED or backlight work.

use embassy_nrf::gpio::Output;
use embassy_time::Timer;

/// A passive buzzer driven by toggling one GPIO.
pub struct PassiveBuzzer<'d> {
    pin: Output<'d>,
}

impl<'d> PassiveBuzzer<'d> {
    /// Take ownership of the buzzer pin and leave it low.
    pub fn new(mut pin: Output<'d>) -> Self {
        pin.set_low();
        Self { pin }
    }

    /// Drive a tone for `duration_ms`, then leave the pin low.
    pub async fn beep(&mut self, frequency_hz: u32, duration_ms: u64) {
        let frequency_hz = frequency_hz.clamp(200, 10_000) as u64;
        let half_period_us = (500_000 / frequency_hz).max(1);
        let cycles = (duration_ms * frequency_hz / 1_000).max(1);

        for _ in 0..cycles {
            self.pin.set_high();
            Timer::after_micros(half_period_us).await;
            self.pin.set_low();
            Timer::after_micros(half_period_us).await;
        }
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

    /// Silence the buzzer.
    pub fn stop(&mut self) {
        self.pin.set_low();
    }
}
