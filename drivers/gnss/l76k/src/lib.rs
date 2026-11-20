//! Quectel L76K GNSS transport.
//!
//! The driver owns power sequencing (standby and active-low reset), UART byte
//! reads and the module's configuration. NMEA parsing and the shared GPS state
//! stay in `ferrix-core`, so a different GNSS chip can reuse the parser and a
//! different protocol can reuse the power/reset sequence.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![allow(async_fn_in_trait)]

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_io_async::Read;

/// Errors returned by the L76K transport.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// A standby/reset GPIO operation failed.
    Pin,
    /// The UART read failed or returned no data.
    Uart,
}

/// Module wiring and timing options.
#[derive(Clone, Copy, Debug)]
pub struct L76kConfig {
    /// Standby is active-high on the ThinkNode M1.
    pub standby_active_high: bool,
    /// Reset is active-low (asserted by driving it low).
    pub reset_active_low: bool,
    /// How long reset is held before release. `0` skips the pulse entirely.
    pub reset_pulse_ms: u32,
}

impl Default for L76kConfig {
    fn default() -> Self {
        Self {
            standby_active_high: true,
            reset_active_low: true,
            reset_pulse_ms: 120,
        }
    }
}

/// A reset "pin" for boards that do not route the module's reset line.
///
/// Writes are accepted and ignored, so [`L76k`] skips the reset pulse
/// without a separate code path.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoPin;

impl embedded_hal::digital::ErrorType for NoPin {
    type Error = core::convert::Infallible;
}

impl OutputPin for NoPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// An L76K on a UART with standby and reset control pins.
pub struct L76k<Uart, Standby, Reset, Delay> {
    uart: Uart,
    standby: Standby,
    reset: Reset,
    delay: Delay,
    config: L76kConfig,
}

impl<Uart, Standby, Reset, Delay> L76k<Uart, Standby, Reset, Delay>
where
    Uart: Read,
    Standby: OutputPin,
    Reset: OutputPin,
    Delay: DelayNs,
{
    /// Create the transport and leave the module powered off with reset held.
    pub fn new(
        uart: Uart,
        standby: Standby,
        reset: Reset,
        delay: Delay,
        config: L76kConfig,
    ) -> Result<Self, Error> {
        let mut this = Self {
            uart,
            standby,
            reset,
            delay,
            config,
        };
        this.write_standby(false)?;
        this.write_reset(true)?;
        Ok(this)
    }

    fn write_standby(&mut self, active: bool) -> Result<(), Error> {
        let high = output_is_high(active, self.config.standby_active_high);
        if high {
            self.standby.set_high().map_err(|_| Error::Pin)
        } else {
            self.standby.set_low().map_err(|_| Error::Pin)
        }
    }

    fn write_reset(&mut self, asserted: bool) -> Result<(), Error> {
        // An active-low reset is asserted by driving the pin low.
        let high = output_is_high(asserted, !self.config.reset_active_low);
        if high {
            self.reset.set_high().map_err(|_| Error::Pin)
        } else {
            self.reset.set_low().map_err(|_| Error::Pin)
        }
    }

    /// Power the module and release reset.
    ///
    /// With `reset_pulse_ms == 0` (boards without a reset line, see
    /// [`NoPin`]) the reset pulse and its delay are skipped.
    pub async fn power_on(&mut self) -> Result<(), Error> {
        self.write_standby(true)?;
        if self.config.reset_pulse_ms == 0 {
            return Ok(());
        }
        self.write_reset(true)?;
        self.delay.delay_ms(self.config.reset_pulse_ms).await;
        self.write_reset(false)?;
        Ok(())
    }

    /// Assert reset and remove standby power.
    pub async fn power_off(&mut self) -> Result<(), Error> {
        self.write_reset(true)?;
        self.write_standby(false)?;
        Ok(())
    }

    /// Read one byte from the module.
    pub async fn read_byte(&mut self, byte: &mut [u8; 1]) -> Result<(), Error> {
        let read = self.uart.read(&mut byte[..]).await.map_err(|_| Error::Uart)?;
        if read == 0 {
            Err(Error::Uart)
        } else {
            Ok(())
        }
    }

    /// Read up to `buf.len()` bytes from the module.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        self.uart.read(buf).await.map_err(|_| Error::Uart)
    }
}

fn output_is_high(active: bool, active_high: bool) -> bool {
    active == active_high
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Recorder(std::vec::Vec<bool>);

    impl embedded_hal::digital::ErrorType for Recorder {
        type Error = core::convert::Infallible;
    }

    impl OutputPin for Recorder {
        fn set_low(&mut self) -> Result<(), Self::Error> {
            self.0.push(false);
            Ok(())
        }
        fn set_high(&mut self) -> Result<(), Self::Error> {
            self.0.push(true);
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        async fn delay_ns(&mut self, _ns: u32) {}
    }

    struct NoUart;
    impl embedded_io_async::ErrorType for NoUart {
        type Error = embedded_io_async::ErrorKind;
    }
    impl Read for NoUart {
        async fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
            Ok(0)
        }
    }

    fn block_on<F: core::future::Future>(f: F) -> F::Output {
        use core::task::{Context, Poll, Waker};
        let mut cx = Context::from_waker(Waker::noop());
        let mut f = core::pin::pin!(f);
        loop {
            if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    #[test]
    fn power_on_without_reset_line_skips_pulse() {
        let cfg = L76kConfig {
            reset_pulse_ms: 0,
            ..L76kConfig::default()
        };
        let mut gps = L76k::new(NoUart, Recorder(std::vec::Vec::new()), NoPin, NoDelay, cfg).unwrap();
        block_on(gps.power_on()).unwrap();
        // new(): standby off (low); power_on(): standby on (high), nothing else.
        assert_eq!(gps.standby.0, std::vec![false, true]);
    }

    #[test]
    fn power_on_with_reset_line_pulses_reset() {
        let mut gps = L76k::new(
            NoUart,
            Recorder(std::vec::Vec::new()),
            Recorder(std::vec::Vec::new()),
            NoDelay,
            L76kConfig::default(),
        )
        .unwrap();
        block_on(gps.power_on()).unwrap();
        // Active-low reset: held (low) in new(), re-asserted, then released (high).
        assert_eq!(gps.reset.0, std::vec![false, false, true]);
    }

    #[test]
    fn standby_polarity() {
        assert!(!output_is_high(false, true));
        assert!(output_is_high(true, true));
        assert!(output_is_high(false, false));
        assert!(!output_is_high(true, false));
    }

    #[test]
    fn active_low_reset_is_low_when_asserted() {
        // write_reset uses `asserted != reset_active_low` as the pin level.
        let reset_active_low = L76kConfig::default().reset_active_low;
        assert!(!output_is_high(true, !reset_active_low));
        assert!(output_is_high(false, !reset_active_low));
    }
}
