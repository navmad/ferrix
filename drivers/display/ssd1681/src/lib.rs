//! SSD1681 monochrome e-paper controller driver.
//!
//! The driver owns the panel command sequence, RAM window setup and refresh
//! timing. It takes an `embedded-hal-async` SPI device and GPIO pins, so it is
//! independent of the MCU HAL, the framebuffer layout and the UI crate.
//!
//! The caller supplies a 1-bit frame where `1` is white and `0` is black,
//! matching the SSD1681 RAM format.

#![no_std]
#![forbid(unsafe_code)]

use defmt::{info, warn};
use embassy_time::Duration;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

/// Errors returned by the panel driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The SPI transfer failed or timed out.
    Spi,
    /// A GPIO operation failed.
    Pin,
    /// The panel BUSY line did not go idle.
    Busy,
    /// The supplied frame does not match the configured panel size.
    FrameSize,
}

/// Panel geometry and RAM layout.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PanelConfig {
    /// Panel width in pixels.
    pub width: u16,
    /// Panel height in pixels.
    pub height: u16,
}

impl PanelConfig {
    /// 1.54" 200x200 panel used by the ThinkNode M1.
    pub const THINKNODE_M1: Self = Self {
        width: 200,
        height: 200,
    };

    /// Number of bytes in one 1-bit frame.
    pub const fn frame_size(&self) -> usize {
        (self.width as usize * self.height as usize) / 8
    }
}

/// SSD1681 controller on an SPI bus with command/data and control pins.
pub struct Ssd1681<Spi, Dc, Reset, Busy, Backlight, Delay> {
    spi: Spi,
    dc: Dc,
    reset: Reset,
    busy: Busy,
    backlight: Option<Backlight>,
    delay: Delay,
    panel: PanelConfig,
}

impl<Spi, Dc, Reset, Busy, Backlight, Delay> Ssd1681<Spi, Dc, Reset, Busy, Backlight, Delay>
where
    Spi: SpiDevice<u8>,
    Dc: OutputPin,
    Reset: OutputPin,
    Busy: InputPin,
    Backlight: OutputPin,
    Delay: DelayNs,
{
    /// Create a driver. `backlight` may be `None` on panels without one.
    pub fn new(
        spi: Spi,
        dc: Dc,
        reset: Reset,
        busy: Busy,
        backlight: Option<Backlight>,
        delay: Delay,
        panel: PanelConfig,
    ) -> Self {
        Self {
            spi,
            dc,
            reset,
            busy,
            backlight,
            delay,
            panel,
        }
    }

    /// Wait until BUSY goes low, with a 2 s timeout.
    async fn wait_idle(&mut self) -> Result<(), Error> {
        let mut count = 0;
        while self.busy.is_high().map_err(|_| Error::Pin)? {
            self.delay.delay_ms(5).await;
            count += 1;
            if count > 400 {
                warn!("ssd1681: BUSY timeout");
                return Err(Error::Busy);
            }
        }
        Ok(())
    }

    async fn send_cmd(&mut self, cmd: u8) -> Result<(), Error> {
        self.wait_idle().await?;
        self.dc.set_low().map_err(|_| Error::Pin)?;
        match embassy_time::with_timeout(
            Duration::from_millis(100),
            self.spi.write(&[cmd]),
        )
        .await
        {
            Ok(Ok(())) => Ok(()),
            _ => Err(Error::Spi),
        }
    }

    async fn send_data(&mut self, data: &[u8]) -> Result<(), Error> {
        self.wait_idle().await?;
        self.dc.set_high().map_err(|_| Error::Pin)?;
        match embassy_time::with_timeout(
            Duration::from_millis(500),
            self.spi.write(data),
        )
        .await
        {
            Ok(Ok(())) => Ok(()),
            _ => Err(Error::Spi),
        }
    }

    /// Turn an optional frontlight or backlight on or off.
    pub fn set_backlight(&mut self, enabled: bool) -> Result<(), Error> {
        match &mut self.backlight {
            Some(pin) => {
                if enabled {
                    pin.set_high().map_err(|_| Error::Pin)
                } else {
                    pin.set_low().map_err(|_| Error::Pin)
                }
            }
            None => Ok(()),
        }
    }

    /// Hardware reset pulse.
    pub async fn reset(&mut self) -> Result<(), Error> {
        self.reset.set_high().map_err(|_| Error::Pin)?;
        self.delay.delay_ms(2).await;
        self.reset.set_low().map_err(|_| Error::Pin)?;
        self.delay.delay_ms(10).await;
        self.reset.set_high().map_err(|_| Error::Pin)?;
        self.delay.delay_ms(10).await;
        self.wait_idle().await
    }

    /// Initialise the controller registers for this panel.
    pub async fn init(&mut self) -> Result<(), Error> {
        info!("ssd1681: initialising {}x{} panel", self.panel.width, self.panel.height);
        self.reset().await?;

        // Software reset.
        self.send_cmd(0x12).await?;
        self.delay.delay_ms(20).await;
        self.wait_idle().await?;

        // Driver output control: number of gate lines and scan order.
        self.send_cmd(0x01).await?;
        self.send_data(&output_control(self.panel.height)).await?;

        // Data entry mode: X increment, Y increment.
        self.send_cmd(0x11).await?;
        self.send_data(&[0x03]).await?;

        self.set_window().await?;

        // Border waveform: actively drive the border white.
        self.send_cmd(0x3C).await?;
        self.send_data(&[0x05]).await?;

        // Use the internal temperature sensor.
        self.send_cmd(0x18).await?;
        self.send_data(&[0x80]).await?;

        self.wait_idle().await?;
        info!("ssd1681: panel ready");
        Ok(())
    }

    async fn set_window(&mut self) -> Result<(), Error> {
        self.send_cmd(0x44).await?;
        self.send_data(&x_boundary(self.panel.width)).await?;

        self.send_cmd(0x45).await?;
        self.send_data(&y_boundary(self.panel.height)).await?;
        Ok(())
    }

    async fn set_cursor(&mut self) -> Result<(), Error> {
        self.send_cmd(0x4E).await?;
        self.send_data(&[0x00]).await?;
        self.send_cmd(0x4F).await?;
        self.send_data(&[0x00, 0x00]).await?;
        Ok(())
    }

    fn check_frame(&self, frame: &[u8]) -> Result<(), Error> {
        if frame.len() != self.panel.frame_size() {
            Err(Error::FrameSize)
        } else {
            Ok(())
        }
    }

    /// Write the previous-frame baseline into red RAM (`0x26`) without
    /// refreshing the panel.
    ///
    /// Call after waking from deep sleep / hardware reset so the next
    /// differential refresh (`display_frame_fast`) compares against the image
    /// currently on the panel even if controller RAM did not survive reset.
    pub async fn sync_previous(&mut self, frame: &[u8]) -> Result<(), Error> {
        self.check_frame(frame)?;
        self.set_window().await?;
        self.set_cursor().await?;
        self.send_cmd(0x26).await?;
        self.send_data(frame).await
    }

    /// Full refresh with the OTP waveform (about 2 s, clears ghosting).
    pub async fn display_frame_full(&mut self, frame: &[u8]) -> Result<(), Error> {
        self.check_frame(frame)?;
        self.send_cmd(0x3C).await?;
        self.send_data(&[0x05]).await?;
        self.set_window().await?;
        self.set_cursor().await?;

        // Current frame in black/white RAM.
        self.send_cmd(0x24).await?;
        self.send_data(frame).await?;

        // Previous frame in red RAM, so the next differential update is clean.
        self.send_cmd(0x26).await?;
        self.send_data(frame).await?;

        self.send_cmd(0x22).await?;
        self.send_data(&[0xF7]).await?;
        self.send_cmd(0x20).await?;
        self.wait_idle().await
    }

    /// Fast differential refresh (about 400 ms, no black/white flash).
    pub async fn display_frame_fast(&mut self, frame: &[u8]) -> Result<(), Error> {
        self.check_frame(frame)?;
        // HiZ border waveform avoids accumulating a 1-px edge residue across
        // repeated partial updates.
        self.send_cmd(0x3C).await?;
        self.send_data(&[0x80]).await?;
        self.set_window().await?;
        self.set_cursor().await?;

        self.send_cmd(0x24).await?;
        self.send_data(frame).await?;

        // 0xFF = Mode 2 differential update + disable analog charge pump and
        // clock afterward (unlike 0xFC which leaves HV analog and osc powered).
        self.send_cmd(0x22).await?;
        self.send_data(&[0xFF]).await?;
        self.send_cmd(0x20).await?;
        self.wait_idle().await?;

        // Keep the previous-frame RAM in sync with the new baseline.
        self.set_cursor().await?;
        self.send_cmd(0x26).await?;
        self.send_data(frame).await?;
        Ok(())
    }

    /// Default update: fast differential refresh.
    pub async fn display_frame(&mut self, frame: &[u8]) -> Result<(), Error> {
        self.display_frame_fast(frame).await
    }

    /// Enter deep sleep mode 1 (`0x10, 0x01`).
    ///
    /// Both `display_frame_full` (`0xF7`) and `display_frame_fast` (`0xFF`)
    /// already power down the analog charge pump and oscillator on completion.
    pub async fn sleep(&mut self) -> Result<(), Error> {
        self.send_cmd(0x10).await?;
        self.send_data(&[0x01]).await?;
        Ok(())
    }
}

fn output_control(height: u16) -> [u8; 3] {
    let lines = height.saturating_sub(1);
    [lines as u8, (lines >> 8) as u8, 0x00]
}

fn x_boundary(width: u16) -> [u8; 2] {
    [0x00, (width / 8).saturating_sub(1) as u8]
}

fn y_boundary(height: u16) -> [u8; 4] {
    let end = height.saturating_sub(1);
    [0x00, 0x00, end as u8, (end >> 8) as u8]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thinknode_panel_geometry() {
        let panel = PanelConfig::THINKNODE_M1;
        assert_eq!(panel.frame_size(), 5000);
        assert_eq!(output_control(panel.height), [0xC7, 0x00, 0x00]);
        assert_eq!(x_boundary(panel.width), [0x00, 0x18]);
        assert_eq!(y_boundary(panel.height), [0x00, 0x00, 0xC7, 0x00]);
    }

    #[test]
    fn other_panel_sizes() {
        let panel = PanelConfig {
            width: 128,
            height: 296,
        };
        assert_eq!(panel.frame_size(), 128 * 296 / 8);
        assert_eq!(x_boundary(panel.width), [0x00, 0x0F]);
        assert_eq!(y_boundary(panel.height), [0x00, 0x00, 0x27, 0x01]);
    }
}
