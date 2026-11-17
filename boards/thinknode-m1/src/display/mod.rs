//! Display subsystem for the ThinkNode M1.
//!
//! The SSD1681 controller lives in `ferrix-ssd1681`; this module owns the
//! board's 200x200 framebuffer, its `embedded-graphics` integration and the
//! `ThinkNodeDisplay` wrapper used by the application loop.

pub mod ui;

pub use ferrix_ssd1681::{PanelConfig, Ssd1681};
pub use ui::render_dashboard;

use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::peripherals::TWISPI1;
use embassy_nrf::spim::Spim;
use embassy_time::Delay;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_hal_bus::spi::ExclusiveDevice;
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_runtime::app::UiState;

pub const EPD_WIDTH: usize = 200;
pub const EPD_HEIGHT: usize = 200;
pub const EPD_BUFFER_SIZE: usize = (EPD_WIDTH * EPD_HEIGHT) / 8;

/// In-memory monochrome framebuffer for the 200x200 panel.
pub struct EpdFrameBuffer {
    pub buffer: [u8; EPD_BUFFER_SIZE],
}

impl EpdFrameBuffer {
    pub const fn new() -> Self {
        // 1 = white in SSD1681 RAM.
        Self {
            buffer: [0xFF; EPD_BUFFER_SIZE],
        }
    }

    pub fn clear(&mut self) {
        self.buffer.fill(0xFF);
    }
}

impl Default for EpdFrameBuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl OriginDimensions for EpdFrameBuffer {
    fn size(&self) -> Size {
        Size::new(EPD_WIDTH as u32, EPD_HEIGHT as u32)
    }
}

impl DrawTarget for EpdFrameBuffer {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x >= 0
                && coord.x < EPD_WIDTH as i32
                && coord.y >= 0
                && coord.y < EPD_HEIGHT as i32
            {
                let x = coord.x as usize;
                let y = coord.y as usize;
                let byte_idx = (y * EPD_WIDTH + x) / 8;
                let bit_mask = 0x80 >> (x % 8);

                match color {
                    // Black: cleared bit.
                    BinaryColor::On => self.buffer[byte_idx] &= !bit_mask,
                    // White: set bit.
                    BinaryColor::Off => self.buffer[byte_idx] |= bit_mask,
                }
            }
        }
        Ok(())
    }
}

type EpdSpi = ExclusiveDevice<Spim<'static, TWISPI1>, Output<'static>, Delay>;

/// Concrete driver type for the ThinkNode panel and its pins.
pub type ThinkNodeEpd = Ssd1681<
    EpdSpi,
    Output<'static>,
    Output<'static>,
    Input<'static>,
    Output<'static>,
    Delay,
>;

/// Maximum number of consecutive fast partial updates before forcing a full
/// clearing refresh to remove DC bias and ghosting.
const FULL_REFRESH_INTERVAL: u32 = 16;

/// ThinkNode M1 display wrapper.
pub struct ThinkNodeDisplay {
    epd: ThinkNodeEpd,
    fb: &'static mut EpdFrameBuffer,
    prev_fb: &'static mut EpdFrameBuffer,
    sleeping: bool,
    has_frame: bool,
    partial_count: u32,
}

impl ThinkNodeDisplay {
    pub fn new(
        epd: ThinkNodeEpd,
        fb: &'static mut EpdFrameBuffer,
        prev_fb: &'static mut EpdFrameBuffer,
    ) -> Self {
        Self {
            epd,
            fb,
            prev_fb,
            sleeping: false,
            has_frame: false,
            partial_count: 0,
        }
    }

    async fn wake(&mut self) -> Result<(), ferrix_ssd1681::Error> {
        if self.sleeping {
            self.epd.init().await?;
            self.sleeping = false;
        }
        Ok(())
    }

    /// Turn the frontlight/backlight on or off.
    pub fn set_backlight(&mut self, enabled: bool) {
        let _ = self.epd.set_backlight(enabled);
    }

    /// Initialise the panel controller.
    pub async fn init(&mut self) -> Result<(), ferrix_ssd1681::Error> {
        self.epd.init().await?;
        self.sleeping = false;
        Ok(())
    }

    /// Render and flush the dashboard, skipping identical frames and forcing a
    /// full clearing refresh every [`FULL_REFRESH_INTERVAL`] partial updates.
    pub async fn render_dashboard(
        &mut self,
        node: &CompanionNode<'_>,
        ui: &UiState,
    ) -> Result<(), ferrix_ssd1681::Error> {
        ui::render_dashboard(node, ui, self.fb);
        if self.has_frame && self.fb.buffer == self.prev_fb.buffer {
            return Ok(());
        }

        if !self.has_frame || self.partial_count >= FULL_REFRESH_INTERVAL {
            self.wake().await?;
            self.epd.display_frame_full(&self.fb.buffer).await?;
            self.partial_count = 0;
        } else {
            if self.sleeping {
                self.epd.init().await?;
                // `prev_fb` holds the exact image currently on the panel;
                // restore it to `0x26` so Mode 2 differential refresh has an
                // intact baseline even if deep sleep / reset wiped RAM.
                self.epd.sync_previous(&self.prev_fb.buffer).await?;
                self.sleeping = false;
            }
            self.epd.display_frame_fast(&self.fb.buffer).await?;
            self.partial_count += 1;
        }

        self.prev_fb.buffer.copy_from_slice(&self.fb.buffer);
        self.has_frame = true;
        Ok(())
    }

    /// Render the dashboard with a full OTP clearing refresh.
    pub async fn render_dashboard_full(
        &mut self,
        node: &CompanionNode<'_>,
        ui: &UiState,
    ) -> Result<(), ferrix_ssd1681::Error> {
        self.wake().await?;
        ui::render_dashboard(node, ui, self.fb);
        self.epd.display_frame_full(&self.fb.buffer).await?;
        self.prev_fb.buffer.copy_from_slice(&self.fb.buffer);
        self.has_frame = true;
        self.partial_count = 0;
        Ok(())
    }

    /// Render the boot splash.
    pub async fn render_boot(
        &mut self,
        node: &CompanionNode<'_>,
    ) -> Result<(), ferrix_ssd1681::Error> {
        self.wake().await?;
        ui::render_boot(node, self.fb);
        self.epd.display_frame_full(&self.fb.buffer).await?;
        self.prev_fb.buffer.copy_from_slice(&self.fb.buffer);
        self.has_frame = true;
        self.partial_count = 0;
        Ok(())
    }

    /// Put the panel into deep sleep.
    pub async fn sleep(&mut self) -> Result<(), ferrix_ssd1681::Error> {
        if !self.sleeping {
            self.epd.sleep().await?;
            self.sleeping = true;
        }
        Ok(())
    }
}
