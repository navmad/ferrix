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
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
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

/// Commands sent from the application loop to [`display_task`].
pub enum DisplayCmd {
    /// Flush the rendered framebuffer to the panel, then return it via
    /// [`DISPLAY_RETURN`].
    Flush {
        fb: &'static mut EpdFrameBuffer,
        full: bool,
        sleep_after: bool,
    },
    /// Put the panel into deep sleep without refreshing.
    Sleep,
}

/// Queue of display refresh/sleep requests processed off the RX/TX loop.
pub static DISPLAY_CMD: Channel<CriticalSectionRawMutex, DisplayCmd, 2> = Channel::new();

/// Returns ownership of the staging framebuffer once [`display_task`] finishes
/// flushing it to the SSD1681 panel.
pub static DISPLAY_RETURN: Channel<CriticalSectionRawMutex, &'static mut EpdFrameBuffer, 1> =
    Channel::new();

/// Hardware-facing half of the panel owned by [`display_task`] after boot.
pub struct DisplayPanel {
    epd: ThinkNodeEpd,
    prev_fb: &'static mut EpdFrameBuffer,
    sleeping: bool,
    has_frame: bool,
    partial_count: u32,
}

impl DisplayPanel {
    async fn wake(&mut self) -> Result<(), ferrix_ssd1681::Error> {
        if self.sleeping {
            self.epd.init().await?;
            self.sleeping = false;
        }
        Ok(())
    }

    async fn flush(
        &mut self,
        fb: &EpdFrameBuffer,
        full: bool,
        sleep_after: bool,
    ) -> Result<(), ferrix_ssd1681::Error> {
        if !full && self.has_frame && fb.buffer == self.prev_fb.buffer {
            return Ok(());
        }

        if full || !self.has_frame || self.partial_count >= FULL_REFRESH_INTERVAL {
            self.wake().await?;
            self.epd.display_frame_full(&fb.buffer).await?;
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
            self.epd.display_frame_fast(&fb.buffer).await?;
            self.partial_count += 1;
        }

        self.prev_fb.buffer.copy_from_slice(&fb.buffer);
        self.has_frame = true;

        if sleep_after {
            let _ = self.sleep().await;
        }
        Ok(())
    }

    async fn sleep(&mut self) -> Result<(), ferrix_ssd1681::Error> {
        if !self.sleeping {
            self.epd.sleep().await?;
            self.sleeping = true;
        }
        Ok(())
    }
}

/// Background task that drives the slow (400 ms - 2 s) SSD1681 SPI + BUSY
/// refresh cycle without stalling LoRa RX/TX or companion commands.
#[embassy_executor::task]
pub async fn display_task(mut panel: DisplayPanel) -> ! {
    loop {
        match DISPLAY_CMD.receive().await {
            DisplayCmd::Flush {
                fb,
                full,
                sleep_after,
            } => {
                let _ = panel.flush(fb, full, sleep_after).await;
                DISPLAY_RETURN.send(fb).await;
            }
            DisplayCmd::Sleep => {
                let _ = panel.sleep().await;
            }
        }
    }
}

/// ThinkNode M1 display wrapper used during synchronous boot UI setup.
pub struct ThinkNodeDisplay {
    panel: DisplayPanel,
    fb: &'static mut EpdFrameBuffer,
    backlight: Output<'static>,
}

impl ThinkNodeDisplay {
    pub fn new(
        epd: ThinkNodeEpd,
        fb: &'static mut EpdFrameBuffer,
        prev_fb: &'static mut EpdFrameBuffer,
        backlight: Output<'static>,
    ) -> Self {
        Self {
            panel: DisplayPanel {
                epd,
                prev_fb,
                sleeping: false,
                has_frame: false,
                partial_count: 0,
            },
            fb,
            backlight,
        }
    }

    /// Split into the background panel task handle, staging framebuffer, and
    /// frontlight pin once boot splash rendering is complete.
    pub fn into_parts(self) -> (DisplayPanel, &'static mut EpdFrameBuffer, Output<'static>) {
        (self.panel, self.fb, self.backlight)
    }

    /// Turn the frontlight/backlight on or off.
    pub fn set_backlight(&mut self, enabled: bool) {
        if enabled {
            self.backlight.set_high();
        } else {
            self.backlight.set_low();
        }
    }

    /// Initialise the panel controller.
    pub async fn init(&mut self) -> Result<(), ferrix_ssd1681::Error> {
        self.panel.epd.init().await?;
        self.panel.sleeping = false;
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
        self.panel.flush(self.fb, false, false).await
    }

    /// Render the boot splash.
    pub async fn render_boot(
        &mut self,
        node: &CompanionNode<'_>,
    ) -> Result<(), ferrix_ssd1681::Error> {
        ui::render_boot(node, self.fb);
        self.panel.flush(self.fb, true, false).await
    }

    /// Put the panel into deep sleep.
    pub async fn sleep(&mut self) -> Result<(), ferrix_ssd1681::Error> {
        self.panel.sleep().await
    }
}
