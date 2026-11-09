//! 1.54" 200x200 Monochrome E-Paper Display (SSD1681) Driver for ThinkNode M1
//!
//! Integrates with `embedded-graphics` for rendering off-grid dashboard metrics.

use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::spim::Spim;
use embassy_time::Timer;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use defmt::info;

pub const EPD_WIDTH: usize = 200;
pub const EPD_HEIGHT: usize = 200;
pub const EPD_BUFFER_SIZE: usize = (EPD_WIDTH * EPD_HEIGHT) / 8; // 5000 bytes

/// In-memory monochrome framebuffer for 200x200 EPD
pub struct EpdFrameBuffer {
    pub buffer: [u8; EPD_BUFFER_SIZE],
}

impl EpdFrameBuffer {
    pub const fn new() -> Self {
        // 0xFF = All pixels white (cleared)
        Self {
            buffer: [0xFF; EPD_BUFFER_SIZE],
        }
    }

    pub fn clear(&mut self) {
        self.buffer.fill(0xFF);
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
            if coord.x >= 0 && coord.x < EPD_WIDTH as i32 && coord.y >= 0 && coord.y < EPD_HEIGHT as i32 {
                let x = coord.x as usize;
                let y = coord.y as usize;
                let byte_idx = (y * EPD_WIDTH + x) / 8;
                let bit_mask = 0x80 >> (x % 8);

                match color {
                    BinaryColor::On => {
                        // Black: bit is cleared to 0 in SSD1681 RAM
                        self.buffer[byte_idx] &= !bit_mask;
                    }
                    BinaryColor::Off => {
                        // White: bit is set to 1 in SSD1681 RAM
                        self.buffer[byte_idx] |= bit_mask;
                    }
                }
            }
        }
        Ok(())
    }
}

/// SSD1681 Hardware Display Controller
pub struct Ssd1681<'a, T: embassy_nrf::spim::Instance> {
    spim: Spim<'a, T>,
    cs: Output<'static>,
    dc: Output<'static>,
    reset: Output<'static>,
    busy: Input<'static>,
    backlight: Output<'static>,
}

impl<'a, T: embassy_nrf::spim::Instance> Ssd1681<'a, T> {
    pub fn new(
        spim: Spim<'a, T>,
        cs: Output<'static>,
        dc: Output<'static>,
        reset: Output<'static>,
        busy: Input<'static>,
        backlight: Output<'static>,
    ) -> Self {
        Self {
            spim,
            cs,
            dc,
            reset,
            busy,
            backlight,
        }
    }

    /// Wait until EPD BUSY signal goes low
    async fn wait_idle(&mut self) {
        while self.busy.is_high() {
            Timer::after_millis(10).await;
        }
    }

    /// Send a command byte to SSD1681
    async fn send_cmd(&mut self, cmd: u8) {
        self.wait_idle().await;
        self.dc.set_low();
        self.cs.set_low();
        let _ = self.spim.write(&[cmd]).await;
        self.cs.set_high();
    }

    /// Send data bytes to SSD1681
    async fn send_data(&mut self, data: &[u8]) {
        self.wait_idle().await;
        self.dc.set_high();
        self.cs.set_low();
        let _ = self.spim.write(data).await;
        self.cs.set_high();
    }

    /// Toggle backlight state
    pub fn set_backlight(&mut self, enabled: bool) {
        if enabled {
            self.backlight.set_high();
        } else {
            self.backlight.set_low();
        }
    }

    /// Hardware reset sequence
    pub async fn reset(&mut self) {
        self.reset.set_low();
        Timer::after_millis(10).await;
        self.reset.set_high();
        Timer::after_millis(10).await;
        self.wait_idle().await;
    }

    /// Initialize SSD1681 display registers
    pub async fn init(&mut self) {
        info!("Initializing SSD1681 E-Paper Display...");
        self.reset().await;

        // Software reset
        self.send_cmd(0x12).await;
        Timer::after_millis(20).await;
        self.wait_idle().await;

        // Driver Output control (200 mux lines, Gate scan sequence)
        self.send_cmd(0x01).await;
        self.send_data(&[0xC7, 0x00, 0x00]).await; // (200 - 1) = 199 = 0xC7

        // Data Entry mode: X increment, Y increment
        self.send_cmd(0x11).await;
        self.send_data(&[0x03]).await;

        // Set RAM X address boundaries: 0x00 to 0x18 (25 bytes = 200 pixels)
        self.send_cmd(0x44).await;
        self.send_data(&[0x00, 0x18]).await;

        // Set RAM Y address boundaries: 0x0000 to 0x00C7 (200 lines)
        self.send_cmd(0x45).await;
        self.send_data(&[0x00, 0x00, 0xC7, 0x00]).await;

        // Border waveform control
        self.send_cmd(0x3C).await;
        self.send_data(&[0x05]).await;

        // Temperature sensor control (internal sensor)
        self.send_cmd(0x18).await;
        self.send_data(&[0x80]).await;

        self.wait_idle().await;
        info!("SSD1681 E-Paper Display initialized!");
    }

    /// Push framebuffer contents to SSD1681 RAM and trigger display update
    pub async fn display_frame(&mut self, fb: &EpdFrameBuffer) {
        // Set RAM X address counter to 0
        self.send_cmd(0x4E).await;
        self.send_data(&[0x00]).await;

        // Set RAM Y address counter to 0
        self.send_cmd(0x4F).await;
        self.send_data(&[0x00, 0x00]).await;

        // Write Black/White RAM
        self.send_cmd(0x24).await;
        self.send_data(&fb.buffer).await;

        // Display Update Control 2: Load LUT from OTP and trigger display mode 1
        self.send_cmd(0x22).await;
        self.send_data(&[0xF7]).await;

        // Master Activation (refresh screen)
        self.send_cmd(0x20).await;
        self.wait_idle().await;
    }

    /// Enter deep sleep mode to conserve battery
    pub async fn sleep(&mut self) {
        self.send_cmd(0x10).await; // Deep sleep mode
        self.send_data(&[0x01]).await;
    }
}
