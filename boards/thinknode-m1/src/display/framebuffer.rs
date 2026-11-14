//! 200x200 monochrome framebuffer for the ThinkNode M1 panel.
//!
//! The pixel layout and `embedded-graphics` integration are board/UI concerns;
//! the controller driver only sees the raw 1-bit slice.

use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;

pub const EPD_WIDTH: usize = 200;
pub const EPD_HEIGHT: usize = 200;
pub const EPD_BUFFER_SIZE: usize = (EPD_WIDTH * EPD_HEIGHT) / 8;

/// In-memory monochrome framebuffer for a 200x200 EPD.
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
