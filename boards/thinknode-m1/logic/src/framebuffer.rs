//! In-memory 200x200 monochrome framebuffer for the SSD1681 e-paper panel.
//!
//! SSD1681 RAM polarity: `1` = white (`BinaryColor::Off`), `0` = black
//! (`BinaryColor::On`), packed row-major MSB-first (25 bytes per row).

use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;

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

    /// Set a single pixel; out-of-bounds coordinates are ignored.
    pub fn set_pixel(&mut self, x: i32, y: i32, black: bool) {
        if x >= 0 && x < EPD_WIDTH as i32 && y >= 0 && y < EPD_HEIGHT as i32 {
            let x = x as usize;
            let y = y as usize;
            let byte_idx = (y * EPD_WIDTH + x) / 8;
            let bit_mask = 0x80 >> (x % 8);
            if black {
                self.buffer[byte_idx] &= !bit_mask;
            } else {
                self.buffer[byte_idx] |= bit_mask;
            }
        }
    }

    /// `true` if pixel `(x, y)` is black (`BinaryColor::On`).
    pub fn is_black(&self, x: usize, y: usize) -> bool {
        if x >= EPD_WIDTH || y >= EPD_HEIGHT {
            return false;
        }
        let byte_idx = (y * EPD_WIDTH + x) / 8;
        let bit_mask = 0x80 >> (x % 8);
        (self.buffer[byte_idx] & bit_mask) == 0
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
            self.set_pixel(coord.x, coord.y, color == BinaryColor::On);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_framebuffer_is_all_white() {
        let fb = EpdFrameBuffer::new();
        assert_eq!(fb.buffer.len(), 5_000);
        assert!(fb.buffer.iter().all(|&b| b == 0xFF));
        assert!(!fb.is_black(0, 0));
        assert!(!fb.is_black(EPD_WIDTH - 1, EPD_HEIGHT - 1));
    }

    #[test]
    fn dimensions_match_200x200_panel() {
        let fb = EpdFrameBuffer::default();
        assert_eq!(fb.size(), Size::new(200, 200));
    }

    #[test]
    fn draw_target_uses_msb_first_inverted_epd_polarity() {
        let mut fb = EpdFrameBuffer::new();
        fb.draw_iter([
            Pixel(Point::new(0, 0), BinaryColor::On),
            Pixel(Point::new(7, 0), BinaryColor::On),
            Pixel(Point::new(8, 0), BinaryColor::On),
            Pixel(Point::new(0, 1), BinaryColor::On),
        ])
        .unwrap();

        // Row 0 byte 0: bits 7 (x=0) and 0 (x=7) cleared -> 0b0111_1110 = 0x7E.
        assert_eq!(fb.buffer[0], 0x7E);
        // Row 0 byte 1: bit 7 (x=8) cleared -> 0b0111_1111 = 0x7F.
        assert_eq!(fb.buffer[1], 0x7F);
        // Row 1 starts at byte 25 (200 / 8): bit 7 cleared -> 0x7F.
        assert_eq!(fb.buffer[25], 0x7F);

        assert!(fb.is_black(0, 0));
        assert!(!fb.is_black(1, 0));
        assert!(fb.is_black(7, 0));
        assert!(fb.is_black(8, 0));
        assert!(fb.is_black(0, 1));

        // Drawing BinaryColor::Off restores white (sets bit back to 1).
        fb.draw_iter([Pixel(Point::new(0, 0), BinaryColor::Off)])
            .unwrap();
        assert_eq!(fb.buffer[0], 0xFE);
        assert!(!fb.is_black(0, 0));
    }

    #[test]
    fn out_of_bounds_pixels_are_ignored() {
        let mut fb = EpdFrameBuffer::new();
        fb.draw_iter([
            Pixel(Point::new(-1, 0), BinaryColor::On),
            Pixel(Point::new(0, -1), BinaryColor::On),
            Pixel(Point::new(EPD_WIDTH as i32, 0), BinaryColor::On),
            Pixel(Point::new(0, EPD_HEIGHT as i32), BinaryColor::On),
            Pixel(Point::new(i32::MIN, i32::MAX), BinaryColor::On),
        ])
        .unwrap();
        assert!(fb.buffer.iter().all(|&b| b == 0xFF));
        assert!(!fb.is_black(EPD_WIDTH, 0));
        assert!(!fb.is_black(0, EPD_HEIGHT));
    }

    #[test]
    fn clear_resets_drawn_pixels_to_white() {
        let mut fb = EpdFrameBuffer::new();
        fb.set_pixel(100, 100, true);
        assert!(fb.is_black(100, 100));
        fb.clear();
        assert!(!fb.is_black(100, 100));
        assert!(fb.buffer.iter().all(|&b| b == 0xFF));
    }
}
