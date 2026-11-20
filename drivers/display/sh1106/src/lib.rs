//! SH1106 monochrome OLED controller driver (I2C).
//!
//! The driver owns the init sequence, contrast, power control and the
//! page-addressed frame flush. It takes an `embedded-hal-async` I2C bus, so it
//! is independent of the MCU HAL, the framebuffer type and the UI.
//!
//! The caller supplies a 1-bit frame in the controller's native layout: 8
//! pages of 128 bytes, where byte `x` of page `p` holds pixels `y = 8p..8p+7`
//! with bit 0 at the top, and `1` means lit.
//!
//! The SH1106 has 132 columns of RAM and a 128-pixel panel centred in it, so
//! every page write starts at column 2.

#![cfg_attr(not(test), no_std)]

use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::geometry::{OriginDimensions, Size};
use embedded_graphics_core::pixelcolor::BinaryColor;
use embedded_graphics_core::Pixel;
use embedded_hal_async::i2c::I2c;

/// Panel width in pixels.
pub const WIDTH: usize = 128;
/// Panel height in pixels.
pub const HEIGHT: usize = 64;
/// Number of 8-pixel-high pages.
pub const PAGES: usize = HEIGHT / 8;
/// Bytes in one frame.
pub const FRAME_SIZE: usize = WIDTH * PAGES;
/// RAM column of the first visible pixel.
pub const COLUMN_OFFSET: u8 = 2;

/// Control byte: the following bytes are commands.
const CTRL_CMD: u8 = 0x00;
/// Control byte: the following bytes are display data.
const CTRL_DATA: u8 = 0x40;

/// Errors returned by the panel driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The I2C transfer failed.
    I2c,
    /// The supplied frame is not [`FRAME_SIZE`] bytes.
    FrameSize,
}

/// In-memory 128x64 frame in the controller's page layout, usable as an
/// `embedded-graphics` draw target. `BinaryColor::On` lights a pixel.
pub struct Framebuffer {
    buf: [u8; FRAME_SIZE],
}

impl Framebuffer {
    pub const fn new() -> Self {
        Self {
            buf: [0; FRAME_SIZE],
        }
    }

    /// Turn every pixel off.
    pub fn clear(&mut self) {
        self.buf.fill(0);
    }

    /// The frame bytes, ready for [`Sh1106::flush`].
    pub fn as_bytes(&self) -> &[u8] {
        &self.buf
    }

    /// Set one pixel; coordinates outside the panel are ignored.
    pub fn set_pixel(&mut self, x: i32, y: i32, on: bool) {
        if x < 0 || y < 0 || x >= WIDTH as i32 || y >= HEIGHT as i32 {
            return;
        }
        let idx = (y as usize / 8) * WIDTH + x as usize;
        let mask = 1u8 << (y as usize % 8);
        if on {
            self.buf[idx] |= mask;
        } else {
            self.buf[idx] &= !mask;
        }
    }
}

impl Default for Framebuffer {
    fn default() -> Self {
        Self::new()
    }
}

impl OriginDimensions for Framebuffer {
    fn size(&self) -> Size {
        Size::new(WIDTH as u32, HEIGHT as u32)
    }
}

impl DrawTarget for Framebuffer {
    type Color = BinaryColor;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(p, color) in pixels {
            self.set_pixel(p.x, p.y, color == BinaryColor::On);
        }
        Ok(())
    }
}

/// SH1106 on an I2C bus.
pub struct Sh1106<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C: I2c> Sh1106<I2C> {
    /// Default 7-bit address used by the Wio Tracker L1 Pro.
    pub const ADDRESS_WIO_TRACKER_L1: u8 = 0x3D;

    /// Wrap an I2C bus. Call [`Sh1106::init`] before drawing.
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    async fn commands(&mut self, cmds: &[u8]) -> Result<(), Error> {
        // One transaction: control byte followed by up to 8 command bytes.
        let mut buf = [0u8; 9];
        let n = cmds.len().min(8);
        buf[0] = CTRL_CMD;
        buf[1..=n].copy_from_slice(&cmds[..n]);
        self.i2c
            .write(self.address, &buf[..=n])
            .await
            .map_err(|_| Error::I2c)
    }

    /// Run the power-up sequence and leave the panel on and blank.
    pub async fn init(&mut self) -> Result<(), Error> {
        self.commands(&[0xAE]).await?; // display off
        self.commands(&[0xD5, 0x80]).await?; // clock divide / oscillator
        self.commands(&[0xA8, 0x3F]).await?; // multiplex ratio 1/64
        self.commands(&[0xD3, 0x00]).await?; // display offset
        self.commands(&[0x40]).await?; // start line 0
        self.commands(&[0xAD, 0x8B]).await?; // internal DC-DC on
        self.commands(&[0xA1]).await?; // segment remap (flip X)
        self.commands(&[0xC8]).await?; // COM scan direction (flip Y)
        self.commands(&[0xDA, 0x12]).await?; // COM pins
        self.commands(&[0x81, 0x80]).await?; // contrast
        self.commands(&[0xD9, 0x22]).await?; // pre-charge
        self.commands(&[0xDB, 0x35]).await?; // VCOM deselect
        self.commands(&[0xA4]).await?; // follow RAM
        self.commands(&[0xA6]).await?; // normal (not inverted)
        self.flush(&[0u8; FRAME_SIZE]).await?;
        self.commands(&[0xAF]).await // display on
    }

    /// Turn the panel on or off. Frame RAM is preserved while off.
    pub async fn set_on(&mut self, on: bool) -> Result<(), Error> {
        self.commands(&[if on { 0xAF } else { 0xAE }]).await
    }

    /// Set the panel contrast (0-255).
    pub async fn set_contrast(&mut self, level: u8) -> Result<(), Error> {
        self.commands(&[0x81, level]).await
    }

    /// Write a full frame to the panel.
    pub async fn flush(&mut self, frame: &[u8]) -> Result<(), Error> {
        if frame.len() != FRAME_SIZE {
            return Err(Error::FrameSize);
        }
        for (page, row) in frame.chunks_exact(WIDTH).enumerate() {
            self.commands(&[
                0xB0 | page as u8,
                COLUMN_OFFSET & 0x0F,
                0x10 | (COLUMN_OFFSET >> 4),
            ])
            .await?;
            // Control byte plus 32 data bytes per transaction keeps the stack
            // buffer small and the bus transactions short.
            for chunk in row.chunks(32) {
                let mut buf = [0u8; 33];
                buf[0] = CTRL_DATA;
                buf[1..=chunk.len()].copy_from_slice(chunk);
                self.i2c
                    .write(self.address, &buf[..=chunk.len()])
                    .await
                    .map_err(|_| Error::I2c)?;
            }
        }
        Ok(())
    }

    /// Release the bus.
    pub fn release(self) -> I2C {
        self.i2c
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_async::i2c::{ErrorType, Operation};

    #[derive(Default)]
    struct MockI2c {
        writes: std::vec::Vec<(u8, std::vec::Vec<u8>)>,
    }

    impl ErrorType for MockI2c {
        type Error = core::convert::Infallible;
    }

    impl I2c for MockI2c {
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                if let Operation::Write(data) = op {
                    self.writes.push((address, data.to_vec()));
                }
            }
            Ok(())
        }
    }

    fn block_on<F: core::future::Future>(f: F) -> F::Output {
        use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};
        fn noop(_: *const ()) {}
        fn clone(_: *const ()) -> RawWaker {
            RawWaker::new(core::ptr::null(), &VTABLE)
        }
        static VTABLE: RawWakerVTable = RawWakerVTable::new(clone, noop, noop, noop);
        let waker = unsafe { Waker::from_raw(RawWaker::new(core::ptr::null(), &VTABLE)) };
        let mut cx = Context::from_waker(&waker);
        let mut f = core::pin::pin!(f);
        loop {
            if let Poll::Ready(v) = f.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    #[test]
    fn flush_addresses_each_page_at_column_offset() {
        let mut d = Sh1106::new(MockI2c::default(), 0x3D);
        let mut frame = [0u8; FRAME_SIZE];
        frame[0] = 0xAA; // page 0, x 0
        frame[FRAME_SIZE - 1] = 0x55; // page 7, x 127
        block_on(d.flush(&frame)).unwrap();
        let w = d.release().writes;

        // Per page: one 4-byte command write + 4 data writes of 32 bytes.
        assert_eq!(w.len(), PAGES * 5);
        assert!(w.iter().all(|(a, _)| *a == 0x3D));
        assert_eq!(w[0].1, std::vec![0x00, 0xB0, 0x02, 0x10]);
        assert_eq!(w[5].1, std::vec![0x00, 0xB1, 0x02, 0x10]);
        assert_eq!(w[35].1, std::vec![0x00, 0xB7, 0x02, 0x10]);
        // First data write: control byte + 32 bytes, first byte is pixel data.
        assert_eq!(w[1].1.len(), 33);
        assert_eq!(w[1].1[..2], [0x40, 0xAA]);
        // Last data write ends with the last frame byte.
        assert_eq!(*w.last().unwrap().1.last().unwrap(), 0x55);
    }

    #[test]
    fn framebuffer_uses_page_layout() {
        let mut fb = Framebuffer::new();
        fb.set_pixel(0, 0, true); // page 0, bit 0
        fb.set_pixel(5, 9, true); // page 1, bit 1
        fb.set_pixel(127, 63, true); // page 7, bit 7
        let b = fb.as_bytes();
        assert_eq!(b[0], 0x01);
        assert_eq!(b[WIDTH + 5], 0x02);
        assert_eq!(b[FRAME_SIZE - 1], 0x80);
        assert_eq!(b.iter().filter(|&&x| x != 0).count(), 3);
        fb.set_pixel(5, 9, false);
        assert_eq!(fb.as_bytes()[WIDTH + 5], 0);
    }

    #[test]
    fn framebuffer_ignores_out_of_range_and_clears() {
        let mut fb = Framebuffer::new();
        fb.set_pixel(-1, 0, true);
        fb.set_pixel(0, -1, true);
        fb.set_pixel(128, 0, true);
        fb.set_pixel(0, 64, true);
        assert!(fb.as_bytes().iter().all(|&b| b == 0));
        fb.set_pixel(10, 10, true);
        fb.clear();
        assert!(fb.as_bytes().iter().all(|&b| b == 0));
    }

    #[test]
    fn framebuffer_draw_target_lights_on_pixels() {
        let mut fb = Framebuffer::new();
        fb.draw_iter([
            Pixel(embedded_graphics_core::geometry::Point::new(3, 2), BinaryColor::On),
            Pixel(embedded_graphics_core::geometry::Point::new(4, 2), BinaryColor::Off),
        ])
        .unwrap();
        assert_eq!(fb.as_bytes()[3], 0x04);
        assert_eq!(fb.as_bytes()[4], 0x00);
    }

    #[test]
    fn flush_rejects_wrong_frame_size() {
        let mut d = Sh1106::new(MockI2c::default(), 0x3D);
        assert_eq!(block_on(d.flush(&[0u8; 10])), Err(Error::FrameSize));
        assert!(d.release().writes.is_empty());
    }

    #[test]
    fn init_ends_with_display_on_and_blank_frame() {
        let mut d = Sh1106::new(MockI2c::default(), 0x3D);
        block_on(d.init()).unwrap();
        let w = d.release().writes;
        assert_eq!(w[0].1, std::vec![0x00, 0xAE]);
        assert_eq!(w.last().unwrap().1, std::vec![0x00, 0xAF]);
        // Data bytes written are all zero (blank frame).
        assert!(w
            .iter()
            .filter(|(_, b)| b[0] == 0x40)
            .all(|(_, b)| b[1..].iter().all(|&x| x == 0)));
    }

    #[test]
    fn power_commands() {
        let mut d = Sh1106::new(MockI2c::default(), 0x3D);
        block_on(d.set_on(false)).unwrap();
        block_on(d.set_on(true)).unwrap();
        block_on(d.set_contrast(0x10)).unwrap();
        let w = d.release().writes;
        assert_eq!(w[0].1, std::vec![0x00, 0xAE]);
        assert_eq!(w[1].1, std::vec![0x00, 0xAF]);
        assert_eq!(w[2].1, std::vec![0x00, 0x81, 0x10]);
    }
}
