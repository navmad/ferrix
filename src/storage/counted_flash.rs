//! Flash wrapper that attributes page erases to a per-store counter.

use core::sync::atomic::{AtomicU32, Ordering};

use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, ReadNorFlash,
};
use nrf_softdevice::Flash;

/// Borrows the SoftDevice flash and counts erases into `erases`.
pub struct CountedFlash<'a> {
    flash: &'a mut Flash,
    erases: &'static AtomicU32,
}

impl<'a> CountedFlash<'a> {
    pub fn new(flash: &'a mut Flash, erases: &'static AtomicU32) -> Self {
        Self { flash, erases }
    }
}

impl ErrorType for CountedFlash<'_> {
    type Error = <Flash as ErrorType>::Error;
}

impl ReadNorFlash for CountedFlash<'_> {
    const READ_SIZE: usize = <Flash as ReadNorFlash>::READ_SIZE;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash.read(offset, bytes).await
    }

    fn capacity(&self) -> usize {
        self.flash.capacity()
    }
}

impl NorFlash for CountedFlash<'_> {
    const WRITE_SIZE: usize = <Flash as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <Flash as NorFlash>::ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let result = self.flash.erase(from, to).await;
        if result.is_ok() {
            let pages = (to.saturating_sub(from) / Self::ERASE_SIZE as u32) as u32;
            self.erases.fetch_add(pages.max(1), Ordering::Relaxed);
        }
        result
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.flash.write(offset, bytes).await
    }
}

impl MultiwriteNorFlash for CountedFlash<'_> {}
