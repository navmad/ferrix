//! Flash wrapper that attributes page erases to a per-store counter.

use core::sync::atomic::{AtomicU32, Ordering};

use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, ReadNorFlash,
};

/// Borrows a flash and counts erases into `erases`.
pub struct CountedFlash<'a, F> {
    flash: &'a mut F,
    erases: &'static AtomicU32,
}

impl<'a, F> CountedFlash<'a, F> {
    pub fn new(flash: &'a mut F, erases: &'static AtomicU32) -> Self {
        Self { flash, erases }
    }
}

impl<F: ErrorType> ErrorType for CountedFlash<'_, F> {
    type Error = F::Error;
}

impl<F: ReadNorFlash> ReadNorFlash for CountedFlash<'_, F> {
    const READ_SIZE: usize = F::READ_SIZE;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash.read(offset, bytes).await
    }

    fn capacity(&self) -> usize {
        self.flash.capacity()
    }
}

impl<F: NorFlash> NorFlash for CountedFlash<'_, F> {
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

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

impl<F: MultiwriteNorFlash> MultiwriteNorFlash for CountedFlash<'_, F> {}
