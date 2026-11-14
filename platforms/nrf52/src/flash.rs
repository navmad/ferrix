//! SoftDevice flash access.

use ferrix_core::storage::MappedFlash;
use nrf_softdevice::Softdevice;

/// Take the singleton SoftDevice flash driver.
pub fn take(sd: &Softdevice) -> nrf_softdevice::Flash {
    nrf_softdevice::Flash::take(sd)
}

/// Synchronous reader over the memory-mapped nRF52 flash window.
pub struct NrfMappedFlash;

impl MappedFlash for NrfMappedFlash {
    fn read(&self, offset: u32, out: &mut [u8]) {
        // SAFETY: callers pass offsets from the board's reserved flash map.
        unsafe {
            core::ptr::copy_nonoverlapping(offset as *const u8, out.as_mut_ptr(), out.len());
        }
    }
}

pub static MAPPED: NrfMappedFlash = NrfMappedFlash;
