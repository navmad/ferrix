//! SoftDevice and boot-time internal flash access.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::peripherals::NVMC;
use embassy_nrf::Peripheral;
use embedded_storage::nor_flash::ReadNorFlash;
use ferrix_core::storage::MappedFlash;
use nrf_softdevice::Softdevice;

/// Take the singleton SoftDevice flash driver.
pub fn take(sd: &Softdevice) -> nrf_softdevice::Flash {
    nrf_softdevice::Flash::take(sd)
}

static SD_VECTORS_CAPTURED: AtomicBool = AtomicBool::new(false);
static SD_VECTOR_W0: AtomicU32 = AtomicU32::new(0);
static SD_VECTOR_W1: AtomicU32 = AtomicU32::new(0);

/// Snapshot the SoftDevice vector table header (`0x1000..0x1008`) safely via
/// NVMC at boot so `stats-ble` can report it if SoftDevice enable fails.
pub fn capture_softdevice_vectors(reader: &dyn MappedFlash) {
    let mut buf = [0u8; 8];
    reader.read(0x1000, &mut buf);
    SD_VECTOR_W0.store(
        u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
        Ordering::Relaxed,
    );
    SD_VECTOR_W1.store(
        u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
        Ordering::Relaxed,
    );
    SD_VECTORS_CAPTURED.store(true, Ordering::Release);
}

/// Return the boot-captured SoftDevice vector words `(w0, w1)`, if available.
pub fn softdevice_vectors() -> Option<(u32, u32)> {
    SD_VECTORS_CAPTURED
        .load(Ordering::Acquire)
        .then(|| (SD_VECTOR_W0.load(Ordering::Relaxed), SD_VECTOR_W1.load(Ordering::Relaxed)))
}

/// Safe synchronous internal-flash reader backed by `embassy_nrf::nvmc::Nvmc`.
///
/// Used at boot before the S140 SoftDevice is enabled, then dropped before
/// handing flash ownership to the SoftDevice.
pub struct NvmcReader<'d> {
    nvmc: RefCell<Nvmc<'d>>,
}

impl<'d> NvmcReader<'d> {
    pub fn new(nvmc: impl Peripheral<P = NVMC> + 'd) -> Self {
        Self {
            nvmc: RefCell::new(Nvmc::new(nvmc)),
        }
    }
}

impl MappedFlash for NvmcReader<'_> {
    fn read(&self, offset: u32, out: &mut [u8]) {
        if self.nvmc.borrow_mut().read(offset, out).is_err() {
            out.fill(0xFF);
        }
    }
}
