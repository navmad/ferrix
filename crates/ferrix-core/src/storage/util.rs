//! Small helpers shared by the storage modules.

use core::future::Future;
use core::ops::Range;
use core::task::{Context, Poll, Waker};

use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use super::MappedFlash;

/// Word-aligned scratch buffer for the SoftDevice and QSPI DMA paths.
#[repr(C, align(4))]
pub struct AlignedBuf<const N: usize>(pub [u8; N]);

impl<const N: usize> AlignedBuf<N> {
    pub const fn new() -> Self {
        Self([0; N])
    }
}

/// FNV-1a initial value.
pub const FNV_INIT: u32 = 0x811C_9DC5;
const FNV_PRIME: u32 = 0x0100_0193;

pub fn fnv_bytes(h: &mut u32, bytes: &[u8]) {
    for &b in bytes {
        *h ^= b as u32;
        *h = h.wrapping_mul(FNV_PRIME);
    }
}

/// Copy a string into a fixed-size, NUL-padded field.
pub fn fixed_str<const N: usize>(s: &str) -> [u8; N] {
    let mut out = [0u8; N];
    let bytes = s.as_bytes();
    let n = bytes.len().min(N);
    out[..n].copy_from_slice(&bytes[..n]);
    out
}

/// Read a NUL-padded fixed-size string (or the whole field when full).
pub fn str_from_fixed(bytes: &[u8]) -> &str {
    let n = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    core::str::from_utf8(&bytes[..n]).unwrap_or("")
}

/// Error returned by a read-only mapped flash adapter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[derive(defmt::Format)]
pub struct MappedFlashError;

impl NorFlashError for MappedFlashError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

/// Read-only async flash view over a memory-mapped region.
///
/// Used to mount `sequential-storage` maps during boot, before the storage
/// task (and therefore the writable flash driver) exists.
pub struct MappedAsyncFlash<'a> {
    reader: &'a dyn MappedFlash,
    end: u32,
}

impl<'a> MappedAsyncFlash<'a> {
    pub fn new(reader: &'a dyn MappedFlash, region: Range<u32>) -> Self {
        Self {
            reader,
            end: region.end,
        }
    }
}

impl ErrorType for MappedAsyncFlash<'_> {
    type Error = MappedFlashError;
}

impl ReadNorFlash for MappedAsyncFlash<'_> {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        // `sequential-storage` passes absolute flash addresses.
        if offset.saturating_add(bytes.len() as u32) > self.end {
            return Err(MappedFlashError);
        }
        self.reader.read(offset, bytes);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.end as usize
    }
}

impl NorFlash for MappedAsyncFlash<'_> {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = 4096;

    async fn erase(&mut self, _from: u32, _to: u32) -> Result<(), Self::Error> {
        Err(MappedFlashError)
    }

    async fn write(&mut self, _offset: u32, _bytes: &[u8]) -> Result<(), Self::Error> {
        Err(MappedFlashError)
    }
}

impl MultiwriteNorFlash for MappedAsyncFlash<'_> {}

/// Drive a future whose I/O completes synchronously.
///
/// The mapped flash adapter reads finish on the first poll, so this is used
/// only for boot-time mount/lookups and cannot block on a real device.
pub fn block_on<F: Future>(future: F) -> F::Output {
    let mut cx = Context::from_waker(Waker::noop());
    let mut future = core::pin::pin!(future);
    loop {
        match future.as_mut().poll(&mut cx) {
            Poll::Ready(value) => return value,
            Poll::Pending => core::hint::spin_loop(),
        }
    }
}
