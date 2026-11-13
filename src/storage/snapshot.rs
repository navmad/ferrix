//! Generic two-bank snapshot store for internal flash.
//!
//! A save erases the inactive bank, writes its records, and writes the header
//! **last**, so a power loss mid-write leaves the previous bank valid. On boot
//! the newest valid snapshot (sequence + checksum) wins.

use defmt::{error, info};
use embedded_storage_async::nor_flash::NorFlash;
use nrf_softdevice::Flash;

pub const HEADER_SIZE: usize = 32;
pub const SNAPSHOT_VERSION: u32 = 1;
pub const FNV_INIT: u32 = 0x811C_9DC5;
const FNV_PRIME: u32 = 0x0100_0193;

/// A fixed-size record that can be stored in a snapshot bank.
pub trait SnapshotRecord: Copy {
    const SIZE: usize;
    const MAGIC: u32;
    const MAX_COUNT: usize;
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct SnapshotHeader {
    pub magic: u32,
    pub version: u32,
    pub seq: u32,
    pub count: u32,
    pub checksum: u32,
    pub _reserved: [u32; 3],
}

const _: () = assert!(core::mem::size_of::<SnapshotHeader>() == HEADER_SIZE);

pub fn fnv_bytes(h: &mut u32, bytes: &[u8]) {
    for &b in bytes {
        *h ^= b as u32;
        *h = h.wrapping_mul(FNV_PRIME);
    }
}

pub fn fnv_struct<T>(h: &mut u32, value: &T) {
    let bytes = unsafe {
        core::slice::from_raw_parts(value as *const T as *const u8, core::mem::size_of::<T>())
    };
    fnv_bytes(h, bytes);
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

pub struct SnapshotStore {
    bank_a: u32,
    bank_b: u32,
    bank_size: u32,
}

impl SnapshotStore {
    pub const fn new(bank_a: u32, bank_b: u32, bank_size: u32) -> Self {
        Self {
            bank_a,
            bank_b,
            bank_size,
        }
    }

    const fn base(&self, bank: u8) -> u32 {
        if bank == 0 {
            self.bank_a
        } else {
            self.bank_b
        }
    }

    fn read_record<T: SnapshotRecord>(&self, bank: u8, index: usize) -> T {
        let addr = self.base(bank) + HEADER_SIZE as u32 + index as u32 * T::SIZE as u32;
        // SAFETY: the address is inside the reserved, memory-mapped flash region.
        unsafe { core::ptr::read_unaligned(addr as *const T) }
    }

    fn checksum<T: SnapshotRecord>(&self, bank: u8, count: usize) -> u32 {
        let mut h = FNV_INIT;
        for i in 0..count.min(T::MAX_COUNT) {
            fnv_struct(&mut h, &self.read_record::<T>(bank, i));
        }
        h
    }

    fn header<T: SnapshotRecord>(&self, bank: u8) -> Option<SnapshotHeader> {
        // SAFETY: see `read_record`.
        let h = unsafe { core::ptr::read_unaligned(self.base(bank) as *const SnapshotHeader) };
        if h.magic != T::MAGIC
            || h.version != SNAPSHOT_VERSION
            || h.count as usize > T::MAX_COUNT
        {
            return None;
        }
        if self.checksum::<T>(bank, h.count as usize) != h.checksum {
            return None;
        }
        Some(h)
    }

    /// Emit the newest valid snapshot. Returns `(bank, count)` when found.
    pub fn load<T: SnapshotRecord>(&self, mut emit: impl FnMut(T)) -> Option<(u8, u32)> {
        let a = self.header::<T>(0).map(|h| (0u8, h));
        let b = self.header::<T>(1).map(|h| (1u8, h));
        let (bank, h) = match (a, b) {
            (Some(x), Some(y)) => {
                if x.1.seq >= y.1.seq {
                    x
                } else {
                    y
                }
            }
            (Some(x), None) => x,
            (None, Some(y)) => y,
            (None, None) => return None,
        };
        for i in 0..h.count as usize {
            emit(self.read_record::<T>(bank, i));
        }
        Some((bank, h.count))
    }

    /// Save a snapshot. `record(i)` must return `Some` for each of `count`
    /// entries. The header is written last.
    pub async fn save<T: SnapshotRecord>(
        &self,
        flash: &mut Flash,
        count: usize,
        mut record: impl FnMut(usize) -> Option<T>,
    ) -> Result<usize, ()> {
        let a = self.header::<T>(0);
        let b = self.header::<T>(1);
        let (target, seq) = match (a, b) {
            (Some(ha), Some(hb)) => {
                if ha.seq >= hb.seq {
                    (1u8, ha.seq.wrapping_add(1))
                } else {
                    (0u8, hb.seq.wrapping_add(1))
                }
            }
            (Some(ha), None) => (1, ha.seq.wrapping_add(1)),
            (None, Some(hb)) => (0, hb.seq.wrapping_add(1)),
            (None, None) => (0, 1),
        };
        let base = self.base(target);

        flash
            .erase(base, base + self.bank_size)
            .await
            .map_err(|e| {
                error!("Snapshot: erase failed: {:?}", e);
            })?;

        let mut checksum = FNV_INIT;
        let mut written = 0usize;
        for i in 0..count {
            let Some(value) = record(i) else {
                break;
            };
            fnv_struct(&mut checksum, &value);
            let bytes = unsafe {
                core::slice::from_raw_parts(&value as *const T as *const u8, T::SIZE)
            };
            let addr = base + HEADER_SIZE as u32 + i as u32 * T::SIZE as u32;
            flash.write(addr, bytes).await.map_err(|e| {
                error!("Snapshot: record write failed: {:?}", e);
            })?;
            written += 1;
        }

        let header = SnapshotHeader {
            magic: T::MAGIC,
            version: SNAPSHOT_VERSION,
            seq,
            count: written as u32,
            checksum,
            _reserved: [0; 3],
        };
        let bytes = unsafe {
            core::slice::from_raw_parts(
                &header as *const SnapshotHeader as *const u8,
                HEADER_SIZE,
            )
        };
        flash.write(base, bytes).await.map_err(|e| {
            error!("Snapshot: header write failed: {:?}", e);
        })?;

        info!(
            "Snapshot: saved {} records to bank {} (seq {})",
            written, target, seq
        );
        Ok(written)
    }
}
