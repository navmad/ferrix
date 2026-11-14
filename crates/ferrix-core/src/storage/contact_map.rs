//! Shared tagged-key map for contacts and advert blobs.
//!
//! Both record types live in the same 256 KB external-flash map:
//!
//! `[0x01, pubkey]` → contact metadata
//! `[0x02, pubkey]` → signed advert blob
//!
//! This gives keyed lookups and real deletion without a separate append log.

use core::ops::Range;
use core::sync::atomic::AtomicU32;

use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};

use super::counted_flash::CountedFlash;

pub const CONTACT_TAG: u8 = 0x01;
pub const BLOB_TAG: u8 = 0x02;
pub const KEY_LEN: usize = 33;
pub const SCRATCH_SIZE: usize = 256;

pub type Key = [u8; KEY_LEN];
pub type CacheType = Cache<Uncached, Uncached, Uncached, Key>;
pub type Map<'a, F> = MapStorage<Key, CountedFlash<'a, F>, CacheType>;

pub fn contact_key(pub_key: &[u8; 32]) -> Key {
    let mut key = [0u8; KEY_LEN];
    key[0] = CONTACT_TAG;
    key[1..].copy_from_slice(pub_key);
    key
}

pub fn blob_key(pub_key: &[u8; 32]) -> Key {
    let mut key = [0u8; KEY_LEN];
    key[0] = BLOB_TAG;
    key[1..].copy_from_slice(pub_key);
    key
}

pub fn key_tag(key: &Key) -> u8 {
    key[0]
}

pub fn key_pub_key(key: &Key) -> [u8; 32] {
    let mut pub_key = [0u8; 32];
    pub_key.copy_from_slice(&key[1..]);
    pub_key
}

pub fn open<'a, F>(
    flash: &'a mut F,
    region: &Range<u32>,
    erases: &'static AtomicU32,
) -> Map<'a, F>
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    MapStorage::new(
        CountedFlash::new(flash, erases),
        MapConfig::new(region.clone()),
        Cache::new_uncached(),
    )
}
