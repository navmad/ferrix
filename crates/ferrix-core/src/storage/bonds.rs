//! BLE bond storage.
//!
//! The SoftDevice leaves bond persistence to the application. This module keeps
//! a small RAM table for synchronous lookups from the BLE event context and
//! persists bonds as a `sequential-storage` map in a 16 KB internal journal.

use core::cell::RefCell;
use core::ops::Range;

#[cfg(not(test))]
use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};
use sequential_storage::Error;

use super::counted_flash::CountedFlash;
use super::util::{self, AlignedBuf, MappedAsyncFlash};
use super::MappedFlash;

pub const MAX_BONDS: usize = 4;
pub const MAX_SYS_ATTRS: usize = 64;

const MAGIC: u32 = 0x424F_4E44; // "BOND"
const VERSION: u16 = 1;
const RECORD_SIZE: usize = 128;
const RECORD_PAYLOAD: usize = RECORD_SIZE - 4;
const SCRATCH_SIZE: usize = 192;

/// One bonded peer.
#[derive(Clone, Copy, Debug)]
pub struct Bond {
    pub master_ediv: u16,
    pub master_rand: [u8; 8],
    pub ltk: [u8; 16],
    pub ltk_flags: u8,
    pub peer_addr_flags: u8,
    pub peer_addr: [u8; 6],
    pub peer_irk: [u8; 16],
    pub sys_len: u8,
    pub sys_attrs: [u8; MAX_SYS_ATTRS],
}

pub static BOND_ERASES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static BOND_SAVES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static BOND_SAVE_ERRORS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static BOND_LOADED: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
/// Last persistence error, encoded by [`error_code`] (0 = none).
pub static BOND_LAST_ERROR: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
/// Bond regions that were reformatted after a failed write.
pub static BOND_RECOVERIES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Set when the boot-time read found a region that cannot be mounted. The
/// storage task erases it before the first save instead of appending to the
/// foreign records.
static NEEDS_FORMAT: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

struct Store {
    bonds: [Option<Bond>; MAX_BONDS],
    dirty: [bool; MAX_BONDS],
    removed: [Option<[u8; 10]>; MAX_BONDS],
    seq: [u32; MAX_BONDS],
    sequence: u32,
}

static STORE: Mutex<CriticalSectionRawMutex, RefCell<Store>> = Mutex::new(RefCell::new(Store {
    bonds: [None; MAX_BONDS],
    dirty: [false; MAX_BONDS],
    removed: [None; MAX_BONDS],
    seq: [0; MAX_BONDS],
    sequence: 0,
}));

static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

type BondCache = Cache<Uncached, Uncached, Uncached, [u8; 10]>;
type BondMap<'a, F> = MapStorage<[u8; 10], CountedFlash<'a, F>, BondCache>;

fn open<'a, F>(flash: &'a mut F, region: &Range<u32>) -> BondMap<'a, F>
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    MapStorage::new(
        CountedFlash::new(flash, &BOND_ERASES),
        MapConfig::new(region.clone()),
        Cache::new_uncached(),
    )
}

fn bond_key(bond: &Bond) -> [u8; 10] {
    let mut key = [0u8; 10];
    key[..2].copy_from_slice(&bond.master_ediv.to_le_bytes());
    key[2..].copy_from_slice(&bond.master_rand);
    key
}

fn checksum(bytes: &[u8]) -> u32 {
    let mut h = 0x811C_9DC5u32;
    for &b in bytes {
        h ^= b as u32;
        h = h.wrapping_mul(0x0100_0193);
    }
    h
}

fn encode(bond: &Bond) -> [u8; RECORD_SIZE] {
    let mut buf = [0u8; RECORD_SIZE];
    buf[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    buf[4..6].copy_from_slice(&VERSION.to_le_bytes());
    // 6..8: _reserved = 0
    buf[8..10].copy_from_slice(&bond.master_ediv.to_le_bytes());
    buf[10..18].copy_from_slice(&bond.master_rand);
    buf[18..34].copy_from_slice(&bond.ltk);
    buf[34] = bond.ltk_flags;
    buf[35] = bond.peer_addr_flags;
    buf[36..42].copy_from_slice(&bond.peer_addr);
    buf[42..58].copy_from_slice(&bond.peer_irk);
    buf[58] = bond.sys_len.min(MAX_SYS_ATTRS as u8);
    buf[59..123].copy_from_slice(&bond.sys_attrs);
    // 123: _pad = 0
    let csum = checksum(&buf[..RECORD_PAYLOAD]);
    buf[124..128].copy_from_slice(&csum.to_le_bytes());
    buf
}

fn decode(bytes: &[u8]) -> Option<Bond> {
    if bytes.len() < RECORD_SIZE {
        return None;
    }
    let magic = u32::from_le_bytes(bytes[0..4].try_into().ok()?);
    let version = u16::from_le_bytes(bytes[4..6].try_into().ok()?);
    let sys_len = bytes[58];
    if magic != MAGIC || version != VERSION || sys_len as usize > MAX_SYS_ATTRS {
        return None;
    }
    let stored_checksum = u32::from_le_bytes(bytes[124..128].try_into().ok()?);
    if checksum(&bytes[..RECORD_PAYLOAD]) != stored_checksum {
        return None;
    }
    Some(Bond {
        master_ediv: u16::from_le_bytes(bytes[8..10].try_into().ok()?),
        master_rand: bytes[10..18].try_into().ok()?,
        ltk: bytes[18..34].try_into().ok()?,
        ltk_flags: bytes[34],
        peer_addr_flags: bytes[35],
        peer_addr: bytes[36..42].try_into().ok()?,
        peer_irk: bytes[42..58].try_into().ok()?,
        sys_len,
        sys_attrs: bytes[59..123].try_into().ok()?,
    })
}

/// Whether two records describe the same physical peer.
///
/// Re-pairing mints a new master ID, so the peer identity (IRK, or a stable
/// address when no IRK was distributed) must also be compared. Otherwise one
/// phone that pairs repeatedly would consume every bond slot.
fn same_peer(a: &Bond, b: &Bond) -> bool {
    if a.master_ediv == b.master_ediv && a.master_rand == b.master_rand {
        return true;
    }
    if a.peer_irk != [0u8; 16] && a.peer_irk == b.peer_irk {
        return true;
    }
    a.peer_addr != [0u8; 6]
        && a.peer_addr_flags == b.peer_addr_flags
        && a.peer_addr == b.peer_addr
}

fn same_address(a: &Bond, flags: u8, addr: [u8; 6]) -> bool {
    a.peer_addr_flags == flags && a.peer_addr == addr
}

/// Load the bond map into RAM. Called once at boot before BLE starts.
pub fn init(reader: &dyn MappedFlash, region: Range<u32>) {
    let mut flash = MappedAsyncFlash::new(reader, region.clone());
    let mut buf = AlignedBuf::<SCRATCH_SIZE>::new();

    let (bonds, count) = util::block_on(async {
        let mut map = open(&mut flash, &region);
        let mut bonds: [Option<Bond>; MAX_BONDS] = [None; MAX_BONDS];
        let mut count = 0usize;
        match map.fetch_all_items(&mut buf.0).await {
            Ok(mut iter) => {
                while let Ok(Some((_key, value))) = iter.next::<&[u8]>(&mut buf.0).await {
                    let Some(bond) = decode(value) else {
                        continue;
                    };
                    if let Some(existing) = bonds
                        .iter_mut()
                        .flatten()
                        .find(|b| same_peer(b, &bond))
                    {
                        *existing = bond;
                    } else if count < MAX_BONDS {
                        bonds[count] = Some(bond);
                        count += 1;
                    }
                }
            }
            Err(e) => {
                // Report why the map could not be read; 0x200 marks a load
                // failure, distinct from a write failure. The region holds
                // no readable bonds, so the storage task reformats it.
                NEEDS_FORMAT.store(true, core::sync::atomic::Ordering::Relaxed);
                BOND_LAST_ERROR.store(
                    0x200 + error_code(&e),
                    core::sync::atomic::Ordering::Relaxed,
                );
            }
        }
        (bonds, count)
    });

    STORE.lock(|s| {
        let mut store = s.borrow_mut();
        store.bonds = bonds;
        store.dirty = [false; MAX_BONDS];
        store.removed = [None; MAX_BONDS];
        let mut seq = [0u32; MAX_BONDS];
        let mut next = 0u32;
        for (i, bond) in store.bonds.iter().enumerate() {
            if bond.is_some() {
                next += 1;
                seq[i] = next;
            }
        }
        store.seq = seq;
        store.sequence = next;
    });
    BOND_LOADED.store(count as u32, core::sync::atomic::Ordering::Relaxed);
    #[cfg(not(test))]
    info!("BLE: loaded {} bond(s)", count);
}

fn request_save() {
    #[cfg(not(test))]
    let _ = SAVE_CHANNEL.try_send(());
}

/// Erase the bond region when the boot-time mount failed.
///
/// A blank region already mounts as an empty map, so a mount failure means
/// the region still holds data from an older layout (contacts, advert log).
/// Nothing in it is readable as a bond, so reclaiming it is lossless.
pub async fn format_if_needed<F>(flash: &mut F, region: &Range<u32>) -> bool
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    if !NEEDS_FORMAT.swap(false, core::sync::atomic::Ordering::Relaxed) {
        return true;
    }
    let mut map = open(flash, region);
    match map.erase_all().await {
        Ok(()) => {
            BOND_LAST_ERROR.store(0, core::sync::atomic::Ordering::Relaxed);
            BOND_RECOVERIES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            #[cfg(not(test))]
            info!("BLE: formatted bond region");
            true
        }
        Err(e) => {
            NEEDS_FORMAT.store(true, core::sync::atomic::Ordering::Relaxed);
            BOND_LAST_ERROR.store(
                0x100 + error_code(&e),
                core::sync::atomic::Ordering::Relaxed,
            );
            #[cfg(not(test))]
            error!("BLE: bond region erase failed: {:?}", e);
            false
        }
    }
}

/// Wait for a pending bond write.
pub async fn wait_save_request() {
    SAVE_CHANNEL.receive().await;
}

/// Number of stored bonds.
pub fn count() -> usize {
    STORE.lock(|s| s.borrow().bonds.iter().flatten().count())
}

/// Insert or update a bond. New bonds evict the oldest entry when full.
pub fn upsert(bond: Bond) {
    STORE.lock(|s| {
        let mut store = s.borrow_mut();
        let slot = if let Some(i) = store
            .bonds
            .iter()
            .position(|b| b.as_ref().is_some_and(|b| same_peer(b, &bond)))
        {
            i
        } else if let Some(i) = store.bonds.iter().position(Option::is_none) {
            i
        } else {
            // Evict the least recently used bond.
            let mut oldest = 0;
            let mut seq = u32::MAX;
            for (i, bond) in store.bonds.iter().enumerate() {
                if bond.is_some() && store.seq[i] < seq {
                    seq = store.seq[i];
                    oldest = i;
                }
            }
            oldest
        };

        let mut new_bond = bond;
        if let Some(existing) = store.bonds[slot] {
            if same_peer(&existing, &new_bond) && new_bond.sys_len == 0 {
                // Keep sys attrs across a re-bond unless new ones are present.
                new_bond.sys_len = existing.sys_len;
                new_bond.sys_attrs = existing.sys_attrs;
            }
            if bond_key(&existing) != bond_key(&new_bond) {
                // The slot may be reused by a new master ID, so drop the old
                // record from flash as well.
                store.removed[slot] = Some(bond_key(&existing));
            }
        }
        store.bonds[slot] = Some(new_bond);
        store.dirty[slot] = true;
        store.sequence = store.sequence.wrapping_add(1);
        store.seq[slot] = store.sequence;
    });
    request_save();
}

/// Find a bond by its master ID.
pub fn get_key(ediv: u16, rand: &[u8; 8]) -> Option<Bond> {
    STORE.lock(|s| {
        s.borrow()
            .bonds
            .iter()
            .flatten()
            .find(|b| b.master_ediv == ediv && b.master_rand == *rand)
            .copied()
    })
}

/// Look up a bond by peer address; if the address is not matched and there is
/// exactly one bond, use it (covers resolvable private addresses).
fn find_by_address(store: &Store, flags: u8, addr: [u8; 6]) -> Option<usize> {
    if let Some(i) = store
        .bonds
        .iter()
        .position(|b| b.as_ref().is_some_and(|b| same_address(b, flags, addr)))
    {
        return Some(i);
    }
    let mut only = None;
    let mut count = 0;
    for (i, b) in store.bonds.iter().enumerate() {
        if b.is_some() {
            count += 1;
            only = Some(i);
        }
    }
    if count == 1 { only } else { None }
}

/// Store GATT system attributes for a peer.
pub fn set_sys_attrs(flags: u8, addr: [u8; 6], attrs: &[u8]) {
    let changed = STORE.lock(|s| {
        let mut store = s.borrow_mut();
        let Some(i) = find_by_address(&store, flags, addr) else {
            return false;
        };
        let Some(bond) = store.bonds[i].as_mut() else {
            return false;
        };
        let len = attrs.len().min(MAX_SYS_ATTRS);
        if bond.sys_len as usize == len && bond.sys_attrs[..len] == attrs[..len] {
            return false;
        }
        bond.sys_len = len as u8;
        bond.sys_attrs[..len].copy_from_slice(&attrs[..len]);
        store.dirty[i] = true;
        true
    });
    if changed {
        request_save();
    }
}

/// Return stored GATT system attributes for a peer.
pub fn sys_attrs(flags: u8, addr: [u8; 6]) -> Option<([u8; MAX_SYS_ATTRS], usize)> {
    STORE.lock(|s| {
        let store = s.borrow();
        let i = find_by_address(&store, flags, addr)?;
        let bond = store.bonds[i]?;
        if bond.sys_len == 0 {
            None
        } else {
            Some((bond.sys_attrs, bond.sys_len as usize))
        }
    })
}

/// Forget all bonds. The next save removes every persisted record.
pub fn clear() {
    STORE.lock(|s| {
        let mut store = s.borrow_mut();
        for i in 0..MAX_BONDS {
            if let Some(bond) = store.bonds[i] {
                store.removed[i] = Some(bond_key(&bond));
            }
            store.bonds[i] = None;
            store.dirty[i] = false;
        }
        store.seq = [0; MAX_BONDS];
    });
    request_save();
}

/// Encode a `sequential-storage` error for the `stats-ble` diagnostic field.
fn error_code<S>(error: &Error<S>) -> u32 {
    match error {
        Error::Storage { .. } => 1,
        Error::FullStorage => 2,
        Error::Corrupted { .. } => 3,
        Error::LogicBug { .. } => 4,
        Error::BufferTooBig => 5,
        Error::BufferTooSmall(_) => 6,
        Error::SerializationError(_) => 7,
        Error::ItemTooBig => 8,
        _ => 9,
    }
}

/// Write pending removals and dirty bonds. `rewrite_all` is used after a
/// reformat, when the map is blank and every RAM bond must be restored.
async fn write_pending<F>(
    flash: &mut F,
    region: &Range<u32>,
    bonds: &[Option<Bond>; MAX_BONDS],
    dirty: &[bool; MAX_BONDS],
    removed: &[Option<[u8; 10]>; MAX_BONDS],
    rewrite_all: bool,
) -> Result<(), Error<F::Error>>
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut buf = AlignedBuf::<SCRATCH_SIZE>::new();
    let mut map = open(flash, region);

    if !rewrite_all {
        for key in removed.iter().flatten() {
            map.remove_item(&mut buf.0, key).await?;
        }
    }

    for (i, bond) in bonds.iter().enumerate() {
        if !rewrite_all && !dirty[i] {
            continue;
        }
        let Some(bond) = bond else {
            continue;
        };
        let rec = encode(bond);
        let bytes: &[u8] = &rec;
        let key = bond_key(bond);
        map.store_item(&mut buf.0, &key, &bytes).await?;
    }

    Ok(())
}

/// Persist dirty/removed bonds to the map.
pub async fn save<F>(flash: &mut F, region: &Range<u32>) -> bool
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let (bonds, dirty, removed) = STORE.lock(|s| {
        let store = s.borrow();
        (store.bonds, store.dirty, store.removed)
    });
    if !dirty.iter().any(|d| *d) && !removed.iter().any(Option::is_some) {
        return true;
    }

    // The bond region used to hold the internal contacts map and advert log.
    // Leftover records from that layout can look like a valid map, so a write
    // fails instead of repairing it. The bonds are already in RAM, so
    // reformatting the region and rewriting them is lossless. Reformat up
    // front when the boot-time mount already failed, otherwise retry once
    // with a clean map after the first write error.
    let rewrite_all = NEEDS_FORMAT.swap(false, core::sync::atomic::Ordering::Relaxed);
    if rewrite_all {
        let mut map = open(flash, region);
        match map.erase_all().await {
            Ok(()) => {
                BOND_RECOVERIES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            }
            Err(e) => {
                NEEDS_FORMAT.store(true, core::sync::atomic::Ordering::Relaxed);
                BOND_LAST_ERROR.store(
                    0x100 + error_code(&e),
                    core::sync::atomic::Ordering::Relaxed,
                );
                #[cfg(not(test))]
                error!("BLE: bond map erase failed: {:?}", e);
                BOND_SAVE_ERRORS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                return false;
            }
        }
    }

    for attempt in 0..2 {
        let all = rewrite_all || attempt > 0;
        match write_pending(flash, region, &bonds, &dirty, &removed, all).await {
            Ok(()) => {
                STORE.lock(|s| {
                    let mut store = s.borrow_mut();
                    store.dirty = [false; MAX_BONDS];
                    store.removed = [None; MAX_BONDS];
                });
                BOND_SAVES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                return true;
            }
            Err(e) => {
                BOND_LAST_ERROR.store(error_code(&e), core::sync::atomic::Ordering::Relaxed);
                #[cfg(not(test))]
                if attempt == 0 {
                    error!("BLE: bond write failed ({:?}), reformatting map", e);
                } else {
                    error!("BLE: bond write failed: {:?}", e);
                }
                if attempt == 0 && !rewrite_all {
                    let mut map = open(flash, region);
                    match map.erase_all().await {
                        Ok(()) => {
                            BOND_RECOVERIES
                                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
                        }
                        Err(e) => {
                            BOND_LAST_ERROR.store(
                                0x100 + error_code(&e),
                                core::sync::atomic::Ordering::Relaxed,
                            );
                            #[cfg(not(test))]
                            error!("BLE: bond map erase failed: {:?}", e);
                            break;
                        }
                    }
                }
            }
        }
    }

    BOND_SAVE_ERRORS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Bond {
        Bond {
            master_ediv: 0x1234,
            master_rand: [1, 2, 3, 4, 5, 6, 7, 8],
            ltk: [0xAA; 16],
            ltk_flags: 2,
            peer_addr_flags: 1,
            peer_addr: [9, 8, 7, 6, 5, 4],
            peer_irk: [0xBB; 16],
            sys_len: 0,
            sys_attrs: [0; MAX_SYS_ATTRS],
        }
    }

    #[test]
    fn bond_table_and_record_roundtrip() {
        clear();

        let bond = sample();
        upsert(bond);
        assert_eq!(count(), 1);
        let found = get_key(bond.master_ediv, &bond.master_rand).expect("bond");
        assert_eq!(found.ltk, bond.ltk);
        assert_eq!(found.peer_addr, bond.peer_addr);

        set_sys_attrs(bond.peer_addr_flags, bond.peer_addr, b"attrs");
        let (attrs, len) = sys_attrs(bond.peer_addr_flags, bond.peer_addr).expect("attrs");
        assert_eq!(&attrs[..len], b"attrs");

        let rec = encode(&bond);
        assert_eq!(rec.len(), RECORD_SIZE);
        assert_eq!(&rec[0..4], &MAGIC.to_le_bytes());
        assert_eq!(&rec[8..10], &bond.master_ediv.to_le_bytes());
        assert_eq!(rec[34], bond.ltk_flags);
        assert_eq!(rec[58], bond.sys_len);
        let decoded = decode(&rec).expect("decode");
        assert_eq!(decoded.master_ediv, bond.master_ediv);
        assert_eq!(decoded.master_rand, bond.master_rand);
        assert_eq!(decoded.ltk, bond.ltk);
        assert_eq!(decoded.peer_addr, bond.peer_addr);
        assert_eq!(decoded.peer_irk, bond.peer_irk);

        // Corrupted checksum is rejected.
        let mut bad = rec;
        bad[124] ^= 0xFF;
        assert!(decode(&bad).is_none());

        clear();
        assert_eq!(count(), 0);
    }

    #[test]
    fn rebond_replaces_the_previous_slot() {
        clear();

        let mut first = sample();
        first.master_ediv = 1;
        first.master_rand = [1; 8];
        first.peer_addr = [1; 6];
        first.peer_irk = [0xCC; 16];
        upsert(first);
        assert_eq!(count(), 1);

        // Re-pairing keeps the peer identity but mints a new master ID.
        let mut second = first;
        second.master_ediv = 2;
        second.master_rand = [2; 8];
        upsert(second);
        assert_eq!(count(), 1);
        assert!(get_key(1, &[1; 8]).is_none());
        assert!(get_key(2, &[2; 8]).is_some());

        // A different peer gets its own slot.
        let mut other = first;
        other.master_ediv = 3;
        other.master_rand = [3; 8];
        other.peer_addr = [2; 6];
        other.peer_irk = [0xDD; 16];
        upsert(other);
        assert_eq!(count(), 2);

        clear();
        assert_eq!(count(), 0);
    }
}
