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

use super::counted_flash::CountedFlash;
use super::util::{self, MappedAsyncFlash};
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

#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistBond {
    magic: u32,
    version: u16,
    _reserved: u16,
    master_ediv: u16,
    master_rand: [u8; 8],
    ltk: [u8; 16],
    ltk_flags: u8,
    peer_addr_flags: u8,
    peer_addr: [u8; 6],
    peer_irk: [u8; 16],
    sys_len: u8,
    sys_attrs: [u8; MAX_SYS_ATTRS],
    _pad: [u8; 1],
    checksum: u32,
}

const _: () = assert!(core::mem::size_of::<PersistBond>() == RECORD_SIZE);

pub static BOND_ERASES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static BOND_SAVES: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static BOND_SAVE_ERRORS: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);
pub static BOND_LOADED: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

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

fn encode(bond: &Bond) -> PersistBond {
    let mut rec = PersistBond {
        magic: MAGIC,
        version: VERSION,
        _reserved: 0,
        master_ediv: bond.master_ediv,
        master_rand: bond.master_rand,
        ltk: bond.ltk,
        ltk_flags: bond.ltk_flags,
        peer_addr_flags: bond.peer_addr_flags,
        peer_addr: bond.peer_addr,
        peer_irk: bond.peer_irk,
        sys_len: bond.sys_len.min(MAX_SYS_ATTRS as u8),
        sys_attrs: bond.sys_attrs,
        _pad: [0],
        checksum: 0,
    };
    let bytes = unsafe {
        core::slice::from_raw_parts(&rec as *const PersistBond as *const u8, RECORD_PAYLOAD)
    };
    rec.checksum = checksum(bytes);
    rec
}

fn decode(bytes: &[u8]) -> Option<Bond> {
    if bytes.len() < RECORD_SIZE {
        return None;
    }
    let rec = unsafe { core::ptr::read_unaligned(bytes.as_ptr() as *const PersistBond) };
    if rec.magic != MAGIC || rec.version != VERSION || rec.sys_len as usize > MAX_SYS_ATTRS {
        return None;
    }
    let payload = unsafe {
        core::slice::from_raw_parts(&rec as *const PersistBond as *const u8, RECORD_PAYLOAD)
    };
    if checksum(payload) != rec.checksum {
        return None;
    }
    Some(Bond {
        master_ediv: rec.master_ediv,
        master_rand: rec.master_rand,
        ltk: rec.ltk,
        ltk_flags: rec.ltk_flags,
        peer_addr_flags: rec.peer_addr_flags,
        peer_addr: rec.peer_addr,
        peer_irk: rec.peer_irk,
        sys_len: rec.sys_len,
        sys_attrs: rec.sys_attrs,
    })
}

fn same_peer(a: &Bond, b: &Bond) -> bool {
    a.master_ediv == b.master_ediv && a.master_rand == b.master_rand
}

fn same_address(a: &Bond, flags: u8, addr: [u8; 6]) -> bool {
    a.peer_addr_flags == flags && a.peer_addr == addr
}

/// Load the bond map into RAM. Called once at boot before BLE starts.
pub fn init(reader: &'static dyn MappedFlash, region: Range<u32>) {
    let mut flash = MappedAsyncFlash::new(reader, region.clone());
    let mut buf = [0u8; SCRATCH_SIZE];

    let (bonds, count) = util::block_on(async {
        let mut map = open(&mut flash, &region);
        let mut bonds: [Option<Bond>; MAX_BONDS] = [None; MAX_BONDS];
        let mut count = 0usize;
        match map.fetch_all_items(&mut buf).await {
            Ok(mut iter) => {
                while let Ok(Some((_key, value))) = iter.next::<&[u8]>(&mut buf).await {
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
            Err(_) => {}
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
            if same_peer(&existing, &new_bond) {
                // Keep sys attrs across a re-bond unless new ones are present.
                if new_bond.sys_len == 0 {
                    new_bond.sys_len = existing.sys_len;
                    new_bond.sys_attrs = existing.sys_attrs;
                }
            } else {
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

    let mut buf = [0u8; SCRATCH_SIZE];
    let mut map = open(flash, region);

    for key in removed.iter().flatten() {
        if let Err(_e) = map.remove_item(&mut buf, key).await {
            BOND_SAVE_ERRORS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            #[cfg(not(test))]
            error!("BLE: bond remove failed: {:?}", _e);
            return false;
        }
    }

    for (i, bond) in bonds.iter().enumerate() {
        if !dirty[i] {
            continue;
        }
        let Some(bond) = bond else {
            continue;
        };
        let rec = encode(bond);
        let bytes = unsafe {
            core::slice::from_raw_parts(&rec as *const PersistBond as *const u8, RECORD_SIZE)
        };
        let key = bond_key(bond);
        if let Err(_e) = map.store_item(&mut buf, &key, &bytes).await {
            BOND_SAVE_ERRORS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            #[cfg(not(test))]
            error!("BLE: bond write failed: {:?}", _e);
            return false;
        }
    }

    STORE.lock(|s| {
        let mut store = s.borrow_mut();
        store.dirty = [false; MAX_BONDS];
        store.removed = [None; MAX_BONDS];
    });
    BOND_SAVES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    true
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
        let bytes = unsafe {
            core::slice::from_raw_parts(&rec as *const PersistBond as *const u8, RECORD_SIZE)
        };
        let decoded = decode(bytes).expect("decode");
        assert_eq!(decoded.master_ediv, bond.master_ediv);
        assert_eq!(decoded.master_rand, bond.master_rand);
        assert_eq!(decoded.ltk, bond.ltk);

        clear();
        assert_eq!(count(), 0);
    }
}
