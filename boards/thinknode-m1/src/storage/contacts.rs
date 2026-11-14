//! Contact storage: a key-value map keyed by public key.
//!
//! Each contact lives in its own record, so adding or updating one contact
//! appends ~190 bytes instead of erasing 14 pages and rewriting the whole
//! 350-contact table. `sequential-storage` handles page rotation and
//! compaction; removals rewrite the item header.

use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::String;
use nrf_softdevice::Flash;
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};

use super::counted_flash::CountedFlash;
use super::util::{fixed_str, fnv_bytes, str_from_fixed, FNV_INIT};
use crate::companion::usb::DISCOVERED_CONTACTS;
use crate::meshcore::mesh::DiscoveredContact;

/// 0xB0000..0xCC000: 28 pages of 4 KB.
const REGION: Range<u32> = 0x000B_0000..0x000C_C000;
const CONTACT_RECORD_SIZE: usize = 164;
/// Size of records written before `sync_since` was added.
const LEGACY_CONTACT_RECORD_SIZE: usize = 156;
/// Key (32) + serialized contact, rounded up to a whole flash word.
const SCRATCH_SIZE: usize = 200;

/// Completed contact record writes and page erases since boot.
pub static SAVES: AtomicU32 = AtomicU32::new(0);
pub static ERASES: AtomicU32 = AtomicU32::new(0);

#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct PersistContact {
    pub_key: [u8; 32],
    node_id: [u8; 4],
    callsign: [u8; 32],
    out_path: [u8; 64],
    latitude: i32,
    longitude: i32,
    last_seen_epoch: u32,
    last_advert_ts: u32,
    rssi: i16,
    snr: i8,
    flags: u8,
    contact_type: u8,
    out_path_len: u8,
    sync_since: u32,
    _pad: [u8; 2],
}

const _: () = assert!(core::mem::size_of::<PersistContact>() == CONTACT_RECORD_SIZE);

#[repr(C, align(4))]
struct Scratch([u8; SCRATCH_SIZE]);

type ContactCache = Cache<Uncached, Uncached, Uncached, [u8; 32]>;
type ContactMap<'a> = MapStorage<[u8; 32], CountedFlash<'a>, ContactCache>;

fn open(flash: &mut Flash) -> ContactMap<'_> {
    MapStorage::new(
        CountedFlash::new(flash, &ERASES),
        MapConfig::new(REGION),
        Cache::new_uncached(),
    )
}

pub(crate) static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

pub fn request() {
    let _ = SAVE_CHANNEL.try_send(());
}

/// Out-of-band mutations that are not expressed by the RAM list.
pub enum ContactOp {
    Remove([u8; 32]),
    Clear,
}

pub static OP_CHANNEL: Channel<CriticalSectionRawMutex, ContactOp, 4> = Channel::new();

pub async fn queue_remove(pub_key: [u8; 32]) {
    let _ = OP_CHANNEL.send(ContactOp::Remove(pub_key)).await;
}

pub async fn queue_clear() {
    let _ = OP_CHANNEL.send(ContactOp::Clear).await;
}

fn contact_to_record(c: &DiscoveredContact) -> PersistContact {
    PersistContact {
        pub_key: c.pub_key,
        node_id: c.node_id,
        callsign: fixed_str(c.callsign.as_str()),
        out_path: c.out_path,
        latitude: c.latitude,
        longitude: c.longitude,
        last_seen_epoch: c.last_seen_epoch,
        last_advert_ts: c.last_advert_ts,
        rssi: c.rssi,
        snr: c.snr,
        flags: c.flags,
        contact_type: c.contact_type,
        out_path_len: c.out_path_len,
        sync_since: c.sync_since,
        _pad: [0; 2],
    }
}

fn record_to_contact(r: &PersistContact) -> DiscoveredContact {
    let mut callsign = String::<32>::new();
    let _ = callsign.push_str(str_from_fixed(&r.callsign));
    DiscoveredContact {
        pub_key: r.pub_key,
        callsign,
        node_id: r.node_id,
        last_seen_epoch: r.last_seen_epoch,
        rssi: r.rssi,
        snr: r.snr,
        latitude: r.latitude,
        longitude: r.longitude,
        flags: r.flags,
        contact_type: r.contact_type,
        out_path: r.out_path,
        out_path_len: r.out_path_len,
        last_advert_ts: r.last_advert_ts,
        saved_hash: 0,
        sync_since: r.sync_since,
    }
}

/// Hash of the fields that are worth persisting. Volatile fields (RSSI, SNR,
/// last-seen time) are excluded so a re-advert does not rewrite the record.
fn structural_hash_of(ct: &DiscoveredContact) -> u32 {
    let mut h = FNV_INIT;
    fnv_bytes(&mut h, &ct.pub_key);
    fnv_bytes(&mut h, &ct.node_id);
    fnv_bytes(&mut h, ct.callsign.as_bytes());
    fnv_bytes(&mut h, &ct.out_path);
    fnv_bytes(&mut h, &ct.latitude.to_le_bytes());
    fnv_bytes(&mut h, &ct.longitude.to_le_bytes());
    fnv_bytes(&mut h, &ct.sync_since.to_le_bytes());
    fnv_bytes(&mut h, &[ct.flags, ct.contact_type, ct.out_path_len]);
    h
}

/// Load the map into `DISCOVERED_CONTACTS`, keeping the newest record per key.
pub async fn load(flash: &mut Flash) {
    DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());

    let mut map = open(flash);

    // The previous firmware kept two whole-table snapshots in this region.
    // Their layout is not a valid map, so clear them once.
    const LEGACY_MAGIC: u32 = 0x434E_5443;
    let legacy = |addr: u32| unsafe {
        (addr as *const u32).read_unaligned() == LEGACY_MAGIC
    };
    if legacy(REGION.start) || legacy(REGION.start + 14 * 4096) {
        info!("Persistence: clearing legacy contact snapshot banks");
        if let Err(e) = map.erase_all().await {
            error!("Persistence: legacy contact erase failed: {:?}", e);
        }
    }

    let mut buf = Scratch([0u8; SCRATCH_SIZE]);
    let mut count = 0usize;
    match map.fetch_all_items(&mut buf.0).await {
        Ok(mut iter) => {
            while let Ok(Some((key, value))) = iter.next::<&[u8]>(&mut buf.0).await {
                if value.len() < LEGACY_CONTACT_RECORD_SIZE {
                    continue;
                }
                // Older records are shorter (no sync_since); copy only what is
                // present into a zeroed struct.
                let mut rec: PersistContact = unsafe { core::mem::zeroed() };
                let n = value.len().min(CONTACT_RECORD_SIZE);
                unsafe {
                    core::ptr::copy_nonoverlapping(
                        value.as_ptr(),
                        &mut rec as *mut PersistContact as *mut u8,
                        n,
                    );
                }
                let mut ct = record_to_contact(&rec);
                ct.saved_hash = structural_hash_of(&ct);
                DISCOVERED_CONTACTS.lock(|c| {
                    let mut list = c.borrow_mut();
                    if let Some(existing) = list.iter().position(|x| x.pub_key == key) {
                        list[existing] = ct;
                    } else if list.len() < list.capacity() {
                        let _ = list.push(ct);
                    }
                });
                count += 1;
            }
        }
        Err(e) => error!("Persistence: contact map load failed: {:?}", e),
    }
    info!("Persistence: loaded {} contact records", count);
}

/// Write contacts whose persisted fields changed since the last save.
pub async fn save(flash: &mut Flash) {
    let total = DISCOVERED_CONTACTS.lock(|c| c.borrow().len());
    let mut map = open(flash);
    let mut buf = Scratch([0u8; SCRATCH_SIZE]);
    let mut written = 0u32;

    for i in 0..total {
        let Some((ct, hash)) = DISCOVERED_CONTACTS.lock(|c| {
            c.borrow()
                .get(i)
                .map(|ct| (ct.clone(), structural_hash_of(ct)))
        }) else {
            continue;
        };
        if ct.saved_hash == hash {
            continue;
        }

        let rec = contact_to_record(&ct);
        let bytes = unsafe {
            core::slice::from_raw_parts(
                &rec as *const PersistContact as *const u8,
                CONTACT_RECORD_SIZE,
            )
        };
        let value: &[u8] = bytes;
        match map.store_item(&mut buf.0, &ct.pub_key, &value).await {
            Ok(()) => {
                written += 1;
                DISCOVERED_CONTACTS.lock(|c| {
                    if let Some(existing) = c
                        .borrow_mut()
                        .iter_mut()
                        .find(|x| x.pub_key == ct.pub_key)
                    {
                        existing.saved_hash = hash;
                    }
                });
            }
            Err(e) => {
                error!("Persistence: contact map store failed: {:?}", e);
                break;
            }
        }
    }

    if written > 0 {
        SAVES.fetch_add(written, Ordering::Relaxed);
    }
}

/// Remove one contact record (overwrite or explicit delete).
pub async fn remove(flash: &mut Flash, pub_key: [u8; 32]) {
    let mut map = open(flash);
    let mut buf = Scratch([0u8; SCRATCH_SIZE]);
    if let Err(e) = map.remove_item(&mut buf.0, &pub_key).await {
        error!("Persistence: contact map remove failed: {:?}", e);
    }
}

/// Erase every stored contact.
pub async fn clear(flash: &mut Flash) {
    let mut map = open(flash);
    if let Err(e) = map.erase_all().await {
        error!("Persistence: contact map clear failed: {:?}", e);
    }
}
