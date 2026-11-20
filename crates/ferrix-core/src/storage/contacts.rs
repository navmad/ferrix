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
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};

use super::contact_map::{self, SCRATCH_SIZE};
use super::util::{fixed_str, fnv_bytes, str_from_fixed, FNV_INIT};
use crate::state::DISCOVERED_CONTACTS;
use crate::types::DiscoveredContact;

const CONTACT_RECORD_SIZE: usize = 164;
/// Size of records written before `sync_since` was added.
const LEGACY_CONTACT_RECORD_SIZE: usize = 156;

/// Completed contact record writes and page erases since boot.
pub static SAVES: AtomicU32 = AtomicU32::new(0);
pub static ERASES: AtomicU32 = AtomicU32::new(0);

const _: () = assert!(CONTACT_RECORD_SIZE == 164);
const _: () = assert!(LEGACY_CONTACT_RECORD_SIZE == 156);
const _: () = assert!(CONTACT_RECORD_SIZE <= SCRATCH_SIZE);

#[repr(C, align(4))]
struct Scratch([u8; SCRATCH_SIZE]);

fn open<'a, F>(flash: &'a mut F, region: &Range<u32>) -> contact_map::Map<'a, F>
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    contact_map::open(flash, region, &ERASES)
}

static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

pub fn request() {
    let _ = SAVE_CHANNEL.try_send(());
}

/// Wait for a coalesced contact-list save request.
pub async fn wait_save_request() {
    SAVE_CHANNEL.receive().await;
}

/// Drop queued save requests after a coalescing delay.
pub fn drain_save_requests() {
    while SAVE_CHANNEL.try_receive().is_ok() {}
}

/// Wait for the next out-of-band contact mutation.
pub async fn wait_operation() -> ContactOp {
    OP_CHANNEL.receive().await
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

/// Non-blocking remove for the radio RX loop so advert eviction never stalls
/// packet handling.
pub fn try_queue_remove(pub_key: [u8; 32]) -> bool {
    OP_CHANNEL.try_send(ContactOp::Remove(pub_key)).is_ok()
}

pub async fn queue_clear() {
    let _ = OP_CHANNEL.send(ContactOp::Clear).await;
}

fn encode_contact(c: &DiscoveredContact) -> [u8; CONTACT_RECORD_SIZE] {
    let mut buf = [0u8; CONTACT_RECORD_SIZE];
    buf[0..32].copy_from_slice(&c.pub_key);
    buf[32..36].copy_from_slice(&c.node_id);
    buf[36..68].copy_from_slice(&fixed_str::<32>(c.callsign.as_str()));
    buf[68..132].copy_from_slice(&c.out_path);
    buf[132..136].copy_from_slice(&c.latitude.to_le_bytes());
    buf[136..140].copy_from_slice(&c.longitude.to_le_bytes());
    buf[140..144].copy_from_slice(&c.last_seen_epoch.to_le_bytes());
    buf[144..148].copy_from_slice(&c.last_advert_ts.to_le_bytes());
    buf[148..150].copy_from_slice(&c.rssi.to_le_bytes());
    buf[150] = c.snr as u8;
    buf[151] = c.flags;
    buf[152] = c.contact_type;
    buf[153] = c.out_path_len;
    // 154..156: _pad_before_sync = 0
    buf[156..160].copy_from_slice(&c.sync_since.to_le_bytes());
    // 160..164: _pad_tail = 0
    buf
}

fn decode_contact(bytes: &[u8]) -> Option<DiscoveredContact> {
    if bytes.len() < LEGACY_CONTACT_RECORD_SIZE {
        return None;
    }
    // Older records are shorter (no sync_since); copy only what is present
    // into a zeroed buffer.
    let mut raw = [0u8; CONTACT_RECORD_SIZE];
    let n = bytes.len().min(CONTACT_RECORD_SIZE);
    raw[..n].copy_from_slice(&bytes[..n]);

    let mut callsign = String::<32>::new();
    let _ = callsign.push_str(str_from_fixed(&raw[36..68]));
    Some(DiscoveredContact {
        pub_key: raw[0..32].try_into().ok()?,
        callsign,
        node_id: raw[32..36].try_into().ok()?,
        last_seen_epoch: u32::from_le_bytes(raw[140..144].try_into().ok()?),
        rssi: i16::from_le_bytes(raw[148..150].try_into().ok()?),
        snr: raw[150] as i8,
        latitude: i32::from_le_bytes(raw[132..136].try_into().ok()?),
        longitude: i32::from_le_bytes(raw[136..140].try_into().ok()?),
        flags: raw[151],
        contact_type: raw[152],
        out_path: raw[68..132].try_into().ok()?,
        out_path_len: raw[153],
        last_advert_ts: u32::from_le_bytes(raw[144..148].try_into().ok()?),
        saved_hash: 0,
        sync_since: u32::from_le_bytes(raw[156..160].try_into().ok()?),
    })
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
async fn has_magic<F>(flash: &mut F, addr: u32, magic: u32) -> bool
where
    F: NorFlash,
    F::Error: defmt::Format,
{
    let mut buf = super::util::AlignedBuf::<4>::new();
    flash.read(addr, &mut buf.0).await.is_ok() && u32::from_le_bytes(buf.0) == magic
}

pub async fn load<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + embedded_storage_async::nor_flash::MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());

    // The previous firmware kept two whole-table snapshots in this region.
    // Their layout is not a valid map, so clear them once.
    const LEGACY_MAGIC: u32 = 0x434E_5443;
    let legacy = has_magic(flash, region.start, LEGACY_MAGIC).await
        || has_magic(flash, region.start + 14 * 4096, LEGACY_MAGIC).await;

    let mut map = open(flash, region);
    if legacy {
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
                if contact_map::key_tag(&key) != contact_map::CONTACT_TAG {
                    continue;
                }
                let Some(mut ct) = decode_contact(value) else {
                    continue;
                };
                let pub_key = contact_map::key_pub_key(&key);
                ct.saved_hash = structural_hash_of(&ct);
                DISCOVERED_CONTACTS.lock(|c| {
                    let mut list = c.borrow_mut();
                    if let Some(existing) = list.iter().position(|x| x.pub_key == pub_key) {
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
pub async fn save<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + embedded_storage_async::nor_flash::MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let total = DISCOVERED_CONTACTS.lock(|c| c.borrow().len());
    let mut map = open(flash, region);
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

        let rec = encode_contact(&ct);
        let value: &[u8] = &rec;
        match map
            .store_item(&mut buf.0, &contact_map::contact_key(&ct.pub_key), &value)
            .await
        {
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
pub async fn remove<F>(flash: &mut F, region: &Range<u32>, pub_key: [u8; 32])
where
    F: NorFlash + embedded_storage_async::nor_flash::MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut map = open(flash, region);
    let mut buf = Scratch([0u8; SCRATCH_SIZE]);
    let contact_key = contact_map::contact_key(&pub_key);
    if let Err(e) = map.remove_item(&mut buf.0, &contact_key).await {
        error!("Persistence: contact map remove failed: {:?}", e);
    }
    // The advert blob shares this map, so remove it with the contact.
    let blob_key = contact_map::blob_key(&pub_key);
    if let Err(e) = map.remove_item(&mut buf.0, &blob_key).await {
        error!("Persistence: blob map remove failed: {:?}", e);
    }
}

/// Erase every stored contact.
pub async fn clear<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + embedded_storage_async::nor_flash::MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut map = open(flash, region);
    if let Err(e) = map.erase_all().await {
        error!("Persistence: contact map clear failed: {:?}", e);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contact_record_roundtrip_and_legacy_layout() {
        let mut callsign = String::<32>::new();
        let _ = callsign.push_str("ch1nn4n");
        let contact = DiscoveredContact {
            pub_key: [0x12; 32],
            callsign,
            node_id: [1, 2, 3, 4],
            last_seen_epoch: 1_700_000_000,
            rssi: -95,
            snr: -12,
            latitude: 37_774_900,
            longitude: -122_419_400,
            flags: 0x05,
            contact_type: 1,
            out_path: [0x33; 64],
            out_path_len: 3,
            last_advert_ts: 1_700_000_123,
            saved_hash: 0,
            sync_since: 42,
        };

        let bytes = encode_contact(&contact);
        assert_eq!(bytes.len(), CONTACT_RECORD_SIZE);
        assert_eq!(&bytes[132..136], &37_774_900i32.to_le_bytes());
        assert_eq!(&bytes[148..150], &(-95i16).to_le_bytes());
        assert_eq!(bytes[150], (-12i8) as u8);
        assert_eq!(&bytes[156..160], &42u32.to_le_bytes());

        let decoded = decode_contact(&bytes).expect("decode current");
        assert_eq!(decoded.pub_key, contact.pub_key);
        assert_eq!(decoded.callsign, contact.callsign);
        assert_eq!(decoded.latitude, contact.latitude);
        assert_eq!(decoded.longitude, contact.longitude);
        assert_eq!(decoded.rssi, -95);
        assert_eq!(decoded.snr, -12);
        assert_eq!(decoded.sync_since, 42);

        // Legacy 156-byte record without sync_since decodes with sync_since == 0.
        let legacy = decode_contact(&bytes[..LEGACY_CONTACT_RECORD_SIZE]).expect("decode legacy");
        assert_eq!(legacy.callsign, contact.callsign);
        assert_eq!(legacy.rssi, -95);
        assert_eq!(legacy.sync_since, 0);
    }
}
