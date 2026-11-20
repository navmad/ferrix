//! Advert blobs in the contact map.
//!
//! The signed advert is a cache used only for Share Contact and Export
//! Contact. It lives under `[0x02, pubkey]` in the same keyed map as contact
//! metadata, so lookups are keyed and removal with the contact is real.
//!
//! Writes are conditional: a blob is queued only when a contact is added or
//! its callsign/type/location changes. Plain liveness adverts do not write.

use core::cell::RefCell;
use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
#[cfg(not(test))]
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use heapless::Vec;

#[cfg(not(test))]
use meshcore_wire::packets::AdvertPacket;

use super::contact_map::{self, SCRATCH_SIZE};
use super::util::AlignedBuf;

/// `pub(32) ‖ ts(4) ‖ sig(64) ‖ app_data(≤32)`.
pub const MAX_ADVERT_PAYLOAD: usize = 132;
const PENDING_CAPACITY: usize = 8;

const BLOB_RECORD_SIZE: usize = 1 + MAX_ADVERT_PAYLOAD + 3;

const _: () = assert!(BLOB_RECORD_SIZE == 136);
const _: () = assert!(BLOB_RECORD_SIZE <= SCRATCH_SIZE);

fn encode_blob(len: u8, payload: &[u8; MAX_ADVERT_PAYLOAD]) -> [u8; BLOB_RECORD_SIZE] {
    let mut buf = [0u8; BLOB_RECORD_SIZE];
    buf[0] = len;
    buf[1..1 + MAX_ADVERT_PAYLOAD].copy_from_slice(payload);
    // 133..136: _pad = 0
    buf
}

fn decode_blob(bytes: &[u8]) -> Option<(u8, [u8; MAX_ADVERT_PAYLOAD])> {
    if bytes.len() < BLOB_RECORD_SIZE {
        return None;
    }
    let len = bytes[0];
    if len as usize > MAX_ADVERT_PAYLOAD {
        return None;
    }
    let payload: [u8; MAX_ADVERT_PAYLOAD] = bytes[1..1 + MAX_ADVERT_PAYLOAD].try_into().ok()?;
    Some((len, payload))
}

#[derive(Clone, Copy)]
struct PendingBlob {
    pub_key: [u8; 32],
    len: u8,
    payload: [u8; MAX_ADVERT_PAYLOAD],
}

static PENDING: Mutex<CriticalSectionRawMutex, RefCell<Vec<PendingBlob, PENDING_CAPACITY>>> =
    Mutex::new(RefCell::new(Vec::new()));

static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();
pub static RECORD_SAVES: AtomicU32 = AtomicU32::new(0);
pub static ERASES: AtomicU32 = AtomicU32::new(0);

/// A blob fetch from a protocol handler.
pub struct FetchRequest {
    pub key: [u8; 32],
}

static FETCH_CHANNEL: Channel<CriticalSectionRawMutex, FetchRequest, 1> = Channel::new();
static FETCH_RESPONSE: Signal<CriticalSectionRawMutex, Option<PendingBlob>> = Signal::new();
/// Serializes USB and BLE advert fetches and prevents a stale response from
/// a cancelled caller being delivered to the next one.
#[cfg(not(test))]
static FETCH_LOCK: AsyncMutex<CriticalSectionRawMutex, ()> = AsyncMutex::new(());

fn request() {
    #[cfg(not(test))]
    let _ = SAVE_CHANNEL.try_send(());
}

/// Wait for a coalesced advert-log save request.
pub async fn wait_save_request() {
    SAVE_CHANNEL.receive().await;
}

/// Wait for a blob fetch request from a protocol handler.
pub async fn wait_fetch_request() -> FetchRequest {
    FETCH_CHANNEL.receive().await
}

/// Drop queued save requests after a coalescing delay.
pub fn drain_save_requests() {
    while SAVE_CHANNEL.try_receive().is_ok() {}
}

/// Keep the signed advert payload for `pub_key`.
pub fn remember(pub_key: &[u8; 32], payload: &[u8]) {
    if payload.len() > MAX_ADVERT_PAYLOAD {
        return;
    }
    let mut rec = PendingBlob {
        pub_key: *pub_key,
        len: payload.len() as u8,
        payload: [0u8; MAX_ADVERT_PAYLOAD],
    };
    rec.payload[..payload.len()].copy_from_slice(payload);

    PENDING.lock(|p| {
        let mut list = p.borrow_mut();
        if let Some(entry) = list.iter_mut().find(|e| e.pub_key == *pub_key) {
            *entry = rec;
        } else if list.is_full() {
            list.remove(0);
            let _ = list.push(rec);
        } else {
            let _ = list.push(rec);
        }
    });
    request();
}

/// Drop a queued (not yet written) blob.
pub fn forget(pub_key: &[u8; 32]) {
    PENDING.lock(|p| {
        let mut list = p.borrow_mut();
        if let Some(pos) = list.iter().position(|e| e.pub_key == *pub_key) {
            list.remove(pos);
        }
    });
}

/// True when a blob for this contact is queued for writing.
pub fn has_blob(pub_key: &[u8; 32]) -> bool {
    PENDING.lock(|p| p.borrow().iter().any(|e| e.pub_key == *pub_key))
}

/// Fetch and verify the latest signed advert payload for a contact.
pub async fn request_payload(pub_key: &[u8; 32]) -> Option<Vec<u8, MAX_ADVERT_PAYLOAD>> {
    #[cfg(test)]
    {
        let _ = pub_key;
        None
    }
    #[cfg(not(test))]
    {
        let _guard = FETCH_LOCK.lock().await;
        FETCH_RESPONSE.reset();
        FETCH_CHANNEL
            .send(FetchRequest { key: *pub_key })
            .await;
        let rec = FETCH_RESPONSE.wait().await?;
        let len = rec.len as usize;
        let adv = AdvertPacket::decode(&rec.payload[..len]).ok()?;
        if !adv.verify_signature() {
            return None;
        }
        let mut out = Vec::<u8, MAX_ADVERT_PAYLOAD>::new();
        out.extend_from_slice(&rec.payload[..len]).ok()?;
        Some(out)
    }
}

/// Persist queued blobs into the contact map.
pub async fn save<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut pending = Vec::<PendingBlob, PENDING_CAPACITY>::new();
    PENDING.lock(|p| {
        let mut list = p.borrow_mut();
        for rec in list.iter() {
            let _ = pending.push(*rec);
        }
        list.clear();
    });
    if pending.is_empty() {
        return;
    }

    let mut buf = AlignedBuf::<SCRATCH_SIZE>::new();
    let mut map = contact_map::open(flash, region, &ERASES);

    for rec in pending.iter() {
        // Keep blobs only for contacts in the discovered list.
        let known = crate::state::DISCOVERED_CONTACTS
            .lock(|c| c.borrow().iter().any(|ct| ct.pub_key == rec.pub_key));
        if !known {
            continue;
        }

        let record = encode_blob(rec.len, &rec.payload);
        let bytes: &[u8] = &record;
        let key = contact_map::blob_key(&rec.pub_key);
        if let Err(e) = map.store_item(&mut buf.0, &key, &bytes).await {
            error!("Persistence: blob map store failed: {:?}", e);
            return;
        }
        RECORD_SAVES.fetch_add(1, Ordering::Relaxed);
    }
    info!("Persistence: advert blobs saved");
}

/// Serve one blob fetch request from the storage task.
pub async fn serve_fetch<F>(flash: &mut F, region: &Range<u32>, req: FetchRequest)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut buf = AlignedBuf::<SCRATCH_SIZE>::new();
    let mut map = contact_map::open(flash, region, &ERASES);
    let key = contact_map::blob_key(&req.key);
    let rec = match map.fetch_item::<&[u8]>(&mut buf.0, &key).await {
        Ok(Some(value)) => decode_blob(value).map(|(len, payload)| PendingBlob {
            pub_key: req.key,
            len,
            payload,
        }),
        _ => None,
    };
    FETCH_RESPONSE.signal(rec);
}

/// Remove a persisted blob (also done by `contacts::remove`).
pub async fn remove<F>(flash: &mut F, region: &Range<u32>, pub_key: &[u8; 32])
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut buf = AlignedBuf::<SCRATCH_SIZE>::new();
    let mut map = contact_map::open(flash, region, &ERASES);
    let key = contact_map::blob_key(pub_key);
    if let Err(e) = map.remove_item(&mut buf.0, &key).await {
        error!("Persistence: blob map remove failed: {:?}", e);
    }
}

/// `(saved blobs, pending blobs, erase pending)` for diagnostics.
pub fn stats() -> (usize, usize, bool) {
    (
        RECORD_SAVES.load(Ordering::Relaxed) as usize,
        PENDING.lock(|p| p.borrow().len()),
        false,
    )
}

/// Legacy slot count. The keyed map has no fixed slot cap.
pub fn slots() -> usize {
    0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pending_blob_is_remembered() {
        forget(&[7u8; 32]);
        remember(&[7u8; 32], &[1, 2, 3]);
        assert!(has_blob(&[7u8; 32]));
        assert!(!has_blob(&[8u8; 32]));
        forget(&[7u8; 32]);
    }

    #[test]
    fn blob_record_roundtrip() {
        let mut payload = [0u8; MAX_ADVERT_PAYLOAD];
        payload[0..4].copy_from_slice(&[0xDE, 0xAD, 0xBE, 0xEF]);
        let bytes = encode_blob(100, &payload);
        assert_eq!(bytes.len(), 136);
        assert_eq!(bytes[0], 100);
        assert_eq!(&bytes[1..5], &[0xDE, 0xAD, 0xBE, 0xEF]);
        let (len, decoded) = decode_blob(&bytes).expect("decode blob");
        assert_eq!(len, 100);
        assert_eq!(decoded, payload);

        let mut bad = bytes;
        bad[0] = (MAX_ADVERT_PAYLOAD + 1) as u8;
        assert!(decode_blob(&bad).is_none());
    }
}
