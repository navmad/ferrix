//! Advert blob log: the original signed advert payload per contact.
//!
//! MeshCore's `EXPORT_CONTACT` returns the raw advert packet that was received
//! for a contact, so sharing preserves the original signature. The contact
//! record itself cannot rebuild that signature, so blobs live in an
//! append-only bank on the external QSPI flash.
//!
//! Blob reads are asynchronous: the storage task owns the QSPI driver, so
//! protocol handlers request a blob and wait for the reply.

use core::cell::RefCell;
use core::ops::Range;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embedded_storage_async::nor_flash::NorFlash;
use heapless::Vec;

#[cfg(not(test))]
use meshcore_wire::packets::AdvertPacket;

/// `pub(32) ‖ ts(4) ‖ sig(64) ‖ app_data(≤32)`.
pub const MAX_ADVERT_PAYLOAD: usize = 132;
const RECORD_SIZE: usize = 172; // magic 4 + key 32 + len 1 + payload 132 + pad 3
const RECORD_MAGIC: u32 = 0x424C_4F42; // "BLOB"
const PENDING_CAPACITY: usize = 8;

#[repr(C)]
#[derive(Clone, Copy)]
struct PersistBlob {
    magic: u32,
    pub_key: [u8; 32],
    len: u8,
    payload: [u8; MAX_ADVERT_PAYLOAD],
    _pad: [u8; 3],
}

const _: () = assert!(core::mem::size_of::<PersistBlob>() == RECORD_SIZE);

static PENDING: Mutex<CriticalSectionRawMutex, RefCell<Vec<PersistBlob, PENDING_CAPACITY>>> =
    Mutex::new(RefCell::new(Vec::new()));

static CURSOR: AtomicUsize = AtomicUsize::new(0);
static NEED_ERASE: AtomicBool = AtomicBool::new(false);
static READY: AtomicBool = AtomicBool::new(false);

static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();
pub static RECORD_SAVES: AtomicU32 = AtomicU32::new(0);
pub static ERASES: AtomicU32 = AtomicU32::new(0);

/// A blob fetch from a protocol handler.
pub struct FetchRequest {
    pub key: [u8; 32],
}

static FETCH_CHANNEL: Channel<CriticalSectionRawMutex, FetchRequest, 1> = Channel::new();
static FETCH_RESPONSE: Signal<CriticalSectionRawMutex, Option<PersistBlob>> = Signal::new();

struct AdvertStore {
    bank: u32,
    size: u32,
    slots: usize,
}

static STORE: Mutex<CriticalSectionRawMutex, RefCell<Option<AdvertStore>>> =
    Mutex::new(RefCell::new(None));

fn store_shape() -> (u32, u32, usize) {
    STORE.lock(|s| {
        let st = s.borrow();
        st.as_ref()
            .map(|st| (st.bank, st.size, st.slots))
            .unwrap_or((0, 0, 0))
    })
}

async fn read_slot<F: NorFlash>(flash: &mut F, index: usize) -> Option<PersistBlob> {
    let (bank, _, slots) = store_shape();
    if index >= slots {
        return None;
    }
    let addr = bank + index as u32 * RECORD_SIZE as u32;
    let mut buf = [0u8; RECORD_SIZE];
    flash.read(addr, &mut buf).await.ok()?;
    Some(unsafe { core::ptr::read_unaligned(buf.as_ptr() as *const PersistBlob) })
}

fn slot_valid(rec: &PersistBlob) -> bool {
    rec.magic == RECORD_MAGIC
        && rec.len != 0
        && rec.len as usize <= MAX_ADVERT_PAYLOAD
        && rec.pub_key[0] != 0xFF
}

/// Locate the append cursor. Called once after the QSPI flash is ready.
pub async fn init<F: NorFlash>(flash: &mut F, region: Range<u32>) {
    let size = region.end - region.start;
    let slots = size as usize / RECORD_SIZE;
    STORE.lock(|s| {
        *s.borrow_mut() = Some(AdvertStore {
            bank: region.start,
            size,
            slots,
        });
    });

    let first = read_slot(flash, 0).await.unwrap_or(PersistBlob {
        magic: 0,
        pub_key: [0xFF; 32],
        len: 0,
        payload: [0; MAX_ADVERT_PAYLOAD],
        _pad: [0; 3],
    });
    let (cursor, need_erase) = if slot_valid(&first) {
        let mut i = 1usize;
        while i < slots {
            match read_slot(flash, i).await {
                Some(rec) if slot_valid(&rec) => i += 1,
                _ => break,
            }
        }
        (i, false)
    } else if first.pub_key[0] == 0xFF {
        (0, false)
    } else {
        // Foreign data (old layout or corrupt head); reclaim on first save.
        (0, true)
    };
    CURSOR.store(cursor, Ordering::Relaxed);
    NEED_ERASE.store(need_erase, Ordering::Relaxed);
    READY.store(true, Ordering::Relaxed);
    #[cfg(not(test))]
    info!(
        "Persistence: advert log at slot {}{}",
        cursor,
        if need_erase { " (erase pending)" } else { "" }
    );
}

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

fn key_matches(rec: &PersistBlob, pub_key: &[u8; 32]) -> bool {
    slot_valid(rec) && rec.pub_key == *pub_key
}

/// Keep the signed advert payload for `pub_key` so it can be exported later.
pub fn remember(pub_key: &[u8; 32], payload: &[u8]) {
    if payload.len() > MAX_ADVERT_PAYLOAD {
        return;
    }
    let mut rec = PersistBlob {
        magic: RECORD_MAGIC,
        pub_key: *pub_key,
        len: payload.len() as u8,
        payload: [0u8; MAX_ADVERT_PAYLOAD],
        _pad: [0; 3],
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

/// Drop a queued (not yet written) blob. Written blobs are left in place:
/// export goes through the contact list first, so they are unreachable.
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

async fn find_blob<F: NorFlash>(flash: &mut F, pub_key: &[u8; 32]) -> Option<PersistBlob> {
    // A blob queued moments ago is already valid; do not make the app wait
    // for the debounced flash write.
    if let Some(rec) = PENDING.lock(|p| {
        p.borrow()
            .iter()
            .rev()
            .find(|e| e.pub_key == *pub_key && e.len != 0)
            .copied()
    }) {
        return Some(rec);
    }

    let cursor = CURSOR.load(Ordering::Relaxed);
    let mut found = None;
    for i in 0..cursor {
        if let Some(rec) = read_slot(flash, i).await {
            if key_matches(&rec, pub_key) {
                found = Some(rec);
            }
        }
    }
    found
}

/// Fetch and verify the latest signed advert payload for a contact.
///
/// Runs in protocol task context; the storage task performs the QSPI read.
pub async fn request_payload(pub_key: &[u8; 32]) -> Option<Vec<u8, MAX_ADVERT_PAYLOAD>> {
    #[cfg(test)]
    {
        let _ = pub_key;
        None
    }
    #[cfg(not(test))]
    {
        let _ = FETCH_CHANNEL.send(FetchRequest { key: *pub_key }).await;
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

/// Serve one fetch request from the storage task.
pub async fn serve_fetch<F: NorFlash>(flash: &mut F, req: FetchRequest) {
    let rec = find_blob(flash, &req.key).await;
    FETCH_RESPONSE.signal(rec);
}

/// Append queued blobs to the log.
pub async fn save<F>(flash: &mut F)
where
    F: NorFlash,
{
    let (bank, size, slots) = store_shape();
    if !READY.load(Ordering::Relaxed) {
        return;
    }
    if NEED_ERASE.swap(false, Ordering::Relaxed) {
        if flash.erase(bank, bank + size).await.is_err() {
            error!("Persistence: advert log erase failed");
            NEED_ERASE.store(true, Ordering::Relaxed);
            return;
        }
        ERASES.fetch_add(1, Ordering::Relaxed);
        CURSOR.store(0, Ordering::Relaxed);
    }

    let mut pending = Vec::<PersistBlob, PENDING_CAPACITY>::new();
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

    let mut cursor = CURSOR.load(Ordering::Relaxed);
    for rec in pending.iter() {
        // Keep blobs only for contacts that are still in the saved/discovered
        // list. This drops orphans from contacts removed before the debounced
        // write, and from adverts that were never accepted.
        let known = crate::state::DISCOVERED_CONTACTS
            .lock(|c| c.borrow().iter().any(|ct| ct.pub_key == rec.pub_key));
        if !known {
            continue;
        }
        if cursor >= slots {
            error!("Persistence: advert log full");
            break;
        }
        let addr = bank + cursor as u32 * RECORD_SIZE as u32;
        let bytes = unsafe {
            core::slice::from_raw_parts(rec as *const PersistBlob as *const u8, RECORD_SIZE)
        };
        if flash.write(addr, bytes).await.is_err() {
            error!("Persistence: advert log write failed");
            break;
        }
        RECORD_SAVES.fetch_add(1, Ordering::Relaxed);
        cursor += 1;
    }
    CURSOR.store(cursor, Ordering::Relaxed);
    info!("Persistence: advert log at slot {}", cursor);
}

/// Number of records the configured log can hold.
pub fn slots() -> usize {
    store_shape().2
}

/// `(cursor, pending, erase_pending)` for diagnostics.
pub fn stats() -> (usize, usize, bool) {
    (
        CURSOR.load(Ordering::Relaxed),
        PENDING.lock(|p| p.borrow().len()),
        NEED_ERASE.load(Ordering::Relaxed),
    )
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
}
