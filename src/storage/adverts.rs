//! Advert blob log: the original signed advert payload per contact.
//!
//! MeshCore's `EXPORT_CONTACT` returns the raw advert packet that was received
//! for a contact, so sharing preserves the original signature. The contact
//! record itself cannot rebuild that signature, so blobs live in a single
//! append-only flash bank above the application image. No double-banking:
//! blobs are re-learned from the next advert, so a torn append is harmless.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use heapless::Vec;
use nrf_softdevice::Flash;

use crate::meshcore::packets::AdvertPacket;

/// `pub(32) ‖ ts(4) ‖ sig(64) ‖ app_data(≤32)`.
pub const MAX_ADVERT_PAYLOAD: usize = 132;
const BANK: u32 = 0x000A_0000;
const BANK_SIZE: u32 = 64 * 1024;
const RECORD_SIZE: usize = 172; // magic 4 + key 32 + len 1 + payload 132 + pad 3
/// Number of records the log can hold.
pub const SLOTS: usize = BANK_SIZE as usize / RECORD_SIZE;
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

pub(crate) static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

fn read_slot(index: usize) -> PersistBlob {
    let addr = BANK + index as u32 * RECORD_SIZE as u32;
    // SAFETY: address is inside the reserved, memory-mapped flash region.
    unsafe { core::ptr::read_unaligned(addr as *const PersistBlob) }
}

fn slot_valid(rec: &PersistBlob) -> bool {
    rec.magic == RECORD_MAGIC
        && rec.len != 0
        && rec.len as usize <= MAX_ADVERT_PAYLOAD
        && rec.pub_key[0] != 0xFF
}

/// Locate the append cursor. Called once at boot before the flash task starts.
pub fn init() {
    let first = read_slot(0);
    let (cursor, need_erase) = if slot_valid(&first) {
        let mut i = 1usize;
        while i < SLOTS && slot_valid(&read_slot(i)) {
            i += 1;
        }
        (i, false)
    } else if first.pub_key[0] == 0xFF {
        // Freshly erased bank.
        (0, false)
    } else {
        // Foreign data (old layout or corrupt head); reclaim on first save.
        (0, true)
    };
    CURSOR.store(cursor, Ordering::Relaxed);
    NEED_ERASE.store(need_erase, Ordering::Relaxed);
    READY.store(true, Ordering::Relaxed);
    info!(
        "Persistence: advert log at slot {}{}",
        cursor,
        if need_erase { " (erase pending)" } else { "" }
    );
}

fn request() {
    let _ = SAVE_CHANNEL.try_send(());
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

/// True when a blob for this contact is queued or already written.
pub fn has_blob(pub_key: &[u8; 32]) -> bool {
    PENDING.lock(|p| p.borrow().iter().any(|e| e.pub_key == *pub_key))
        || (0..CURSOR.load(Ordering::Relaxed)).any(|i| key_matches(&read_slot(i), pub_key))
}

fn find_blob(pub_key: &[u8; 32]) -> Option<PersistBlob> {
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
        let rec = read_slot(i);
        if key_matches(&rec, pub_key) {
            found = Some(rec);
        }
    }
    found
}

/// Verified advert payload for a contact, or `None` if no valid blob exists.
pub fn payload(pub_key: &[u8; 32]) -> Option<Vec<u8, MAX_ADVERT_PAYLOAD>> {
    let rec = find_blob(pub_key)?;
    let len = rec.len as usize;
    let adv = AdvertPacket::decode(&rec.payload[..len]).ok()?;
    if !adv.verify_signature() {
        return None;
    }
    let mut out = Vec::<u8, MAX_ADVERT_PAYLOAD>::new();
    out.extend_from_slice(&rec.payload[..len]).ok()?;
    Some(out)
}

/// Build a complete advert packet (`header ‖ path_len ‖ payload`) for export.
pub fn export(pub_key: &[u8; 32]) -> Option<Vec<u8, { MAX_ADVERT_PAYLOAD + 2 }>> {
    let payload = payload(pub_key)?;
    let mut out = Vec::<u8, { MAX_ADVERT_PAYLOAD + 2 }>::new();
    // Flood-routed advert with no path, as MeshCore exports it.
    out.push(0x11).ok()?;
    out.push(0x00).ok()?;
    out.extend_from_slice(&payload).ok()?;
    Some(out)
}

/// Append queued blobs to the log.
pub async fn save(flash: &mut Flash) {
    use embedded_storage_async::nor_flash::NorFlash;

    if !READY.load(Ordering::Relaxed) {
        return;
    }
    if NEED_ERASE.swap(false, Ordering::Relaxed) {
        if flash.erase(BANK, BANK + BANK_SIZE).await.is_err() {
            error!("Persistence: advert log erase failed");
            NEED_ERASE.store(true, Ordering::Relaxed);
            return;
        }
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
        if cursor >= SLOTS {
            error!("Persistence: advert log full");
            break;
        }
        let addr = BANK + cursor as u32 * RECORD_SIZE as u32;
        let bytes = unsafe {
            core::slice::from_raw_parts(rec as *const PersistBlob as *const u8, RECORD_SIZE)
        };
        if flash.write(addr, bytes).await.is_err() {
            error!("Persistence: advert log write failed");
            break;
        }
        cursor += 1;
    }
    CURSOR.store(cursor, Ordering::Relaxed);
    info!("Persistence: advert log at slot {}", cursor);
}

/// `(cursor, pending, erase_pending)` for diagnostics.
pub fn stats() -> (usize, usize, bool) {
    (
        CURSOR.load(Ordering::Relaxed),
        PENDING.lock(|p| p.borrow().len()),
        NEED_ERASE.load(Ordering::Relaxed),
    )
}
