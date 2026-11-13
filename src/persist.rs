//! Persistent contact and message snapshots on the nRF52840 internal flash.
//!
//! Each store has two snapshot banks. A save erases the inactive bank, writes
//! a header followed by the records, and only then makes it active. If power
//! fails mid-write, the previous bank is still valid. On boot the newest valid
//! snapshot (highest sequence number + matching checksum) is loaded.
//!
//! Contacts are only rewritten when their *structural* fields change
//! (key/name/type/flags/path). Adverts that only update RSSI or timestamps do
//! not wear the flash.
//!
//! Flash layout (the application image is linked to end before 0xB0000):
//!   0xB0000..0xBE000  contacts bank A (14 pages)
//!   0xBE000..0xCC000  contacts bank B (14 pages)
//!   0xCC000..0xCD000  messages bank A (1 page)
//!   0xCD000..0xCE000  messages bank B (1 page)
//!   0xEC000..0xED000  existing persistent config page

use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embedded_storage_async::nor_flash::NorFlash;
use heapless::String;
use nrf_softdevice::Flash;

use crate::companion::usb::{DISCOVERED_CONTACTS, MESSAGE_LOG, MAX_CONTACTS, MAX_MESSAGES};
use crate::meshcore::mesh::{DiscoveredContact, ReceivedMessage};

pub const CONTACTS_BANK_A: u32 = 0x000B_0000;
pub const CONTACTS_BANK_B: u32 = 0x000B_E000;
pub const CONTACTS_BANK_SIZE: u32 = 14 * 4096;

pub const MESSAGES_BANK_A: u32 = 0x000C_C000;
pub const MESSAGES_BANK_B: u32 = 0x000C_D000;
pub const MESSAGES_BANK_SIZE: u32 = 4096;

const CONTACTS_MAGIC: u32 = 0x434E_5443;
const MESSAGES_MAGIC: u32 = 0x4D53_474D;
const SNAPSHOT_VERSION: u32 = 1;

const HEADER_SIZE: usize = 32;
const CONTACT_RECORD_SIZE: usize = 156;
const MESSAGE_RECORD_SIZE: usize = 236;

const FNV_INIT: u32 = 0x811C_9DC5;
const FNV_PRIME: u32 = 0x0100_0193;

#[repr(C)]
#[derive(Clone, Copy)]
struct SnapshotHeader {
    magic: u32,
    version: u32,
    seq: u32,
    count: u32,
    checksum: u32,
    _reserved: [u32; 3],
}

#[repr(C)]
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
    _pad: [u8; 2],
}

#[repr(C)]
#[derive(Clone, Copy)]
struct PersistMessage {
    channel_idx: u8,
    path_len: u8,
    is_direct: u8,
    _pad: u8,
    sender: [u8; 32],
    sender_pub: [u8; 32],
    text: [u8; 160],
    timestamp: u32,
    rssi: i16,
    snr: i8,
    _pad2: u8,
}

const _: () = {
    assert!(core::mem::size_of::<SnapshotHeader>() == HEADER_SIZE);
    assert!(core::mem::size_of::<PersistContact>() == CONTACT_RECORD_SIZE);
    assert!(core::mem::size_of::<PersistMessage>() == MESSAGE_RECORD_SIZE);
    assert!(HEADER_SIZE + MAX_CONTACTS * CONTACT_RECORD_SIZE <= CONTACTS_BANK_SIZE as usize);
    assert!(HEADER_SIZE + MAX_MESSAGES * MESSAGE_RECORD_SIZE <= MESSAGES_BANK_SIZE as usize);
};

static CONTACTS_BANK: AtomicU8 = AtomicU8::new(1);
static CONTACTS_SEQ: AtomicU32 = AtomicU32::new(0);
static CONTACTS_HASH: AtomicU32 = AtomicU32::new(0);

static MESSAGES_BANK: AtomicU8 = AtomicU8::new(1);
static MESSAGES_SEQ: AtomicU32 = AtomicU32::new(0);

pub static CONTACTS_SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();
pub static MESSAGES_SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

/// Ask the storage task to snapshot the contact list.
pub fn request_contacts() {
    let _ = CONTACTS_SAVE_CHANNEL.try_send(());
}

/// Ask the storage task to snapshot the message log.
pub fn request_messages() {
    let _ = MESSAGES_SAVE_CHANNEL.try_send(());
}

fn fnv_bytes(h: &mut u32, bytes: &[u8]) {
    for &b in bytes {
        *h ^= b as u32;
        *h = h.wrapping_mul(FNV_PRIME);
    }
}

fn fnv_struct<T>(h: &mut u32, value: &T) {
    let bytes = unsafe {
        core::slice::from_raw_parts(value as *const T as *const u8, core::mem::size_of::<T>())
    };
    fnv_bytes(h, bytes);
}

fn fixed_str<const N: usize>(s: &str) -> [u8; N] {
    let mut out = [0u8; N];
    let bytes = s.as_bytes();
    let n = bytes.len().min(N);
    out[..n].copy_from_slice(&bytes[..n]);
    out
}

fn str_from_fixed(bytes: &[u8]) -> &str {
    let n = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    core::str::from_utf8(&bytes[..n]).unwrap_or("")
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
    }
}

fn message_to_record(m: &ReceivedMessage) -> PersistMessage {
    PersistMessage {
        channel_idx: m.channel_idx,
        path_len: m.path_len,
        is_direct: if m.is_direct { 1 } else { 0 },
        _pad: 0,
        sender: fixed_str(m.sender.as_str()),
        sender_pub: m.sender_pub,
        text: fixed_str(m.text.as_str()),
        timestamp: m.timestamp,
        rssi: m.rssi,
        snr: m.snr,
        _pad2: 0,
    }
}

fn record_to_message(r: &PersistMessage) -> ReceivedMessage {
    let mut sender = String::<32>::new();
    let _ = sender.push_str(str_from_fixed(&r.sender));
    let mut text = String::<160>::new();
    let _ = text.push_str(str_from_fixed(&r.text));
    ReceivedMessage {
        channel_idx: r.channel_idx,
        path_len: r.path_len,
        sender,
        sender_pub: r.sender_pub,
        text,
        timestamp: r.timestamp,
        rssi: r.rssi,
        snr: r.snr,
        is_direct: r.is_direct != 0,
    }
}

fn read_contact(base: u32, index: usize) -> PersistContact {
    let addr = base + HEADER_SIZE as u32 + index as u32 * CONTACT_RECORD_SIZE as u32;
    // SAFETY: the address is inside the reserved, memory-mapped flash region.
    unsafe { core::ptr::read_unaligned(addr as *const PersistContact) }
}

fn read_message(base: u32, index: usize) -> PersistMessage {
    let addr = base + HEADER_SIZE as u32 + index as u32 * MESSAGE_RECORD_SIZE as u32;
    // SAFETY: see `read_contact`.
    unsafe { core::ptr::read_unaligned(addr as *const PersistMessage) }
}

fn read_header(base: u32) -> Option<SnapshotHeader> {
    // SAFETY: see `read_contact`.
    Some(unsafe { core::ptr::read_unaligned(base as *const SnapshotHeader) })
}

fn contacts_checksum(base: u32, count: usize) -> u32 {
    let mut h = FNV_INIT;
    for i in 0..count.min(MAX_CONTACTS) {
        fnv_struct(&mut h, &read_contact(base, i));
    }
    h
}

fn messages_checksum(base: u32, count: usize) -> u32 {
    let mut h = FNV_INIT;
    for i in 0..count.min(MAX_MESSAGES) {
        fnv_struct(&mut h, &read_message(base, i));
    }
    h
}

fn valid_contacts_header(base: u32) -> Option<SnapshotHeader> {
    let h = read_header(base)?;
    if h.magic != CONTACTS_MAGIC
        || h.version != SNAPSHOT_VERSION
        || h.count as usize > MAX_CONTACTS
    {
        return None;
    }
    if contacts_checksum(base, h.count as usize) != h.checksum {
        return None;
    }
    Some(h)
}

fn valid_messages_header(base: u32) -> Option<SnapshotHeader> {
    let h = read_header(base)?;
    if h.magic != MESSAGES_MAGIC
        || h.version != SNAPSHOT_VERSION
        || h.count as usize > MAX_MESSAGES
    {
        return None;
    }
    if messages_checksum(base, h.count as usize) != h.checksum {
        return None;
    }
    Some(h)
}

/// Structural checksum of the contact list (volatile fields excluded).
fn structural_hash() -> u32 {
    let mut h = FNV_INIT;
    DISCOVERED_CONTACTS.lock(|c| {
        let list = c.borrow();
        fnv_bytes(&mut h, &(list.len() as u32).to_le_bytes());
        for ct in list.iter() {
            fnv_bytes(&mut h, &ct.pub_key);
            fnv_bytes(&mut h, &ct.node_id);
            fnv_bytes(&mut h, ct.callsign.as_bytes());
            fnv_bytes(&mut h, &ct.out_path);
            fnv_bytes(&mut h, &[ct.flags, ct.contact_type, ct.out_path_len]);
        }
    });
    h
}

/// Load both snapshots. Called once from `main` before the tasks start.
pub fn load() {
    load_contacts();
    load_messages();
}

fn load_contacts() {
    let a = valid_contacts_header(CONTACTS_BANK_A).map(|h| (CONTACTS_BANK_A, 0u8, h));
    let b = valid_contacts_header(CONTACTS_BANK_B).map(|h| (CONTACTS_BANK_B, 1u8, h));
    let chosen = match (a, b) {
        (Some(x), Some(y)) => Some(if x.2.seq >= y.2.seq { x } else { y }),
        (Some(x), None) => Some(x),
        (None, Some(y)) => Some(y),
        (None, None) => None,
    };

    if let Some((base, bank, h)) = chosen {
        DISCOVERED_CONTACTS.lock(|c| {
            let mut list = c.borrow_mut();
            list.clear();
            for i in 0..h.count as usize {
                let _ = list.push(record_to_contact(&read_contact(base, i)));
            }
        });
        CONTACTS_BANK.store(bank, Ordering::Relaxed);
        CONTACTS_SEQ.store(h.seq, Ordering::Relaxed);
        CONTACTS_HASH.store(structural_hash(), Ordering::Relaxed);
        info!("Persistence: loaded {} contacts from bank {}", h.count, bank);
    } else {
        CONTACTS_BANK.store(1, Ordering::Relaxed);
        CONTACTS_SEQ.store(0, Ordering::Relaxed);
        CONTACTS_HASH.store(structural_hash(), Ordering::Relaxed);
        info!("Persistence: no contact snapshot");
    }
}

fn load_messages() {
    let a = valid_messages_header(MESSAGES_BANK_A).map(|h| (MESSAGES_BANK_A, 0u8, h));
    let b = valid_messages_header(MESSAGES_BANK_B).map(|h| (MESSAGES_BANK_B, 1u8, h));
    let chosen = match (a, b) {
        (Some(x), Some(y)) => Some(if x.2.seq >= y.2.seq { x } else { y }),
        (Some(x), None) => Some(x),
        (None, Some(y)) => Some(y),
        (None, None) => None,
    };

    if let Some((base, bank, h)) = chosen {
        MESSAGE_LOG.lock(|m| {
            let mut list = m.borrow_mut();
            list.clear();
            for i in 0..h.count as usize {
                let _ = list.push(record_to_message(&read_message(base, i)));
            }
        });
        MESSAGES_BANK.store(bank, Ordering::Relaxed);
        MESSAGES_SEQ.store(h.seq, Ordering::Relaxed);
        info!("Persistence: loaded {} messages from bank {}", h.count, bank);
    } else {
        MESSAGES_BANK.store(1, Ordering::Relaxed);
        MESSAGES_SEQ.store(0, Ordering::Relaxed);
        info!("Persistence: no message snapshot");
    }
}

fn header_bytes(h: &SnapshotHeader) -> &[u8] {
    // SAFETY: the header is a plain `repr(C)` struct; the slice is read-only.
    unsafe { core::slice::from_raw_parts(h as *const SnapshotHeader as *const u8, HEADER_SIZE) }
}

async fn write_contacts_snapshot(flash: &mut Flash) -> Result<(), ()> {
    let count = DISCOVERED_CONTACTS.lock(|c| c.borrow().len());
    let hash = structural_hash();
    if hash == CONTACTS_HASH.load(Ordering::Relaxed) {
        return Ok(());
    }

    let active = CONTACTS_BANK.load(Ordering::Relaxed);
    let target = 1 - active;
    let base = if target == 0 {
        CONTACTS_BANK_A
    } else {
        CONTACTS_BANK_B
    };
    let seq = CONTACTS_SEQ.load(Ordering::Relaxed).wrapping_add(1);

    let mut checksum = FNV_INIT;
    for i in 0..count {
        if let Some(c) = DISCOVERED_CONTACTS.lock(|c| c.borrow().get(i).cloned()) {
            fnv_struct(&mut checksum, &contact_to_record(&c));
        }
    }

    let header = SnapshotHeader {
        magic: CONTACTS_MAGIC,
        version: SNAPSHOT_VERSION,
        seq,
        count: count as u32,
        checksum,
        _reserved: [0; 3],
    };

    flash
        .erase(base, base + CONTACTS_BANK_SIZE)
        .await
        .map_err(|e| {
            error!("Persistence: contacts erase failed: {:?}", e);
        })?;
    flash.write(base, header_bytes(&header)).await.map_err(|e| {
        error!("Persistence: contacts header write failed: {:?}", e);
    })?;

    for i in 0..count {
        let Some(c) = DISCOVERED_CONTACTS.lock(|c| c.borrow().get(i).cloned()) else {
            continue;
        };
        let rec = contact_to_record(&c);
        let bytes = unsafe {
            core::slice::from_raw_parts(
                &rec as *const PersistContact as *const u8,
                CONTACT_RECORD_SIZE,
            )
        };
        let addr = base + HEADER_SIZE as u32 + i as u32 * CONTACT_RECORD_SIZE as u32;
        flash.write(addr, bytes).await.map_err(|e| {
            error!("Persistence: contact write failed: {:?}", e);
        })?;
    }

    CONTACTS_BANK.store(target, Ordering::Relaxed);
    CONTACTS_SEQ.store(seq, Ordering::Relaxed);
    CONTACTS_HASH.store(hash, Ordering::Relaxed);
    info!(
        "Persistence: saved {} contacts to bank {} (seq {})",
        count, target, seq
    );
    Ok(())
}

async fn write_messages_snapshot(flash: &mut Flash) -> Result<(), ()> {
    let count = MESSAGE_LOG.lock(|m| m.borrow().len());
    let active = MESSAGES_BANK.load(Ordering::Relaxed);
    let target = 1 - active;
    let base = if target == 0 {
        MESSAGES_BANK_A
    } else {
        MESSAGES_BANK_B
    };
    let seq = MESSAGES_SEQ.load(Ordering::Relaxed).wrapping_add(1);

    let mut checksum = FNV_INIT;
    for i in 0..count {
        if let Some(m) = MESSAGE_LOG.lock(|m| m.borrow().get(i).cloned()) {
            fnv_struct(&mut checksum, &message_to_record(&m));
        }
    }

    let header = SnapshotHeader {
        magic: MESSAGES_MAGIC,
        version: SNAPSHOT_VERSION,
        seq,
        count: count as u32,
        checksum,
        _reserved: [0; 3],
    };

    flash
        .erase(base, base + MESSAGES_BANK_SIZE)
        .await
        .map_err(|e| {
            error!("Persistence: messages erase failed: {:?}", e);
        })?;
    flash.write(base, header_bytes(&header)).await.map_err(|e| {
        error!("Persistence: messages header write failed: {:?}", e);
    })?;

    for i in 0..count {
        let Some(m) = MESSAGE_LOG.lock(|m| m.borrow().get(i).cloned()) else {
            continue;
        };
        let rec = message_to_record(&m);
        let bytes = unsafe {
            core::slice::from_raw_parts(
                &rec as *const PersistMessage as *const u8,
                MESSAGE_RECORD_SIZE,
            )
        };
        let addr = base + HEADER_SIZE as u32 + i as u32 * MESSAGE_RECORD_SIZE as u32;
        flash.write(addr, bytes).await.map_err(|e| {
            error!("Persistence: message write failed: {:?}", e);
        })?;
    }

    MESSAGES_BANK.store(target, Ordering::Relaxed);
    MESSAGES_SEQ.store(seq, Ordering::Relaxed);
    info!(
        "Persistence: saved {} messages to bank {} (seq {})",
        count, target, seq
    );
    Ok(())
}

/// Called by the storage task.
pub async fn save_contacts(flash: &mut Flash) {
    let _ = write_contacts_snapshot(flash).await;
}

/// Called by the storage task.
pub async fn save_messages(flash: &mut Flash) {
    let _ = write_messages_snapshot(flash).await;
}
