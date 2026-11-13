//! Contact snapshot: fixed-size records, structural-change filtering.

use core::sync::atomic::{AtomicU32, Ordering};

use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::String;
use nrf_softdevice::Flash;

use super::snapshot::{
    fixed_str, fnv_bytes, str_from_fixed, SnapshotRecord, SnapshotStore, FNV_INIT,
};
use crate::companion::usb::{DISCOVERED_CONTACTS, MAX_CONTACTS};
use crate::meshcore::mesh::DiscoveredContact;

const CONTACT_RECORD_SIZE: usize = 156;
const STORE: SnapshotStore =
    SnapshotStore::new(0x000B_0000, 0x000B_E000, 14 * 4096);

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

impl SnapshotRecord for PersistContact {
    const SIZE: usize = CONTACT_RECORD_SIZE;
    const MAGIC: u32 = 0x434E_5443;
    const MAX_COUNT: usize = MAX_CONTACTS;
}

const _: () = assert!(core::mem::size_of::<PersistContact>() == CONTACT_RECORD_SIZE);

/// Skip snapshots when only volatile fields (RSSI/timestamps) changed.
static HASH: AtomicU32 = AtomicU32::new(0);

pub(crate) static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

pub fn request() {
    let _ = SAVE_CHANNEL.try_send(());
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

/// Load the newest contact snapshot into `DISCOVERED_CONTACTS`.
pub fn load() {
    let mut cleared = false;
    let found = STORE.load::<PersistContact>(|rec| {
        let ct = record_to_contact(&rec);
        DISCOVERED_CONTACTS.lock(|c| {
            let mut list = c.borrow_mut();
            if !cleared {
                list.clear();
                cleared = true;
            }
            let _ = list.push(ct);
        });
    });
    if !cleared {
        DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());
    }
    HASH.store(structural_hash(), Ordering::Relaxed);
    match found {
        Some((bank, count)) => {
            info!(
                "Persistence: loaded {} contacts from bank {}",
                count, bank
            )
        }
        None => info!("Persistence: no contact snapshot"),
    }
}

/// Snapshot the contact list if its structural fields changed.
pub async fn save(flash: &mut Flash) {
    let hash = structural_hash();
    if hash == HASH.load(Ordering::Relaxed) {
        return;
    }
    let count = DISCOVERED_CONTACTS.lock(|c| c.borrow().len());
    let result = STORE
        .save::<PersistContact>(flash, count, |i| {
            DISCOVERED_CONTACTS
                .lock(|c| c.borrow().get(i).cloned())
                .map(|c| contact_to_record(&c))
        })
        .await;
    if result.is_ok() {
        HASH.store(hash, Ordering::Relaxed);
    }
}
