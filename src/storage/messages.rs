//! Message log snapshot.

use defmt::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::String;
use nrf_softdevice::Flash;

use super::snapshot::{fixed_str, str_from_fixed, SnapshotRecord, SnapshotStore};
use crate::companion::usb::{MESSAGE_LOG, MAX_MESSAGES};
use crate::meshcore::mesh::ReceivedMessage;

const MESSAGE_RECORD_SIZE: usize = 236;
// Two 64 KB banks fill 0xCC000..0xEC000; the config page starts at 0xEC000.
// 256 records * 236 bytes + 32-byte header = 60,480 bytes per bank.
const MESSAGE_BANK_SIZE: u32 = 16 * 4096;
const STORE: SnapshotStore =
    SnapshotStore::new(0x000C_C000, 0x000D_C000, MESSAGE_BANK_SIZE);

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

impl SnapshotRecord for PersistMessage {
    const SIZE: usize = MESSAGE_RECORD_SIZE;
    const MAGIC: u32 = 0x4D53_474D;
    const MAX_COUNT: usize = MAX_MESSAGES;
}

const _: () = assert!(core::mem::size_of::<PersistMessage>() == MESSAGE_RECORD_SIZE);
const _: () = assert!(
    super::snapshot::HEADER_SIZE + MAX_MESSAGES * MESSAGE_RECORD_SIZE
        <= MESSAGE_BANK_SIZE as usize
);

pub(crate) static SAVE_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

pub fn request() {
    let _ = SAVE_CHANNEL.try_send(());
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

/// Load the newest message snapshot into `MESSAGE_LOG`.
pub fn load() {
    let mut cleared = false;
    let found = STORE.load::<PersistMessage>(|rec| {
        let msg = record_to_message(&rec);
        MESSAGE_LOG.lock(|m| {
            let mut list = m.borrow_mut();
            if !cleared {
                list.clear();
                cleared = true;
            }
            let _ = list.push(msg);
        });
    });
    if !cleared {
        MESSAGE_LOG.lock(|m| m.borrow_mut().clear());
    }
    match found {
        Some((bank, count)) => {
            info!("Persistence: loaded {} messages from bank {}", count, bank)
        }
        None => info!("Persistence: no message snapshot"),
    }
}

/// Snapshot the message log.
pub async fn save(flash: &mut Flash) {
    let count = MESSAGE_LOG.lock(|m| m.borrow().len());
    let _ = STORE
        .save::<PersistMessage>(flash, count, |i| {
            MESSAGE_LOG
                .lock(|m| m.borrow().get(i).cloned())
                .map(|m| message_to_record(&m))
        })
        .await;
}
