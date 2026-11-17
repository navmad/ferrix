//! Command and push queues shared by companion transports and the engine.
//!
//! The protocol layer only ever enqueues these values; the board owns the
//! consumers (radio task, contact task, BLE push drain), so the types and
//! channels live in the core crate.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use heapless::{String, Vec};

/// Commands from the companion protocol to the board's mesh/radio loop.
#[derive(Clone, Debug)]
pub enum UsbCommand {
    SetName(String<32>),
    SetPrivateKey([u8; 64]),
    SetTime(u64),
    TriggerAdvert { flood: bool },
    SetAdvertLatLon { lat: i32, lon: i32 },
    SetAutoAddConfig { config: u8, max_hops: u8 },
    SendGroupText {
        channel_idx: u8,
        timestamp: u32,
        text: String<160>,
    },
    SendDirectText {
        /// First six bytes of the destination public key.
        dest_prefix: [u8; 6],
        timestamp: u32,
        txt_type: u8,
        attempt: u8,
        text: String<160>,
    },
    SendChannelData {
        channel_idx: u8,
        path_len: u8,
        path: [u8; 64],
        data_type: u16,
        data: Vec<u8, 165>,
    },
    SendControlData {
        data: Vec<u8, 183>,
    },
    SendRawData {
        path_len: u8,
        path: [u8; 64],
        payload: Vec<u8, 184>,
    },
    SendRawPacket {
        raw: Vec<u8, 255>,
    },
    SendTracePath {
        tag: u32,
        auth: u32,
        flags: u8,
        path_len: u8,
        path: [u8; 64],
    },
    SetFrequency(u32),
    SetRadio {
        frequency_hz: u32,
        bw_khz: u32,
        sf: u8,
        cr: u8,
    },
    SetTxPower(i8),
    SetRepeat(bool),
    SetSpreadingFactor(u8),
    SetBandwidth(u32),
    SetCodingRate(u8),
    SetPathHashMode(u8),
    RebootToOtaDfu,
}

pub static USB_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, UsbCommand, 4> = Channel::new();

/// Contact operation deferred out of the command handler so contact storage
/// never runs inside the BLE/SoftDevice path.
pub enum ContactOperation {
    AddOrUpdate {
        pub_key: [u8; 32],
        callsign: String<32>,
        node_id: [u8; 4],
        last_seen_epoch: u32,
        latitude: i32,
        longitude: i32,
        flags: u8,
        contact_type: u8,
        out_path: [u8; 64],
        out_path_len: u8,
        last_advert_ts: u32,
    },
    /// Clear a contact's learned path.
    ResetPath {
        pub_key: [u8; 32],
    },
    Remove {
        pub_key: [u8; 32],
    },
}

/// Channel for deferring contact mutations outside the GATT callback's
/// SoftDevice critical section.
pub static CONTACT_OP_CHANNEL: Channel<CriticalSectionRawMutex, ContactOperation, 4> =
    Channel::new();

/// Maximum number of registered push transports.
pub const MAX_PUSH_SINKS: usize = 4;

/// A transport that receives asynchronous companion pushes.
///
/// `try_push` is synchronous and may drop when the transport queue is full;
/// producers must never block the radio/app loop on a transport.
pub trait PushSink: Send + Sync {
    fn is_connected(&self) -> bool;
    fn try_push(&self, frame: &[u8]) -> bool;
}

static PUSH_SINKS: Mutex<
    CriticalSectionRawMutex,
    RefCell<[Option<&'static dyn PushSink>; MAX_PUSH_SINKS]>,
> = Mutex::new(RefCell::new([None; MAX_PUSH_SINKS]));

static PUSH_DROPS: AtomicU32 = AtomicU32::new(0);

/// Register a transport sink. Returns false when the table is full.
pub fn register_push_sink(sink: &'static dyn PushSink) -> bool {
    PUSH_SINKS.lock(|s| {
        let mut list = s.borrow_mut();
        for slot in list.iter_mut() {
            if let Some(existing) = slot {
                if core::ptr::eq(*existing, sink) {
                    return true;
                }
            }
        }
        for slot in list.iter_mut() {
            if slot.is_none() {
                *slot = Some(sink);
                return true;
            }
        }
        false
    })
}

/// Broadcast one asynchronous push to every connected transport.
pub fn push_companion(frame: &[u8]) {
    PUSH_SINKS.lock(|s| {
        for sink in s.borrow().iter().flatten() {
            if sink.is_connected() && !sink.try_push(frame) {
                PUSH_DROPS.fetch_add(1, Ordering::Relaxed);
            }
        }
    });
}

/// Number of pushes a transport refused because its queue was full.
pub fn push_drops() -> u32 {
    PUSH_DROPS.load(Ordering::Relaxed)
}
