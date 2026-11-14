//! Command and push queues shared by companion transports and the engine.
//!
//! The protocol layer only ever enqueues these values; the board owns the
//! consumers (radio task, contact task, BLE push drain), so the types and
//! channels live in the core crate.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
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

/// Outbound companion push frames (advert notifications, status responses).
pub static COMPANION_PUSH_CHANNEL: Channel<CriticalSectionRawMutex, Vec<u8, 256>, 40> =
    Channel::new();
