//! MeshCore Companion Protocol Command & Response Definitions
//!
//! Handles command parsing from companion apps (Mobile/Desktop/Web)
//! and generates protocol response frames.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use heapless::{String, Vec};

use meshcore_wire::packet::{
    Packet, PAYLOAD_TYPE_ADVERT, PAYLOAD_TYPE_REQ, ROUTE_TYPE_DIRECT, ROUTE_TYPE_FLOOD,
};
use meshcore_wire::packets::{AddressedPacket, AdvertPacket, AnonReqPacket};

/// Signing scratch buffer. Advertised to the app by `SIGN_START`.
pub const MAX_SIGN_DATA: usize = 4096;
static SIGN_ACTIVE: AtomicBool = AtomicBool::new(false);
static SIGN_BUFFER: Mutex<CriticalSectionRawMutex, RefCell<Vec<u8, MAX_SIGN_DATA>>> =
    Mutex::new(RefCell::new(Vec::new()));

/// Diagnostics for companion app capability negotiation. These are shared by
/// every transport so `get companion` can show what the connected app asked.
pub static DEVICE_QUERY_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GET_AUTOADD_COUNT: AtomicU32 = AtomicU32::new(0);
pub static SET_AUTOADD_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LAST_CMD: AtomicU8 = AtomicU8::new(0);
/// Diagnostics for advert processing and blob backfill.
pub static ADVERT_COUNT: AtomicU32 = AtomicU32::new(0);

/// Tag of the outstanding status/ping request, if any.
static PENDING_STATUS: Mutex<CriticalSectionRawMutex, RefCell<Option<(u32, [u8; 32])>>> =
    Mutex::new(RefCell::new(None));
static LAST_TAG: AtomicU32 = AtomicU32::new(0);
const REQ_TYPE_GET_STATUS: u8 = 0x01;

/// Strictly increasing request tag anchored to wall time, like MeshCore's
/// `getCurrentTimeUnique()`. Never add a lifetime counter: that drifts ahead
/// of the clock and makes post-reboot tags look like replays to a server.
fn next_tag() -> u32 {
    let now = crate::state::get_device_time();
    let mut prev = LAST_TAG.load(Ordering::Relaxed);
    loop {
        let next = if now > prev { now } else { prev.wrapping_add(1) };
        match LAST_TAG.compare_exchange_weak(
            prev,
            next,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => return next,
            Err(current) => prev = current,
        }
    }
}

fn note_status(tag: u32, pub_key: [u8; 32]) {
    PENDING_STATUS.lock(|p| *p.borrow_mut() = Some((tag, pub_key)));
}

use crate::state::{Connection, CONNECTIONS, MAX_CONNECTIONS, PENDING_LOGIN};

const RESP_SERVER_LOGIN_OK: u8 = 0;
const REQ_TYPE_KEEP_ALIVE: u8 = 0x02;

// Login diagnostics.
pub static LOGIN_TX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LOGIN_TX_TAG: AtomicU32 = AtomicU32::new(0);
pub static LOGIN_TX_KEY: AtomicU32 = AtomicU32::new(0);
pub static LOGIN_TX_FLOOD: AtomicBool = AtomicBool::new(false);
pub static LOGIN_RX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LOGIN_RX_TAG: AtomicU32 = AtomicU32::new(0);
pub static LOGIN_RX_LEN: AtomicU32 = AtomicU32::new(0);
pub static LOGIN_RESULT: AtomicU8 = AtomicU8::new(0);
pub static LOGIN_PENDING: AtomicBool = AtomicBool::new(false);

/// Request results are delayed briefly: clients that register their pending
/// tag/wait after receiving PACKET_SENT (e.g. meshcore-cli) otherwise miss it.
static DELAYED_PUSH: Channel<CriticalSectionRawMutex, Vec<u8, 256>, 2> = Channel::new();

/// Wait for the next deferred push frame and its settling delay.
pub async fn next_delayed_push() -> Vec<u8, 256> {
    let frame = DELAYED_PUSH.receive().await;
    embassy_time::Timer::after_millis(800).await;
    frame
}
// Binary/anonymous request diagnostics.
pub static REQ_TX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static REQ_TX_TAG: AtomicU32 = AtomicU32::new(0);
pub static REQ_TX_TYPE: AtomicU8 = AtomicU8::new(0);
pub static REQ_RX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static REQ_RX_TAG: AtomicU32 = AtomicU32::new(0);
pub static REQ_RX_LEN: AtomicU32 = AtomicU32::new(0);

pub static DISC_TX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISC_TX_TAG: AtomicU32 = AtomicU32::new(0);
pub static DISC_RX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static DISC_RX_TAG: AtomicU32 = AtomicU32::new(0);
pub static DISC_RX_OUT: AtomicU8 = AtomicU8::new(0);
pub static DISC_RX_IN: AtomicU8 = AtomicU8::new(0);
pub static DISC_PENDING: AtomicBool = AtomicBool::new(false);
pub static DISC_TX_MS: AtomicU32 = AtomicU32::new(0);
pub static DISC_RX_RTT_MS: AtomicU32 = AtomicU32::new(0);

/// Outstanding path-discovery request: (tag, destination).
static PENDING_DISCOVERY: Mutex<CriticalSectionRawMutex, RefCell<Option<(u32, [u8; 32])>>> =
    Mutex::new(RefCell::new(None));

/// Outstanding telemetry request: (tag, destination).
static PENDING_TELEMETRY: Mutex<CriticalSectionRawMutex, RefCell<Option<(u32, [u8; 32])>>> =
    Mutex::new(RefCell::new(None));
const REQ_TYPE_GET_TELEMETRY_DATA: u8 = 0x03;

/// Outstanding binary/anon request: (tag, destination).
static PENDING_REQ: Mutex<CriticalSectionRawMutex, RefCell<Option<(u32, [u8; 32])>>> =
    Mutex::new(RefCell::new(None));

fn now_ms() -> u64 {
    embassy_time::Instant::now().as_millis()
}

fn note_pending_login(pub_key: [u8; 32]) {
    PENDING_LOGIN.lock(|p| *p.borrow_mut() = Some((pub_key, now_ms() + 15000)));
    LOGIN_PENDING.store(true, Ordering::Relaxed);
}

fn note_pending_req(tag: u32, pub_key: [u8; 32]) {
    PENDING_REQ.lock(|p| *p.borrow_mut() = Some((tag, pub_key)));
}

fn note_pending_telemetry(tag: u32, pub_key: [u8; 32]) {
    PENDING_TELEMETRY.lock(|p| *p.borrow_mut() = Some((tag, pub_key)));
}

fn note_pending_discovery(tag: u32, pub_key: [u8; 32]) {
    PENDING_DISCOVERY.lock(|p| *p.borrow_mut() = Some((tag, pub_key)));
    DISC_TX_COUNT.fetch_add(1, Ordering::Relaxed);
    DISC_TX_TAG.store(tag, Ordering::Relaxed);
    DISC_TX_MS.store(embassy_time::Instant::now().as_millis() as u32, Ordering::Relaxed);
    DISC_PENDING.store(true, Ordering::Relaxed);
}

/// Emit PATH_DISCOVERY_RESP when a PATH reply carries the pending tag.
pub fn handle_path_discovery(
    tag: u32,
    sender_pub: &[u8; 32],
    out_path_len: u8,
    out_path: &[u8],
    in_path_len: u8,
    in_path: &[u8],
) -> bool {
    let matched = PENDING_DISCOVERY.lock(|p| {
        let mut pending = p.borrow_mut();
        if pending.map_or(false, |(t, k)| t == tag && k == *sender_pub) {
            *pending = None;
            true
        } else {
            false
        }
    });
    if !matched {
        return false;
    }
    DISC_RX_COUNT.fetch_add(1, Ordering::Relaxed);
    DISC_RX_TAG.store(tag, Ordering::Relaxed);
    DISC_RX_OUT.store(out_path_len, Ordering::Relaxed);
    DISC_RX_IN.store(in_path_len, Ordering::Relaxed);
    DISC_RX_RTT_MS.store(
        (embassy_time::Instant::now().as_millis() as u32)
            .wrapping_sub(DISC_TX_MS.load(Ordering::Relaxed)),
        Ordering::Relaxed,
    );
    DISC_PENDING.store(false, Ordering::Relaxed);
    let mut push = Vec::<u8, 256>::new();
    let _ = push.push(PUSH_CODE_PATH_DISCOVERY_RESP);
    let _ = push.push(0);
    let _ = push.extend_from_slice(&sender_pub[..6]);
    let _ = push.push(out_path_len);
    let on = (out_path_len & 63) as usize * (((out_path_len >> 6) & 3) as usize + 1);
    let _ = push.extend_from_slice(&out_path[..on.min(out_path.len())]);
    let _ = push.push(in_path_len);
    let inn = (in_path_len & 63) as usize * (((in_path_len >> 6) & 3) as usize + 1);
    let _ = push.extend_from_slice(&in_path[..inn.min(in_path.len())]);
    let _ = DELAYED_PUSH.try_send(push);
    true
}

fn start_connection(pub_key: [u8; 32], keep_alive_secs: u16) {
    let now = now_ms();
    let interval = keep_alive_secs as u64 * 1000;
    CONNECTIONS.lock(|c| {
        let mut list = c.borrow_mut();
        if let Some(entry) = list.iter_mut().find(|e| e.pub_key == pub_key) {
            entry.last_activity_ms = now;
            entry.next_keepalive_ms = now.saturating_add(interval);
            entry.expected_ack = 0;
            entry.keep_alive_secs = keep_alive_secs;
        } else if !list.is_full() {
            let _ = list.push(Connection {
                pub_key,
                last_activity_ms: now,
                next_keepalive_ms: now.saturating_add(interval),
                expected_ack: 0,
                keep_alive_secs,
            });
        }
    });
}

fn connection_alive(entry: &Connection, now: u64) -> bool {
    let interval = entry.keep_alive_secs as u64 * 1000;
    entry.last_activity_ms.saturating_add(interval * 5 / 2) > now
}

fn has_connection(pub_key: &[u8; 32]) -> bool {
    let now = now_ms();
    CONNECTIONS.lock(|c| {
        let mut list = c.borrow_mut();
        list.retain(|e| connection_alive(e, now));
        list.iter().any(|e| e.pub_key == *pub_key)
    })
}

fn stop_connection(pub_key: &[u8; 32]) {
    CONNECTIONS.lock(|c| {
        let mut list = c.borrow_mut();
        if let Some(pos) = list.iter().position(|e| e.pub_key == *pub_key) {
            list.remove(pos);
        }
    });
}

/// Match an ACK to a pending keep-alive and refresh the connection.
pub fn handle_keepalive_ack(crc: u32) -> bool {
    if crc == 0 {
        return false;
    }
    let now = now_ms();
    CONNECTIONS.lock(|c| {
        let mut list = c.borrow_mut();
        for entry in list.iter_mut() {
            if entry.expected_ack == crc {
                entry.expected_ack = 0;
                entry.last_activity_ms = now;
                entry.next_keepalive_ms = now
                    .saturating_add((entry.keep_alive_secs as u64 * 1000).max(1000));
                return true;
            }
        }
        false
    })
}

fn send_keepalive(tag: u32, dest_key: &[u8; 32], expected_ack: u32) -> bool {
    let callsign = crate::state::NODE_CALLSIGN.lock(|c| c.borrow().clone());
    let private_key = crate::state::current_private_key();
    let public_key = crate::state::current_public_key();
    let identity = meshcore_wire::identity::LocalIdentity::from_private(
        callsign.as_str(),
        &private_key,
    );
    let Some(id) = identity else { return false };
    let Some(secret) = id.shared_secret(dest_key) else { return false };
    let contact = crate::state::DISCOVERED_CONTACTS.lock(|c| {
        c.borrow()
            .iter()
            .find(|ct| ct.pub_key == *dest_key)
            .map(|ct| (ct.out_path_len, ct.out_path))
    });
    let Some((out_path_len, out_path)) = contact else { return false };
    if out_path_len == 0xFF || !Packet::is_valid_path_len(out_path_len) {
        return false;
    }
    let mut plain = [0u8; 9];
    plain[..4].copy_from_slice(&tag.to_le_bytes());
    plain[4] = REQ_TYPE_KEEP_ALIVE;
    let Ok(addressed) = AddressedPacket::new(dest_key[0], public_key[0], &plain) else {
        return false;
    };
    let Ok(mut pkt) = addressed.to_packet(PAYLOAD_TYPE_REQ, ROUTE_TYPE_DIRECT, &secret) else {
        return false;
    };
    pkt.path_len = out_path_len;
    let n = pkt.path_byte_len();
    pkt.path[..n].copy_from_slice(&out_path[..n]);
    let mut buf = [0u8; 255];
    let Ok(n) = pkt.write_to(&mut buf) else { return false };
    let mut raw = heapless::Vec::<u8, 255>::new();
    if raw.extend_from_slice(&buf[..n]).is_err() {
        return false;
    }
    if crate::command::USB_COMMAND_CHANNEL
        .try_send(crate::command::UsbCommand::SendRawPacket { raw })
        .is_err()
    {
        return false;
    }
    let _ = expected_ack;
    true
}

/// Send due keepalives and expire stale connections. Called from the main
/// ticker (every 2 s).
pub fn keepalive_tick() {
    let now = now_ms();
    PENDING_LOGIN.lock(|p| {
        let mut pending = p.borrow_mut();
        let expired = pending.map_or(false, |(_, deadline)| now > deadline);
        if expired {
            *pending = None;
            LOGIN_PENDING.store(false, Ordering::Relaxed);
        }
    });
    let public_key = crate::state::current_public_key();
    let mut sends = heapless::Vec::<(u32, [u8; 32], u32), MAX_CONNECTIONS>::new();
    CONNECTIONS.lock(|c| {
        let mut list = c.borrow_mut();
        list.retain(|e| connection_alive(e, now));
        for entry in list.iter_mut() {
            if now < entry.next_keepalive_ms {
                continue;
            }
            let tag = next_tag();
            let expected = meshcore_wire::crypto::ack_crc(
                tag,
                REQ_TYPE_KEEP_ALIVE,
                &[0u8; 4],
                &public_key,
            );
            entry.expected_ack = expected;
            entry.next_keepalive_ms =
                now.saturating_add((entry.keep_alive_secs as u64 * 1000).max(1000));
            let _ = sends.push((tag, entry.pub_key, expected));
        }
    });
    for (tag, key, expected) in sends.iter() {
        let _ = send_keepalive(*tag, key, *expected);
    }
}

/// Route a received RESPONSE to the app when it matches a pending login or
/// status request.
/// Returns true when the response was consumed.
pub fn handle_response(plain: &[u8], sender_pub: &[u8; 32]) -> bool {
    if plain.len() < 4 {
        return false;
    }
    let tag = u32::from_le_bytes([plain[0], plain[1], plain[2], plain[3]]);

    // Login reply: the repeater's first 4 bytes are its own clock, so match
    // the sender and the login-reply shape instead of the tag. A late
    // status/neighbour response must not be mistaken for a login result.
    // Login success is `server_time(4) ‖ RESP_SERVER_LOGIN_OK ‖ ...`; newer
    // firmwares may append extra fields, so allow up to 32 bytes. Status
    // (60 bytes) and neighbour replies are longer and are not consumed here.
    let login_shape = plain.len() >= 5
        && plain.len() <= 32
        && (plain[4] == RESP_SERVER_LOGIN_OK
            || (plain.len() >= 6 && &plain[4..6] == b"OK"));
    let login = PENDING_LOGIN.lock(|p| {
        let mut pending = p.borrow_mut();
        if let Some((key, deadline)) = *pending {
            if key == *sender_pub && login_shape && now_ms() <= deadline {
                *pending = None;
                true
            } else {
                false
            }
        } else {
            false
        }
    });
    if login {
        LOGIN_RX_COUNT.fetch_add(1, Ordering::Relaxed);
        LOGIN_RX_TAG.store(tag, Ordering::Relaxed);
        LOGIN_RX_LEN.store(plain.len() as u32, Ordering::Relaxed);
        LOGIN_PENDING.store(false, Ordering::Relaxed);
        let mut push = Vec::<u8, 256>::new();
        let _ = push.push(PUSH_CODE_LOGIN_SUCCESS);
        let reply = &plain[4..];
        if reply.len() >= 2 && &reply[..2] == b"OK" {
            let _ = push.push(0);
            let _ = push.extend_from_slice(&sender_pub[..6]);
            LOGIN_RESULT.store(1, Ordering::Relaxed);
        } else if !reply.is_empty() && reply[0] == RESP_SERVER_LOGIN_OK {
            let keep_alive_secs = reply.get(1).copied().unwrap_or(0) as u16 * 16;
            if keep_alive_secs > 0 {
                start_connection(*sender_pub, keep_alive_secs);
            }
            let _ = push.push(reply.get(2).copied().unwrap_or(0));
            let _ = push.extend_from_slice(&sender_pub[..6]);
            let _ = push.extend_from_slice(&tag.to_le_bytes());
            let _ = push.push(reply.get(3).copied().unwrap_or(0));
            let _ = push.push(reply.get(8).copied().unwrap_or(0));
            LOGIN_RESULT.store(1, Ordering::Relaxed);
        } else {
            push[0] = PUSH_CODE_LOGIN_FAIL;
            LOGIN_RESULT.store(2, Ordering::Relaxed);
            let _ = push.push(0);
            let _ = push.extend_from_slice(&sender_pub[..6]);
        }
        let _ = DELAYED_PUSH.try_send(push);
        return true;
    }

    // Binary / anonymous request response: [tag][data].
    let req_match = PENDING_REQ.lock(|p| {
        let mut pending = p.borrow_mut();
        if pending.map_or(false, |(t, k)| t == tag && k == *sender_pub) {
            *pending = None;
            true
        } else {
            false
        }
    });
    if req_match {
        REQ_RX_COUNT.fetch_add(1, Ordering::Relaxed);
        REQ_RX_TAG.store(tag, Ordering::Relaxed);
        REQ_RX_LEN.store(plain.len() as u32, Ordering::Relaxed);
        let mut push = Vec::<u8, 256>::new();
        let _ = push.push(PUSH_CODE_BINARY_RESPONSE);
        let _ = push.push(0);
        let _ = push.extend_from_slice(&tag.to_le_bytes());
        let data = &plain[4..];
        let _ = push.extend_from_slice(&data[..data.len().min(256 - push.len())]);
        let _ = DELAYED_PUSH.try_send(push);
        return true;
    }

    // Telemetry response.
    let telem_match = PENDING_TELEMETRY.lock(|p| {
        let mut pending = p.borrow_mut();
        if pending.map_or(false, |(t, k)| t == tag && k == *sender_pub) {
            *pending = None;
            true
        } else {
            false
        }
    });
    if telem_match {
        let mut push = Vec::<u8, 256>::new();
        let _ = push.push(PUSH_CODE_TELEMETRY_RESPONSE);
        let _ = push.push(0);
        let _ = push.extend_from_slice(&sender_pub[..6]);
        let data = &plain[4..];
        let _ = push.extend_from_slice(&data[..data.len().min(256 - push.len())]);
        let _ = DELAYED_PUSH.try_send(push);
        return true;
    }

    let matched = PENDING_STATUS.lock(|p| {
        let mut pending = p.borrow_mut();
        if pending.map_or(false, |(t, k)| t == tag && k == *sender_pub) {
            *pending = None;
            true
        } else {
            false
        }
    });
    if !matched {
        return false;
    }

    let mut push = Vec::<u8, 256>::new();
    let _ = push.push(PUSH_CODE_STATUS_RESPONSE);
    let _ = push.push(0);
    let _ = push.extend_from_slice(&sender_pub[..6]);
    let data = &plain[4..];
    let _ = push.extend_from_slice(&data[..data.len().min(256 - push.len())]);
    let _ = DELAYED_PUSH.try_send(push);
    true
}
pub static ADVERT_LAST_KEY: AtomicU32 = AtomicU32::new(0);
pub static ADVERT_LAST_TS: AtomicU32 = AtomicU32::new(0);
pub static ADVERT_LAST_HAD_BLOB: AtomicBool = AtomicBool::new(false);

/// Last TRACE command sent to the app and last TRACE received.
pub static TRACE_TX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TRACE_TX_TAG: AtomicU32 = AtomicU32::new(0);
pub static TRACE_TX_FLAGS: AtomicU8 = AtomicU8::new(0);
pub static TRACE_TX_PATH_LEN: AtomicU8 = AtomicU8::new(0);
pub static TRACE_TX_PATH: AtomicU32 = AtomicU32::new(0);
pub static TRACE_RX_COUNT: AtomicU32 = AtomicU32::new(0);
pub static TRACE_RX_TAG: AtomicU32 = AtomicU32::new(0);
pub static TRACE_RX_FLAGS: AtomicU8 = AtomicU8::new(0);
pub static TRACE_RX_HASHES: AtomicU8 = AtomicU8::new(0);
pub static TRACE_RX_SNRS: AtomicU8 = AtomicU8::new(0);

pub fn note_trace_rx(tag: u32, flags: u8, hashes: usize, snrs: usize) {
    TRACE_RX_COUNT.fetch_add(1, Ordering::Relaxed);
    TRACE_RX_TAG.store(tag, Ordering::Relaxed);
    TRACE_RX_FLAGS.store(flags, Ordering::Relaxed);
    TRACE_RX_HASHES.store(hashes.min(255) as u8, Ordering::Relaxed);
    TRACE_RX_SNRS.store(snrs.min(255) as u8, Ordering::Relaxed);
}

pub fn note_advert(pub_key: &[u8; 32], timestamp: u32, had_blob: bool) {
    ADVERT_COUNT.fetch_add(1, Ordering::Relaxed);
    ADVERT_LAST_HAD_BLOB.store(had_blob, Ordering::Relaxed);
    ADVERT_LAST_KEY.store(
        u32::from_le_bytes([pub_key[0], pub_key[1], pub_key[2], pub_key[3]]),
        Ordering::Relaxed,
    );
    ADVERT_LAST_TS.store(timestamp, Ordering::Relaxed);
}

pub use crate::companion::contacts::{
    encode_contacts_end, encode_contacts_start, RESP_CODE_CONTACT, RESP_CODE_CONTACTS_START,
    RESP_CODE_END_OF_CONTACTS,
};
pub(crate) use crate::companion::contacts::serialize_contact;

pub use crate::state::{
    advert_location, apply_flood_scope, apply_flood_scope_default, flood_scope_default,
    flood_scope_send, prefs, scope_override, update_prefs, CompanionPrefs, DEFAULT_PREFS, SCOPE,
    SCOPED_FLOODS,
};

pub const CMD_APP_START: u8 = 1;
pub const CMD_SEND_MSG: u8 = 2;
pub const CMD_SEND_CHANNEL_MSG: u8 = 3;
pub const CMD_GET_CONTACTS: u8 = 4;
pub const CMD_GET_DEVICE_TIME: u8 = 5;
pub const CMD_SET_DEVICE_TIME: u8 = 6;
pub const CMD_SEND_SELF_ADVERT: u8 = 7;
pub const CMD_SET_ADVERT_NAME: u8 = 8;
pub const CMD_ADD_UPDATE_CONTACT: u8 = 9;
pub const CMD_SYNC_NEXT_MESSAGE: u8 = 10;
pub const CMD_SET_RADIO_PARAMS: u8 = 0x0B;
pub const CMD_SET_RADIO_TX_POWER: u8 = 0x0C;
pub const CMD_RESET_PATH: u8 = 0x0D;
pub const CMD_SET_ADVERT_LATLON: u8 = 0x0E;
pub const CMD_REMOVE_CONTACT: u8 = 15;
pub const CMD_SHARE_CONTACT: u8 = 0x10;
pub const CMD_EXPORT_CONTACT: u8 = 0x11;
pub const CMD_IMPORT_CONTACT: u8 = 0x12;
pub const CMD_REBOOT: u8 = 0x13;
pub const CMD_GET_BATT_AND_STORAGE: u8 = 0x14;
pub const CMD_SET_TUNING_PARAMS: u8 = 0x15;
pub const CMD_EXPORT_PRIVATE_KEY: u8 = 0x17;
pub const CMD_IMPORT_PRIVATE_KEY: u8 = 0x18;
pub const CMD_GET_CONTACT_BY_KEY: u8 = 0x1E;
pub const CMD_SIGN_START: u8 = 0x21;
pub const CMD_SIGN_DATA: u8 = 0x22;
pub const CMD_SIGN_FINISH: u8 = 0x23;
pub const CMD_SEND_LOGIN: u8 = 0x1A;
pub const CMD_SEND_STATUS_REQ: u8 = 0x1B;
pub const CMD_HAS_CONNECTION: u8 = 0x1C;
pub const CMD_LOGOUT: u8 = 0x1D;
pub const CMD_SEND_TRACE_PATH: u8 = 0x24;
pub const CMD_SET_DEVICE_PIN: u8 = 0x25;
pub const CMD_SET_OTHER_PARAMS: u8 = 0x26;
pub const CMD_SEND_TELEMETRY_REQ: u8 = 0x27;
pub const CMD_GET_CUSTOM_VARS: u8 = 0x28;
pub const CMD_SET_CUSTOM_VAR: u8 = 0x29;
pub const CMD_DEVICE_QUERY: u8 = 22;
pub const CMD_GET_ADVERT_PATH: u8 = 0x2A;
pub const CMD_GET_TUNING_PARAMS: u8 = 0x2B;
pub const CMD_GET_CHANNEL: u8 = 31;
pub const CMD_SET_CHANNEL: u8 = 32;
pub const CMD_SEND_BINARY_REQ: u8 = 0x32;
pub const CMD_FACTORY_RESET: u8 = 0x33;
pub const CMD_SEND_PATH_DISCOVERY_REQ: u8 = 0x34;
pub const CMD_SET_FLOOD_SCOPE_KEY: u8 = 0x36;
pub const CMD_SEND_CONTROL_DATA: u8 = 0x37;
pub const CMD_GET_STATS: u8 = 0x38;
pub const CMD_SET_AUTOADD_CONFIG: u8 = 0x3A;
pub const CMD_GET_AUTOADD_CONFIG: u8 = 0x3B;
pub const CMD_GET_ALLOWED_REPEAT_FREQ: u8 = 0x3C;
pub const CMD_SET_PATH_HASH_MODE: u8 = 0x3D;
pub const CMD_SEND_ANON_REQ: u8 = 0x39;
pub const CMD_SEND_CHANNEL_DATA: u8 = 0x3E;
pub const CMD_SET_DEFAULT_FLOOD_SCOPE: u8 = 0x3F;
pub const CMD_GET_DEFAULT_FLOOD_SCOPE: u8 = 0x40;
pub const CMD_SEND_RAW_PACKET: u8 = 0x41;
pub const CMD_SEND_RAW_DATA: u8 = 0x19;
pub const CMD_RUN_CLI_COMMAND: u8 = 0x42;

pub const RESP_CODE_OK: u8 = 0;
pub const RESP_CODE_ERR: u8 = 1;
/// Second byte of an error frame: `[RESP_CODE_ERR, code]`.
pub const ERR_UNSUPPORTED: u8 = 0x01;
pub const ERR_NOT_FOUND: u8 = 0x02;
pub const ERR_TABLE_FULL: u8 = 0x03;
pub const ERR_BAD_STATE: u8 = 0x04;
pub const ERR_ILLEGAL_ARG: u8 = 0x06;
pub const RESP_CODE_SELF_INFO: u8 = 5;
/// Reply to a direct send: `[is_flood][expected_ack:4][est_timeout_ms:4]`.
pub const RESP_CODE_SENT: u8 = 0x06;
pub const RESP_CODE_CURR_TIME: u8 = 9;
pub const RESP_CODE_NO_MORE_MESSAGES: u8 = 10;
pub const RESP_CODE_DEVICE_INFO: u8 = 13;
pub const RESP_CODE_EXPORT_CONTACT: u8 = 0x0B;
pub const RESP_CODE_BATTERY: u8 = 0x0C;
pub const RESP_CODE_PRIVATE_KEY: u8 = 0x0E;
pub const RESP_CODE_CONTACT_MSG_V3: u8 = 0x10;
pub const RESP_CODE_CHANNEL_MSG_V3: u8 = 0x11;
pub const RESP_CODE_CHANNEL_INFO: u8 = 18;
pub const RESP_CODE_SIGN_START: u8 = 0x13;
pub const RESP_CODE_SIGNATURE: u8 = 0x14;
pub const RESP_CODE_CUSTOM_VARS: u8 = 0x15;
pub const RESP_CODE_ADVERT_PATH: u8 = 0x16;
pub const RESP_CODE_TUNING: u8 = 0x17;
pub const RESP_CODE_STATS: u8 = 0x18;
pub const RESP_CODE_AUTOADD: u8 = 0x19;
pub const RESP_CODE_ALLOWED_REPEAT_FREQ: u8 = 0x1A;
pub const RESP_CODE_DEFAULT_FLOOD_SCOPE: u8 = 0x1C;
pub const RESP_CODE_TIME: u8 = 20;
pub const RESP_CODE_MSG: u8 = 21;

// Keep these in sync with MeshCore's CompanionMesh.h. Several push codes
// currently have no producer; they are defined so future handlers use the
// right values instead of colliding with another event.
pub const PUSH_CODE_ADVERT: u8 = 0x80;
pub const PUSH_CODE_PATH_UPDATED: u8 = 0x81;
pub const PUSH_CODE_SEND_CONFIRMED: u8 = 0x82;
pub const PUSH_CODE_MSG_WAITING: u8 = 0x83;
pub const PUSH_CODE_RAW_DATA: u8 = 0x84;
pub const PUSH_CODE_LOGIN_SUCCESS: u8 = 0x85;
pub const PUSH_CODE_LOGIN_FAIL: u8 = 0x86;
pub const PUSH_CODE_STATUS_RESPONSE: u8 = 0x87;
pub const PUSH_CODE_LOG_RX_DATA: u8 = 0x88;
pub const PUSH_CODE_TRACE_DATA: u8 = 0x89;
pub const PUSH_CODE_NEW_ADVERT: u8 = 0x8A;
pub const PUSH_CODE_TELEMETRY_RESPONSE: u8 = 0x8B;
pub const PUSH_CODE_BINARY_RESPONSE: u8 = 0x8C;
pub const PUSH_CODE_PATH_DISCOVERY_RESP: u8 = 0x8D;
pub const PUSH_CODE_CONTROL_DATA: u8 = 0x8E;
/// Inbound channel datagram frame (same value as the reference packet code).
pub const PACKET_CHANNEL_DATA_RECV: u8 = 0x1B;
/// Local CLI reply for `CMD_RUN_CLI_COMMAND`.
pub const PACKET_CLI_REPLY: u8 = 0x1D;
pub const PUSH_CODE_CONTACT_DELETED: u8 = 0x8F;
pub const PUSH_CODE_CONTACTS_FULL: u8 = 0x90;

/// Auto-add configuration bitmask (matches MeshCore)
pub const AUTO_ADD_OVERWRITE_OLDEST: u8 = 1 << 0;
pub const AUTO_ADD_CHAT: u8 = 1 << 1;
pub const AUTO_ADD_REPEATER: u8 = 1 << 2;
pub const AUTO_ADD_ROOM_SERVER: u8 = 1 << 3;
pub const AUTO_ADD_SENSOR: u8 = 1 << 4;

/// Contact type constants (from advert flags)
pub const ADV_TYPE_CHAT: u8 = 1;
pub const ADV_TYPE_REPEATER: u8 = 2;
pub const ADV_TYPE_ROOM: u8 = 3;
pub const ADV_TYPE_SENSOR: u8 = 4;

/// Contact flags bitmask
pub const CONTACT_FLAG_FAVORITE: u8 = 1 << 0;

/// Device capabilities bitmask
pub const CAP_ROUTER: u32 = 1 << 0;
pub const CAP_CLIENT: u32 = 1 << 1;
pub const CAP_GPS: u32 = 1 << 2;
pub const CAP_EPD: u32 = 1 << 3;
pub const CAP_BLE: u32 = 1 << 4;

/// Board services the companion protocol needs.
///
/// The protocol owns command parsing and response encoding; persistence,
/// radio reconfiguration, advert storage and battery sampling stay on the
/// board side of this trait.
pub trait CompanionHost: Send + Sync {
    fn battery_millivolts(&self) -> u16;
    fn set_callsign(&self, name: &str);
    fn set_custom_priv_key(&self, key: [u8; 64]);
    fn set_path_hash_mode(&self, mode: u8);
    fn request_save_config(&self);
    fn request_radio_config(&self);
    fn request_factory_reset(&self);
    fn queue_consume_now(&self);
    fn advert_remember(&self, pub_key: &[u8; 32], payload: &[u8]);
    /// Reset the node. Does not return.
    fn reboot(&self) -> !;
}

/// MeshCore Companion Node State
pub struct CompanionNode<'a> {
    pub callsign: String<32>,
    pub hw_model: &'static str,
    pub capabilities: u32,
    /// Node type reported in SELF_INFO and used for adverts.
    pub advert_type: u8,
    pub public_key: [u8; 32],
    /// MeshCore's 64-byte expanded private key (`scalar ‖ nonce_prefix`).
    pub private_key: [u8; 64],
    pub battery_millivolts: u16,
    pub battery_percentage: u8,
    pub epoch_time: u64,
    pub gps_fix: bool,
    pub latitude: f32,
    pub longitude: f32,
    pub tx_beacons: u32,
    pub rx_packets: u32,
    pub last_rssi: i16,
    pub last_snr: i8,
    pub current_page: u8,
    pub path_hash_mode: u8,
    /// Auto-add configuration bitmask (matches MeshCore)
    pub autoadd_config: u8,
    /// Max hops for auto-adding contacts (0 = no limit)
    pub autoadd_max_hops: u8,
    /// Receive delay base in milliseconds (tuning parameter).
    pub rx_delay_ms: u32,
    /// Airtime factor × 1000 (tuning parameter).
    pub airtime_factor_milli: u32,
    pub manual_add_contacts: u8,
    pub telemetry_mode_base: u8,
    pub telemetry_mode_loc: u8,
    pub telemetry_mode_env: u8,
    pub advert_loc_policy: u8,
    pub multi_acks: u8,
    /// Tracks how many channels have been synced since the last connection.
    /// Used to defer connection parameter updates until after the sync burst.
    pub channels_synced: u8,
    host: &'a dyn CompanionHost,
}

impl<'a> CompanionNode<'a> {
    pub fn new(
        callsign: &str,
        hw_model: &'static str,
        capabilities: u32,
        public_key: [u8; 32],
        private_key: [u8; 64],
        host: &'a dyn CompanionHost,
    ) -> Self {
        let mut cs = String::new();
        let _ = cs.push_str(callsign);
        let auto = crate::state::AUTOADD_SETTINGS.lock(|a| *a.borrow());
        // Prefer the shared identity: another transport may have imported a
        // key after this task was spawned.
        let shared_pub = crate::state::current_public_key();
        let (public_key, private_key) = if shared_pub == [0u8; 32] {
            (public_key, private_key)
        } else {
            (
                shared_pub,
                crate::state::current_private_key(),
            )
        };
        let prefs = prefs();
        let battery_mv = host.battery_millivolts();
        let (battery_millivolts, battery_percentage) =
            if battery_mv >= crate::battery::MIN_PLAUSIBLE_BATTERY_MV {
                (battery_mv, crate::battery::millivolts_to_percentage(battery_mv))
            } else {
                (4120, 95)
            };

        Self {
            callsign: cs,
            hw_model,
            advert_type: meshcore_wire::packets::advert::ADV_TYPE_CHAT,
            capabilities,
            public_key,
            private_key,
            battery_millivolts,
            battery_percentage,
            epoch_time: 0,
            gps_fix: false,
            latitude: 0.0,
            longitude: 0.0,
            tx_beacons: 0,
            rx_packets: 0,
            last_rssi: 0,
            last_snr: 0,
            current_page: 0,
            path_hash_mode: crate::state::RADIO_SETTINGS.lock(|r| r.borrow().path_hash_mode),
            autoadd_config: auto.config,
            autoadd_max_hops: auto.max_hops,
            rx_delay_ms: 0,
            airtime_factor_milli: 9000,
            manual_add_contacts: auto.manual_add_contacts,
            telemetry_mode_base: prefs.telemetry_mode_base,
            telemetry_mode_loc: prefs.telemetry_mode_loc,
            telemetry_mode_env: prefs.telemetry_mode_env,
            advert_loc_policy: prefs.advert_loc_policy,
            multi_acks: prefs.multi_acks,
            channels_synced: 0,
            host,
        }
    }

    /// Pull identity, callsign, battery, clock and preferences from the
    /// shared state. The app node and each transport session call this so a
    /// key import, rename or app time sync is visible everywhere.
    pub fn sync_from_shared(&mut self) {
        let callsign = crate::storage::get_callsign();
        if !callsign.is_empty() {
            self.callsign = callsign;
        }

        let public_key = crate::state::current_public_key();
        if public_key != [0u8; 32] {
            self.public_key = public_key;
            self.private_key = crate::state::current_private_key();
        }

        let mv = self.host.battery_millivolts();
        if mv >= crate::battery::MIN_PLAUSIBLE_BATTERY_MV {
            self.battery_millivolts = mv;
            self.battery_percentage = crate::battery::millivolts_to_percentage(mv);
        }

        let epoch = crate::state::get_device_time();
        if epoch != 0 {
            self.epoch_time = epoch as u64;
        }

        let (path_hash_mode, airtime_tenths) = crate::state::RADIO_SETTINGS.lock(|r| {
            let s = r.borrow();
            (s.path_hash_mode, s.airtime_factor_tenths)
        });
        self.path_hash_mode = path_hash_mode;
        self.airtime_factor_milli = airtime_tenths as u32 * 100;

        let auto = crate::state::AUTOADD_SETTINGS.lock(|a| *a.borrow());
        self.autoadd_config = auto.config;
        self.autoadd_max_hops = auto.max_hops;
        self.manual_add_contacts = auto.manual_add_contacts;

        let prefs = crate::state::prefs();
        self.telemetry_mode_base = prefs.telemetry_mode_base;
        self.telemetry_mode_loc = prefs.telemetry_mode_loc;
        self.telemetry_mode_env = prefs.telemetry_mode_env;
        self.advert_loc_policy = prefs.advert_loc_policy;
        self.multi_acks = prefs.multi_acks;
    }

    /// Determine the contact type from advert flags.
    /// The low nibble is a single kind value: 1 chat, 2 repeater, 3 room, 4 sensor.
    pub fn get_contact_type_from_flags(flags: u8) -> u8 {
        match flags & 0x0F {
            2 => ADV_TYPE_REPEATER,
            3 => ADV_TYPE_ROOM,
            4 => ADV_TYPE_SENSOR,
            _ => ADV_TYPE_CHAT,
        }
    }

    /// Check if a contact type should be auto-added based on current config
    /// Returns true if the contact type is enabled for auto-add
    pub fn should_auto_add_contact_type(&self, contact_type: u8) -> bool {
        // The shared policy is authoritative: the app may have changed it on
        // the BLE transport while this instance serves USB (or vice versa).
        let auto = crate::state::AUTOADD_SETTINGS.lock(|a| *a.borrow());
        // Bit 0 of manual_add_contacts: 0 = add every advert, 1 = only the
        // contact types enabled in the autoadd_config bitmask.
        if (auto.manual_add_contacts & 1) == 0 {
            return true;
        }
        let type_bit = match contact_type {
            ADV_TYPE_CHAT => AUTO_ADD_CHAT,
            ADV_TYPE_REPEATER => AUTO_ADD_REPEATER,
            ADV_TYPE_ROOM => AUTO_ADD_ROOM_SERVER,
            ADV_TYPE_SENSOR => AUTO_ADD_SENSOR,
            _ => return false,
        };
        (auto.config & type_bit) != 0
    }

    /// Check whether an advert's hop count is within the auto-add limit.
    pub fn autoadd_allows_hops(&self, hops: u8) -> bool {
        let max_hops = crate::state::AUTOADD_SETTINGS.lock(|a| a.borrow().max_hops);
        max_hops == 0 || hops < max_hops
    }

    /// Check if oldest non-favorite contacts should be overwritten when the list is full
    pub fn should_overwrite_when_full(&self) -> bool {
        let config = crate::state::AUTOADD_SETTINGS.lock(|a| a.borrow().config);
        (config & AUTO_ADD_OVERWRITE_OLDEST) != 0
    }

    /// Process an inbound frame payload from the companion app
    /// and generate corresponding response frames via the `emit` callback.
    pub async fn handle_command<F: FnMut(&[u8])>(&mut self, payload: &[u8], mut emit: F) {
        if payload.is_empty() {
            return;
        }

        self.sync_from_shared();

        let cmd = payload[0];
        LAST_CMD.store(cmd, Ordering::Relaxed);
        match cmd {
            CMD_DEVICE_QUERY => {
                DEVICE_QUERY_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            CMD_GET_AUTOADD_CONFIG => {
                GET_AUTOADD_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            CMD_SET_AUTOADD_CONFIG => {
                SET_AUTOADD_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
        // NOTE: this runs in `command_fut` (normal task context), never inside
        // the SoftDevice GATT callback. Logging is therefore safe here.

        match cmd {
            CMD_DEVICE_QUERY => {
                // Reset channel sync counter on each new device query (app reconnection).
                self.channels_synced = 0;
                // RESP_CODE_DEVICE_INFO (13)
                // Protocol v3+ official layout:
                // [code(13), firmware_ver(14), max_contacts_div_2(50), max_channels(8), ble_pin(u32_le), build_date(12), model(40), semver(20), offgrid(0), path_hash_mode(u8)]
                let mut resp = Vec::<u8, 128>::new();
                let _ = resp.push(RESP_CODE_DEVICE_INFO);
                let _ = resp.push(14); // firmware_ver: 14 (v10+ required for path_hash_mode in apps)
                let _ = resp.push((crate::types::MAX_CONTACTS / 2).min(255) as u8);
                let _ = resp.push(crate::types::MAX_CHANNELS as u8);
                let _ = resp.extend_from_slice(
                    &crate::state::BLE_PIN
                        .load(Ordering::Relaxed)
                        .to_le_bytes(),
                );

                let mut bdate = [0u8; 12];
                let d = b"Nov 10 2026";
                bdate[..d.len()].copy_from_slice(d);
                let _ = resp.extend_from_slice(&bdate);

                let mut mmodel = [0u8; 40];
                let m = b"Elecrow ThinkNode M1";
                mmodel[..m.len()].copy_from_slice(m);
                let _ = resp.extend_from_slice(&mmodel);

                let mut sver = [0u8; 20];
                // The app gates several settings on the firmware version
                // string, so report the MeshCore-compatible one.
                let s = b"v1.18.0";
                sver[..s.len()].copy_from_slice(s);
                let _ = resp.extend_from_slice(&sver);

                let _ = resp.push(if crate::state::offgrid_active() { 1 } else { 0 }); // offgrid mode
                let _ = resp.push(self.path_hash_mode); // path_hash_mode (0=1B, 1=2B, 2=3B)

                emit(&resp);
            }

            CMD_APP_START => {
                // RESP_CODE_SELF_INFO (5)
                // Layout:
                // [code(5), type, tx_power, max_tx_power, pubkey(32), adv_lat(i32_le), adv_lon(i32_le),
                //  reserved(0), advert_loc_policy(0), telemetry_modes(0), manual_add_contacts(0),
                //  radio_freq(u32_le_khz), radio_bw(u32_le_hz), radio_sf(u8), radio_cr(u8), name(str)]
                let mut resp = Vec::<u8, 128>::new();
                let _ = resp.push(RESP_CODE_SELF_INFO);
                let (f, tx, sf, bw, cr) = crate::state::RADIO_SETTINGS.lock(|r| {
                    let s = r.borrow();
                    (s.frequency_hz, s.tx_power_dbm, s.sf, s.bw_khz, s.cr)
                });
                let _ = resp.push(self.advert_type); // 1 = client, 2 = repeater
                let _ = resp.push(tx as u8); // current tx power
                let _ = resp.push(22); // max tx power
                let public_key = crate::state::current_public_key();
                let _ = resp.extend_from_slice(&public_key);
                let _ = resp.extend_from_slice(&0i32.to_le_bytes()); // adv_lat
                let _ = resp.extend_from_slice(&0i32.to_le_bytes()); // adv_lon
                let _ = resp.push(0); // reserved
                let _ = resp.push(self.advert_loc_policy);
                let _ = resp.push(
                    (self.telemetry_mode_env << 4)
                        | (self.telemetry_mode_loc << 2)
                        | self.telemetry_mode_base,
                );
                let _ = resp.push(self.manual_add_contacts);
                // MeshCore SELF_INFO carries frequency in kHz and bandwidth in
                // Hz (the app divides freq by 1000 to show MHz).
                let _ = resp.extend_from_slice(&(f / 1000).to_le_bytes()); // radio_freq in kHz
                let bw_hz = crate::airtime::bw_khz_to_hz(bw);
                let _ = resp.extend_from_slice(&bw_hz.to_le_bytes()); // radio_bw in Hz
                let _ = resp.push(sf);
                let _ = resp.push(cr);
                let active_name = crate::state::NODE_CALLSIGN.lock(|c| c.borrow().clone());
                let _ = resp.extend_from_slice(active_name.as_bytes());

                emit(&resp);
            }

            CMD_GET_BATT_AND_STORAGE => {
                // [PACKET_BATTERY][mV u16][storage used KiB u32][storage total KiB u32]
                let mut resp = Vec::<u8, 16>::new();
                let _ = resp.push(RESP_CODE_BATTERY);
                let _ = resp.extend_from_slice(&self.battery_millivolts.to_le_bytes());
                let _ = resp.extend_from_slice(&0u32.to_le_bytes());
                let _ = resp.extend_from_slice(&0u32.to_le_bytes());
                emit(&resp);
            }

            CMD_SET_ADVERT_NAME => {
                let mut ok = false;
                if payload.len() >= 2 {
                    let raw = &payload[1..];
                    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
                    if let Ok(text) = core::str::from_utf8(&raw[..end]) {
                        let mut name = String::<32>::new();
                        if !text.is_empty() && name.push_str(text).is_ok() {
                            self.callsign = name.clone();
                            self.host.set_callsign(text);
                            let _ = crate::command::USB_COMMAND_CHANNEL
                                .try_send(crate::command::UsbCommand::SetName(name));
                            ok = true;
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_RADIO_PARAMS => {
                let mut ok = false;
                if payload.len() >= 11 {
                    let freq_khz =
                        u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
                    let bw_hz =
                        u32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);
                    let sf = payload[9];
                    let cr = payload[10];
                    // v9+ appends the client-repeat (off-grid) flag. It is
                    // only valid on an allowed repeat band.
                    let repeat = payload.get(11).map_or(false, |&b| b != 0);
                    if (100_000..=1_000_000).contains(&freq_khz)
                        && (7_800..=500_000).contains(&bw_hz)
                        && (5..=12).contains(&sf)
                        && (5..=8).contains(&cr)
                        && (!repeat || crate::state::is_offgrid_freq_khz(freq_khz))
                    {
                        // The app re-sends its stored settings on every connect.
                        // Only touch flash and interrupt RX when something changed.
                        let (rf_changed, repeat_changed, forward_changed, forward) =
                            crate::state::RADIO_SETTINGS.lock(|r| {
                                let mut s = r.borrow_mut();
                                let old_forward = s.client_repeat
                                    && crate::state::is_offgrid_freq_hz(s.frequency_hz);
                                let new_freq = freq_khz * 1000;
                                let new_bw = bw_hz / 1000;
                                let rf_changed = s.frequency_hz != new_freq
                                    || s.bw_khz != new_bw
                                    || s.sf != sf
                                    || s.cr != cr;
                                let repeat_changed = s.client_repeat != repeat;
                                s.frequency_hz = new_freq;
                                s.bw_khz = new_bw;
                                s.sf = sf;
                                s.cr = cr;
                                s.client_repeat = repeat;
                                // The app owns the frequency, so forget the
                                // saved home frequency.
                                s.offgrid_home_freq = 0;
                                let forward = repeat
                                    && crate::state::is_offgrid_freq_khz(freq_khz);
                                (rf_changed, repeat_changed, forward != old_forward, forward)
                            });
                        if rf_changed || repeat_changed {
                            self.host.request_save_config();
                        }
                        if rf_changed {
                            self.host.request_radio_config();
                        }
                        if forward_changed {
                            let _ = crate::command::USB_COMMAND_CHANNEL.try_send(
                                crate::command::UsbCommand::SetRepeat(forward),
                            );
                        }
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_RADIO_TX_POWER => {
                let mut ok = false;
                if payload.len() >= 2 {
                    let power = payload[1] as i8;
                    if (-9..=22).contains(&power) {
                        let changed = crate::state::RADIO_SETTINGS.lock(|r| {
                            let mut s = r.borrow_mut();
                            let changed = s.tx_power_dbm != power;
                            s.tx_power_dbm = power;
                            changed
                        });
                        if changed {
                            self.host.request_save_config();
                            self.host.request_radio_config();
                        }
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_ADVERT_LATLON => {
                let mut ok = false;
                if payload.len() >= 9 {
                    let lat =
                        i32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
                    let lon =
                        i32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);
                    if (-90_000_000..=90_000_000).contains(&lat)
                        && (-180_000_000..=180_000_000).contains(&lon)
                    {
                        self.latitude = lat as f32 / 1_000_000.0;
                        self.longitude = lon as f32 / 1_000_000.0;
                        let _ = crate::command::USB_COMMAND_CHANNEL.try_send(
                            crate::command::UsbCommand::SetAdvertLatLon { lat, lon },
                        );
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_RESET_PATH => {
                let mut ok = false;
                if payload.len() >= 33 {
                    let mut pub_key = [0u8; 32];
                    pub_key.copy_from_slice(&payload[1..33]);
                    ok = crate::command::CONTACT_OP_CHANNEL
                        .try_send(crate::command::ContactOperation::ResetPath { pub_key })
                        .is_ok();
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_EXPORT_PRIVATE_KEY => {
                // [PACKET_PRIVATE_KEY][64-byte expanded private key]
                let mut resp = Vec::<u8, 66>::new();
                let _ = resp.push(RESP_CODE_PRIVATE_KEY);
                let _ = resp.extend_from_slice(&crate::state::current_private_key());
                emit(&resp);
            }

            CMD_IMPORT_PRIVATE_KEY => {
                let mut ok = false;
                if payload.len() >= 65 {
                    let mut key = [0u8; 64];
                    key.copy_from_slice(&payload[1..65]);
                    if let Some(id) = meshcore_wire::identity::LocalIdentity::from_private(
                        self.callsign.as_str(),
                        &key,
                    ) {
                        self.private_key = key;
                        self.public_key = *id.pub_key();
                        crate::state::set_identity(*id.pub_key(), key);
                        // Persist before acknowledging: otherwise the import
                        // appears to succeed but the old identity returns on
                        // the next reboot.
                        self.host.set_custom_priv_key(key);
                        let _ = crate::command::USB_COMMAND_CHANNEL
                            .try_send(crate::command::UsbCommand::SetPrivateKey(key));
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_TUNING_PARAMS => {
                let mut ok = false;
                if payload.len() >= 9 {
                    self.rx_delay_ms =
                        u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
                    let af_milli =
                        u32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);
                    let af_tenths = ((af_milli / 100) as u8).min(90);
                    let changed = crate::state::RADIO_SETTINGS.lock(|r| {
                        let mut s = r.borrow_mut();
                        let changed = s.airtime_factor_tenths != af_tenths;
                        s.airtime_factor_tenths = af_tenths;
                        changed
                    });
                    self.airtime_factor_milli = af_tenths as u32 * 100;
                    if changed {
                        self.host.request_save_config();
                    }
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_TUNING_PARAMS => {
                let mut resp = Vec::<u8, 9>::new();
                let _ = resp.push(RESP_CODE_TUNING);
                let _ = resp.extend_from_slice(&self.rx_delay_ms.to_le_bytes());
                let _ = resp.extend_from_slice(&self.airtime_factor_milli.to_le_bytes());
                emit(&resp);
            }

            CMD_SET_AUTOADD_CONFIG => {
                let mut ok = false;
                if payload.len() >= 2 {
                    let config = payload[1];
                    let max_hops = payload.get(2).map_or(self.autoadd_max_hops, |&b| b.min(64));
                    let changed = config != self.autoadd_config || max_hops != self.autoadd_max_hops;
                    self.autoadd_config = config;
                    self.autoadd_max_hops = max_hops;
                    crate::state::AUTOADD_SETTINGS.lock(|a| {
                        let mut st = a.borrow_mut();
                        st.config = config;
                        st.max_hops = max_hops;
                    });
                    if changed {
                        self.host.request_save_config();
                    }
                    let _ = crate::command::USB_COMMAND_CHANNEL.try_send(
                        crate::command::UsbCommand::SetAutoAddConfig {
                            config: self.autoadd_config,
                            max_hops: self.autoadd_max_hops,
                        },
                    );
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_AUTOADD_CONFIG => {
                let mut resp = Vec::<u8, 3>::new();
                let _ = resp.push(RESP_CODE_AUTOADD);
                let _ = resp.push(self.autoadd_config);
                let _ = resp.push(self.autoadd_max_hops);
                emit(&resp);
            }

            CMD_GET_ADVERT_PATH => {
                let mut found = None;
                if payload.len() >= 9 {
                    let prefix = &payload[2..9];
                    found = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| &ct.pub_key[..7] == prefix)
                            .map(|ct| (ct.out_path_len, ct.out_path, ct.last_advert_ts))
                    });
                }
                if let Some((path_len, path, ts)) = found {
                    if path_len != 0xFF
                        && meshcore_wire::packet::Packet::is_valid_path_len(path_len)
                    {
                        let n = (path_len & 63) as usize * (((path_len >> 6) as usize) + 1);
                        let mut resp = Vec::<u8, 72>::new();
                        let _ = resp.push(RESP_CODE_ADVERT_PATH);
                        let _ = resp.extend_from_slice(&ts.to_le_bytes());
                        let _ = resp.push(path_len);
                        let _ = resp.extend_from_slice(&path[..n]);
                        emit(&resp);
                    } else {
                        let mut resp = Vec::<u8, 4>::new();
                        let _ = resp.push(RESP_CODE_ERR);
                        emit(&resp);
                    }
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    emit(&resp);
                }
            }

            CMD_GET_STATS => {
                let mut handled = false;
                if payload.len() >= 2 {
                    let kind = payload[1];
                    if kind == 0 {
                        let uptime = embassy_time::Instant::now().as_secs() as u32;
                        let mut resp = Vec::<u8, 11>::new();
                        let _ = resp.push(RESP_CODE_STATS);
                        let _ = resp.push(0);
                        let _ = resp.extend_from_slice(&self.battery_millivolts.to_le_bytes());
                        let _ = resp.extend_from_slice(&uptime.to_le_bytes());
                        let _ = resp.extend_from_slice(&0u16.to_le_bytes());
                        let _ = resp.push(0);
                        emit(&resp);
                        handled = true;
                    } else if kind == 1 {
                        let (rssi, snr, tx_air_s, rx_air_s) =
                            crate::state::RADIO_TELEMETRY.lock(|t| {
                                let tel = t.borrow();
                                (
                                    tel.last_rssi,
                                    tel.last_snr,
                                    tel.tx_air_ms / 1000,
                                    tel.rx_air_ms / 1000,
                                )
                            });
                        let mut resp = Vec::<u8, 14>::new();
                        let _ = resp.push(RESP_CODE_STATS);
                        let _ = resp.push(1);
                        let _ = resp.extend_from_slice(&(-118i16).to_le_bytes());
                        let _ = resp.push(rssi as i8 as u8);
                        let _ = resp.push((snr * 4) as u8);
                        let _ = resp.extend_from_slice(&tx_air_s.to_le_bytes());
                        let _ = resp.extend_from_slice(&rx_air_s.to_le_bytes());
                        emit(&resp);
                        handled = true;
                    }
                }
                if !handled {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    emit(&resp);
                }
            }

            CMD_REBOOT => {
                if payload.len() >= 7 && &payload[1..7] == b"reboot" {
                    // The app reconnects after a reboot; make sure pending
                    // contacts/blobs/config are durable before resetting.
                    let _ = crate::storage::flush_pending().await;
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_OK);
                    emit(&resp);
                    self.host.reboot();
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    emit(&resp);
                }
            }

            CMD_GET_DEVICE_TIME => {
                // RESP_CODE_CURR_TIME (9)
                // Payload: [RESP_CODE(9), epoch_seconds_u32_le(4)]
                let mut resp = Vec::<u8, 8>::new();
                let _ = resp.push(RESP_CODE_CURR_TIME);
                let epoch = crate::state::get_device_time();
                let _ = resp.extend_from_slice(&epoch.to_le_bytes());
                emit(&resp);
            }

            CMD_SET_DEVICE_TIME => {
                // Payload from app: [CMD(6), epoch_seconds_u32_le(4)]
                if payload.len() >= 5 {
                    let mut time_bytes = [0u8; 4];
                    time_bytes.copy_from_slice(&payload[1..5]);
                    let epoch = u32::from_le_bytes(time_bytes);
                    self.epoch_time = epoch as u64;
                    crate::state::set_device_time(epoch);
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_GET_CHANNEL => {
                // Inbound: [CMD(31 / 0x1F), channel_idx]
                // The app walks indices upward until an error, so out-of-range
                // requests must fail instead of returning empty channels.
                if payload.len() < 2
                    || payload[1] >= crate::types::MAX_CHANNELS as u8
                {
                    let mut resp = Vec::<u8, 2>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                    emit(&resp);
                } else {
                    let channel_idx = payload[1];
                    self.channels_synced = self.channels_synced.saturating_add(1);
                    let mut resp = Vec::<u8, 64>::new();
                    let _ = resp.push(RESP_CODE_CHANNEL_INFO); // 18 (0x12)
                    let _ = resp.push(channel_idx);

                    crate::state::CHANNELS.lock(|c| {
                        let chs = c.borrow();
                        if (channel_idx as usize) < chs.len() {
                            let ch = &chs[channel_idx as usize];
                            let _ = resp.extend_from_slice(&ch.name);
                            let _ = resp.extend_from_slice(&ch.secret);
                        } else {
                            let empty_name = [0u8; 32];
                            let empty_secret = [0u8; 16];
                            let _ = resp.extend_from_slice(&empty_name);
                            let _ = resp.extend_from_slice(&empty_secret);
                        }
                    });

                    emit(&resp);
                }
            }

            CMD_SET_CHANNEL => {
                // Inbound: [CMD(32 / 0x20), channel_idx, name(32 bytes), secret(16 bytes)]
                if payload.len() >= 50 {
                    let channel_idx = payload[1] as usize;
                    if channel_idx < crate::types::MAX_CHANNELS {
                        let mut name = [0u8; 32];
                        name.copy_from_slice(&payload[2..34]);
                        let mut secret = [0u8; 16];
                        secret.copy_from_slice(&payload[34..50]);
                        let changed = crate::state::CHANNELS.lock(|c| {
                            let mut chs = c.borrow_mut();
                            let old = &chs[channel_idx];
                            let changed = old.name != name || old.secret != secret;
                            chs[channel_idx] =
                                crate::types::ChannelData { name, secret };
                            changed
                        });
                        if changed {
                            crate::storage::config::mark_channel_dirty(channel_idx);
                            self.host.request_save_config();
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_SYNC_NEXT_MESSAGE => {
                let next_item = crate::state::MESSAGE_LOG.lock(|m| {
                    let mut list = m.borrow_mut();
                    if !list.is_empty() {
                        Some(list.remove(0))
                    } else {
                        None
                    }
                });

                if let Some(item) = next_item {
                    let mut resp = Vec::<u8, 256>::new();
                    match item {
                        crate::types::InboxItem::Text(msg) => {
                            // Pop the record from flash too, so a reboot does
                            // not resurrect a message the app already received.
                            self.host.queue_consume_now();
                            if msg.is_direct {
                                // V3 DM: [0x10, (snr*4), 0, 0, 6B pubkey, path_len, txt_type, 4B ts, text]
                                let _ = resp.push(RESP_CODE_CONTACT_MSG_V3);
                                let _ = resp.push((msg.snr * 4) as u8);
                                let _ = resp.push(0);
                                let _ = resp.push(0);
                                let _ = resp.extend_from_slice(&msg.sender_pub[..6]);
                                let _ = resp.push(msg.path_len);
                                let _ = resp.push(msg.txt_type);
                                let _ = resp.extend_from_slice(&msg.timestamp.to_le_bytes());
                                // Signed messages carry the signing-key prefix
                                // between the timestamp and the text.
                                if msg.txt_type
                                    == meshcore_wire::packets::txtmsg::TXT_TYPE_SIGNED_PLAIN
                                {
                                    let _ = resp.extend_from_slice(&msg.signed_prefix);
                                }
                                let _ = resp.extend_from_slice(msg.text.as_bytes());
                            } else {
                                // V3 Channel: [0x11, (snr*4), 0, 0, channel_idx, path_len, txt_type, 4B ts, text]
                                let _ = resp.push(RESP_CODE_CHANNEL_MSG_V3);
                                let _ = resp.push((msg.snr * 4) as u8);
                                let _ = resp.push(0);
                                let _ = resp.push(0);
                                let _ = resp.push(msg.channel_idx);
                                let _ = resp.push(msg.path_len);
                                let _ = resp.push(0); // TXT_TYPE_PLAIN
                                let _ = resp.extend_from_slice(&msg.timestamp.to_le_bytes());
                                let _ = resp.extend_from_slice(msg.sender.as_bytes());
                                let _ = resp.extend_from_slice(b": ");
                                let _ = resp.extend_from_slice(msg.text.as_bytes());
                            }
                        }
                        crate::types::InboxItem::ChannelData(data) => {
                            // Datagrams are persisted too: pop the record once
                            // the app has taken it.
                            self.host.queue_consume_now();
                            // [0x1B, (snr*4), 0, 0, channel_idx, path_len,
                            //  data_type u16 LE, data_len, data...]
                            let _ = resp.push(PACKET_CHANNEL_DATA_RECV);
                            let _ = resp.push((data.snr * 4) as u8);
                            let _ = resp.push(0);
                            let _ = resp.push(0);
                            let _ = resp.push(data.channel_idx);
                            let _ = resp.push(data.path_len);
                            let _ = resp.extend_from_slice(&data.data_type.to_le_bytes());
                            let _ = resp.push(data.data.len() as u8);
                            let _ = resp.extend_from_slice(&data.data);
                        }
                    }
                    emit(&resp);
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_NO_MORE_MESSAGES);
                    emit(&resp);
                }
            }

            CMD_GET_CONTACTS => {
                // The transports handle this with async streaming: with up to
                // 350 contacts, the start + N contacts + end burst does not fit
                // the bounded push channel. See `ble::stream_contacts` and
                // `usb::stream_contacts_usb`.
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_ERR);
                emit(&resp);
            }

            CMD_SEND_CHANNEL_MSG => {
                // [CMD(3), txt_type, channel_idx, timestamp u32 LE, text...]
                let mut err = None;
                if payload.len() < 8 {
                    err = Some(ERR_ILLEGAL_ARG);
                } else {
                    let txt_type = payload[1];
                    let channel_idx = payload[2];
                    let timestamp = u32::from_le_bytes([
                        payload[3],
                        payload[4],
                        payload[5],
                        payload[6],
                    ]);
                    let text_bytes = &payload[7..];
                    let channel_ok = crate::state::CHANNELS.lock(|c| {
                        c.borrow()
                            .get(channel_idx as usize)
                            .map_or(false, |ch| ch.secret != [0u8; 16])
                    });
                    if txt_type != 0 {
                        // Only plain channel text is accepted from companions.
                        err = Some(ERR_UNSUPPORTED);
                    } else if !channel_ok {
                        err = Some(ERR_NOT_FOUND);
                    } else if let Ok(msg_str) = core::str::from_utf8(text_bytes) {
                        let msg_str = msg_str.trim_end_matches('\0');
                        let n = msg_str.len().min(160);
                        let n = msg_str.floor_char_boundary(n);
                        let mut text = String::<160>::new();
                        if text.push_str(&msg_str[..n]).is_err()
                            || crate::command::USB_COMMAND_CHANNEL
                                .try_send(crate::command::UsbCommand::SendGroupText {
                                    channel_idx,
                                    timestamp,
                                    text,
                                })
                                .is_err()
                        {
                            // Destination task queue full / internal state.
                            err = Some(ERR_BAD_STATE);
                        }
                    } else {
                        err = Some(ERR_ILLEGAL_ARG);
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                match err {
                    None => {
                        let _ = resp.push(RESP_CODE_OK);
                    }
                    Some(code) => {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(code);
                    }
                }
                emit(&resp);
            }

            CMD_SEND_ANON_REQ => {
                // [cmd][public_key 32][anon data...]
                let mut resp = Vec::<u8, 10>::new();
                let mut reply = None;
                if payload.len() >= 34 {
                    let mut dest_key = [0u8; 32];
                    dest_key.copy_from_slice(&payload[1..33]);
                    let data = &payload[33..];
                    let contact = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| ct.pub_key == dest_key)
                            .map(|ct| (ct.out_path_len, ct.out_path))
                    });
                    // Unknown contacts use a zero-hop direct anon request.
                    let (out_path_len, out_path) =
                        contact.unwrap_or((0, [0u8; 64]));
                    let identity = meshcore_wire::identity::LocalIdentity::from_private(
                        self.callsign.as_str(),
                        &self.private_key,
                    );
                    if let Some(id) = identity {
                        if let Some(secret) = id.shared_secret(&dest_key) {
                            let tag = next_tag();
                            let mut plain = heapless::Vec::<u8, 144>::new();
                            let _ = plain.extend_from_slice(&tag.to_le_bytes());
                            let _ = plain.extend_from_slice(data);
                            if let Ok(anon) =
                                AnonReqPacket::new(dest_key[0], self.public_key, &plain)
                            {
                                let direct = out_path_len != 0xFF
                                    && Packet::is_valid_path_len(out_path_len);
                                let route = if direct {
                                    ROUTE_TYPE_DIRECT
                                } else {
                                    ROUTE_TYPE_FLOOD
                                };
                                if let Ok(mut pkt) = anon.to_packet(&secret, route) {
                                    if direct {
                                        pkt.path_len = out_path_len;
                                        let pn = pkt.path_byte_len();
                                        pkt.path[..pn].copy_from_slice(&out_path[..pn]);
                                    } else {
                                        apply_flood_scope(&mut pkt);
                                    }
                                    let mut buf = [0u8; 255];
                                    if let Ok(n) = pkt.write_to(&mut buf) {
                                        let mut raw = heapless::Vec::<u8, 255>::new();
                                        if raw.extend_from_slice(&buf[..n]).is_ok()
                                            && crate::command::USB_COMMAND_CHANNEL
                                                .try_send(
                                                    crate::command::UsbCommand::SendRawPacket {
                                                        raw,
                                                    },
                                                )
                                                .is_ok()
                                        {
                                            let (sf, bw_khz, cr) =
                                                crate::state::RADIO_SETTINGS.lock(|r| {
                                                    let s = r.borrow();
                                                    (s.sf, s.bw_khz, s.cr)
                                                });
                                            let air_ms = crate::airtime::estimate_airtime_ms(
                                                n,
                                                sf,
                                                bw_khz,
                                                cr,
                                                if sf <= 8 { 32 } else { 16 },
                                            );
                                            let hops = if direct {
                                                (out_path_len & 63) as u32
                                            } else {
                                                0
                                            };
                                            let est_timeout = if direct {
                                                500 + (air_ms * 6 + 250) * (hops + 1)
                                            } else {
                                                500 + 16 * air_ms
                                            };
                                            note_pending_req(tag, dest_key);
                                            reply = Some((direct, tag, est_timeout));
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                match reply {
                    Some((direct, tag, est_timeout)) => {
                        let _ = resp.push(RESP_CODE_SENT);
                        let _ = resp.push(if direct { 0 } else { 1 });
                        let _ = resp.extend_from_slice(&tag.to_le_bytes());
                        let _ = resp.extend_from_slice(&est_timeout.to_le_bytes());
                    }
                    None => {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_ILLEGAL_ARG);
                    }
                }
                emit(&resp);
            }

            CMD_SEND_CHANNEL_DATA => {
                // [CMD(0x3E), channel_idx, path_len, path..., data_type u16 LE, data...]
                let mut ok = false;
                if payload.len() >= 5 {
                    let channel_idx = payload[1];
                    let path_len = payload[2];
                    let path_bytes = if path_len == 0xFF {
                        0
                    } else if Packet::is_valid_path_len(path_len) {
                        (path_len & 63) as usize * (((path_len >> 6) as usize) + 1)
                    } else {
                        usize::MAX
                    };
                    if path_bytes != usize::MAX && payload.len() >= 3 + path_bytes + 2 {
                        let mut path = [0u8; 64];
                        if path_bytes > 0 {
                            path[..path_bytes]
                                .copy_from_slice(&payload[3..3 + path_bytes]);
                        }
                        let type_offset = 3 + path_bytes;
                        let data_type = u16::from_le_bytes([
                            payload[type_offset],
                            payload[type_offset + 1],
                        ]);
                        let raw = &payload[type_offset + 2..];
                        let channel_ok = crate::state::CHANNELS.lock(|c| {
                            c.borrow()
                                .get(channel_idx as usize)
                                .map_or(false, |ch| ch.secret != [0u8; 16])
                        });
                        // Data type 0 is reserved; the radio limit is 165 bytes.
                        if channel_ok && data_type != 0 && raw.len() <= 165 {
                            let mut data = Vec::<u8, 165>::new();
                            if data.extend_from_slice(raw).is_ok() {
                                ok = crate::command::USB_COMMAND_CHANNEL
                                    .try_send(
                                        crate::command::UsbCommand::SendChannelData {
                                            channel_idx,
                                            path_len,
                                            path,
                                            data_type,
                                            data,
                                        },
                                    )
                                    .is_ok();
                            }
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SEND_MSG => {
                // [cmd][txt_type][attempt][timestamp u32 LE][pubkey_prefix 6][text...]
                // Minimum: 1 + 1 + 1 + 4 + 6 + 1 = 14 bytes.
                //
                // Reply with RESP_CODE_SENT so the app can arm its ACK-based
                // retry timer: [is_flood][expected_ack:4][est_timeout:4].
                // Retries themselves are app-driven; the node sends one packet
                // per request, matching MeshCore.
                let mut resp = Vec::<u8, 10>::new();
                let mut reply = None;
                if payload.len() >= 14 {
                    let txt_type = payload[1];
                    let attempt = payload[2];
                    let timestamp = u32::from_le_bytes([
                        payload[3], payload[4], payload[5], payload[6],
                    ]);
                    let mut dest_prefix = [0u8; 6];
                    dest_prefix.copy_from_slice(&payload[7..13]);
                    if let Ok(text_str) = core::str::from_utf8(&payload[13..]) {
                        let text_str = text_str.trim_end_matches('\0');
                        let path = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                            c.borrow()
                                .iter()
                                .find(|ct| &ct.pub_key[..6] == &dest_prefix[..])
                                .map(|ct| ct.out_path_len)
                        });
                        if let Some(out_path_len) = path {
                            let mut s = String::<160>::new();
                            if s.push_str(text_str).is_ok()
                                && crate::command::USB_COMMAND_CHANNEL
                                    .try_send(
                                        crate::command::UsbCommand::SendDirectText {
                                            dest_prefix,
                                            timestamp,
                                            txt_type,
                                            attempt,
                                            text: s,
                                        },
                                    )
                                    .is_ok()
                            {
                                reply = Some((txt_type, attempt, timestamp, text_str, out_path_len));
                            }
                        }
                    }
                }

                if let Some((txt_type, attempt, timestamp, text, out_path_len)) = reply {
                    let is_direct = out_path_len != 0xFF
                        && Packet::is_valid_path_len(out_path_len);
                    let hops = if is_direct {
                        (out_path_len & 63) as u32
                    } else {
                        0
                    };
                    let flags = ((txt_type & 0x3F) << 2) | (attempt & 0x03);
                    let expected_ack = meshcore_wire::crypto::ack_crc(
                        timestamp,
                        flags,
                        text.as_bytes(),
                        &self.public_key,
                    );

                    // Estimated on-air time of the complete packet.
                    let plain_len = 5 + text.len() + if attempt > 3 { 2 } else { 0 };
                    let path_bytes = if is_direct {
                        (out_path_len & 63) as usize
                            * (((out_path_len >> 6) & 3) as usize + 1)
                    } else {
                        0
                    };
                    let raw_len = 2 + path_bytes + 4 + plain_len;
                    let (sf, bw_khz, cr) =
                        crate::state::RADIO_SETTINGS.lock(|r| {
                            let s = r.borrow();
                            (s.sf, s.bw_khz, s.cr)
                        });
                    let air_ms = crate::airtime::estimate_airtime_ms(
                        raw_len,
                        sf,
                        bw_khz,
                        cr,
                        if sf <= 8 { 32 } else { 16 },
                    );
                    let est_timeout = if is_direct {
                        500 + (air_ms * 6 + 250) * (hops + 1)
                    } else {
                        500 + 16 * air_ms
                    };

                    let _ = resp.push(RESP_CODE_SENT);
                    let _ = resp.push(if is_direct { 0 } else { 1 });
                    let _ = resp.extend_from_slice(&expected_ack.to_le_bytes());
                    let _ = resp.extend_from_slice(&est_timeout.to_le_bytes());
                } else {
                    let _ = resp.push(RESP_CODE_ERR);
                }
                emit(&resp);
            }

            CMD_SET_PATH_HASH_MODE => {
                // Inbound: [0x3D, 0, mode] or [0x3D, mode]
                let mode_opt = if payload.len() >= 3 && payload[1] == 0 {
                    Some(payload[2])
                } else if payload.len() >= 2 {
                    Some(payload[1])
                } else {
                    None
                };

                if let Some(mode) = mode_opt {
                    if mode <= 2 {
                        self.path_hash_mode = mode;
                        self.host.set_path_hash_mode(mode);
                        let _ = crate::command::USB_COMMAND_CHANNEL.try_send(crate::command::UsbCommand::SetPathHashMode(mode));
                        let mut resp = Vec::<u8, 4>::new();
                        let _ = resp.push(RESP_CODE_OK);
                        emit(&resp);
                        return;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_ERR);
                emit(&resp);
            }

            CMD_SEND_SELF_ADVERT => {
                let flood = payload.len() >= 2 && payload[1] == 1;
                let _ = crate::command::USB_COMMAND_CHANNEL
                    .try_send(crate::command::UsbCommand::TriggerAdvert { flood });
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_ADD_UPDATE_CONTACT => {
                // Inbound: [CMD(9), public_key(32), type(1), flags(1), out_path_len(1), out_path(64), adv_name(32), last_advert(4)]
                // Optional: gps_lat(4), gps_lon(4), lastmod(4)
                // Minimum required: 1 + 32 + 1 + 1 + 1 + 64 + 32 + 4 = 136 bytes (matches MeshCore)
                //
                // This handler runs in `command_fut` (normal task context), but contact
                // storage is still deferred to `contact_task` via CONTACT_OP_CHANNEL so
                // a slow or full contact list cannot back up the BLE task.
                if payload.len() >= 1 + 32 + 1 + 1 + 1 + 64 + 32 + 4 {
                    let mut i = 1;
                    let pub_key: [u8; 32] = payload[i..i+32].try_into().unwrap(); i += 32;
                    let contact_type = payload[i]; i += 1;
                    let flags = payload[i]; i += 1;
                    let out_path_len = payload[i]; i += 1;
                    let mut out_path = [0u8; 64];
                    out_path.copy_from_slice(&payload[i..i+64]); i += 64;
                    let adv_name_bytes = &payload[i..i+32]; i += 32;
                    let adv_name = core::str::from_utf8(adv_name_bytes).unwrap_or("").trim_end_matches('\0');
                    let last_advert = u32::from_le_bytes([payload[i], payload[i+1], payload[i+2], payload[i+3]]); i += 4;

                    // Optional fields after the required 136 bytes
                    let (adv_lat, adv_lon, lastmod) = if payload.len() >= i + 8 {
                        let lat = i32::from_le_bytes([payload[i], payload[i+1], payload[i+2], payload[i+3]]); i += 4;
                        let lon = i32::from_le_bytes([payload[i], payload[i+1], payload[i+2], payload[i+3]]); i += 4;
                        if payload.len() >= i + 4 {
                            let lm = u32::from_le_bytes([payload[i], payload[i+1], payload[i+2], payload[i+3]]);
                            (lat, lon, lm)
                        } else {
                            let lm = last_advert;
                            (lat, lon, lm)
                        }
                    } else {
                        (0, 0, last_advert)
                    };

                    let node_id = [pub_key[0], pub_key[1], pub_key[2], pub_key[3]];

                    let mut callsign = heapless::String::new();
                    let _ = callsign.push_str(adv_name);

                    // Defer the actual contact storage to the background task to avoid
                    // locking DISCOVERED_CONTACTS inside the SoftDevice critical section.
                    // If the channel is full, we drop the operation — this is acceptable
                    // since the app can retry.
                    let _ = crate::command::CONTACT_OP_CHANNEL.try_send(
                        crate::command::ContactOperation::AddOrUpdate {
                            pub_key,
                            callsign,
                            node_id,
                            last_seen_epoch: lastmod,
                            latitude: adv_lat,
                            longitude: adv_lon,
                            flags,
                            contact_type,
                            out_path,
                            out_path_len,
                            last_advert_ts: last_advert,
                        }
                    );

                    // Return OK immediately — the actual storage happens asynchronously.
                    // The companion also returns OK before the contact is fully persisted.
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_OK);
                    emit(&resp);
                    return;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_ERR);
                emit(&resp);
            }

            CMD_REMOVE_CONTACT => {
                // Inbound: [CMD(15), public_key(32)]
                // Defer the actual removal to `contact_task`, like AddOrUpdate.
                // Report NOT_FOUND synchronously for an unknown key, as the
                // reference does.
                let mut resp = Vec::<u8, 2>::new();
                if payload.len() >= 33 {
                    let pub_key: [u8; 32] = payload[1..33].try_into().unwrap();
                    let found = crate::state::DISCOVERED_CONTACTS
                        .lock(|c| c.borrow().iter().any(|ct| ct.pub_key == pub_key));
                    if !found {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    } else if crate::command::CONTACT_OP_CHANNEL
                        .try_send(crate::command::ContactOperation::Remove { pub_key })
                        .is_ok()
                    {
                        let _ = resp.push(RESP_CODE_OK);
                    } else {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_BAD_STATE);
                    }
                } else {
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                }
                emit(&resp);
            }

            CMD_SHARE_CONTACT => {
                // Inbound: [CMD(0x10), public_key(32)]
                // Re-broadcast the stored advert as a zero-hop transport-direct
                // advert so nearby nodes can add the contact.
                let mut resp = Vec::<u8, 2>::new();
                if payload.len() >= 33 {
                    let mut key = [0u8; 32];
                    key.copy_from_slice(&payload[1..33]);
                    let known = crate::state::DISCOVERED_CONTACTS
                        .lock(|c| c.borrow().iter().any(|ct| ct.pub_key == key));
                    if !known {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    } else if let Some(advert) =
                        crate::storage::adverts::request_payload(&key).await
                    {
                        let mut pkt = Packet::new(
                            meshcore_wire::packet::ROUTE_TYPE_TRANSPORT_DIRECT,
                            PAYLOAD_TYPE_ADVERT,
                        );
                        pkt.transport_codes = [0, 0];
                        pkt.path_len = 0;
                        let mut raw = heapless::Vec::<u8, 255>::new();
                        let mut buf = [0u8; 255];
                        let sent = if pkt.set_payload(&advert).is_ok() {
                            if let Ok(n) = pkt.write_to(&mut buf) {
                                raw.extend_from_slice(&buf[..n]).is_ok()
                                    && crate::command::USB_COMMAND_CHANNEL
                                        .try_send(crate::command::UsbCommand::SendRawPacket {
                                            raw,
                                        })
                                        .is_ok()
                            } else {
                                false
                            }
                        } else {
                            false
                        };
                        if sent {
                            let _ = resp.push(RESP_CODE_OK);
                        } else {
                            let _ = resp.push(RESP_CODE_ERR);
                            let _ = resp.push(ERR_BAD_STATE);
                        }
                    } else {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_BAD_STATE);
                    }
                } else {
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                }
                emit(&resp);
            }

            CMD_GET_CONTACT_BY_KEY => {
                let found = if payload.len() >= 33 {
                    let mut key = [0u8; 32];
                    key.copy_from_slice(&payload[1..33]);
                    crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow().iter().find(|ct| ct.pub_key == key).cloned()
                    })
                } else {
                    None
                };
                match found {
                    Some(ct) => emit(&serialize_contact(&ct)),
                    None => {
                        let mut resp = Vec::<u8, 4>::new();
                        let _ = resp.push(RESP_CODE_ERR);
                        emit(&resp);
                    }
                }
            }

            CMD_SIGN_START => {
                SIGN_ACTIVE.store(true, Ordering::Relaxed);
                SIGN_BUFFER.lock(|b| b.borrow_mut().clear());
                let mut resp = Vec::<u8, 6>::new();
                let _ = resp.push(RESP_CODE_SIGN_START);
                let _ = resp.push(0);
                let _ = resp.extend_from_slice(&(MAX_SIGN_DATA as u32).to_le_bytes());
                emit(&resp);
            }

            CMD_SIGN_DATA => {
                let mut ok = false;
                if payload.len() > 1 && SIGN_ACTIVE.load(Ordering::Relaxed) {
                    ok = SIGN_BUFFER.lock(|b| {
                        b.borrow_mut().extend_from_slice(&payload[1..]).is_ok()
                    });
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SIGN_FINISH => {
                let mut signature = None;
                if SIGN_ACTIVE.swap(false, Ordering::Relaxed) {
                    SIGN_BUFFER.lock(|b| {
                        let data = b.borrow();
                        if !data.is_empty() {
                            let private_key = crate::state::current_private_key();
                            if let Some(id) = meshcore_wire::identity::LocalIdentity::from_private(
                                self.callsign.as_str(),
                                &private_key,
                            ) {
                                signature = Some(id.sign(&data));
                            }
                        }
                    });
                    SIGN_BUFFER.lock(|b| b.borrow_mut().clear());
                }
                match signature {
                    Some(sig) => {
                        let mut resp = Vec::<u8, 65>::new();
                        let _ = resp.push(RESP_CODE_SIGNATURE);
                        let _ = resp.extend_from_slice(&sig);
                        emit(&resp);
                    }
                    None => {
                        let mut resp = Vec::<u8, 4>::new();
                        let _ = resp.push(RESP_CODE_ERR);
                        emit(&resp);
                    }
                }
            }

            CMD_EXPORT_CONTACT => {
                // No key: export self as a fresh signed advert. With a key:
                // return the signed advert blob retained when the contact was
                // discovered, as MeshCore does.
                if payload.len() < 33 {
                    let private_key = crate::state::current_private_key();
                    let id = meshcore_wire::identity::LocalIdentity::from_private(
                        self.callsign.as_str(),
                        &private_key,
                    );
                    match id {
                        Some(id) => {
                            let ts = crate::state::get_device_time();
                            let (adv_lat, adv_lon) =
                                advert_location(self.latitude, self.longitude);
                            let lat = (adv_lat * 1_000_000.0) as i32;
                            let lon = (adv_lon * 1_000_000.0) as i32;
                            let mut adv = AdvertPacket::new(
                                *id.pub_key(),
                                ts,
                                meshcore_wire::packets::advert::ADV_TYPE_CHAT,
                                lat,
                                lon,
                                self.callsign.as_str(),
                            );
                            adv.sign(&id);
                            if let Ok(pkt) =
                                adv.to_packet(meshcore_wire::packet::ROUTE_TYPE_FLOOD)
                            {
                                let mut raw = [0u8; 184];
                                if let Ok(n) = pkt.write_to(&mut raw) {
                                    let mut resp = Vec::<u8, 185>::new();
                                    let _ = resp.push(RESP_CODE_EXPORT_CONTACT);
                                    let _ = resp.extend_from_slice(&raw[..n]);
                                    emit(&resp);
                                }
                            }
                        }
                        None => {
                            let mut resp = Vec::<u8, 4>::new();
                            let _ = resp.push(RESP_CODE_ERR);
                            emit(&resp);
                        }
                    }
                } else {
                    let mut key = [0u8; 32];
                    key.copy_from_slice(&payload[1..33]);
                    let known = crate::state::DISCOVERED_CONTACTS
                        .lock(|c| c.borrow().iter().any(|ct| ct.pub_key == key));
                    let mut resp = Vec::<u8, 140>::new();
                    if !known {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    } else if let Some(advert) =
                        crate::storage::adverts::request_payload(&key).await
                    {
                        // [0x11, 0x00, signed advert payload]: a flood-routed
                        // advert with no path, matching MeshCore's export.
                        let _ = resp.push(RESP_CODE_EXPORT_CONTACT);
                        let _ = resp.push(0x11);
                        let _ = resp.push(0x00);
                        let _ = resp.extend_from_slice(&advert);
                    } else {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_BAD_STATE);
                    }
                    emit(&resp);
                }
            }

            CMD_IMPORT_CONTACT => {
                let mut ok = false;
                if payload.len() > 1 {
                    if let Ok(pkt) = Packet::read_from(&payload[1..]) {
                        if pkt.payload_type() == PAYLOAD_TYPE_ADVERT {
                            if let Ok(adv) = AdvertPacket::decode(&pkt.payload[..pkt.payload_len]) {
                                if adv.verify_signature() {
                                    let mut callsign = String::<32>::new();
                                    let _ = callsign.push_str(adv.name.as_str());
                                    let mut out_path = [0u8; 64];
                                    let plen = pkt.path_byte_len();
                                    out_path[..plen].copy_from_slice(&pkt.path[..plen]);
                                    let node_id = [
                                        adv.pub_key[0],
                                        adv.pub_key[1],
                                        adv.pub_key[2],
                                        adv.pub_key[3],
                                    ];
                                    let last_seen_epoch =
                                        crate::state::get_device_time();
                                    let contact_type =
                                        CompanionNode::get_contact_type_from_flags(adv.flags);

                                    // The reference loops the imported advert back
                                    // through the receive path, which also emits the
                                    // discovery push the app expects.
                                    let existing = crate::state::DISCOVERED_CONTACTS
                                        .lock(|c| {
                                            c.borrow()
                                                .iter()
                                                .find(|ct| ct.pub_key == adv.pub_key)
                                                .map(|ct| ct.last_advert_ts)
                                        });
                                    if existing.map_or(true, |ts| adv.timestamp > ts) {
                                        let mut frame = Vec::<u8, 256>::new();
                                        if existing.is_some() {
                                            let _ = frame.push(PUSH_CODE_ADVERT);
                                            let _ = frame.extend_from_slice(&adv.pub_key);
                                        } else {
                                            let discovered =
                                                crate::types::DiscoveredContact {
                                                    pub_key: adv.pub_key,
                                                    callsign: callsign.clone(),
                                                    node_id,
                                                    last_seen_epoch,
                                                    rssi: 0,
                                                    snr: 0,
                                                    latitude: adv.latitude,
                                                    longitude: adv.longitude,
                                                    flags: 0,
                                                    contact_type,
                                                    out_path,
                                                    out_path_len: pkt.path_len,
                                                    last_advert_ts: adv.timestamp,
                                                    saved_hash: 0,
                            sync_since: 0,
                                                };
                                            let record = serialize_contact(&discovered);
                                            let _ = frame.push(PUSH_CODE_NEW_ADVERT);
                                            let _ = frame.extend_from_slice(&record[1..]);
                                        }
                                        crate::command::push_companion(&frame);
                                        self.host.advert_remember(
                                            &adv.pub_key,
                                            &pkt.payload[..pkt.payload_len],
                                        );
                                    }

                                    ok = crate::command::CONTACT_OP_CHANNEL
                                        .try_send(crate::command::ContactOperation::AddOrUpdate {
                                            pub_key: adv.pub_key,
                                            callsign,
                                            node_id,
                                            last_seen_epoch,
                                            latitude: adv.latitude,
                                            longitude: adv.longitude,
                                            // Imported adverts carry no contact flags.
                                            flags: 0,
                                            contact_type,
                                            out_path,
                                            out_path_len: pkt.path_len,
                                            last_advert_ts: adv.timestamp,
                                        })
                                        .is_ok();
                                }
                            }
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_OTHER_PARAMS => {
                let mut ok = false;
                if payload.len() >= 2 {
                    let manual = payload[1];
                    let auto_changed = manual != self.manual_add_contacts;
                    self.manual_add_contacts = manual;
                    crate::state::AUTOADD_SETTINGS.lock(|a| {
                        a.borrow_mut().manual_add_contacts = manual;
                    });
                    if auto_changed {
                        self.host.request_save_config();
                    }
                    if payload.len() >= 3 {
                        let new_base = payload[2] & 0x03;
                        let new_loc = (payload[2] >> 2) & 0x03;
                        let new_env = (payload[2] >> 4) & 0x03;
                        let new_policy = if payload.len() >= 4 {
                            payload[3]
                        } else {
                            self.advert_loc_policy
                        };
                        let new_multi = if payload.len() >= 5 {
                            payload[4]
                        } else {
                            self.multi_acks
                        };
                        let changed = new_base != self.telemetry_mode_base
                            || new_loc != self.telemetry_mode_loc
                            || new_env != self.telemetry_mode_env
                            || new_policy != self.advert_loc_policy
                            || new_multi != self.multi_acks;
                        self.telemetry_mode_base = new_base;
                        self.telemetry_mode_loc = new_loc;
                        self.telemetry_mode_env = new_env;
                        self.advert_loc_policy = new_policy;
                        self.multi_acks = new_multi;
                        update_prefs(|p| {
                            p.telemetry_mode_base = new_base;
                            p.telemetry_mode_loc = new_loc;
                            p.telemetry_mode_env = new_env;
                            p.advert_loc_policy = new_policy;
                            p.multi_acks = new_multi;
                        });
                        if changed {
                            self.host.request_save_config();
                        }
                    }
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_DEVICE_PIN => {
                let mut ok = false;
                if payload.len() >= 5 {
                    let raw = u32::from_le_bytes([
                        payload[1], payload[2], payload[3], payload[4],
                    ]);
                    // Zero keeps the static default rather than programming
                    // an unusable all-zero passkey.
                    let pin = if raw == 0 { 123_456 } else { raw };
                    if (100_000..=999_999).contains(&pin) {
                        let changed = crate::state::BLE_PIN.swap(pin, Ordering::Relaxed) != pin;
                        // The app applies a new PIN by rebooting the node as
                        // soon as it gets this reply. Wait for the flash write,
                        // otherwise the next boot restores the old PIN.
                        ok = !changed
                            || crate::storage::config::request_save_config_and_wait().await;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_CUSTOM_VARS => {
                // No device-specific custom variables are exposed yet.
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_CUSTOM_VARS);
                emit(&resp);
            }

            CMD_SET_CUSTOM_VAR => {
                let ok = payload.len() >= 4;
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_ALLOWED_REPEAT_FREQ => {
                // Allowed client-repeat (off-grid) bands (kHz, lower/upper).
                let mut resp = Vec::<u8, 32>::new();
                let _ = resp.push(RESP_CODE_ALLOWED_REPEAT_FREQ);
                for (lo, hi) in crate::state::OFFGRID_FREQ_RANGES {
                    let _ = resp.extend_from_slice(&lo.to_le_bytes());
                    let _ = resp.extend_from_slice(&hi.to_le_bytes());
                }
                emit(&resp);
            }

            CMD_SET_FLOOD_SCOPE_KEY => {
                let mut ok = false;
                if payload.len() >= 2 && payload[1] == 0 {
                    SCOPE.lock(|s| {
                        let mut scope = s.borrow_mut();
                        if payload.len() >= 18 {
                            let mut key = [0u8; 16];
                            key.copy_from_slice(&payload[2..18]);
                            scope.send_key = Some(key);
                        } else {
                            scope.send_key = None;
                        }
                        scope.force_unscoped = false;
                    });
                    ok = true;
                } else if payload.len() == 2 && payload[1] == 1 {
                    SCOPE.lock(|s| {
                        let mut scope = s.borrow_mut();
                        scope.send_key = None;
                        scope.force_unscoped = true;
                    });
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_DEFAULT_FLOOD_SCOPE => {
                let mut ok = false;
                if payload.len() >= 48 {
                    let name = &payload[1..32];
                    let n = name.iter().position(|&b| b == 0).unwrap_or(31);
                    if n > 0 && n < 31 {
                        let mut new_name = [0u8; 31];
                        new_name[..n].copy_from_slice(&name[..n]);
                        let mut new_key = [0u8; 16];
                        new_key.copy_from_slice(&payload[32..48]);
                        let changed = {
                            let p = prefs();
                            p.scope_name != new_name || p.scope_key != new_key
                        };
                        update_prefs(|p| {
                            p.scope_name = new_name;
                            p.scope_key = new_key;
                        });
                        if changed {
                            self.host.request_save_config();
                        }
                        ok = true;
                    }
                } else if payload.len() == 1 {
                    let changed = {
                        let p = prefs();
                        p.scope_name != [0u8; 31] || p.scope_key != [0u8; 16]
                    };
                    update_prefs(|p| {
                        p.scope_name = [0u8; 31];
                        p.scope_key = [0u8; 16];
                    });
                    if changed {
                        self.host.request_save_config();
                    }
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_DEFAULT_FLOOD_SCOPE => {
                let p = prefs();
                let (name, key) = (p.scope_name, p.scope_key);
                if name[0] != 0 {
                    let mut resp = Vec::<u8, 48>::new();
                    let _ = resp.push(RESP_CODE_DEFAULT_FLOOD_SCOPE);
                    let _ = resp.extend_from_slice(&name);
                    let _ = resp.extend_from_slice(&key);
                    emit(&resp);
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_DEFAULT_FLOOD_SCOPE);
                    emit(&resp);
                }
            }

            CMD_SEND_CONTROL_DATA => {
                let mut ok = false;
                if payload.len() >= 2 && (payload[1] & 0x80) != 0 {
                    let mut data = Vec::<u8, 183>::new();
                    if data.extend_from_slice(&payload[1..]).is_ok() {
                        ok = crate::command::USB_COMMAND_CHANNEL
                            .try_send(crate::command::UsbCommand::SendControlData { data })
                            .is_ok();
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SEND_RAW_DATA => {
                let mut ok = false;
                if payload.len() >= 6 {
                    let path_len = payload[1];
                    if Packet::is_valid_path_len(path_len) {
                        let n = (path_len & 63) as usize
                            * (((path_len >> 6) as usize) + 1);
                        if payload.len() >= 2 + n + 4 {
                            let mut path = [0u8; 64];
                            path[..n].copy_from_slice(&payload[2..2 + n]);
                            let mut data = Vec::<u8, 184>::new();
                            if data.extend_from_slice(&payload[2 + n..]).is_ok() {
                                ok = crate::command::USB_COMMAND_CHANNEL
                                    .try_send(
                                        crate::command::UsbCommand::SendRawData {
                                            path_len,
                                            path,
                                            payload: data,
                                        },
                                    )
                                    .is_ok();
                            }
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SEND_RAW_PACKET => {
                let mut ok = false;
                if payload.len() >= 4 && Packet::read_from(&payload[2..]).is_ok() {
                    let mut raw = Vec::<u8, 255>::new();
                    if raw.extend_from_slice(&payload[2..]).is_ok() {
                        ok = crate::command::USB_COMMAND_CHANNEL
                            .try_send(crate::command::UsbCommand::SendRawPacket { raw })
                            .is_ok();
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SEND_LOGIN => {
                // [cmd][public_key 32][password...]
                let mut resp = Vec::<u8, 10>::new();
                let mut reply = None;
                if payload.len() >= 33 {
                    let mut dest_key = [0u8; 32];
                    dest_key.copy_from_slice(&payload[1..33]);
                    let password = core::str::from_utf8(&payload[33..])
                        .unwrap_or("")
                        .trim_end_matches('\0');
                    let contact = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| ct.pub_key == dest_key)
                            .map(|ct| (ct.out_path_len, ct.out_path, ct.contact_type, ct.sync_since))
                    });
                    if let Some((out_path_len, out_path, contact_type, sync_since)) = contact {
                        let identity = meshcore_wire::identity::LocalIdentity::from_private(
                            self.callsign.as_str(),
                            &self.private_key,
                        );
                        if let Some(id) = identity {
                            if let Some(secret) = id.shared_secret(&dest_key) {
                                let tag = next_tag();
                                let pw = &password.as_bytes()[..password.len().min(15)];
                                let mut plain = [0u8; 4 + 4 + 15];
                                plain[..4].copy_from_slice(&tag.to_le_bytes());
                                let n = if contact_type
                                    == meshcore_wire::packets::advert::ADV_TYPE_ROOM
                                {
                                    // [tag][sync_since][password]
                                    plain[4..8].copy_from_slice(&sync_since.to_le_bytes());
                                    plain[8..8 + pw.len()].copy_from_slice(pw);
                                    8 + pw.len()
                                } else {
                                    plain[4..4 + pw.len()].copy_from_slice(pw);
                                    4 + pw.len()
                                };
                                if let Ok(anon) = AnonReqPacket::new(
                                    dest_key[0],
                                    self.public_key,
                                    &plain[..n],
                                ) {
                                    let direct = out_path_len != 0xFF
                                        && Packet::is_valid_path_len(out_path_len);
                                    let route = if direct {
                                        ROUTE_TYPE_DIRECT
                                    } else {
                                        ROUTE_TYPE_FLOOD
                                    };
                                    if let Ok(mut pkt) = anon.to_packet(&secret, route) {
                                        if direct {
                                            pkt.path_len = out_path_len;
                                            let pn = pkt.path_byte_len();
                                            pkt.path[..pn].copy_from_slice(&out_path[..pn]);
                                        } else {
                                            apply_flood_scope(&mut pkt);
                                        }
                                        let mut buf = [0u8; 255];
                                        if let Ok(n) = pkt.write_to(&mut buf) {
                                            let mut raw = heapless::Vec::<u8, 255>::new();
                                            if raw.extend_from_slice(&buf[..n]).is_ok()
                                                && crate::command::USB_COMMAND_CHANNEL
                                                    .try_send(
                                                        crate::command::UsbCommand::SendRawPacket {
                                                            raw,
                                                        },
                                                    )
                                                    .is_ok()
                                            {
                                                let (sf, bw_khz, cr) =
                                                    crate::state::RADIO_SETTINGS.lock(|r| {
                                                        let s = r.borrow();
                                                        (s.sf, s.bw_khz, s.cr)
                                                    });
                                                let air_ms = crate::airtime::estimate_airtime_ms(
                                                    n,
                                                    sf,
                                                    bw_khz,
                                                    cr,
                                                    if sf <= 8 { 32 } else { 16 },
                                                );
                                                let hops = if direct {
                                                    (out_path_len & 63) as u32
                                                } else {
                                                    0
                                                };
                                                let est_timeout = if direct {
                                                    500 + (air_ms * 6 + 250) * (hops + 1)
                                                } else {
                                                    500 + 16 * air_ms
                                                };
                                                // Login replies are flooded even for a
                                                // direct request; give clients more
                                                // time than the ACK formula.
                                                let est_timeout = est_timeout.max(8000);
                                                note_pending_login(dest_key);
                                                LOGIN_TX_COUNT.fetch_add(1, Ordering::Relaxed);
                                                LOGIN_TX_TAG.store(tag, Ordering::Relaxed);
                                                LOGIN_TX_KEY.store(
                                                    u32::from_le_bytes([
                                                        dest_key[0],
                                                        dest_key[1],
                                                        dest_key[2],
                                                        dest_key[3],
                                                    ]),
                                                    Ordering::Relaxed,
                                                );
                                                LOGIN_TX_FLOOD.store(!direct, Ordering::Relaxed);
                                                LOGIN_RESULT.store(0, Ordering::Relaxed);
                                                let ack_tag = u32::from_le_bytes([
                                                    dest_key[0],
                                                    dest_key[1],
                                                    dest_key[2],
                                                    dest_key[3],
                                                ]);
                                                reply = Some((direct, ack_tag, est_timeout));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                match reply {
                    Some((direct, tag, est_timeout)) => {
                        let _ = resp.push(RESP_CODE_SENT);
                        let _ = resp.push(if direct { 0 } else { 1 });
                        let _ = resp.extend_from_slice(&tag.to_le_bytes());
                        let _ = resp.extend_from_slice(&est_timeout.to_le_bytes());
                    }
                    None => {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    }
                }
                emit(&resp);
            }

            CMD_HAS_CONNECTION => {
                let mut resp = Vec::<u8, 2>::new();
                if payload.len() >= 33 {
                    let mut key = [0u8; 32];
                    key.copy_from_slice(&payload[1..33]);
                    if has_connection(&key) {
                        let _ = resp.push(RESP_CODE_OK);
                    } else {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    }
                } else {
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                }
                emit(&resp);
            }

            CMD_LOGOUT => {
                if payload.len() >= 33 {
                    let mut key = [0u8; 32];
                    key.copy_from_slice(&payload[1..33]);
                    stop_connection(&key);
                }
                let mut resp = Vec::<u8, 2>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_SEND_TELEMETRY_REQ => {
                // Self: [cmd][0][0][0]; contact: [cmd][3 reserved][pubkey 32]
                if payload.len() == 4 {
                    // Cayenne LPP voltage: channel 1, type 0x74, 0.01 V BE.
                    let mv = self.host.battery_millivolts();
                    let scaled = ((mv as u32 + 5) / 10) as u16;
                    let mut push = Vec::<u8, 256>::new();
                    let _ = push.push(PUSH_CODE_TELEMETRY_RESPONSE);
                    let _ = push.push(0);
                    let _ = push.extend_from_slice(&self.public_key[..6]);
                    let _ = push.push(1);
                    let _ = push.push(0x74);
                    let _ = push.push((scaled >> 8) as u8);
                    let _ = push.push(scaled as u8);
                    crate::command::push_companion(&push);
                } else if payload.len() >= 4 + 32 {
                    let mut dest_key = [0u8; 32];
                    dest_key.copy_from_slice(&payload[4..36]);
                    let mut resp = Vec::<u8, 10>::new();
                    let mut reply = None;
                    let contact = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| ct.pub_key == dest_key)
                            .map(|ct| (ct.out_path_len, ct.out_path))
                    });
                    if let Some((out_path_len, out_path)) = contact {
                        let identity =
                            meshcore_wire::identity::LocalIdentity::from_private(
                                self.callsign.as_str(),
                                &self.private_key,
                            );
                        if let Some(id) = identity {
                            if let Some(secret) = id.shared_secret(&dest_key) {
                                let tag = next_tag();
                                let mut plain = [0u8; 13];
                                plain[..4].copy_from_slice(&tag.to_le_bytes());
                                plain[4] = REQ_TYPE_GET_TELEMETRY_DATA;
                                plain[9..13].copy_from_slice(
                                    &tag.wrapping_mul(0x9E37_79B1).to_le_bytes(),
                                );
                                if let Ok(addressed) = AddressedPacket::new(
                                    dest_key[0],
                                    self.public_key[0],
                                    &plain,
                                ) {
                                    let direct = out_path_len != 0xFF
                                        && Packet::is_valid_path_len(out_path_len);
                                    let route = if direct {
                                        ROUTE_TYPE_DIRECT
                                    } else {
                                        ROUTE_TYPE_FLOOD
                                    };
                                    if let Ok(mut pkt) =
                                        addressed.to_packet(PAYLOAD_TYPE_REQ, route, &secret)
                                    {
                                        if direct {
                                            pkt.path_len = out_path_len;
                                            let n = pkt.path_byte_len();
                                            pkt.path[..n].copy_from_slice(&out_path[..n]);
                                        } else {
                                            apply_flood_scope(&mut pkt);
                                        }
                                        let mut buf = [0u8; 255];
                                        if let Ok(n) = pkt.write_to(&mut buf) {
                                            let mut raw = heapless::Vec::<u8, 255>::new();
                                            if raw.extend_from_slice(&buf[..n]).is_ok()
                                                && crate::command::USB_COMMAND_CHANNEL
                                                    .try_send(
                                                        crate::command::UsbCommand::SendRawPacket {
                                                            raw,
                                                        },
                                                    )
                                                    .is_ok()
                                            {
                                                let (sf, bw_khz, cr) =
                                                    crate::state::RADIO_SETTINGS.lock(|r| {
                                                        let s = r.borrow();
                                                        (s.sf, s.bw_khz, s.cr)
                                                    });
                                                let air_ms = crate::airtime::estimate_airtime_ms(
                                                    n,
                                                    sf,
                                                    bw_khz,
                                                    cr,
                                                    if sf <= 8 { 32 } else { 16 },
                                                );
                                                let hops = if direct {
                                                    (out_path_len & 63) as u32
                                                } else {
                                                    0
                                                };
                                                let est_timeout = if direct {
                                                    500 + (air_ms * 6 + 250) * (hops + 1)
                                                } else {
                                                    500 + 16 * air_ms
                                                };
                                                note_pending_telemetry(tag, dest_key);
                                                reply = Some((direct, tag, est_timeout.max(6000)));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                    match reply {
                        Some((direct, tag, est_timeout)) => {
                            let _ = resp.push(RESP_CODE_SENT);
                            let _ = resp.push(if direct { 0 } else { 1 });
                            let _ = resp.extend_from_slice(&tag.to_le_bytes());
                            let _ = resp.extend_from_slice(&est_timeout.to_le_bytes());
                        }
                        None => {
                            let _ = resp.push(RESP_CODE_ERR);
                            let _ = resp.push(ERR_NOT_FOUND);
                        }
                    }
                    emit(&resp);
                } else {
                    let mut resp = Vec::<u8, 2>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                    emit(&resp);
                }
            }

            CMD_FACTORY_RESET => {
                // Arduino-compatible safety magic: "reset".
                let mut resp = Vec::<u8, 2>::new();
                if payload.len() >= 6 && &payload[1..6] == b"reset" {
                    let _ = resp.push(RESP_CODE_OK);
                    emit(&resp);
                    self.host.request_factory_reset();
                } else {
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                    emit(&resp);
                }
            }

            CMD_SEND_PATH_DISCOVERY_REQ => {
                // [cmd][0][public_key 32]
                let mut resp = Vec::<u8, 10>::new();
                let mut reply = None;
                if payload.len() >= 34 && payload[1] == 0 {
                    let mut dest_key = [0u8; 32];
                    dest_key.copy_from_slice(&payload[2..34]);
                    let known = crate::state::DISCOVERED_CONTACTS
                        .lock(|c| c.borrow().iter().any(|ct| ct.pub_key == dest_key));
                    if known {
                        let identity =
                            meshcore_wire::identity::LocalIdentity::from_private(
                                self.callsign.as_str(),
                                &self.private_key,
                            );
                        if let Some(id) = identity {
                            if let Some(secret) = id.shared_secret(&dest_key) {
                                let tag = next_tag();
                                let mut req = [0u8; 9];
                                req[0] = REQ_TYPE_GET_TELEMETRY_DATA;
                                req[1] = !0x01; // inverse BASE permission mask
                                req[5..9].copy_from_slice(
                                    &tag.wrapping_mul(0x9E37_79B1).to_le_bytes(),
                                );
                                let mut plain = [0u8; 13];
                                plain[..4].copy_from_slice(&tag.to_le_bytes());
                                plain[4..].copy_from_slice(&req);
                                if let Ok(addressed) = AddressedPacket::new(
                                    dest_key[0],
                                    self.public_key[0],
                                    &plain,
                                ) {
                                    // Discovery always floods.
                                    if let Ok(mut pkt) = addressed.to_packet(
                                        PAYLOAD_TYPE_REQ,
                                        ROUTE_TYPE_FLOOD,
                                        &secret,
                                    ) {
                                        apply_flood_scope(&mut pkt);
                                        let mut buf = [0u8; 255];
                                        if let Ok(n) = pkt.write_to(&mut buf) {
                                            let mut raw = heapless::Vec::<u8, 255>::new();
                                            if raw.extend_from_slice(&buf[..n]).is_ok()
                                                && crate::command::USB_COMMAND_CHANNEL
                                                    .try_send(
                                                        crate::command::UsbCommand::SendRawPacket {
                                                            raw,
                                                        },
                                                    )
                                                    .is_ok()
                                            {
                                                let (sf, bw_khz, cr) =
                                                    crate::state::RADIO_SETTINGS.lock(|r| {
                                                        let s = r.borrow();
                                                        (s.sf, s.bw_khz, s.cr)
                                                    });
                                                let air_ms = crate::airtime::estimate_airtime_ms(
                                                    n,
                                                    sf,
                                                    bw_khz,
                                                    cr,
                                                    if sf <= 8 { 32 } else { 16 },
                                                );
                                                let est_timeout = 500 + 16 * air_ms;
                                                note_pending_discovery(tag, dest_key);
                                                reply = Some((tag, est_timeout.max(15000)));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                match reply {
                    Some((tag, est_timeout)) => {
                        let _ = resp.push(RESP_CODE_SENT);
                        let _ = resp.push(1); // flood
                        let _ = resp.extend_from_slice(&tag.to_le_bytes());
                        let _ = resp.extend_from_slice(&est_timeout.to_le_bytes());
                    }
                    None => {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    }
                }
                emit(&resp);
            }

            CMD_SEND_BINARY_REQ => {
                // [cmd][public_key 32][request type][request data...]
                let mut resp = Vec::<u8, 10>::new();
                let mut reply = None;
                if payload.len() >= 34 {
                    let mut dest_key = [0u8; 32];
                    dest_key.copy_from_slice(&payload[1..33]);
                    let req_data = &payload[33..];
                    let contact = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| ct.pub_key == dest_key)
                            .map(|ct| (ct.out_path_len, ct.out_path))
                    });
                    if let Some((out_path_len, out_path)) = contact {
                        let identity = meshcore_wire::identity::LocalIdentity::from_private(
                            self.callsign.as_str(),
                            &self.private_key,
                        );
                        if let Some(id) = identity {
                            if let Some(secret) = id.shared_secret(&dest_key) {
                                let tag = next_tag();
                                let mut plain = heapless::Vec::<u8, 184>::new();
                                let _ = plain.extend_from_slice(&tag.to_le_bytes());
                                let _ = plain.extend_from_slice(req_data);
                                if let Ok(addressed) = AddressedPacket::new(
                                    dest_key[0],
                                    self.public_key[0],
                                    &plain,
                                ) {
                                    let direct = out_path_len != 0xFF
                                        && Packet::is_valid_path_len(out_path_len);
                                    let route = if direct {
                                        ROUTE_TYPE_DIRECT
                                    } else {
                                        ROUTE_TYPE_FLOOD
                                    };
                                    if let Ok(mut pkt) =
                                        addressed.to_packet(PAYLOAD_TYPE_REQ, route, &secret)
                                    {
                                        if direct {
                                            pkt.path_len = out_path_len;
                                            let n = pkt.path_byte_len();
                                            pkt.path[..n].copy_from_slice(&out_path[..n]);
                                        } else {
                                            apply_flood_scope(&mut pkt);
                                        }
                                        let mut buf = [0u8; 255];
                                        if let Ok(n) = pkt.write_to(&mut buf) {
                                            let mut raw = heapless::Vec::<u8, 255>::new();
                                            if raw.extend_from_slice(&buf[..n]).is_ok()
                                                && crate::command::USB_COMMAND_CHANNEL
                                                    .try_send(
                                                        crate::command::UsbCommand::SendRawPacket {
                                                            raw,
                                                        },
                                                    )
                                                    .is_ok()
                                            {
                                                let (sf, bw_khz, cr) =
                                                    crate::state::RADIO_SETTINGS.lock(|r| {
                                                        let s = r.borrow();
                                                        (s.sf, s.bw_khz, s.cr)
                                                    });
                                                let air_ms = crate::airtime::estimate_airtime_ms(
                                                    n,
                                                    sf,
                                                    bw_khz,
                                                    cr,
                                                    if sf <= 8 { 32 } else { 16 },
                                                );
                                                let hops = if direct {
                                                    (out_path_len & 63) as u32
                                                } else {
                                                    0
                                                };
                                                let est_timeout = if direct {
                                                    500 + (air_ms * 6 + 250) * (hops + 1)
                                                } else {
                                                    500 + 16 * air_ms
                                                };
                                                REQ_TX_COUNT.fetch_add(1, Ordering::Relaxed);
                                                REQ_TX_TAG.store(tag, Ordering::Relaxed);
                                                REQ_TX_TYPE.store(
                                                    req_data.first().copied().unwrap_or(0),
                                                    Ordering::Relaxed,
                                                );
                                                note_pending_req(tag, dest_key);
                                                reply = Some((direct, tag, est_timeout.max(6000)));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                match reply {
                    Some((direct, tag, est_timeout)) => {
                        let _ = resp.push(RESP_CODE_SENT);
                        let _ = resp.push(if direct { 0 } else { 1 });
                        let _ = resp.extend_from_slice(&tag.to_le_bytes());
                        let _ = resp.extend_from_slice(&est_timeout.to_le_bytes());
                    }
                    None => {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    }
                }
                emit(&resp);
            }

            CMD_SEND_STATUS_REQ => {
                // Inbound: [CMD(0x1B), public_key(32)]
                let mut resp = Vec::<u8, 10>::new();
                let mut reply = None;
                if payload.len() >= 33 {
                    let mut dest_key = [0u8; 32];
                    dest_key.copy_from_slice(&payload[1..33]);
                    let contact = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| ct.pub_key == dest_key)
                            .map(|ct| (ct.out_path_len, ct.out_path))
                    });
                    if let Some((out_path_len, out_path)) = contact {
                        let identity = meshcore_wire::identity::LocalIdentity::from_private(
                            self.callsign.as_str(),
                            &self.private_key,
                        );
                        if let Some(id) = identity {
                            if let Some(secret) = id.shared_secret(&dest_key) {
                                let tag = next_tag();
                                let mut plain = [0u8; 13];
                                plain[..4].copy_from_slice(&tag.to_le_bytes());
                                plain[4] = REQ_TYPE_GET_STATUS;
                                plain[9..13]
                                    .copy_from_slice(&tag.wrapping_mul(0x9E37_79B1).to_le_bytes());
                                if let Ok(addressed) = AddressedPacket::new(
                                    dest_key[0],
                                    self.public_key[0],
                                    &plain,
                                ) {
                                    let direct = out_path_len != 0xFF
                                        && Packet::is_valid_path_len(out_path_len);
                                    let route = if direct {
                                        ROUTE_TYPE_DIRECT
                                    } else {
                                        ROUTE_TYPE_FLOOD
                                    };
                                    if let Ok(mut pkt) =
                                        addressed.to_packet(PAYLOAD_TYPE_REQ, route, &secret)
                                    {
                                        if direct {
                                            pkt.path_len = out_path_len;
                                            let n = pkt.path_byte_len();
                                            pkt.path[..n].copy_from_slice(&out_path[..n]);
                                        } else {
                                            apply_flood_scope(&mut pkt);
                                        }
                                        let mut buf = [0u8; 255];
                                        if let Ok(n) = pkt.write_to(&mut buf) {
                                            let mut raw = heapless::Vec::<u8, 255>::new();
                                            if raw.extend_from_slice(&buf[..n]).is_ok()
                                                && crate::command::USB_COMMAND_CHANNEL
                                                    .try_send(
                                                        crate::command::UsbCommand::SendRawPacket {
                                                            raw,
                                                        },
                                                    )
                                                    .is_ok()
                                            {
                                                let (sf, bw_khz, cr) =
                                                    crate::state::RADIO_SETTINGS.lock(|r| {
                                                        let s = r.borrow();
                                                        (s.sf, s.bw_khz, s.cr)
                                                    });
                                                let air_ms = crate::airtime::estimate_airtime_ms(
                                                    n,
                                                    sf,
                                                    bw_khz,
                                                    cr,
                                                    if sf <= 8 { 32 } else { 16 },
                                                );
                                                let hops = if direct {
                                                    (out_path_len & 63) as u32
                                                } else {
                                                    0
                                                };
                                                let est_timeout = if direct {
                                                    500 + (air_ms * 6 + 250) * (hops + 1)
                                                } else {
                                                    500 + 16 * air_ms
                                                };
                                                note_status(tag, dest_key);
                                                reply = Some((direct, tag, est_timeout));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                match reply {
                    Some((direct, tag, est_timeout)) => {
                        let _ = resp.push(RESP_CODE_SENT);
                        let _ = resp.push(if direct { 0 } else { 1 });
                        let _ = resp.extend_from_slice(&tag.to_le_bytes());
                        let _ = resp.extend_from_slice(&est_timeout.to_le_bytes());
                    }
                    None => {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    }
                }
                emit(&resp);
            }

            CMD_SEND_TRACE_PATH => {
                TRACE_TX_COUNT.fetch_add(1, Ordering::Relaxed);
                if payload.len() >= 10 {
                    TRACE_TX_TAG.store(
                        u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]),
                        Ordering::Relaxed,
                    );
                    TRACE_TX_FLAGS.store(payload[9], Ordering::Relaxed);
                    let plen = payload.len() - 10;
                    TRACE_TX_PATH_LEN.store(plen.min(255) as u8, Ordering::Relaxed);
                    let mut path0 = [0u8; 4];
                    let n = plen.min(4);
                    path0[..n].copy_from_slice(&payload[10..10 + n]);
                    TRACE_TX_PATH.store(u32::from_le_bytes(path0), Ordering::Relaxed);
                }
                let mut resp = Vec::<u8, 10>::new();
                let mut reply = None;
                if payload.len() > 10 {
                    let tag = u32::from_le_bytes([
                        payload[1], payload[2], payload[3], payload[4],
                    ]);
                    let auth = u32::from_le_bytes([
                        payload[5], payload[6], payload[7], payload[8],
                    ]);
                    let flags = payload[9];
                    let path_len = payload.len() - 10;
                    let hash_bytes = 1usize << (flags & 0x03);
                    if path_len <= 64 && path_len % hash_bytes == 0 {
                        let mut path = [0u8; 64];
                        path[..path_len].copy_from_slice(&payload[10..]);
                        if crate::command::USB_COMMAND_CHANNEL
                            .try_send(crate::command::UsbCommand::SendTracePath {
                                tag,
                                auth,
                                flags,
                                path_len: path_len as u8,
                                path,
                            })
                            .is_ok()
                        {
                            let (sf, bw_khz, cr) =
                                crate::state::RADIO_SETTINGS.lock(|r| {
                                    let s = r.borrow();
                                    (s.sf, s.bw_khz, s.cr)
                                });
                            let air_ms = crate::airtime::estimate_airtime_ms(
                                11 + path_len,
                                sf,
                                bw_khz,
                                cr,
                                if sf <= 8 { 32 } else { 16 },
                            );
                            let hops = (path_len >> (flags & 0x03)) as u32;
                            let est_timeout = 500 + (air_ms * 6 + 250) * (hops + 1);
                            reply = Some((tag, est_timeout));
                        }
                    }
                }

                match reply {
                    Some((tag, est_timeout)) => {
                        let _ = resp.push(RESP_CODE_SENT);
                        let _ = resp.push(0); // direct, never flood
                        let _ = resp.extend_from_slice(&tag.to_le_bytes());
                        let _ = resp.extend_from_slice(&est_timeout.to_le_bytes());
                    }
                    None => {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_ILLEGAL_ARG);
                    }
                }
                emit(&resp);
            }

            _ => {
                // Unsupported command: report an error rather than a misleading OK.
                let mut resp = Vec::<u8, 2>::new();
                let _ = resp.push(RESP_CODE_ERR);
                let _ = resp.push(ERR_UNSUPPORTED);
                emit(&resp);
            }
        }
    }
}


/// Number of stored contacts.
pub fn contacts_count() -> u32 {
    crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow().len()) as u32
}

/// Clone one contact out of the shared list (a small, fixed-size copy).
pub fn contact_at(idx: usize) -> Option<crate::types::DiscoveredContact> {
    crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow().get(idx).cloned())
}


#[cfg(test)]
mod tests {
    use super::*;

    struct TestHost;

    impl CompanionHost for TestHost {
        fn battery_millivolts(&self) -> u16 {
            4_000
        }
        fn set_callsign(&self, _name: &str) {}
        fn set_custom_priv_key(&self, _key: [u8; 64]) {}
        fn set_path_hash_mode(&self, _mode: u8) {}
        fn request_save_config(&self) {}
        fn request_radio_config(&self) {}
        fn request_factory_reset(&self) {}
        fn queue_consume_now(&self) {}
        fn advert_remember(&self, _pub_key: &[u8; 32], _payload: &[u8]) {}
        fn reboot(&self) -> ! {
            panic!("reboot in host test")
        }
    }

    #[test]
    fn node_seeds_battery_from_the_host() {
        let node = CompanionNode::new(
            "test",
            "test-board",
            0,
            [1u8; 32],
            [2u8; 64],
            &TestHost,
        );
        assert_eq!(node.battery_millivolts, 4_000);
        assert_eq!(node.battery_percentage, 77);
    }
}

/// Dispatch a companion frame, including the local-CLI command.
///
/// Transports call this instead of [`CompanionNode::handle_command`] so the
/// text CLI and binary protocol share one entry point.
pub async fn handle_command<F: FnMut(&[u8])>(
    node: &mut CompanionNode<'_>,
    payload: &[u8],
    host: &dyn crate::companion::cli::CliHost,
    mut emit: F,
) {
    if payload.first() == Some(&CMD_RUN_CLI_COMMAND) {
        let line = core::str::from_utf8(&payload[1..])
            .unwrap_or("")
            .trim_end_matches('\0');
        let mut out = String::<512>::new();
        crate::companion::cli::reply(line, node, host, &mut out).await;
        let mut resp =
            Vec::<u8, { crate::companion::framing::MAX_FRAME_PAYLOAD_SIZE }>::new();
        let _ = resp.push(PACKET_CLI_REPLY);
        let bytes = out.as_bytes();
        let max = crate::companion::framing::MAX_FRAME_PAYLOAD_SIZE - 1;
        let _ = resp.extend_from_slice(&bytes[..bytes.len().min(max)]);
        emit(&resp);
    } else {
        node.handle_command(payload, emit).await;
    }
}
