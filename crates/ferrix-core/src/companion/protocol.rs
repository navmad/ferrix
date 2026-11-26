//! MeshCore Companion Protocol Command & Response Definitions
//!
//! Handles command parsing from companion apps (Mobile/Desktop/Web)
//! and generates protocol response frames.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
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

/// Peer public key of the outstanding status/ping request, if any.
static PENDING_STATUS: Mutex<CriticalSectionRawMutex, RefCell<Option<[u8; 32]>>> =
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

fn note_status(pub_key: [u8; 32]) {
    PENDING_STATUS.lock(|p| *p.borrow_mut() = Some(pub_key));
}

use crate::state::{Connection, CONNECTIONS, MAX_CONNECTIONS, PENDING_LOGIN};

const RESP_SERVER_LOGIN_OK: u8 = 0;
const REQ_TYPE_KEEP_ALIVE: u8 = 0x02;

/// Outstanding path-discovery request: (tag, destination).
static PENDING_DISCOVERY: Mutex<CriticalSectionRawMutex, RefCell<Option<(u32, [u8; 32])>>> =
    Mutex::new(RefCell::new(None));

/// Outstanding telemetry request: (tag, destination).
static PENDING_TELEMETRY: Mutex<CriticalSectionRawMutex, RefCell<Option<(u32, [u8; 32])>>> =
    Mutex::new(RefCell::new(None));
pub const REQ_TYPE_GET_TELEMETRY_DATA: u8 = 0x03;
pub const TELEM_PERM_BASE: u8 = 0x01;
pub const TELEM_PERM_LOCATION: u8 = 0x02;
pub const TELEM_PERM_ENVIRONMENT: u8 = 0x04;
pub const TELEM_MODE_DENY: u8 = 0;
pub const TELEM_MODE_ALLOW_FLAGS: u8 = 1;
pub const TELEM_MODE_ALLOW_ALL: u8 = 2;
pub const TELEM_CHANNEL_SELF: u8 = 1;
pub const LPP_TEMPERATURE: u8 = 0x67;
pub const LPP_VOLTAGE: u8 = 0x74;
pub const LPP_GPS: u8 = 0x88;

/// Outstanding binary/anon request: (tag, destination).
static PENDING_REQ: Mutex<CriticalSectionRawMutex, RefCell<Option<(u32, [u8; 32])>>> =
    Mutex::new(RefCell::new(None));

static REBOOT_PENDING: AtomicBool = AtomicBool::new(false);

/// Consume any deferred reboot requested by `CMD_REBOOT`.
pub fn take_pending_reboot() -> bool {
    REBOOT_PENDING.swap(false, Ordering::AcqRel)
}

/// Return the pending request target public key matching `hash`, if any.
pub fn pending_anon_peer(hash: u8) -> Option<[u8; 32]> {
    PENDING_REQ.lock(|p| {
        p.borrow()
            .as_ref()
            .and_then(|(_, key)| (key[0] == hash).then_some(*key))
    })
}

fn now_ms() -> u64 {
    embassy_time::Instant::now().as_millis()
}

/// Clear all pending request slots when dispatching a new companion request so
/// stale matchers from earlier uncompleted requests cannot misclassify replies.
#[inline(never)]
fn clear_pending_reqs() {
    PENDING_LOGIN.lock(|p| *p.borrow_mut() = None);
    PENDING_STATUS.lock(|p| *p.borrow_mut() = None);
    PENDING_TELEMETRY.lock(|p| *p.borrow_mut() = None);
    PENDING_DISCOVERY.lock(|p| *p.borrow_mut() = None);
    PENDING_REQ.lock(|p| *p.borrow_mut() = None);
}

fn note_pending_login(pub_key: [u8; 32]) {
    PENDING_LOGIN.lock(|p| *p.borrow_mut() = Some((pub_key, now_ms() + 15000)));
}

fn note_pending_req(tag: u32, pub_key: [u8; 32]) {
    PENDING_REQ.lock(|p| *p.borrow_mut() = Some((tag, pub_key)));
}

fn note_pending_telemetry(tag: u32, pub_key: [u8; 32]) {
    PENDING_TELEMETRY.lock(|p| *p.borrow_mut() = Some((tag, pub_key)));
}

fn note_pending_discovery(tag: u32, pub_key: [u8; 32]) {
    PENDING_DISCOVERY.lock(|p| *p.borrow_mut() = Some((tag, pub_key)));
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
    if !Packet::is_valid_path_len(out_path_len) || !Packet::is_valid_path_len(in_path_len) {
        return false;
    }
    let on = (out_path_len & 63) as usize * (((out_path_len >> 6) & 3) as usize + 1);
    let inn = (in_path_len & 63) as usize * (((in_path_len >> 6) & 3) as usize + 1);
    if out_path.len() < on || in_path.len() < inn {
        return false;
    }
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
    let mut push = Vec::<u8, 256>::new();
    let _ = push.push(PUSH_CODE_PATH_DISCOVERY_RESP);
    let _ = push.push(0);
    let _ = push.extend_from_slice(&sender_pub[..6]);
    let _ = push.push(out_path_len);
    let _ = push.extend_from_slice(&out_path[..on]);
    let _ = push.push(in_path_len);
    let _ = push.extend_from_slice(&in_path[..inn]);
    crate::command::push_companion(&push);
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

fn shared_secret_for_keypair(
    pub_key: &[u8; 32],
    prv_key: &[u8; 64],
    peer: &[u8; 32],
) -> Option<[u8; 32]> {
    if pub_key[0] == 0x00 || pub_key[0] == 0xFF {
        return None;
    }
    crate::ecdh::shared_secret(prv_key, peer)
}

fn send_keepalive(tag: u32, dest_key: &[u8; 32], expected_ack: u32) -> bool {
    let private_key = crate::state::current_private_key();
    let public_key = crate::state::current_public_key();
    let Some(secret) = shared_secret_for_keypair(&public_key, &private_key, dest_key) else {
        return false;
    };
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

    // Match explicit 4-byte tag + 32-byte key slots first so short status or
    // telemetry responses are never mistaken for a login reply.
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
        let mut push = Vec::<u8, 256>::new();
        let _ = push.push(PUSH_CODE_BINARY_RESPONSE);
        let _ = push.push(0);
        let _ = push.extend_from_slice(&tag.to_le_bytes());
        let data = &plain[4..];
        let _ = push.extend_from_slice(&data[..data.len().min(256 - push.len())]);
        crate::command::push_companion(&push);
        return true;
    }

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
        crate::command::push_companion(&push);
        return true;
    }

    // Status reply: servers may put their own clock in bytes 0..4 rather than
    // echoing the request tag, so match by sender key and require status bytes.
    let matched = plain.len() > 4
        && PENDING_STATUS.lock(|p| {
            let mut pending = p.borrow_mut();
            if pending.map_or(false, |k| k == *sender_pub) {
                *pending = None;
                true
            } else {
                false
            }
        });
    if matched {
        let mut push = Vec::<u8, 256>::new();
        let _ = push.push(PUSH_CODE_STATUS_RESPONSE);
        let _ = push.push(0);
        let _ = push.extend_from_slice(&sender_pub[..6]);
        let data = &plain[4..];
        let _ = push.extend_from_slice(&data[..data.len().min(256 - push.len())]);
        crate::command::push_companion(&push);
        return true;
    }

    // Login reply: the server's first 4 bytes are its own clock, so match the
    // pending login peer and reply length (<= 32 B) after tag-matched slots.
    let login = plain.len() >= 5
        && plain.len() <= 32
        && PENDING_LOGIN.lock(|p| {
            let mut pending = p.borrow_mut();
            if let Some((key, deadline)) = *pending {
                if key == *sender_pub && now_ms() <= deadline {
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
        let mut push = Vec::<u8, 256>::new();
        let _ = push.push(PUSH_CODE_LOGIN_SUCCESS);
        let reply = &plain[4..];
        if reply.len() >= 2 && &reply[..2] == b"OK" {
            let _ = push.push(0);
            let _ = push.extend_from_slice(&sender_pub[..6]);
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
        } else {
            push[0] = PUSH_CODE_LOGIN_FAIL;
            let _ = push.push(0);
            let _ = push.extend_from_slice(&sender_pub[..6]);
        }
        crate::command::push_companion(&push);
        return true;
    }

    // Forward unknown/unhandled responses (e.g. unsolicited sensor push
    // telemetry or overlapping binary requests) to the app as
    // PUSH_CODE_BINARY_RESPONSE.
    if plain.len() > 4 {
        let mut push = Vec::<u8, 256>::new();
        let _ = push.push(PUSH_CODE_BINARY_RESPONSE);
        let _ = push.push(0);
        let _ = push.extend_from_slice(&plain[..plain.len().min(256 - push.len())]);
        crate::command::push_companion(&push);
        return true;
    }

    false
}

#[inline]
fn round_f32_to_i32(v: f32) -> i32 {
    if v >= 0.0 {
        (v + 0.5) as i32
    } else {
        (v - 0.5) as i32
    }
}

/// Build a response payload (`sender_timestamp(4) ‖ payload`) for an inbound
/// `PAYLOAD_TYPE_REQ`.
///
/// Returns `Some(reply)` when the request is supported and permitted by the
/// node's telemetry modes and contact flags, or `None` when ignored/denied.
pub fn handle_contact_request(
    node: &CompanionNode<'_>,
    plain: &[u8],
    sender_pub: &[u8; 32],
) -> Option<Vec<u8, 64>> {
    if plain.len() < 5 {
        return None;
    }
    let req_type = plain[4];
    if req_type != REQ_TYPE_GET_TELEMETRY_DATA {
        return None;
    }

    let contact_flags = crate::state::DISCOVERED_CONTACTS
        .lock(|c| {
            c.borrow()
                .iter()
                .find(|ct| ct.pub_key == *sender_pub)
                .map(|ct| ct.flags)
        })
        .unwrap_or(0);
    // LSB is CONTACT_FLAG_FAVORITE; upper bits carry per-contact telemetry permissions.
    let cp = contact_flags >> 1;
    let p = prefs();

    let mut permissions: u8 = match p.telemetry_mode_base {
        TELEM_MODE_ALLOW_ALL => TELEM_PERM_BASE,
        TELEM_MODE_ALLOW_FLAGS => cp & TELEM_PERM_BASE,
        _ => 0,
    };
    permissions |= match p.telemetry_mode_loc {
        TELEM_MODE_ALLOW_ALL => TELEM_PERM_LOCATION,
        TELEM_MODE_ALLOW_FLAGS => cp & TELEM_PERM_LOCATION,
        _ => 0,
    };
    permissions |= match p.telemetry_mode_env {
        TELEM_MODE_ALLOW_ALL => TELEM_PERM_ENVIRONMENT,
        TELEM_MODE_ALLOW_FLAGS => cp & TELEM_PERM_ENVIRONMENT,
        _ => 0,
    };

    // First reserved byte after req_type is the inverse mask of permissions
    // requested by the sender (e.g. Path Discovery sets `~TELEM_PERM_BASE`).
    let perm_mask = !plain.get(5).copied().unwrap_or(0);
    permissions &= perm_mask;

    if (permissions & TELEM_PERM_BASE) == 0 {
        return None;
    }

    let mut reply = Vec::<u8, 64>::new();
    let _ = reply.extend_from_slice(&plain[..4]); // reflect sender_timestamp as tag
    node.encode_self_telemetry_lpp(permissions, &mut reply);
    Some(reply)
}

pub use crate::companion::contacts::{
    encode_contacts_end, encode_contacts_start, RESP_CODE_CONTACT, RESP_CODE_CONTACTS_START,
    RESP_CODE_END_OF_CONTACTS,
};
pub(crate) use crate::companion::contacts::serialize_contact;

pub use crate::state::{
    advert_location, apply_flood_scope, apply_flood_scope_default, flood_scope_default,
    flood_scope_send, prefs, update_prefs, CompanionPrefs, DEFAULT_PREFS, SCOPE,
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
    /// Built-in MCU die temperature in `0.1 °C` units (`i16`), or `None` if
    /// unsupported/unavailable.
    fn mcu_temperature_tenths_c(&self) -> Option<i16> {
        None
    }
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
            self.battery_percentage = crate::battery::apply_percentage_hysteresis(
                self.battery_percentage,
                crate::battery::millivolts_to_percentage(mv),
            );
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

        let (lat_e6, lon_e6) = crate::state::advert_lat_lon_e6();
        if lat_e6 != 0 || lon_e6 != 0 {
            self.latitude = lat_e6 as f32 / 1_000_000.0;
            self.longitude = lon_e6 as f32 / 1_000_000.0;
        }
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
                let d = crate::version::BUILD_DATE.as_bytes();
                let d_len = d.len().min(bdate.len());
                bdate[..d_len].copy_from_slice(&d[..d_len]);
                let _ = resp.extend_from_slice(&bdate);

                let mut mmodel = [0u8; 40];
                let m = self.hw_model.as_bytes();
                let m_len = m.len().min(mmodel.len());
                mmodel[..m_len].copy_from_slice(&m[..m_len]);
                let _ = resp.extend_from_slice(&mmodel);

                let mut sver = [0u8; 20];
                // The app gates several settings on the firmware version
                // string, so report the MeshCore-compatible semver.
                let s = crate::version::PROTOCOL_SEMVER.as_bytes();
                let s_len = s.len().min(sver.len());
                sver[..s_len].copy_from_slice(&s[..s_len]);
                let _ = resp.extend_from_slice(&sver);

                let _ = resp.push(if crate::state::offgrid_active() { 1 } else { 0 }); // offgrid mode
                let _ = resp.push(self.path_hash_mode); // path_hash_mode (0=1B, 1=2B, 2=3B)

                emit(&resp);
            }

            CMD_APP_START => {
                emit(&self.build_self_info());
            }

            CMD_GET_BATT_AND_STORAGE => {
                // [PACKET_BATTERY][mV u16][storage used KiB u32][storage total KiB u32]
                let used_kb = crate::storage::used_kb();
                let total_kb = crate::storage::total_kb();
                let mut resp = Vec::<u8, 16>::new();
                let _ = resp.push(RESP_CODE_BATTERY);
                let _ = resp.extend_from_slice(&self.battery_millivolts.to_le_bytes());
                let _ = resp.extend_from_slice(&used_kb.to_le_bytes());
                let _ = resp.extend_from_slice(&total_kb.to_le_bytes());
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
                let mut err = None;
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
                        crate::state::set_advert_lat_lon_e6(lat, lon);
                        let _ = crate::command::USB_COMMAND_CHANNEL.try_send(
                            crate::command::UsbCommand::SetAdvertLatLon { lat, lon },
                        );
                    } else {
                        err = Some(ERR_ILLEGAL_ARG);
                    }
                } else {
                    err = Some(ERR_ILLEGAL_ARG);
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

            CMD_RESET_PATH => {
                let mut resp = Vec::<u8, 4>::new();
                if payload.len() < 33 {
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                } else {
                    let mut pub_key = [0u8; 32];
                    pub_key.copy_from_slice(&payload[1..33]);
                    let found = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        if let Some(ct) =
                            c.borrow_mut().iter_mut().find(|ct| ct.pub_key == pub_key)
                        {
                            ct.out_path = [0u8; 64];
                            ct.out_path_len = 0xFF;
                            true
                        } else {
                            false
                        }
                    });
                    if found {
                        crate::storage::request_contacts();
                        let _ = resp.push(RESP_CODE_OK);
                    } else {
                        let _ = resp.push(RESP_CODE_ERR);
                        let _ = resp.push(ERR_NOT_FOUND);
                    }
                }
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
                if payload.len() < 2 + crate::state::ADVERT_PATH_PREFIX_LEN {
                    emit(&[RESP_CODE_ERR, ERR_ILLEGAL_ARG]);
                } else {
                    let mut prefix = [0u8; crate::state::ADVERT_PATH_PREFIX_LEN];
                    prefix.copy_from_slice(&payload[2..2 + crate::state::ADVERT_PATH_PREFIX_LEN]);
                    if let Some(entry) = crate::state::lookup_advert_path(&prefix) {
                        let n = (entry.path_len & 63) as usize
                            * (((entry.path_len >> 6) as usize) + 1);
                        let mut resp = Vec::<u8, 72>::new();
                        let _ = resp.push(RESP_CODE_ADVERT_PATH);
                        let _ = resp.extend_from_slice(&entry.recv_timestamp.to_le_bytes());
                        let _ = resp.push(entry.path_len);
                        let _ = resp.extend_from_slice(&entry.path[..n]);
                        emit(&resp);
                    } else {
                        emit(&[RESP_CODE_ERR, ERR_NOT_FOUND]);
                    }
                }
            }

            CMD_GET_STATS => {
                let mut handled = false;
                if payload.len() >= 2 {
                    let kind = payload[1];
                    if kind == 0 {
                        let uptime = embassy_time::Instant::now().as_secs() as u32;
                        let err_flags = crate::state::core_err_flags();
                        let queue_len = crate::state::tx_queue_len();
                        let mut resp = Vec::<u8, 11>::new();
                        let _ = resp.push(RESP_CODE_STATS);
                        let _ = resp.push(0);
                        let _ = resp.extend_from_slice(&self.battery_millivolts.to_le_bytes());
                        let _ = resp.extend_from_slice(&uptime.to_le_bytes());
                        let _ = resp.extend_from_slice(&err_flags.to_le_bytes());
                        let _ = resp.push(queue_len);
                        emit(&resp);
                        handled = true;
                    } else if kind == 1 {
                        let (noise_floor, rssi, snr, tx_air_s, rx_air_s) =
                            crate::state::RADIO_TELEMETRY.lock(|t| {
                                let tel = t.borrow();
                                (
                                    tel.effective_noise_floor_dbm(),
                                    tel.last_rssi,
                                    tel.last_snr,
                                    tel.tx_air_ms / 1000,
                                    tel.rx_air_ms / 1000,
                                )
                            });
                        let mut resp = Vec::<u8, 14>::new();
                        let _ = resp.push(RESP_CODE_STATS);
                        let _ = resp.push(1);
                        let _ = resp.extend_from_slice(&noise_floor.to_le_bytes());
                        let _ = resp.push(rssi as i8 as u8);
                        let _ = resp.push((snr * 4) as u8);
                        let _ = resp.extend_from_slice(&tx_air_s.to_le_bytes());
                        let _ = resp.extend_from_slice(&rx_air_s.to_le_bytes());
                        emit(&resp);
                        handled = true;
                    } else if kind == 2 {
                        let (
                            recv,
                            sent,
                            sent_flood,
                            sent_direct,
                            recv_flood,
                            recv_direct,
                            recv_errors,
                        ) = crate::state::RADIO_TELEMETRY.lock(|t| {
                            let tel = t.borrow();
                            (
                                tel.rx_packets,
                                tel.tx_beacons,
                                tel.tx_flood_sent,
                                tel.tx_direct,
                                tel.rx_flood,
                                tel.rx_direct,
                                tel.rx_errors,
                            )
                        });
                        let mut resp = Vec::<u8, 32>::new();
                        let _ = resp.push(RESP_CODE_STATS);
                        let _ = resp.push(2);
                        let _ = resp.extend_from_slice(&recv.to_le_bytes());
                        let _ = resp.extend_from_slice(&sent.to_le_bytes());
                        let _ = resp.extend_from_slice(&sent_flood.to_le_bytes());
                        let _ = resp.extend_from_slice(&sent_direct.to_le_bytes());
                        let _ = resp.extend_from_slice(&recv_flood.to_le_bytes());
                        let _ = resp.extend_from_slice(&recv_direct.to_le_bytes());
                        let _ = resp.extend_from_slice(&recv_errors.to_le_bytes());
                        emit(&resp);
                        handled = true;
                    }
                }
                if !handled {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
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
                    REBOOT_PENDING.store(true, Ordering::Release);
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
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
                let mut ok = false;
                if payload.len() >= 5 {
                    let mut time_bytes = [0u8; 4];
                    time_bytes.copy_from_slice(&payload[1..5]);
                    let epoch = u32::from_le_bytes(time_bytes);
                    if epoch >= crate::state::get_device_time() {
                        self.epoch_time = epoch as u64;
                        crate::state::set_device_time(epoch);
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                if ok {
                    let _ = resp.push(RESP_CODE_OK);
                } else {
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                }
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
                let next_item = crate::storage::messages::pop_inbox();

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
                    if let Some(secret) = shared_secret_for_keypair(
                        &self.public_key,
                        &self.private_key,
                        &dest_key,
                    ) {
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
                                    pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
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
                                            crate::scheduler::preamble_for(sf),
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
                                        clear_pending_reqs();
                                        note_pending_req(tag, dest_key);
                                        reply = Some((direct, tag, est_timeout));
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
                use meshcore_wire::packets::txtmsg::{
                    MAX_TEXT_LEN, TXT_TYPE_CLI_COMMAND, TXT_TYPE_CLI_DATA, TXT_TYPE_PLAIN,
                };
                let mut resp = Vec::<u8, 10>::new();
                let mut err = None;
                let mut sent_info = None;

                if payload.len() < 14 {
                    err = Some(ERR_ILLEGAL_ARG);
                } else {
                    let txt_type = payload[1];
                    let attempt = payload[2];
                    let app_timestamp = u32::from_le_bytes([
                        payload[3], payload[4], payload[5], payload[6],
                    ]);
                    let mut dest_prefix = [0u8; 6];
                    dest_prefix.copy_from_slice(&payload[7..13]);

                    let path = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| &ct.pub_key[..6] == &dest_prefix[..])
                            .map(|ct| ct.out_path_len)
                    });

                    if let Some(out_path_len) = path {
                        if txt_type != TXT_TYPE_PLAIN
                            && txt_type != TXT_TYPE_CLI_DATA
                            && txt_type != TXT_TYPE_CLI_COMMAND
                        {
                            err = Some(ERR_UNSUPPORTED);
                        } else if let Ok(text_str) = core::str::from_utf8(&payload[13..]) {
                            let text_str = text_str.trim_end_matches('\0');
                            if text_str.contains('\0')
                                || text_str.len() > MAX_TEXT_LEN
                                || (attempt > 3 && text_str.len() > MAX_TEXT_LEN - 2)
                            {
                                err = Some(ERR_ILLEGAL_ARG);
                            } else {
                                let is_cli = txt_type == TXT_TYPE_CLI_DATA
                                    || txt_type == TXT_TYPE_CLI_COMMAND;
                                let send_ts = if is_cli { next_tag() } else { app_timestamp };
                                let mut s = String::<MAX_TEXT_LEN>::new();
                                if s.push_str(text_str).is_err() {
                                    err = Some(ERR_ILLEGAL_ARG);
                                } else if crate::command::USB_COMMAND_CHANNEL
                                    .try_send(
                                        crate::command::UsbCommand::SendDirectText {
                                            dest_prefix,
                                            timestamp: send_ts,
                                            txt_type,
                                            attempt,
                                            text: s,
                                        },
                                    )
                                    .is_err()
                                {
                                    err = Some(ERR_TABLE_FULL);
                                } else {
                                    sent_info = Some((
                                        txt_type,
                                        attempt,
                                        send_ts,
                                        text_str,
                                        out_path_len,
                                        is_cli,
                                    ));
                                }
                            }
                        } else {
                            err = Some(ERR_ILLEGAL_ARG);
                        }
                    } else {
                        err = Some(ERR_NOT_FOUND);
                    }
                }

                if let Some((txt_type, attempt, send_ts, text, out_path_len, is_cli)) = sent_info {
                    let is_direct = out_path_len != 0xFF
                        && Packet::is_valid_path_len(out_path_len);
                    let hops = if is_direct {
                        (out_path_len & 63) as u32
                    } else {
                        0
                    };
                    let expected_ack = if is_cli {
                        0
                    } else {
                        let flags = ((txt_type & 0x3F) << 2) | (attempt & 0x03);
                        meshcore_wire::crypto::ack_crc(
                            send_ts,
                            flags,
                            text.as_bytes(),
                            &self.public_key,
                        )
                    };

                    // Estimated on-air time of the complete encrypted packet.
                    let plain_len = 5 + text.len() + if attempt > 3 { 2 } else { 0 };
                    let cipher_len = ((plain_len + 15) / 16) * 16;
                    let path_bytes = if is_direct {
                        (out_path_len & 63) as usize
                            * (((out_path_len >> 6) & 3) as usize + 1)
                    } else {
                        0
                    };
                    let raw_len = 2 + path_bytes + 4 + cipher_len;
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
                        crate::scheduler::preamble_for(sf),
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
                    let _ = resp.push(err.unwrap_or(ERR_ILLEGAL_ARG));
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
                // Inbound: [CMD(9), public_key(32), type(1), flags(1), out_path_len(1),
                //           out_path(64)?, adv_name(32)?, last_advert(4)?,
                //           gps_lat(4)?, gps_lon(4)?, lastmod(4)?]
                // Minimum required: 1 + 32 + 1 + 1 + 1 = 36 bytes.
                let mut resp = Vec::<u8, 4>::new();
                if payload.len() < 36 {
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_ILLEGAL_ARG);
                    emit(&resp);
                    return;
                }

                let pub_key: [u8; 32] = payload[1..33].try_into().unwrap();
                let contact_type = payload[33];
                let flags = payload[34];
                let out_path_len = payload[35];

                let has_out_path = payload.len() >= 100;
                let mut out_path = [0u8; 64];
                if has_out_path {
                    out_path.copy_from_slice(&payload[36..100]);
                } else if payload.len() > 36 {
                    let n = payload.len() - 36;
                    out_path[..n].copy_from_slice(&payload[36..]);
                }

                let has_name = payload.len() >= 132;
                let mut callsign = String::<32>::new();
                if has_name {
                    let adv_name = core::str::from_utf8(&payload[100..132])
                        .unwrap_or("")
                        .trim_end_matches('\0');
                    let _ = callsign.push_str(adv_name);
                }

                let has_last_advert = payload.len() >= 136;
                let last_advert = if has_last_advert {
                    u32::from_le_bytes([
                        payload[132], payload[133], payload[134], payload[135],
                    ])
                } else {
                    0
                };

                let has_latlon = payload.len() >= 144;
                let (adv_lat, adv_lon) = if has_latlon {
                    (
                        i32::from_le_bytes([
                            payload[136], payload[137], payload[138], payload[139],
                        ]),
                        i32::from_le_bytes([
                            payload[140], payload[141], payload[142], payload[143],
                        ]),
                    )
                } else {
                    (0, 0)
                };

                let lastmod = if payload.len() >= 148 {
                    u32::from_le_bytes([
                        payload[144], payload[145], payload[146], payload[147],
                    ])
                } else {
                    crate::state::get_device_time()
                };

                let node_id = [pub_key[0], pub_key[1], pub_key[2], pub_key[3]];
                let overwrite_when_full = self.should_overwrite_when_full();
                let mut overwritten_key: Option<[u8; 32]> = None;

                let ok = crate::state::DISCOVERED_CONTACTS.lock(|c| {
                    let mut list = c.borrow_mut();
                    if let Some(ct) = list.iter_mut().find(|ct| ct.pub_key == pub_key) {
                        ct.node_id = node_id;
                        ct.contact_type = contact_type;
                        ct.flags = flags;
                        ct.out_path_len = out_path_len;
                        if payload.len() > 36 {
                            ct.out_path = out_path;
                        }
                        if has_name {
                            ct.callsign = callsign;
                        }
                        if has_last_advert {
                            ct.last_advert_ts = last_advert;
                        }
                        if has_latlon {
                            ct.latitude = adv_lat;
                            ct.longitude = adv_lon;
                        }
                        ct.last_seen_epoch = lastmod;
                        true
                    } else {
                        let contact = crate::types::DiscoveredContact {
                            pub_key,
                            callsign,
                            node_id,
                            last_seen_epoch: lastmod,
                            rssi: 0,
                            snr: 0,
                            latitude: adv_lat,
                            longitude: adv_lon,
                            flags,
                            contact_type,
                            out_path,
                            out_path_len,
                            last_advert_ts: last_advert,
                            saved_hash: 0,
                            sync_since: 0,
                        };
                        if list.len() < list.capacity() {
                            list.push(contact).is_ok()
                        } else if overwrite_when_full {
                            let mut oldest_idx = None;
                            let mut oldest_lastmod = u32::MAX;
                            for (idx, ct) in list.iter().enumerate() {
                                if (ct.flags & CONTACT_FLAG_FAVORITE) == 0
                                    && ct.last_seen_epoch < oldest_lastmod
                                {
                                    oldest_lastmod = ct.last_seen_epoch;
                                    oldest_idx = Some(idx);
                                }
                            }
                            if let Some(idx) = oldest_idx {
                                overwritten_key = Some(list[idx].pub_key);
                                list[idx] = contact;
                                true
                            } else {
                                false
                            }
                        } else {
                            false
                        }
                    }
                });

                if let Some(evicted) = overwritten_key {
                    let mut push = Vec::<u8, 40>::new();
                    let _ = push.push(PUSH_CODE_CONTACT_DELETED);
                    let _ = push.extend_from_slice(&evicted);
                    crate::command::push_companion(&push);
                    crate::storage::adverts::forget(&evicted);
                    let _ = crate::storage::contacts::OP_CHANNEL
                        .try_send(crate::storage::contacts::ContactOp::Remove(evicted));
                }

                if ok {
                    crate::storage::request_contacts();
                    let _ = resp.push(RESP_CODE_OK);
                } else {
                    let _ = resp.push(RESP_CODE_ERR);
                    let _ = resp.push(ERR_TABLE_FULL);
                }
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
                            if let Some(id) = meshcore_wire::identity::LocalIdentity::from_keypair(
                                self.callsign.as_str(),
                                &self.public_key,
                                &self.private_key,
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
                    let id = meshcore_wire::identity::LocalIdentity::from_keypair(
                        self.callsign.as_str(),
                        &self.public_key,
                        &self.private_key,
                    );
                    let mut sent = false;
                    if let Some(id) = id {
                        let ts = crate::state::get_device_time();
                        let (adv_lat, adv_lon) = advert_location(self.latitude, self.longitude);
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
                        if let Ok(pkt) = adv.to_packet(meshcore_wire::packet::ROUTE_TYPE_FLOOD) {
                            let mut raw = [0u8; 184];
                            if let Ok(n) = pkt.write_to(&mut raw) {
                                let mut resp = Vec::<u8, 185>::new();
                                let _ = resp.push(RESP_CODE_EXPORT_CONTACT);
                                let _ = resp.extend_from_slice(&raw[..n]);
                                emit(&resp);
                                sent = true;
                            }
                        }
                    }
                    if !sent {
                        emit(&[RESP_CODE_ERR, ERR_TABLE_FULL]);
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
                        let _ = resp.push(ERR_NOT_FOUND);
                    }
                    emit(&resp);
                }
            }

            CMD_IMPORT_CONTACT => {
                // Loop the imported advert back through the normal receive path
                // (matching upstream BaseChatMesh::importContact), so replay
                // checks, auto-add type/hop rules, blob storage, advert-path
                // caching, and discovery pushes all apply identically and
                // `out_path_len` stays unknown (0xFF).
                let mut err = Some(ERR_ILLEGAL_ARG);
                if payload.len() > 2 + 32 + 64 {
                    let raw_slice = &payload[1..];
                    if raw_slice.len() <= 255 {
                        if let Ok(pkt) = Packet::read_from(raw_slice) {
                            if pkt.payload_type() == PAYLOAD_TYPE_ADVERT {
                                if let Ok(adv) =
                                    AdvertPacket::decode(&pkt.payload[..pkt.payload_len])
                                {
                                    if adv.verify_signature() {
                                        let mut raw = Vec::<u8, 255>::new();
                                        let _ = raw.extend_from_slice(raw_slice);
                                        if crate::command::USB_COMMAND_CHANNEL
                                            .try_send(crate::command::UsbCommand::ImportContact {
                                                raw,
                                            })
                                            .is_ok()
                                        {
                                            err = None;
                                        } else {
                                            err = Some(ERR_TABLE_FULL);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                match err {
                    None => emit(&[RESP_CODE_OK]),
                    Some(code) => emit(&[RESP_CODE_ERR, code]),
                }
            }

            CMD_SET_OTHER_PARAMS => {
                let ok = self.apply_other_params(payload);
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
                        if payload.len() >= 18 && payload[2..18].iter().any(|&b| b != 0) {
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
                        if let Some(secret) = shared_secret_for_keypair(
                            &self.public_key,
                            &self.private_key,
                            &dest_key,
                        ) {
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
                                        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
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
                                                crate::scheduler::preamble_for(sf),
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
                                            clear_pending_reqs();
                                            note_pending_login(dest_key);
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
                    let mut push = Vec::<u8, 256>::new();
                    let _ = push.push(PUSH_CODE_TELEMETRY_RESPONSE);
                    let _ = push.push(0);
                    let _ = push.extend_from_slice(&self.public_key[..6]);
                    self.encode_self_telemetry_lpp(0xFF, &mut push);
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
                        if let Some(secret) = shared_secret_for_keypair(
                            &self.public_key,
                            &self.private_key,
                            &dest_key,
                        ) {
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
                                        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
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
                                                crate::scheduler::preamble_for(sf),
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
                                            clear_pending_reqs();
                                            note_pending_telemetry(tag, dest_key);
                                            reply = Some((direct, tag, est_timeout.max(6000)));
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
                        if let Some(secret) = shared_secret_for_keypair(
                            &self.public_key,
                            &self.private_key,
                            &dest_key,
                        ) {
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
                                    pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
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
                                                crate::scheduler::preamble_for(sf),
                                            );
                                            let est_timeout = 500 + 16 * air_ms;
                                            clear_pending_reqs();
                                            note_pending_discovery(tag, dest_key);
                                            reply = Some((tag, est_timeout.max(15000)));
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
                        if let Some(secret) = shared_secret_for_keypair(
                            &self.public_key,
                            &self.private_key,
                            &dest_key,
                        ) {
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
                                        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
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
                                                crate::scheduler::preamble_for(sf),
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
                                            clear_pending_reqs();
                                            note_pending_req(tag, dest_key);
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
                        if let Some(secret) = shared_secret_for_keypair(
                            &self.public_key,
                            &self.private_key,
                            &dest_key,
                        ) {
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
                                        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
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
                                                crate::scheduler::preamble_for(sf),
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
                                            clear_pending_reqs();
                                            note_status(dest_key);
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
                        let _ = resp.push(ERR_NOT_FOUND);
                    }
                }
                emit(&resp);
            }

            CMD_SEND_TRACE_PATH => {
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
                                crate::scheduler::preamble_for(sf),
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

    fn build_self_info(&self) -> Vec<u8, 128> {
        // RESP_CODE_SELF_INFO (5)
        // Layout:
        // [code(5), type, tx_power, max_tx_power, pubkey(32), adv_lat(i32_le), adv_lon(i32_le),
        //  multi_acks(0..=1), advert_loc_policy(0), telemetry_modes(0), manual_add_contacts(0),
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
        let (lat_e6, lon_e6) = crate::state::advert_lat_lon_e6();
        let _ = resp.extend_from_slice(&lat_e6.to_le_bytes()); // adv_lat
        let _ = resp.extend_from_slice(&lon_e6.to_le_bytes()); // adv_lon
        let _ = resp.push(self.multi_acks.min(1)); // multi_acks (v7+: 0 or 1)
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
        resp
    }

    fn apply_other_params(&mut self, payload: &[u8]) -> bool {
        if payload.len() < 2 {
            return false;
        }
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
                payload[4].min(1)
            } else {
                self.multi_acks.min(1)
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
        true
    }

    /// Encode this node's Cayenne LPP telemetry payload into `out` according
    /// to `permissions` (`TELEM_PERM_BASE`, `TELEM_PERM_LOCATION`,
    /// `TELEM_PERM_ENVIRONMENT`) for both self-telemetry and inbound
    /// `REQ_TYPE_GET_TELEMETRY_DATA` responses.
    pub fn encode_self_telemetry_lpp<const N: usize>(&self, permissions: u8, out: &mut Vec<u8, N>) {
        if (permissions & TELEM_PERM_BASE) == 0 {
            return;
        }
        // 1. Battery voltage: channel 1, type 0x74 (LPP_VOLTAGE), 0.01 V BE.
        let mv = self.host.battery_millivolts();
        let scaled = ((mv as u32 + 5) / 10) as u16;
        let _ = out.extend_from_slice(&[
            TELEM_CHANNEL_SELF,
            LPP_VOLTAGE,
            (scaled >> 8) as u8,
            scaled as u8,
        ]);

        // 2. GPS location: channel 1, type 0x88 (LPP_GPS), 9 bytes:
        //    24-bit signed BE lat * 10000, lon * 10000, alt * 100.
        if (permissions & TELEM_PERM_LOCATION) != 0 {
            let gps = crate::state::gps_state();
            let loc = if gps.fix {
                Some((gps.latitude, gps.longitude, gps.altitude_m))
            } else if self.latitude != 0.0 || self.longitude != 0.0 {
                Some((self.latitude, self.longitude, 0.0))
            } else {
                None
            };
            if let Some((lat, lon, alt)) = loc {
                let lat_raw = round_f32_to_i32(lat * 10_000.0);
                let lon_raw = round_f32_to_i32(lon * 10_000.0);
                let alt_raw = round_f32_to_i32(alt * 100.0);
                let _ = out.extend_from_slice(&[
                    TELEM_CHANNEL_SELF,
                    LPP_GPS,
                    (lat_raw >> 16) as u8,
                    (lat_raw >> 8) as u8,
                    lat_raw as u8,
                    (lon_raw >> 16) as u8,
                    (lon_raw >> 8) as u8,
                    lon_raw as u8,
                    (alt_raw >> 16) as u8,
                    (alt_raw >> 8) as u8,
                    alt_raw as u8,
                ]);
            }
        }

        // 3. Built-in MCU temperature: channel 1, type 0x67 (LPP_TEMPERATURE),
        //    signed 16-bit BE in 0.1 °C units.
        if let Some(tenths_c) = self.host.mcu_temperature_tenths_c() {
            let [hi, lo] = tenths_c.to_be_bytes();
            let _ = out.extend_from_slice(&[TELEM_CHANNEL_SELF, LPP_TEMPERATURE, hi, lo]);
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

    struct TempHost {
        mv: u16,
        temp_tenths_c: Option<i16>,
    }

    impl CompanionHost for TempHost {
        fn battery_millivolts(&self) -> u16 {
            self.mv
        }
        fn mcu_temperature_tenths_c(&self) -> Option<i16> {
            self.temp_tenths_c
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

    fn decode_i24_be(bytes: &[u8]) -> i32 {
        let raw = ((bytes[0] as i32) << 16) | ((bytes[1] as i32) << 8) | (bytes[2] as i32);
        if (raw & 0x80_0000) != 0 {
            raw | !0xFF_FFFF
        } else {
            raw
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

    #[test]
    fn multi_acks_clamped_in_set_other_params_and_reported_in_self_info() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut node = CompanionNode::new(
            "test",
            "test-board",
            0,
            [1u8; 32],
            [2u8; 64],
            &TestHost,
        );

        assert!(node.apply_other_params(&[CMD_SET_OTHER_PARAMS, 0, 0, 0, 7]));
        assert_eq!(node.multi_acks, 1);
        assert_eq!(crate::state::prefs().multi_acks, 1);

        let self_info = node.build_self_info();
        assert_eq!(self_info[0], RESP_CODE_SELF_INFO);
        assert_eq!(self_info[44], 1);

        assert!(node.apply_other_params(&[CMD_SET_OTHER_PARAMS, 0, 0, 0, 0]));
        assert_eq!(node.multi_acks, 0);
        assert_eq!(crate::state::prefs().multi_acks, 0);
        assert_eq!(node.build_self_info()[44], 0);
    }

    fn device_info_model_field(hw_model: &'static str) -> [u8; 40] {
        let mut node = CompanionNode::new("test", hw_model, 0, [1u8; 32], [2u8; 64], &TestHost);
        let mut resp = Vec::<u8, 128>::new();
        poll_sync(node.handle_command(&[CMD_DEVICE_QUERY, 3], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp[0], RESP_CODE_DEVICE_INFO);
        // code, fw_ver, max_contacts/2, max_channels, ble_pin(4), build_date(12)
        resp[20..60].try_into().unwrap()
    }

    #[test]
    fn device_info_reports_the_board_model() {
        let field = device_info_model_field("Seeed Wio Tracker L1");
        let mut expected = [0u8; 40];
        expected[..20].copy_from_slice(b"Seeed Wio Tracker L1");
        assert_eq!(field, expected);
    }

    #[test]
    fn device_info_truncates_an_oversized_model() {
        let field = device_info_model_field(
            "0123456789012345678901234567890123456789-overflow-is-dropped",
        );
        assert_eq!(&field, b"0123456789012345678901234567890123456789");
    }

    #[test]
    fn self_telemetry_lpp_encodes_voltage_gps_and_temperature() {
        crate::state::clear_gps_state();
        let host = TempHost {
            mv: 4_123, // rounds to 412 (4.12 V -> 0x019C)
            temp_tenths_c: Some(235), // 23.5 °C -> 0x00EB
        };
        let mut node = CompanionNode::new(
            "test",
            "test-board",
            0,
            [1u8; 32],
            [2u8; 64],
            &host,
        );
        node.latitude = 37.7749;
        node.longitude = -122.4194;

        let mut lpp = Vec::<u8, 64>::new();
        node.encode_self_telemetry_lpp(0xFF, &mut lpp);

        // Voltage (4B) + GPS (11B) + Temperature (4B) = 19 bytes.
        assert_eq!(lpp.len(), 19);
        assert_eq!(&lpp[0..4], &[1, LPP_VOLTAGE, 0x01, 0x9C]);
        assert_eq!(lpp[4], 1);
        assert_eq!(lpp[5], LPP_GPS);
        assert_eq!(decode_i24_be(&lpp[6..9]), 377_749);
        assert_eq!(decode_i24_be(&lpp[9..12]), -1_224_194);
        assert_eq!(decode_i24_be(&lpp[12..15]), 0);
        assert_eq!(&lpp[15..19], &[1, LPP_TEMPERATURE, 0x00, 0xEB]);
    }

    use crate::state::TEST_LOCK;

    fn poll_sync<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, Waker};
        let mut cx = Context::from_waker(Waker::noop());
        let mut fut = core::pin::pin!(fut);
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(out) => out,
            Poll::Pending => panic!("unexpected Pending in synchronous test"),
        }
    }

    fn drain_usb_commands() {
        while crate::command::USB_COMMAND_CHANNEL.try_receive().is_ok() {}
    }

    #[test]
    fn contact_request_permissions_and_inverse_mask() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::state::clear_gps_state();
        let host = TempHost {
            mv: 3_850, // 385 -> 0x0181
            temp_tenths_c: Some(-42), // -4.2 °C -> 0xFFD6
        };
        let mut node = CompanionNode::new(
            "test",
            "test-board",
            0,
            [1u8; 32],
            [2u8; 64],
            &host,
        );
        node.latitude = 48.8566;
        node.longitude = 2.3522;

        let sender = [0x44u8; 32];
        crate::state::DISCOVERED_CONTACTS.lock(|c| {
            let mut list = c.borrow_mut();
            list.clear();
            let mut cs = String::new();
            let _ = cs.push_str("peer");
            let _ = list.push(crate::types::DiscoveredContact {
                pub_key: sender,
                callsign: cs,
                node_id: [0x44; 4],
                last_seen_epoch: 0,
                rssi: 0,
                snr: 0,
                latitude: 0,
                longitude: 0,
                // bit 0 = favorite, bit 1 = base perm (cp bit 0), bit 2 = loc perm (cp bit 1)
                flags: 0b0000_0011,
                contact_type: 1,
                out_path: [0u8; 64],
                out_path_len: 0xFF,
                last_advert_ts: 0,
                saved_hash: 0,
                sync_since: 0,
            });
        });

        // 1. Default modes are DENY (0) -> no response.
        update_prefs(|p| {
            p.telemetry_mode_base = TELEM_MODE_DENY;
            p.telemetry_mode_loc = TELEM_MODE_DENY;
            p.telemetry_mode_env = TELEM_MODE_DENY;
        });
        let req = [0x78, 0x56, 0x34, 0x12, REQ_TYPE_GET_TELEMETRY_DATA, 0x00];
        assert!(handle_contact_request(&node, &req, &sender).is_none());

        // 2. ALLOW_FLAGS: contact has base perm (flags bit 1) but not loc perm (flags bit 2).
        update_prefs(|p| {
            p.telemetry_mode_base = TELEM_MODE_ALLOW_FLAGS;
            p.telemetry_mode_loc = TELEM_MODE_ALLOW_FLAGS;
        });
        let reply = handle_contact_request(&node, &req, &sender).expect("base permitted by flags");
        // Tag (4B) + Voltage (4B) + Temp (4B) = 12B (no GPS since loc flag not set on contact).
        assert_eq!(reply.len(), 12);
        assert_eq!(&reply[0..4], &[0x78, 0x56, 0x34, 0x12]);
        assert_eq!(&reply[4..8], &[1, LPP_VOLTAGE, 0x01, 0x81]);
        assert_eq!(&reply[8..12], &[1, LPP_TEMPERATURE, 0xFF, 0xD6]);

        // 3. ALLOW_ALL for both base and location -> includes GPS, UNLESS masked out by Path Discovery (~0x01).
        update_prefs(|p| {
            p.telemetry_mode_base = TELEM_MODE_ALLOW_ALL;
            p.telemetry_mode_loc = TELEM_MODE_ALLOW_ALL;
        });
        let full_reply = handle_contact_request(&node, &req, &sender).expect("base + loc permitted");
        assert_eq!(full_reply.len(), 23); // Tag (4B) + Voltage (4B) + GPS (11B) + Temp (4B)

        // Path discovery request sets req[5] = !TELEM_PERM_BASE (0xFE) -> strips location!
        let disc_req = [0xAA, 0xBB, 0xCC, 0xDD, REQ_TYPE_GET_TELEMETRY_DATA, !TELEM_PERM_BASE];
        let disc_reply = handle_contact_request(&node, &disc_req, &sender).expect("discovery base reply");
        assert_eq!(disc_reply.len(), 12);
        assert_eq!(&disc_reply[0..4], &[0xAA, 0xBB, 0xCC, 0xDD]);

        // Restore default prefs.
        update_prefs(|p| *p = DEFAULT_PREFS);
    }

    #[test]
    fn unhandled_response_forwards_as_binary_response() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        clear_pushes();
        clear_pending_reqs();
        let sender = [0x99u8; 32];
        let plain = [0x11, 0x22, 0x33, 0x44, 0x01, 0x74, 0x01, 0x90];
        assert!(handle_response(&plain, &sender));
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        let pushed = &pushes[0];
        assert_eq!(pushed[0], PUSH_CODE_BINARY_RESPONSE);
        assert_eq!(pushed[1], 0);
        assert_eq!(&pushed[2..], &plain[..]);
    }

    #[test]
    fn login_fail_push_and_response_slot_precedence() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        clear_pushes();
        clear_pending_reqs();

        let peer = [0x7Au8; 32];
        let req_tag = 0x1234_5678u32;
        note_pending_login(peer);
        note_pending_req(req_tag, peer);

        // Even with PENDING_LOGIN active for the same peer, tag-matched
        // PENDING_REQ wins first and leaves PENDING_LOGIN intact.
        let mut req_reply = [0u8; 6];
        req_reply[0..4].copy_from_slice(&req_tag.to_le_bytes());
        req_reply[4] = 0xAA;
        req_reply[5] = 0xBB;
        assert!(handle_response(&req_reply, &peer));
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        let pushed = &pushes[0];
        assert_eq!(pushed[0], PUSH_CODE_BINARY_RESPONSE);
        assert_eq!(&pushed[2..6], &req_tag.to_le_bytes());
        assert_eq!(&pushed[6..8], &[0xAA, 0xBB]);

        // Now deliver a failed server login response (status byte != 0 and != "OK").
        let login_fail = [0x10, 0x00, 0x00, 0x00, 0x01];
        assert!(handle_response(&login_fail, &peer));
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        let fail_push = &pushes[0];
        assert_eq!(fail_push[0], PUSH_CODE_LOGIN_FAIL);
        assert_eq!(fail_push[1], 0);
        assert_eq!(&fail_push[2..8], &peer[..6]);
        assert_eq!(fail_push.len(), 8);
    }

    #[test]
    fn status_response_matches_peer_with_repeater_clock_and_rejects_empty() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        clear_pushes();
        clear_pending_reqs();

        let peer = [0x61u8; 32];
        let other = [0x62u8; 32];
        note_status(peer);

        // A 4-byte reply with no status payload must not consume PENDING_STATUS or emit a push.
        let empty = [0xDE, 0xAD, 0xBE, 0xEF];
        assert!(!handle_response(&empty, &peer));
        assert!(take_pushes().is_empty());

        // A reply from a different sender falls through as binary response without consuming PENDING_STATUS.
        let other_reply = [0x01, 0x02, 0x03, 0x04, 0xAA, 0xBB];
        assert!(handle_response(&other_reply, &other));
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        assert_eq!(pushes[0][0], PUSH_CODE_BINARY_RESPONSE);

        // Repeater status reply with its own clock in bytes 0..4 matches by peer pub_key
        // and immediately emits PUSH_CODE_STATUS_RESPONSE (0x87).
        let repeater_reply = [0x78, 0x56, 0x34, 0x12, 0xE8, 0x0F, 0x01, 0x02];
        assert!(handle_response(&repeater_reply, &peer));
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        assert_eq!(pushes[0][0], PUSH_CODE_STATUS_RESPONSE);
        assert_eq!(pushes[0][1], 0);
        assert_eq!(&pushes[0][2..8], &peer[..6]);
        assert_eq!(&pushes[0][8..], &repeater_reply[4..]);

        // Slot was consumed; a subsequent reply does not emit another status response.
        assert!(handle_response(&repeater_reply, &peer));
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        assert_eq!(pushes[0][0], PUSH_CODE_BINARY_RESPONSE);
    }

    #[test]
    fn request_commands_clear_stale_pending_slots() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        drain_usb_commands();
        clear_pushes();
        clear_pending_reqs();

        let local = meshcore_wire::identity::LocalIdentity::from_seed("Node", &[0x41u8; 32]);
        let peer_id = meshcore_wire::identity::LocalIdentity::from_seed("Peer", &[0x42u8; 32]);
        let peer = *peer_id.pub_key();

        crate::state::DISCOVERED_CONTACTS.lock(|c| {
            let mut list = c.borrow_mut();
            list.clear();
            let _ = list.push(crate::types::DiscoveredContact {
                pub_key: peer,
                callsign: String::new(),
                node_id: [peer[0], peer[1], peer[2], peer[3]],
                last_seen_epoch: 100,
                rssi: 0,
                snr: 0,
                latitude: 0,
                longitude: 0,
                flags: 0,
                contact_type: meshcore_wire::packets::advert::ADV_TYPE_REPEATER,
                out_path: [0u8; 64],
                out_path_len: 0,
                last_advert_ts: 100,
                sync_since: 0,
                saved_hash: 0,
            });
        });

        let mut node = CompanionNode::new(
            "Node",
            "Board",
            0,
            local.pub_key_bytes(),
            local.private_key_bytes(),
            &TestHost,
        );

        let send_cmd = |node: &mut CompanionNode<'_>, cmd: &[u8]| {
            let mut resp = Vec::<u8, 32>::new();
            poll_sync(node.handle_command(cmd, |r| {
                let _ = resp.extend_from_slice(r);
            }));
            assert_eq!(resp[0], RESP_CODE_SENT);
            drain_usb_commands();
        };

        // 1. CMD_SEND_LOGIN sets PENDING_LOGIN, clearing any stale status/disc/req/telem.
        note_status(peer);
        note_pending_telemetry(1, peer);
        note_pending_discovery(2, peer);
        note_pending_req(3, peer);
        let mut login_cmd = [0u8; 37];
        login_cmd[0] = CMD_SEND_LOGIN;
        login_cmd[1..33].copy_from_slice(&peer);
        login_cmd[33..37].copy_from_slice(b"pass");
        send_cmd(&mut node, &login_cmd);
        assert!(PENDING_LOGIN.lock(|p| p.borrow().is_some()));
        assert!(PENDING_STATUS.lock(|p| p.borrow().is_none()));
        assert!(PENDING_TELEMETRY.lock(|p| p.borrow().is_none()));
        assert!(PENDING_DISCOVERY.lock(|p| p.borrow().is_none()));
        assert!(PENDING_REQ.lock(|p| p.borrow().is_none()));

        // 2. CMD_SEND_STATUS_REQ clears PENDING_LOGIN and sets only PENDING_STATUS.
        let mut status_cmd = [0u8; 33];
        status_cmd[0] = CMD_SEND_STATUS_REQ;
        status_cmd[1..33].copy_from_slice(&peer);
        send_cmd(&mut node, &status_cmd);
        assert!(PENDING_STATUS.lock(|p| *p.borrow() == Some(peer)));
        assert!(PENDING_LOGIN.lock(|p| p.borrow().is_none()));

        // 3. CMD_SEND_PATH_DISCOVERY_REQ clears PENDING_STATUS and sets PENDING_DISCOVERY.
        let mut disc_cmd = [0u8; 34];
        disc_cmd[0] = CMD_SEND_PATH_DISCOVERY_REQ;
        disc_cmd[1] = 0;
        disc_cmd[2..34].copy_from_slice(&peer);
        send_cmd(&mut node, &disc_cmd);
        assert!(PENDING_DISCOVERY.lock(|p| p.borrow().is_some()));
        assert!(PENDING_STATUS.lock(|p| p.borrow().is_none()));

        // 4. CMD_SEND_TELEMETRY_REQ clears PENDING_DISCOVERY and sets PENDING_TELEMETRY.
        let mut telem_cmd = [0u8; 36];
        telem_cmd[0] = CMD_SEND_TELEMETRY_REQ;
        telem_cmd[4..36].copy_from_slice(&peer);
        send_cmd(&mut node, &telem_cmd);
        assert!(PENDING_TELEMETRY.lock(|p| p.borrow().is_some()));
        assert!(PENDING_DISCOVERY.lock(|p| p.borrow().is_none()));

        // 5. CMD_SEND_BINARY_REQ clears PENDING_TELEMETRY and sets PENDING_REQ.
        let mut bin_cmd = [0u8; 35];
        bin_cmd[0] = CMD_SEND_BINARY_REQ;
        bin_cmd[1..33].copy_from_slice(&peer);
        bin_cmd[33] = 0x01;
        bin_cmd[34] = 0x02;
        send_cmd(&mut node, &bin_cmd);
        assert!(PENDING_REQ.lock(|p| p.borrow().is_some()));
        assert!(PENDING_TELEMETRY.lock(|p| p.borrow().is_none()));

        // 6. CMD_SEND_ANON_REQ clears stale login/discovery/status and sets PENDING_REQ.
        note_pending_login(peer);
        note_status(peer);
        let mut anon_cmd = [0u8; 35];
        anon_cmd[0] = CMD_SEND_ANON_REQ;
        anon_cmd[1..33].copy_from_slice(&peer);
        anon_cmd[33] = 0xAA;
        anon_cmd[34] = 0xBB;
        send_cmd(&mut node, &anon_cmd);
        assert!(PENDING_REQ.lock(|p| p.borrow().is_some()));
        assert!(PENDING_LOGIN.lock(|p| p.borrow().is_none()));
        assert!(PENDING_STATUS.lock(|p| p.borrow().is_none()));

        clear_pending_reqs();
    }

    #[test]
    fn path_discovery_validates_path_lens_and_slice_bounds() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        clear_pushes();
        clear_pending_reqs();

        let peer = [0x88u8; 32];
        let tag = 0xAABB_CCDDu32;
        note_pending_discovery(tag, peer);

        // Invalid path_len (hash_size == 4 -> 0xC0 / 0xFF, or byte_len > 64 -> 0x40 | 33 = 66 B).
        assert!(!handle_path_discovery(tag, &peer, 0xFF, &[0; 64], 0x00, &[]));
        assert!(!handle_path_discovery(tag, &peer, 0xC0, &[0; 64], 0x00, &[]));
        assert!(!handle_path_discovery(tag, &peer, 0x40 | 33, &[0; 64], 0x00, &[]));
        assert!(!handle_path_discovery(tag, &peer, 0x00, &[], 0xFF, &[0; 64]));
        assert!(!handle_path_discovery(tag, &peer, 0x00, &[], 0x80 | 22, &[0; 64]));

        // Truncated path slices (declared byte len exceeds provided slice).
        assert!(!handle_path_discovery(tag, &peer, 0x02, &[0x11], 0x00, &[]));
        assert!(!handle_path_discovery(tag, &peer, 0x42, &[0x11, 0x22, 0x33], 0x00, &[]));
        assert!(!handle_path_discovery(tag, &peer, 0x00, &[], 0x02, &[0xAA]));

        // None of the invalid attempts may consume PENDING_DISCOVERY or emit a push.
        assert!(PENDING_DISCOVERY.lock(|p| *p.borrow() == Some((tag, peer))));
        assert!(take_pushes().is_empty());

        // Valid multi-byte out_path (2 hops * 2 B = 4 B -> 0x42) and 1-byte in_path (1 hop -> 0x01).
        let out_path = [0x11, 0x22, 0x33, 0x44, 0x99];
        let in_path = [0x55, 0x77];
        assert!(handle_path_discovery(tag, &peer, 0x42, &out_path, 0x01, &in_path));
        assert!(PENDING_DISCOVERY.lock(|p| p.borrow().is_none()));

        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        let p = &pushes[0];
        assert_eq!(p[0], PUSH_CODE_PATH_DISCOVERY_RESP);
        assert_eq!(p[1], 0);
        assert_eq!(&p[2..8], &peer[..6]);
        assert_eq!(p[8], 0x42);
        assert_eq!(&p[9..13], &[0x11, 0x22, 0x33, 0x44]);
        assert_eq!(p[13], 0x01);
        assert_eq!(&p[14..15], &[0x55]);
        assert_eq!(p.len(), 15);

        // Zero-hop discovery (out_path_len = 0, in_path_len = 0) is also valid.
        note_pending_discovery(tag + 1, peer);
        assert!(handle_path_discovery(tag + 1, &peer, 0, &[], 0, &[]));
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        assert_eq!(pushes[0].as_slice(), &[
            PUSH_CODE_PATH_DISCOVERY_RESP, 0,
            0x88, 0x88, 0x88, 0x88, 0x88, 0x88,
            0, 0
        ]);
    }

    #[test]
    fn pending_anon_peer_matches_hash_prefix() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        PENDING_REQ.lock(|p| *p.borrow_mut() = None);
        let peer = [0x5Cu8; 32];
        assert!(pending_anon_peer(0x5C).is_none());
        note_pending_req(42, peer);
        assert_eq!(pending_anon_peer(0x5C), Some(peer));
        assert!(pending_anon_peer(0x5D).is_none());
        PENDING_REQ.lock(|p| *p.borrow_mut() = None);
    }

    #[test]
    fn cmd_send_msg_validation_and_cli_expected_ack_zero() {
        use meshcore_wire::packets::txtmsg::{TXT_TYPE_CLI_COMMAND, TXT_TYPE_PLAIN};
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        drain_usb_commands();

        let peer = [0x31u8; 32];
        crate::state::DISCOVERED_CONTACTS.lock(|c| {
            let mut list = c.borrow_mut();
            list.clear();
            let _ = list.push(crate::types::DiscoveredContact {
                pub_key: peer,
                callsign: String::new(),
                node_id: [0x31; 4],
                last_seen_epoch: 100,
                rssi: 0,
                snr: 0,
                latitude: 0,
                longitude: 0,
                flags: 0,
                contact_type: 1,
                out_path: [0u8; 64],
                out_path_len: 0xFF,
                last_advert_ts: 0,
                saved_hash: 0,
                sync_since: 0,
            });
        });

        let mut node = CompanionNode::new("n", "b", 0, [1u8; 32], [2u8; 64], &TestHost);

        // 1. Unknown contact -> ERR_NOT_FOUND.
        let mut frame = Vec::<u8, 64>::new();
        let _ = frame.push(CMD_SEND_MSG);
        let _ = frame.push(TXT_TYPE_PLAIN);
        let _ = frame.push(0);
        let _ = frame.extend_from_slice(&1000u32.to_le_bytes());
        let _ = frame.extend_from_slice(&[0x99; 6]);
        let _ = frame.extend_from_slice(b"hi");
        let mut resp = Vec::<u8, 32>::new();
        poll_sync(node.handle_command(&frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_ERR, ERR_NOT_FOUND]);

        // 2. Unsupported txt_type -> ERR_UNSUPPORTED.
        frame[1] = 2; // TXT_TYPE_SIGNED_PLAIN not allowed from companion
        frame[7..13].copy_from_slice(&peer[..6]);
        resp.clear();
        poll_sync(node.handle_command(&frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_ERR, ERR_UNSUPPORTED]);

        // 3. CLI command -> RESP_CODE_SENT with expected_ack == 0 and monotonic tag.
        crate::state::set_device_time(2_000_000);
        frame[1] = TXT_TYPE_CLI_COMMAND;
        resp.clear();
        poll_sync(node.handle_command(&frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 10);
        assert_eq!(resp[0], RESP_CODE_SENT);
        assert_eq!(resp[1], 1); // flood (out_path_len = 0xFF)
        assert_eq!(&resp[2..6], &[0, 0, 0, 0]);
        drain_usb_commands();

        // 4. Plain message -> non-zero expected_ack.
        frame[1] = TXT_TYPE_PLAIN;
        resp.clear();
        poll_sync(node.handle_command(&frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 10);
        assert_eq!(resp[0], RESP_CODE_SENT);
        assert_ne!(&resp[2..6], &[0, 0, 0, 0]);
        drain_usb_commands();
    }

    /// `EngineContext` for a two-party round trip: one known peer at a fixed path.
    struct PairContext {
        peer: [u8; 32],
        out_path_len: u8,
    }

    impl crate::engine::EngineContext for PairContext {
        fn peers_by_hash(&self, hash: u8) -> heapless::Vec<[u8; 32], 32> {
            let mut peers = heapless::Vec::new();
            if self.peer[0] == hash {
                let _ = peers.push(self.peer);
            }
            peers
        }

        fn peer_path(&self, _pub_key: &[u8; 32]) -> (u8, [u8; 64]) {
            (self.out_path_len, [0x77; 64])
        }
    }

    /// What the companion tells the phone about a send must match what goes on
    /// the air: `RESP_CODE_SENT` carries `expected_ack` (the phone waits for that
    /// CRC) and `est_timeout` (its retry timer). Both are computed in the
    /// companion, independently of the engine that builds the packet and of the
    /// receiver that builds the ACK, so check all three agree.
    #[test]
    fn sent_response_matches_the_engine_packet_and_the_receivers_ack() {
        use crate::engine::{InboundAction, MeshEngine};
        use meshcore_wire::identity::LocalIdentity;
        use meshcore_wire::packet::{Packet, PAYLOAD_TYPE_ACK, ROUTE_TYPE_DIRECT};
        use meshcore_wire::packets::advert::ADV_TYPE_CHAT;
        use meshcore_wire::packets::txtmsg::TXT_TYPE_PLAIN;
        use meshcore_wire::packets::AckPacket;

        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let alice = LocalIdentity::from_seed("alice", &[1u8; 32]);
        let bob = LocalIdentity::from_seed("bob", &[2u8; 32]);
        let alice_pub = alice.pub_key_bytes();
        let bob_pub = bob.pub_key_bytes();
        assert_ne!(alice_pub[..6], bob_pub[..6]);

        let mut node = CompanionNode::new("alice", "b", 0, [1u8; 32], [2u8; 64], &TestHost);
        node.public_key = alice_pub;

        // Text lengths either side of the 16-byte AES block boundaries for the
        // normal (5 + len) and extended-attempt (7 + len) plaintext layouts.
        let lengths = [1usize, 9, 10, 11, 12, 26, 27, 28, 100];
        let attempts = [0u8, 1, 3, 4, 5];
        // Unknown path (flood), zero-hop direct, and one-hop direct.
        let paths = [0xFFu8, 0, 1];

        let mut ts = 1_700_000_000u32;
        for &out_path_len in &paths {
            // Fresh engines per path so duplicate suppression never interferes.
            let ctx_a = PairContext { peer: bob_pub, out_path_len };
            let ctx_b = PairContext { peer: alice_pub, out_path_len: 0 };
            let mut engine_a = MeshEngine::new(
                LocalIdentity::from_seed("alice", &[1u8; 32]),
                ADV_TYPE_CHAT,
                &ctx_a,
            );
            let mut engine_b = MeshEngine::new(
                LocalIdentity::from_seed("bob", &[2u8; 32]),
                ADV_TYPE_CHAT,
                &ctx_b,
            );

            crate::state::DISCOVERED_CONTACTS.lock(|c| {
                let mut list = c.borrow_mut();
                list.clear();
                let _ = list.push(crate::types::DiscoveredContact {
                    pub_key: bob_pub,
                    callsign: String::new(),
                    node_id: [bob_pub[0]; 4],
                    last_seen_epoch: 100,
                    rssi: 0,
                    snr: 0,
                    latitude: 0,
                    longitude: 0,
                    flags: 0,
                    contact_type: 1,
                    out_path: [0x77; 64],
                    out_path_len,
                    last_advert_ts: 0,
                    saved_hash: 0,
                    sync_since: 0,
                });
            });

            for &len in &lengths {
                for &attempt in &attempts {
                    ts += 1;
                    let text: std::string::String = (0..len)
                        .map(|i| (b'a' + (i % 26) as u8) as char)
                        .collect();
                    let case = format!("path_len={out_path_len:#x} attempt={attempt} len={len}");

                    // 1. Companion command -> RESP_CODE_SENT.
                    drain_usb_commands();
                    let mut frame = Vec::<u8, 200>::new();
                    let _ = frame.push(CMD_SEND_MSG);
                    let _ = frame.push(TXT_TYPE_PLAIN);
                    let _ = frame.push(attempt);
                    let _ = frame.extend_from_slice(&ts.to_le_bytes());
                    let _ = frame.extend_from_slice(&bob_pub[..6]);
                    let _ = frame.extend_from_slice(text.as_bytes());
                    let mut resp = Vec::<u8, 16>::new();
                    poll_sync(node.handle_command(&frame, |r| {
                        let _ = resp.extend_from_slice(r);
                    }));
                    assert_eq!(resp.len(), 10, "{case}: {:02x?}", resp.as_slice());
                    assert_eq!(resp[0], RESP_CODE_SENT, "{case}");
                    let is_flood = resp[1] == 1;
                    assert_eq!(is_flood, out_path_len == 0xFF, "{case}");
                    let expected_ack = u32::from_le_bytes(resp[2..6].try_into().unwrap());
                    let est_timeout = u32::from_le_bytes(resp[6..10].try_into().unwrap());

                    // 2. The command handed to the app loop carries the same fields.
                    let Ok(crate::command::UsbCommand::SendDirectText {
                        dest_prefix,
                        timestamp,
                        txt_type,
                        attempt: queued_attempt,
                        text: queued_text,
                    }) = crate::command::USB_COMMAND_CHANNEL.try_receive()
                    else {
                        panic!("{case}: no SendDirectText queued");
                    };
                    assert_eq!(dest_prefix, bob_pub[..6]);
                    assert_eq!((timestamp, txt_type, queued_attempt), (ts, TXT_TYPE_PLAIN, attempt));
                    assert_eq!(queued_text.as_str(), text);

                    // 3. The engine, given that command, tracks the same CRC.
                    let hop_path = [0x77u8; 64];
                    let engine_crc = engine_a
                        .queue_direct_text(
                            &bob_pub,
                            queued_text.as_str(),
                            timestamp,
                            txt_type,
                            queued_attempt,
                            out_path_len,
                            &hop_path,
                        )
                        .unwrap_or_else(|_| panic!("{case}: queue_direct_text failed"));
                    assert_eq!(engine_crc, expected_ack, "{case}: expected_ack vs engine CRC");

                    // 4. est_timeout is derived from the packet length the
                    //    companion predicts; it must match the packet we built.
                    let tx = engine_a.tx_queue.pop().expect("DM queued");
                    let mut buf = [0u8; 256];
                    let wire_len = tx.packet.write_to(&mut buf).expect("serialise");
                    let (sf, bw_khz, cr) = crate::state::RADIO_SETTINGS.lock(|r| {
                        let s = r.borrow();
                        (s.sf, s.bw_khz, s.cr)
                    });
                    let air_ms = crate::airtime::estimate_airtime_ms(
                        wire_len,
                        sf,
                        bw_khz,
                        cr,
                        crate::scheduler::preamble_for(sf),
                    );
                    let expected_timeout = if is_flood {
                        500 + 16 * air_ms
                    } else {
                        500 + (air_ms * 6 + 250) * ((out_path_len & 63) as u32 + 1)
                    };
                    assert_eq!(
                        est_timeout, expected_timeout,
                        "{case}: est_timeout does not match the formula applied to the \
                         {wire_len}-byte packet actually sent (wrong length estimate or \
                         wrong formula)"
                    );

                    // 5. The receiver's ACK carries that CRC.
                    let mut rx = Packet::read_from(&buf[..wire_len]).expect("parse");
                    for _ in 0..(out_path_len & 63) {
                        // Stand in for the repeaters on the path.
                        if out_path_len != 0xFF {
                            assert!(rx.pop_front_hop(), "{case}");
                        }
                    }
                    match engine_b.handle_inbound_packet(rx, 0) {
                        InboundAction::ReceivedDirectText { msg, .. } => {
                            assert_eq!(msg.text.as_str(), text, "{case}");
                            assert_eq!(msg.ack_crc, expected_ack, "{case}: decoded CRC");
                        }
                        _ => panic!("{case}: receiver did not accept the DM"),
                    }
                    let reply = engine_b.tx_queue.pop().expect("receiver replies").packet;
                    if is_flood {
                        // A flooded DM is answered with PATH+ACK.
                        match engine_a.handle_inbound_packet(reply, 0) {
                            InboundAction::ReceivedPath { packet, .. } => {
                                let ack = AckPacket::decode(packet.extra.as_slice())
                                    .unwrap_or_else(|_| panic!("{case}: no ACK in PATH extra"));
                                assert_eq!(ack.crc, expected_ack, "{case}: PATH+ACK CRC");
                            }
                            _ => panic!("{case}: sender did not accept the PATH"),
                        }
                    } else {
                        assert_eq!(reply.route_type(), ROUTE_TYPE_DIRECT, "{case}");
                        assert_eq!(reply.payload_type(), PAYLOAD_TYPE_ACK, "{case}");
                        let ack = AckPacket::decode(&reply.payload[..reply.payload_len])
                            .expect("ACK payload");
                        assert_eq!(ack.crc, expected_ack, "{case}: ACK CRC");
                    }
                    engine_a.tx_queue.clear();
                    engine_b.tx_queue.clear();
                }
            }
        }
        drain_usb_commands();
    }

    #[test]
    fn cmd_get_stats_packets_and_set_device_time_monotonic() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let mut node = CompanionNode::new("n", "b", 0, [1u8; 32], [2u8; 64], &TestHost);

        crate::state::RADIO_TELEMETRY.lock(|t| {
            let mut tel = t.borrow_mut();
            tel.rx_packets = 11;
            tel.tx_beacons = 22;
            tel.tx_flood_sent = 7;
            tel.tx_direct = 15;
            tel.rx_flood = 4;
            tel.rx_direct = 7;
            tel.rx_errors = 3;
        });

        let mut resp = Vec::<u8, 64>::new();
        poll_sync(node.handle_command(&[CMD_GET_STATS, 2], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 30);
        assert_eq!(resp[0], RESP_CODE_STATS);
        assert_eq!(resp[1], 2);
        assert_eq!(u32::from_le_bytes(resp[2..6].try_into().unwrap()), 11);
        assert_eq!(u32::from_le_bytes(resp[6..10].try_into().unwrap()), 22);
        assert_eq!(u32::from_le_bytes(resp[10..14].try_into().unwrap()), 7);
        assert_eq!(u32::from_le_bytes(resp[14..18].try_into().unwrap()), 15);
        assert_eq!(u32::from_le_bytes(resp[18..22].try_into().unwrap()), 4);
        assert_eq!(u32::from_le_bytes(resp[22..26].try_into().unwrap()), 7);
        assert_eq!(u32::from_le_bytes(resp[26..30].try_into().unwrap()), 3);

        // CMD_GET_STATS(0): real err_flags and live TX queue_len.
        let saved_tel = crate::state::RADIO_TELEMETRY.lock(|t| *t.borrow());
        crate::state::clear_core_err_flags();
        crate::state::set_tx_queue_len(0);
        crate::state::RADIO_TELEMETRY.lock(|t| {
            let mut tel = t.borrow_mut();
            tel.tx_dropped = 0;
            tel.rx_dropped = 0;
        });
        for c in &crate::state::TX_QUEUE_FULL {
            c.store(0, Ordering::Relaxed);
        }
        crate::storage::EXTERNAL_UNAVAILABLE.store(false, Ordering::Relaxed);
        crate::storage::bonds::BOND_SAVE_ERRORS.store(0, Ordering::Relaxed);

        crate::state::note_core_err(
            crate::state::CORE_ERR_RADIO_RX | crate::state::CORE_ERR_TX_FAILED,
        );
        crate::state::RADIO_TELEMETRY.lock(|t| t.borrow_mut().tx_dropped = 1);
        crate::state::set_tx_queue_len(5);

        resp.clear();
        poll_sync(node.handle_command(&[CMD_GET_STATS, 0], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 11);
        assert_eq!(resp[0], RESP_CODE_STATS);
        assert_eq!(resp[1], 0);
        assert_eq!(u16::from_le_bytes(resp[2..4].try_into().unwrap()), 4_000);
        let expected_err = crate::state::CORE_ERR_RADIO_RX
            | crate::state::CORE_ERR_TX_FAILED
            | crate::state::CORE_ERR_QUEUE_DROP;
        assert_eq!(
            u16::from_le_bytes(resp[8..10].try_into().unwrap()),
            expected_err
        );
        assert_eq!(resp[10], 5);

        // Storage unavailable sets CORE_ERR_STORAGE.
        crate::storage::EXTERNAL_UNAVAILABLE.store(true, Ordering::Relaxed);
        resp.clear();
        poll_sync(node.handle_command(&[CMD_GET_STATS, 0], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_ne!(
            u16::from_le_bytes(resp[8..10].try_into().unwrap()) & crate::state::CORE_ERR_STORAGE,
            0
        );

        // Restore core error and queue state.
        crate::storage::EXTERNAL_UNAVAILABLE.store(false, Ordering::Relaxed);
        crate::state::clear_core_err_flags();
        crate::state::set_tx_queue_len(0);
        crate::state::RADIO_TELEMETRY.lock(|t| *t.borrow_mut() = saved_tel);

        // CMD_GET_BATT_AND_STORAGE: reports real storage used_kb / total_kb.
        let saved_total = crate::storage::total_kb();
        let saved_queued = crate::storage::messages::QUEUED.load(Ordering::Relaxed);
        crate::storage::set_total_kb(0);
        resp.clear();
        poll_sync(node.handle_command(&[CMD_GET_BATT_AND_STORAGE], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 11);
        assert_eq!(resp[0], RESP_CODE_BATTERY);
        assert_eq!(u16::from_le_bytes(resp[1..3].try_into().unwrap()), 4_000);
        assert_eq!(u32::from_le_bytes(resp[3..7].try_into().unwrap()), 0);
        assert_eq!(u32::from_le_bytes(resp[7..11].try_into().unwrap()), 0);

        crate::storage::set_total_kb(1360);
        crate::state::DISCOVERED_CONTACTS.lock(|c| {
            let mut list = c.borrow_mut();
            list.clear();
            for i in 0..10u8 {
                let mut key = [0u8; 32];
                key[0] = i + 1;
                let _ = list.push(crate::types::DiscoveredContact {
                    pub_key: key,
                    callsign: String::new(),
                    node_id: [i + 1; 4],
                    last_seen_epoch: 100,
                    rssi: 0,
                    snr: 0,
                    latitude: 0,
                    longitude: 0,
                    flags: 0,
                    contact_type: 1,
                    out_path: [0u8; 64],
                    out_path_len: 0xFF,
                    last_advert_ts: 100,
                    sync_since: 0,
                    saved_hash: 0x1234_5678,
                });
            }
        });
        crate::storage::messages::QUEUED.store(20, Ordering::Relaxed);
        resp.clear();
        poll_sync(node.handle_command(&[CMD_GET_BATT_AND_STORAGE], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        let used_kb = u32::from_le_bytes(resp[3..7].try_into().unwrap());
        let total_kb = u32::from_le_bytes(resp[7..11].try_into().unwrap());
        assert_eq!(total_kb, 1360);
        assert!(used_kb >= 8 && used_kb <= total_kb, "unexpected used_kb={used_kb}");

        // Restore storage globals.
        crate::storage::set_total_kb(saved_total);
        crate::storage::messages::QUEUED.store(saved_queued, Ordering::Relaxed);
        crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());

        // CMD_GET_STATS(1): default -118 when unseeded, then live noise_floor_dbm.
        crate::state::RADIO_TELEMETRY.lock(|t| t.borrow_mut().noise_floor_dbm = 0);
        resp.clear();
        poll_sync(node.handle_command(&[CMD_GET_STATS, 1], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 14);
        assert_eq!(resp[0], RESP_CODE_STATS);
        assert_eq!(resp[1], 1);
        assert_eq!(i16::from_le_bytes(resp[2..4].try_into().unwrap()), -118);

        crate::state::RADIO_TELEMETRY.lock(|t| t.borrow_mut().noise_floor_dbm = -111);
        resp.clear();
        poll_sync(node.handle_command(&[CMD_GET_STATS, 1], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 14);
        assert_eq!(i16::from_le_bytes(resp[2..4].try_into().unwrap()), -111);
        crate::state::RADIO_TELEMETRY.lock(|t| t.borrow_mut().noise_floor_dbm = 0);

        resp.clear();
        poll_sync(node.handle_command(&[CMD_GET_STATS, 99], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_ERR, ERR_ILLEGAL_ARG]);

        crate::state::set_device_time(1_700_000_100);
        let mut set_time = [CMD_SET_DEVICE_TIME, 0, 0, 0, 0];
        set_time[1..5].copy_from_slice(&1_700_000_050u32.to_le_bytes());
        resp.clear();
        poll_sync(node.handle_command(&set_time, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_ERR, ERR_ILLEGAL_ARG]);

        set_time[1..5].copy_from_slice(&1_700_000_200u32.to_le_bytes());
        resp.clear();
        poll_sync(node.handle_command(&set_time, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_OK]);
        assert!(crate::state::get_device_time() >= 1_700_000_200);
    }

    #[test]
    fn cmd_add_update_contact_reset_path_and_advert_latlon() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        drain_usb_commands();
        crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());
        let mut node = CompanionNode::new("n", "b", 0, [1u8; 32], [2u8; 64], &TestHost);

        let key = [0x6Au8; 32];
        let mut frame = Vec::<u8, 160>::new();
        let _ = frame.push(CMD_ADD_UPDATE_CONTACT);
        let _ = frame.extend_from_slice(&key);
        let _ = frame.push(ADV_TYPE_CHAT);
        let _ = frame.push(0x01); // favorite
        let _ = frame.push(0x02); // 2 hops, 1-byte hashes
        let _ = frame.extend_from_slice(&[0xAA, 0xBB]);
        let _ = frame.extend_from_slice(&[0u8; 62]);
        let mut name = [0u8; 32];
        name[..3].copy_from_slice(b"bob");
        let _ = frame.extend_from_slice(&name);
        let _ = frame.extend_from_slice(&1234u32.to_le_bytes());
        let _ = frame.extend_from_slice(&37_000_000i32.to_le_bytes());
        let _ = frame.extend_from_slice(&(-122_000_000i32).to_le_bytes());
        let _ = frame.extend_from_slice(&5678u32.to_le_bytes());

        let mut resp = Vec::<u8, 128>::new();
        poll_sync(node.handle_command(&frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_OK]);

        let saved = crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow()[0].clone());
        assert_eq!(saved.pub_key, key);
        assert_eq!(saved.callsign.as_str(), "bob");
        assert_eq!(saved.out_path_len, 0x02);
        assert_eq!(&saved.out_path[..2], &[0xAA, 0xBB]);
        assert_eq!(saved.last_seen_epoch, 5678);

        // CMD_RESET_PATH clears out_path synchronously.
        let mut reset_frame = Vec::<u8, 40>::new();
        let _ = reset_frame.push(CMD_RESET_PATH);
        let _ = reset_frame.extend_from_slice(&key);
        resp.clear();
        poll_sync(node.handle_command(&reset_frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_OK]);
        let after_reset = crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow()[0].out_path_len);
        assert_eq!(after_reset, 0xFF);

        // CMD_SET_ADVERT_LATLON updates shared advert lat/lon reflected in SELF_INFO.
        let mut latlon_frame = Vec::<u8, 16>::new();
        let _ = latlon_frame.push(CMD_SET_ADVERT_LATLON);
        let _ = latlon_frame.extend_from_slice(&51_507_400i32.to_le_bytes());
        let _ = latlon_frame.extend_from_slice(&(-127_800i32).to_le_bytes());
        resp.clear();
        poll_sync(node.handle_command(&latlon_frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_OK]);
        drain_usb_commands();

        let self_info = node.build_self_info();
        assert_eq!(
            i32::from_le_bytes(self_info[36..40].try_into().unwrap()),
            51_507_400
        );
        assert_eq!(
            i32::from_le_bytes(self_info[40..44].try_into().unwrap()),
            -127_800
        );
        crate::state::set_advert_lat_lon_e6(0, 0);
    }

    #[test]
    fn cmd_get_advert_path_uses_inbound_cache_and_returns_not_found() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::state::clear_advert_paths();
        crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());

        let key = [0x42u8; 32];
        // Put a contact in DISCOVERED_CONTACTS with a *different* outbound path
        // and sender timestamp so we prove GET_ADVERT_PATH ignores them.
        crate::state::DISCOVERED_CONTACTS.lock(|c| {
            let _ = c.borrow_mut().push(crate::types::DiscoveredContact {
                pub_key: key,
                callsign: String::new(),
                node_id: [0x42; 4],
                last_seen_epoch: 100,
                rssi: -80,
                snr: 5,
                latitude: 0,
                longitude: 0,
                flags: 0,
                contact_type: 1,
                out_path: [0xEE; 64],
                out_path_len: 1,
                last_advert_ts: 9999,
                saved_hash: 0,
                sync_since: 0,
            });
        });

        let mut node = CompanionNode::new("n", "b", 0, [1u8; 32], [2u8; 64], &TestHost);

        // Short payload -> ERR_ILLEGAL_ARG.
        let mut resp = Vec::<u8, 80>::new();
        poll_sync(node.handle_command(&[CMD_GET_ADVERT_PATH, 0, 1, 2], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_ERR, ERR_ILLEGAL_ARG]);

        // Not in advert path cache (even though contact has an out_path) -> ERR_NOT_FOUND.
        let mut query = Vec::<u8, 40>::new();
        let _ = query.push(CMD_GET_ADVERT_PATH);
        let _ = query.push(0);
        let _ = query.extend_from_slice(&key);
        resp.clear();
        poll_sync(node.handle_command(&query, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_ERR, ERR_NOT_FOUND]);

        // Record inbound advert path and verify response returns local recv_ts + inbound path.
        let mut inbound = [0u8; 64];
        inbound[0] = 0x11;
        inbound[1] = 0x22;
        crate::state::record_advert_path(&key, 1_700_000_777, 2, &inbound);

        resp.clear();
        poll_sync(node.handle_command(&query, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 1 + 4 + 1 + 2);
        assert_eq!(resp[0], RESP_CODE_ADVERT_PATH);
        assert_eq!(
            u32::from_le_bytes(resp[1..5].try_into().unwrap()),
            1_700_000_777
        );
        assert_eq!(resp[5], 2);
        assert_eq!(&resp[6..8], &[0x11, 0x22]);

        // 0-hop direct advert also succeeds with path_len = 0.
        let key_direct = [0x43u8; 32];
        crate::state::record_advert_path(&key_direct, 1_700_000_888, 0, &[0u8; 64]);
        query.clear();
        let _ = query.push(CMD_GET_ADVERT_PATH);
        let _ = query.push(0);
        let _ = query.extend_from_slice(&key_direct);
        resp.clear();
        poll_sync(node.handle_command(&query, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.len(), 1 + 4 + 1);
        assert_eq!(resp[0], RESP_CODE_ADVERT_PATH);
        assert_eq!(
            u32::from_le_bytes(resp[1..5].try_into().unwrap()),
            1_700_000_888
        );
        assert_eq!(resp[5], 0);

        crate::state::clear_advert_paths();
        crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());
    }

    #[test]
    fn cmd_import_contact_enqueues_loopback_and_rejects_invalid() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        drain_usb_commands();
        while crate::command::CONTACT_OP_CHANNEL.try_receive().is_ok() {}

        let mut node = CompanionNode::new("n", "b", 0, [1u8; 32], [2u8; 64], &TestHost);

        // Short / malformed payload -> ERR_ILLEGAL_ARG.
        let mut resp = Vec::<u8, 16>::new();
        poll_sync(node.handle_command(&[CMD_IMPORT_CONTACT, 0x11, 0x00], |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_ERR, ERR_ILLEGAL_ARG]);

        // Valid signed advert packet -> RESP_CODE_OK and enqueues UsbCommand::ImportContact,
        // without touching CONTACT_OP_CHANNEL directly.
        let id = meshcore_wire::identity::LocalIdentity::from_seed("Peer", &[0x5Au8; 32]);
        let mut adv = AdvertPacket::new(
            *id.pub_key(),
            1_700_000_100,
            meshcore_wire::packets::advert::ADV_TYPE_CHAT,
            0,
            0,
            "Peer",
        );
        adv.sign(&id);
        let mut pkt = adv
            .to_packet(meshcore_wire::packet::ROUTE_TYPE_FLOOD)
            .unwrap();
        pkt.append_hop(0xAA);
        let mut raw_pkt = [0u8; 184];
        let raw_len = pkt.write_to(&mut raw_pkt).unwrap();

        let mut frame = Vec::<u8, 200>::new();
        let _ = frame.push(CMD_IMPORT_CONTACT);
        let _ = frame.extend_from_slice(&raw_pkt[..raw_len]);

        resp.clear();
        poll_sync(node.handle_command(&frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_OK]);
        assert!(crate::command::CONTACT_OP_CHANNEL.try_receive().is_err());

        match crate::command::USB_COMMAND_CHANNEL.try_receive() {
            Ok(crate::command::UsbCommand::ImportContact { raw }) => {
                assert_eq!(raw.as_slice(), &raw_pkt[..raw_len]);
            }
            other => panic!("expected UsbCommand::ImportContact, got {:?}", other),
        }

        // Tampered signature -> ERR_ILLEGAL_ARG.
        let mut bad_frame = frame.clone();
        bad_frame[raw_len] ^= 0xFF;
        resp.clear();
        poll_sync(node.handle_command(&bad_frame, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp.as_slice(), &[RESP_CODE_ERR, ERR_ILLEGAL_ARG]);
        assert!(crate::command::USB_COMMAND_CHANNEL.try_receive().is_err());
    }

    struct CaptureSink;
    static CAPTURED_PUSHES: embassy_sync::blocking_mutex::Mutex<
        embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
        core::cell::RefCell<Vec<Vec<u8, 256>, 16>>,
    > = embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(Vec::new()));

    impl crate::command::PushSink for CaptureSink {
        fn is_connected(&self) -> bool {
            true
        }
        fn try_push(&self, frame: &[u8]) -> bool {
            CAPTURED_PUSHES.lock(|p| {
                let mut v = Vec::<u8, 256>::new();
                let _ = v.extend_from_slice(frame);
                let _ = p.borrow_mut().push(v);
            });
            true
        }
    }

    static TEST_PUSH_SINK: CaptureSink = CaptureSink;

    fn clear_pushes() {
        crate::command::register_push_sink(&TEST_PUSH_SINK);
        CAPTURED_PUSHES.lock(|p| p.borrow_mut().clear());
    }

    fn take_pushes() -> Vec<Vec<u8, 256>, 16> {
        CAPTURED_PUSHES.lock(|p| {
            let mut out = Vec::new();
            for item in p.borrow().iter() {
                let _ = out.push(item.clone());
            }
            p.borrow_mut().clear();
            out
        })
    }

    fn signed_advert_packet(seed: u8, name: &str, ts: u32, hops: &[u8]) -> (AdvertPacket, Packet) {
        let id = meshcore_wire::identity::LocalIdentity::from_seed(name, &[seed; 32]);
        let mut adv = AdvertPacket::new(
            *id.pub_key(),
            ts,
            meshcore_wire::packets::advert::ADV_TYPE_CHAT,
            0,
            0,
            name,
        );
        adv.sign(&id);
        let mut pkt = adv
            .to_packet(meshcore_wire::packet::ROUTE_TYPE_FLOOD)
            .unwrap();
        for &h in hops {
            pkt.append_hop(h);
        }
        (adv, pkt)
    }

    #[test]
    fn advert_push_rules_import_loopback_and_advert_path_cache() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        clear_pushes();
        crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());
        crate::state::clear_advert_paths();
        crate::state::set_device_time(1_700_000_500);

        let local_id = meshcore_wire::identity::LocalIdentity::from_seed("Me", &[0x01u8; 32]);
        let mut node = CompanionNode::new(
            "Me",
            "test",
            0,
            *local_id.pub_key(),
            *local_id.private_key(),
            &TestHost,
        );
        node.epoch_time = 1_700_000_500;
        let set_autoadd = |manual: u8, config: u8, max_hops: u8| {
            crate::state::AUTOADD_SETTINGS.lock(|a| {
                let mut s = a.borrow_mut();
                s.manual_add_contacts = manual;
                s.config = config;
                s.max_hops = max_hops;
            });
        };
        set_autoadd(0, 0, 0); // auto-add enabled

        // 1. F3 + F5: New contact auto-added -> emits PUSH_CODE_ADVERT (not NEW_ADVERT),
        //    stores contact with out_path_len = 0xFF, and caches inbound advert path + local recv_ts.
        let (adv1, pkt1) = signed_advert_packet(0x11, "Alice", 1_700_000_100, &[0xAA, 0xBB]);
        let stored_changed = process_inbound_advert(
            &node,
            adv1.clone(),
            pkt1.path_len,
            &pkt1.path,
            Some(&pkt1.payload[..pkt1.payload_len]),
            -70,
            6,
        );
        assert!(stored_changed);
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        assert_eq!(pushes[0][0], PUSH_CODE_ADVERT);
        assert_eq!(pushes[0].len(), 33);
        assert_eq!(&pushes[0][1..33], &adv1.pub_key);

        let stored = crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow()[0].clone());
        assert_eq!(stored.pub_key, adv1.pub_key);
        assert_eq!(stored.out_path_len, 0xFF);
        assert_eq!(stored.callsign.as_str(), "Alice");

        // CMD_GET_ADVERT_PATH returns inbound path + local recv_ts even while out_path_len == 0xFF.
        let mut query = Vec::<u8, 40>::new();
        let _ = query.push(CMD_GET_ADVERT_PATH);
        let _ = query.push(0);
        let _ = query.extend_from_slice(&adv1.pub_key);
        let mut resp = Vec::<u8, 80>::new();
        poll_sync(node.handle_command(&query, |r| {
            let _ = resp.extend_from_slice(r);
        }));
        assert_eq!(resp[0], RESP_CODE_ADVERT_PATH);
        let recv_ts = u32::from_le_bytes(resp[1..5].try_into().unwrap());
        assert!(recv_ts >= 1_700_000_500);
        assert_ne!(recv_ts, 1_700_000_100);
        assert_eq!(resp[5], 2);
        assert_eq!(&resp[6..8], &[0xAA, 0xBB]);

        // 2. Anti-replay: older or equal timestamp advert does NOT overwrite, push, or touch cache.
        let (adv1_old, pkt1_old) = signed_advert_packet(0x11, "AliceOld", 1_700_000_090, &[0xCC]);
        let changed = process_inbound_advert(
            &node,
            adv1_old,
            pkt1_old.path_len,
            &pkt1_old.path,
            Some(&pkt1_old.payload[..pkt1_old.payload_len]),
            -90,
            1,
        );
        assert!(!changed);
        assert!(take_pushes().is_empty());
        assert_eq!(
            crate::state::DISCOVERED_CONTACTS
                .lock(|c| c.borrow()[0].callsign.clone())
                .as_str(),
            "Alice"
        );
        let cached_after_replay =
            crate::state::lookup_advert_path(&adv1.pub_key[..7].try_into().unwrap()).unwrap();
        assert_eq!(&cached_after_replay.path[..2], &[0xAA, 0xBB]);

        // 3. F3 + F5: Manual-add mode declines storing -> emits PUSH_CODE_NEW_ADVERT,
        //    does NOT store, still caches inbound advert path.
        set_autoadd(1, 0, 0);
        let (adv2, pkt2) = signed_advert_packet(0x22, "Bob", 1_700_000_200, &[0xDD]);
        let changed = process_inbound_advert(
            &node,
            adv2.clone(),
            pkt2.path_len,
            &pkt2.path,
            Some(&pkt2.payload[..pkt2.payload_len]),
            -75,
            5,
        );
        assert!(!changed);
        assert_eq!(
            crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow().len()),
            1
        );
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        assert_eq!(pushes[0][0], PUSH_CODE_NEW_ADVERT);
        assert_eq!(&pushes[0][1..33], &adv2.pub_key);
        let bob_cached =
            crate::state::lookup_advert_path(&adv2.pub_key[..7].try_into().unwrap()).unwrap();
        assert_eq!(bob_cached.path_len, 1);
        assert_eq!(bob_cached.path[0], 0xDD);

        // 4. F4: Imported contact goes through normal receive path:
        //    - in manual-add mode, does NOT add contact (emits PUSH_CODE_NEW_ADVERT).
        let (adv3, pkt3) = signed_advert_packet(0x33, "Carol", 1_700_000_300, &[0x11, 0x22]);
        let mut raw3 = [0u8; 184];
        let raw3_len = pkt3.write_to(&mut raw3).unwrap();

        let changed = process_imported_contact(&node, &raw3[..raw3_len]);
        assert!(!changed);
        assert_eq!(
            crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow().len()),
            1
        );
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        assert_eq!(pushes[0][0], PUSH_CODE_NEW_ADVERT);

        //    - in auto-add mode, adds Carol with out_path_len = 0xFF (NOT pkt.path_len = 2!),
        //      emits PUSH_CODE_ADVERT, and rejects an older re-import.
        set_autoadd(0, 0, 0);
        let changed = process_imported_contact(&node, &raw3[..raw3_len]);
        assert!(changed);
        let pushes = take_pushes();
        assert_eq!(pushes.len(), 1);
        assert_eq!(pushes[0][0], PUSH_CODE_ADVERT);

        let carol = crate::state::DISCOVERED_CONTACTS.lock(|c| {
            c.borrow()
                .iter()
                .find(|ct| ct.pub_key == adv3.pub_key)
                .cloned()
                .unwrap()
        });
        assert_eq!(carol.callsign.as_str(), "Carol");
        assert_eq!(carol.out_path_len, 0xFF);

        // Older imported advert for Carol is rejected by anti-replay.
        let (_adv3_old, pkt3_old) = signed_advert_packet(0x33, "CarolStale", 1_700_000_250, &[]);
        let mut raw3_old = [0u8; 184];
        let raw3_old_len = pkt3_old.write_to(&mut raw3_old).unwrap();
        assert!(!process_imported_contact(&node, &raw3_old[..raw3_old_len]));
        assert!(take_pushes().is_empty());
        let carol_after = crate::state::DISCOVERED_CONTACTS.lock(|c| {
            c.borrow()
                .iter()
                .find(|ct| ct.pub_key == adv3.pub_key)
                .cloned()
                .unwrap()
        });
        assert_eq!(carol_after.callsign.as_str(), "Carol");

        set_autoadd(0, 0, 0);
        crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());
        crate::state::clear_advert_paths();
    }

    #[test]
    fn companion_tx_caches_ecdh_and_invalidates_on_key_import() {
        let _guard = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let saved_pub = crate::state::current_public_key();
        let saved_prv = crate::state::current_private_key();
        while crate::command::USB_COMMAND_CHANNEL.try_receive().is_ok() {}
        crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());
        crate::ecdh::invalidate();

        let local_a = meshcore_wire::identity::LocalIdentity::from_seed("NodeA", &[0x11; 32]);
        let local_b = meshcore_wire::identity::LocalIdentity::from_seed("NodeB", &[0x22; 32]);
        let peer = meshcore_wire::identity::LocalIdentity::from_seed("Peer", &[0x33; 32]);

        crate::state::set_identity(*local_a.pub_key(), *local_a.private_key());
        crate::state::DISCOVERED_CONTACTS.lock(|c| {
            let mut cs = heapless::String::new();
            let _ = cs.push_str("Peer");
            let _ = c.borrow_mut().push(crate::types::DiscoveredContact {
                pub_key: *peer.pub_key(),
                callsign: cs,
                node_id: peer.pub_key()[..4].try_into().unwrap(),
                last_seen_epoch: 100,
                rssi: -80,
                snr: 5,
                latitude: 0,
                longitude: 0,
                flags: 0,
                contact_type: meshcore_wire::packets::advert::ADV_TYPE_REPEATER,
                out_path: [0u8; 64],
                out_path_len: 0xFF,
                last_advert_ts: 100,
                saved_hash: 0,
                sync_since: 0,
            });
        });

        let host = TestHost;
        let mut node = CompanionNode::new(
            "NodeA",
            "TestBoard",
            0,
            *local_a.pub_key(),
            *local_a.private_key(),
            &host,
        );

        let mut status_req = [0u8; 33];
        status_req[0] = CMD_SEND_STATUS_REQ;
        status_req[1..33].copy_from_slice(peer.pub_key());

        let (hits0, misses0) = crate::ecdh::stats();

        // First status request: cache miss, encrypts with local_a <-> peer secret.
        let mut resp1 = Vec::<u8, 32>::new();
        poll_sync(node.handle_command(&status_req, |r| {
            let _ = resp1.extend_from_slice(r);
        }));
        assert_eq!(resp1[0], RESP_CODE_SENT);
        let (hits1, misses1) = crate::ecdh::stats();
        assert_eq!(hits1, hits0);
        assert_eq!(misses1, misses0 + 1);

        let raw1 = match crate::command::USB_COMMAND_CHANNEL.try_receive().unwrap() {
            crate::command::UsbCommand::SendRawPacket { raw } => raw,
            _ => panic!("expected SendRawPacket"),
        };
        let pkt1 = Packet::read_from(&raw1).unwrap();
        let secret_a = peer.shared_secret(local_a.pub_key()).unwrap();
        assert!(AddressedPacket::decode(&pkt1.payload[..pkt1.payload_len], &secret_a).is_ok());

        // Second status request: cache hit!
        let mut resp2 = Vec::<u8, 32>::new();
        poll_sync(node.handle_command(&status_req, |r| {
            let _ = resp2.extend_from_slice(r);
        }));
        assert_eq!(resp2[0], RESP_CODE_SENT);
        let (hits2, misses2) = crate::ecdh::stats();
        assert_eq!(hits2, hits1 + 1);
        assert_eq!(misses2, misses1);
        let _ = crate::command::USB_COMMAND_CHANNEL.try_receive();

        // Import new private key -> invalidates ECDH cache.
        let mut import_cmd = [0u8; 65];
        import_cmd[0] = CMD_IMPORT_PRIVATE_KEY;
        import_cmd[1..65].copy_from_slice(local_b.private_key());
        let mut import_resp = Vec::<u8, 8>::new();
        poll_sync(node.handle_command(&import_cmd, |r| {
            let _ = import_resp.extend_from_slice(r);
        }));
        assert_eq!(import_resp.as_slice(), &[RESP_CODE_OK]);
        let _ = crate::command::USB_COMMAND_CHANNEL.try_receive();

        // Next status request must miss cache and encrypt under local_b <-> peer secret.
        let mut resp3 = Vec::<u8, 32>::new();
        poll_sync(node.handle_command(&status_req, |r| {
            let _ = resp3.extend_from_slice(r);
        }));
        assert_eq!(resp3[0], RESP_CODE_SENT);
        let (hits3, misses3) = crate::ecdh::stats();
        assert_eq!(hits3, hits2);
        assert_eq!(misses3, misses2 + 1);

        let raw3 = match crate::command::USB_COMMAND_CHANNEL.try_receive().unwrap() {
            crate::command::UsbCommand::SendRawPacket { raw } => raw,
            _ => panic!("expected SendRawPacket"),
        };
        let pkt3 = Packet::read_from(&raw3).unwrap();
        let secret_b = peer.shared_secret(local_b.pub_key()).unwrap();
        assert!(AddressedPacket::decode(&pkt3.payload[..pkt3.payload_len], &secret_b).is_ok());
        assert!(AddressedPacket::decode(&pkt3.payload[..pkt3.payload_len], &secret_a).is_err());

        // Self-export advert via from_keypair produces a valid signature under local_b.
        let mut exp_resp = Vec::<u8, 200>::new();
        poll_sync(node.handle_command(&[CMD_EXPORT_CONTACT], |r| {
            let _ = exp_resp.extend_from_slice(r);
        }));
        assert_eq!(exp_resp[0], RESP_CODE_EXPORT_CONTACT);
        let exp_pkt = Packet::read_from(&exp_resp[1..]).unwrap();
        let exp_adv = AdvertPacket::decode(&exp_pkt.payload[..exp_pkt.payload_len]).unwrap();
        assert_eq!(&exp_adv.pub_key, local_b.pub_key());
        assert!(exp_adv.verify_signature());

        // Restore global state.
        crate::state::DISCOVERED_CONTACTS.lock(|c| c.borrow_mut().clear());
        crate::state::set_identity(saved_pub, saved_prv);
        crate::ecdh::invalidate();
        while crate::command::USB_COMMAND_CHANNEL.try_receive().is_ok() {}
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AdvertOutcome {
    Replayed,
    Stored,
    NotStored,
}

fn push_new_advert(ct: &crate::types::DiscoveredContact) {
    let record = serialize_contact(ct);
    let mut frame = Vec::<u8, 256>::new();
    let _ = frame.push(PUSH_CODE_NEW_ADVERT);
    let _ = frame.extend_from_slice(&record[1..]);
    crate::command::push_companion(&frame);
}

fn push_advert(pub_key: &[u8; 32]) {
    let mut frame = Vec::<u8, 256>::new();
    let _ = frame.push(PUSH_CODE_ADVERT);
    let _ = frame.extend_from_slice(pub_key);
    crate::command::push_companion(&frame);
}

/// Process a verified inbound (or looped-back imported) advertisement.
///
/// Mirrors upstream `BaseChatMesh::onAdvertRecv` + `MyMesh::onDiscoveredContact`:
/// - rejects replays (`timestamp <= existing.last_advert_ts`),
/// - checks auto-add type/hop rules and optional oldest-non-favorite eviction,
/// - initializes new contacts with `out_path_len = 0xFF` (unknown),
/// - emits `PUSH_CODE_ADVERT` when stored and `PUSH_CODE_NEW_ADVERT` only when
///   not stored,
/// - records the inbound arrival path and local receive timestamp in
///   [`crate::state::record_advert_path`],
/// - queues blob/contact persistence only when stored.
///
/// Returns `true` when `DISCOVERED_CONTACTS` was stored or updated.
#[inline(never)]
#[allow(clippy::too_many_arguments)]
pub fn process_inbound_advert(
    node: &CompanionNode<'_>,
    adv: AdvertPacket,
    path_len: u8,
    path: &[u8; 64],
    advert_payload: Option<&[u8]>,
    rssi: i16,
    snr: i8,
) -> bool {
    defmt::info!("MeshCore Advert from: {}", adv.name.as_str());
    let node_id = [
        adv.pub_key[0],
        adv.pub_key[1],
        adv.pub_key[2],
        adv.pub_key[3],
    ];

    let advert_hops = (path_len & 63) as u8;
    let recv_ts = {
        let t = crate::state::get_device_time();
        if t != 0 {
            t
        } else {
            node.epoch_time as u32
        }
    };

    let contact_type = CompanionNode::get_contact_type_from_flags(adv.flags);
    let should_add =
        node.should_auto_add_contact_type(contact_type) && node.autoadd_allows_hops(advert_hops);

    let mut store_blob = false;
    let mut overwritten_key: Option<[u8; 32]> = None;
    let mut outcome = AdvertOutcome::NotStored;
    crate::state::DISCOVERED_CONTACTS.lock(|c| {
        let mut list = c.borrow_mut();
        if let Some(existing) = list.iter_mut().find(|ct| ct.pub_key == adv.pub_key) {
            // Anti-replay: ignore an advert no newer than the last accepted one.
            if existing.last_advert_ts != 0 && adv.timestamp <= existing.last_advert_ts {
                outcome = AdvertOutcome::Replayed;
                return;
            }
            if existing.callsign != adv.name
                || existing.contact_type != contact_type
                || existing.latitude != adv.latitude
                || existing.longitude != adv.longitude
            {
                store_blob = true;
            }
            existing.callsign = adv.name.clone();
            existing.last_seen_epoch = recv_ts;
            existing.last_advert_ts = adv.timestamp;
            existing.rssi = rssi;
            existing.snr = snr;
            existing.latitude = adv.latitude;
            existing.longitude = adv.longitude;
            existing.contact_type = contact_type;
            // Contact flags (favorite/telemetry permissions) are
            // independent of the advert flags; keep the stored ones.
            push_advert(&adv.pub_key);
            outcome = AdvertOutcome::Stored;
            return;
        }

        // New contact. The app is told even when auto-add declines it.
        let mut cs = String::<32>::new();
        let _ = cs.push_str(adv.name.as_str());
        let discovered = crate::types::DiscoveredContact {
            pub_key: adv.pub_key,
            callsign: cs,
            node_id,
            last_seen_epoch: recv_ts,
            rssi,
            snr,
            latitude: adv.latitude,
            longitude: adv.longitude,
            flags: 0,
            contact_type,
            out_path: [0u8; 64],
            out_path_len: 0xFF,
            last_advert_ts: adv.timestamp,
            saved_hash: 0,
            sync_since: 0,
        };

        let mut added = false;
        if should_add {
            if list.len() < list.capacity() {
                let _ = list.push(discovered.clone());
                added = true;
            } else if node.should_overwrite_when_full() {
                let mut oldest_idx = None;
                let mut oldest_lastmod = u32::MAX;
                for (idx, ct) in list.iter().enumerate() {
                    let is_favorite = (ct.flags & CONTACT_FLAG_FAVORITE) != 0;
                    if !is_favorite && ct.last_seen_epoch < oldest_lastmod {
                        oldest_lastmod = ct.last_seen_epoch;
                        oldest_idx = Some(idx);
                    }
                }
                if let Some(idx) = oldest_idx {
                    let overwritten = list[idx].pub_key;
                    let mut push = Vec::<u8, 256>::new();
                    let _ = push.push(PUSH_CODE_CONTACT_DELETED);
                    let _ = push.extend_from_slice(&overwritten);
                    crate::command::push_companion(&push);
                    crate::storage::adverts::forget(&overwritten);
                    overwritten_key = Some(overwritten);
                    list[idx] = discovered.clone();
                    added = true;
                }
            }
        }

        if added {
            store_blob = true;
            push_advert(&adv.pub_key);
            outcome = AdvertOutcome::Stored;
        } else {
            push_new_advert(&discovered);
            if should_add && list.len() >= list.capacity() {
                let mut push = Vec::<u8, 256>::new();
                let _ = push.push(PUSH_CODE_CONTACTS_FULL);
                crate::command::push_companion(&push);
            }
            outcome = AdvertOutcome::NotStored;
        }
    });

    if outcome == AdvertOutcome::Replayed {
        return false;
    }

    crate::state::record_advert_path(&adv.pub_key, recv_ts, path_len, path);

    if store_blob {
        if let Some(payload) = advert_payload {
            crate::storage::adverts::remember(&adv.pub_key, payload);
        }
    }
    if let Some(key) = overwritten_key {
        let _ = crate::storage::contacts::try_queue_remove(key);
    }
    if outcome == AdvertOutcome::Stored {
        crate::storage::request_contacts();
        return true;
    }
    false
}

/// Process a raw imported contact advert packet through [`process_inbound_advert`].
#[inline(never)]
pub fn process_imported_contact(node: &CompanionNode<'_>, raw: &[u8]) -> bool {
    if let Ok(pkt) = Packet::read_from(raw) {
        if pkt.payload_type() == PAYLOAD_TYPE_ADVERT {
            if let Ok(adv) = AdvertPacket::decode(&pkt.payload[..pkt.payload_len]) {
                if adv.verify_signature() {
                    return process_inbound_advert(
                        node,
                        adv,
                        pkt.path_len,
                        &pkt.path,
                        Some(&pkt.payload[..pkt.payload_len]),
                        0,
                        0,
                    );
                }
            }
        }
    }
    false
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
        let mut out = String::<256>::new();
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
