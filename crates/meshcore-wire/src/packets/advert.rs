//! Node Advertisement (`PAYLOAD_TYPE_ADVERT = 0x04`).
//!
//! Payload: `pub(32) ‖ ts u32 ‖ sig(64) ‖ app_data(≤32)`, signature over
//! `pub ‖ ts ‖ app_data`. `app_data[0]` carries a single kind value in the low
//! nibble plus the optional-field masks.

use heapless::String;

use crate::identity::{PUB_KEY_SIZE, SIGNATURE_SIZE};
use crate::packet::{Packet, PAYLOAD_TYPE_ADVERT};

pub const APP_DATA_MAX: usize = 32;

pub const ADV_TYPE_NONE: u8 = 0;
pub const ADV_TYPE_CHAT: u8 = 1;
pub const ADV_TYPE_REPEATER: u8 = 2;
pub const ADV_TYPE_ROOM: u8 = 3;
pub const ADV_TYPE_SENSOR: u8 = 4;

pub const ADV_FLAG_HAS_LOCATION: u8 = 0x10;
pub const ADV_FLAG_HAS_FEATURE1: u8 = 0x20;
pub const ADV_FLAG_HAS_FEATURE2: u8 = 0x40;
pub const ADV_FLAG_HAS_NAME: u8 = 0x80;

// Backward-compatible names for the old bitmask constants.
pub const ADV_FLAG_IS_CHAT: u8 = ADV_TYPE_CHAT;
pub const ADV_FLAG_IS_REPEATER: u8 = ADV_TYPE_REPEATER;
pub const ADV_FLAG_IS_ROOM: u8 = ADV_TYPE_ROOM;
pub const ADV_FLAG_IS_SENSOR: u8 = ADV_TYPE_SENSOR;

#[derive(Clone, Debug, PartialEq)]
pub struct AdvertPacket {
    pub pub_key: [u8; PUB_KEY_SIZE],
    pub timestamp: u32,
    pub signature: [u8; SIGNATURE_SIZE],
    /// Low nibble of `app_data[0]`: 0 none, 1 chat, 2 repeater, 3 room, 4 sensor.
    pub kind: u8,
    /// Raw `app_data[0]`, including the optional-field masks.
    pub flags: u8,
    pub latitude: i32,
    pub longitude: i32,
    pub feat1: Option<u16>,
    pub feat2: Option<u16>,
    pub name: String<32>,
}

impl AdvertPacket {
    /// Create an unsigned advert. Call `sign()` before sending.
    pub fn new(
        pub_key: [u8; PUB_KEY_SIZE],
        timestamp: u32,
        kind: u8,
        latitude: i32,
        longitude: i32,
        name: &str,
    ) -> Self {
        let mut n = String::new();
        let _ = n.push_str(name);
        Self {
            pub_key,
            timestamp,
            signature: [0u8; SIGNATURE_SIZE],
            kind: kind & 0x0F,
            flags: 0,
            latitude,
            longitude,
            feat1: None,
            feat2: None,
            name: n,
        }
    }

    /// Encode the full payload: `pub ‖ ts ‖ sig ‖ app_data`.
    pub fn encode(&self, dest: &mut [u8]) -> Result<usize, ()> {
        let mut app = [0u8; APP_DATA_MAX];
        let app_len = self.encode_app_data(&mut app);
        let needed = 32 + 4 + 64 + app_len;
        if dest.len() < needed {
            return Err(());
        }

        dest[..32].copy_from_slice(&self.pub_key);
        dest[32..36].copy_from_slice(&self.timestamp.to_le_bytes());
        dest[36..100].copy_from_slice(&self.signature);
        dest[100..100 + app_len].copy_from_slice(&app[..app_len]);
        Ok(needed)
    }

    /// Decode an advert payload. `app_data` is capped at 32 bytes, as the C++
    /// does before signature verification.
    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.len() < 32 + 4 + 64 + 1 {
            return Err(());
        }

        let mut pub_key = [0u8; PUB_KEY_SIZE];
        pub_key.copy_from_slice(&src[..32]);
        let timestamp = u32::from_le_bytes([src[32], src[33], src[34], src[35]]);
        let mut signature = [0u8; SIGNATURE_SIZE];
        signature.copy_from_slice(&src[36..100]);

        let app_end = src.len().min(100 + APP_DATA_MAX);
        let app = &src[100..app_end];
        let flags = app[0];
        let mut i = 1;

        let mut latitude = 0i32;
        let mut longitude = 0i32;
        if flags & ADV_FLAG_HAS_LOCATION != 0 {
            if app.len() < i + 8 {
                return Err(());
            }
            latitude = i32::from_le_bytes([app[i], app[i + 1], app[i + 2], app[i + 3]]);
            longitude = i32::from_le_bytes([app[i + 4], app[i + 5], app[i + 6], app[i + 7]]);
            i += 8;
        }

        let feat1 = if flags & ADV_FLAG_HAS_FEATURE1 != 0 {
            if app.len() < i + 2 {
                return Err(());
            }
            let value = u16::from_le_bytes([app[i], app[i + 1]]);
            i += 2;
            Some(value)
        } else {
            None
        };
        let feat2 = if flags & ADV_FLAG_HAS_FEATURE2 != 0 {
            if app.len() < i + 2 {
                return Err(());
            }
            let value = u16::from_le_bytes([app[i], app[i + 1]]);
            i += 2;
            Some(value)
        } else {
            None
        };

        let mut name = String::new();
        if flags & ADV_FLAG_HAS_NAME != 0 && app.len() > i {
            let raw = &app[i..];
            let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
            if let Ok(text) = core::str::from_utf8(&raw[..end]) {
                let _ = name.push_str(text);
            }
        }

        Ok(Self {
            pub_key,
            timestamp,
            signature,
            kind: flags & 0x0F,
            flags,
            latitude,
            longitude,
            feat1,
            feat2,
            name,
        })
    }

    /// Build the signed message: `pub ‖ ts ‖ app_data`.
    pub fn build_message_for_signing(&self, buf: &mut [u8]) -> usize {
        let mut app = [0u8; APP_DATA_MAX];
        let app_len = self.encode_app_data(&mut app);
        let needed = 32 + 4 + app_len;
        if buf.len() < needed {
            return 0;
        }
        buf[..32].copy_from_slice(&self.pub_key);
        buf[32..36].copy_from_slice(&self.timestamp.to_le_bytes());
        buf[36..36 + app_len].copy_from_slice(&app[..app_len]);
        needed
    }

    pub fn sign(&mut self, local_identity: &crate::identity::LocalIdentity) {
        let mut msg_buf = [0u8; 100 + APP_DATA_MAX];
        let msg_len = self.build_message_for_signing(&mut msg_buf);
        self.signature = local_identity.sign(&msg_buf[..msg_len]);
    }

    pub fn verify_signature(&self) -> bool {
        let mut msg_buf = [0u8; 100 + APP_DATA_MAX];
        let msg_len = self.build_message_for_signing(&mut msg_buf);
        let identity = crate::identity::Identity::new(self.pub_key, "");
        identity.verify(&msg_buf[..msg_len], &self.signature)
    }

    pub fn latitude_f32(&self) -> f32 {
        self.latitude as f32 / 1_000_000.0
    }

    pub fn longitude_f32(&self) -> f32 {
        self.longitude as f32 / 1_000_000.0
    }

    pub fn is_chat(&self) -> bool {
        self.kind == ADV_TYPE_CHAT
    }

    pub fn is_repeater(&self) -> bool {
        self.kind == ADV_TYPE_REPEATER
    }

    /// Pack into a complete MeshCore Packet.
    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_ADVERT);
        let mut buf = [0u8; 184];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }

    fn encode_app_data(&self, out: &mut [u8; APP_DATA_MAX]) -> usize {
        let mut flags = self.kind & 0x0F;
        let mut i = 1usize;

        if self.flags & ADV_FLAG_HAS_LOCATION != 0
            || self.latitude != 0
            || self.longitude != 0
        {
            flags |= ADV_FLAG_HAS_LOCATION;
            out[i..i + 4].copy_from_slice(&self.latitude.to_le_bytes());
            i += 4;
            out[i..i + 4].copy_from_slice(&self.longitude.to_le_bytes());
            i += 4;
        }
        if let Some(value) = self.feat1 {
            if value != 0 || self.flags & ADV_FLAG_HAS_FEATURE1 != 0 {
                flags |= ADV_FLAG_HAS_FEATURE1;
                out[i..i + 2].copy_from_slice(&value.to_le_bytes());
                i += 2;
            }
        }
        if let Some(value) = self.feat2 {
            if value != 0 || self.flags & ADV_FLAG_HAS_FEATURE2 != 0 {
                flags |= ADV_FLAG_HAS_FEATURE2;
                out[i..i + 2].copy_from_slice(&value.to_le_bytes());
                i += 2;
            }
        }
        let name = self.name.as_bytes();
        let room = APP_DATA_MAX.saturating_sub(i);
        let n = utf8_prefix_len(name, room);
        if n > 0 {
            flags |= ADV_FLAG_HAS_NAME;
            out[i..i + n].copy_from_slice(&name[..n]);
            i += n;
        }

        out[0] = flags;
        i
    }
}

/// Longest valid UTF-8 prefix of `data`, at most `limit` bytes.
fn utf8_prefix_len(data: &[u8], limit: usize) -> usize {
    let mut n = data.len().min(limit);
    while n > 0 && core::str::from_utf8(&data[..n]).is_err() {
        n -= 1;
    }
    n
}
