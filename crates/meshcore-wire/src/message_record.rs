//! Variable-length persisted records for the message queue.
//!
//! The flash queue stores one compact record per inbox entry. Only the fields
//! that are actually present are written, so a short channel message costs a
//! couple of dozen bytes instead of a fixed 236-byte slot. Channel datagrams
//! use a separate record kind so undelivered `GRP_DATA` survives a reboot.
//!
//! Text record (version 2, little endian):
//!
//! ```text
//! 0      format version (2)
//! 1      flags: bit 0 = direct message
//! 2      channel index
//! 3      path length
//! 4..6   RSSI (i16)
//! 6      SNR (i8)
//! 7..11  timestamp (u32)
//! 11     sender length
//! 12     text length
//! 13     text type (0 plain, 1 CLI data, 2 signed, 3 CLI command)
//! 14..   sender public key (32 bytes, direct messages only)
//!        sender display name
//!        text
//!        signing-key prefix (4 bytes, signed messages only)
//! ```
//!
//! Channel-datagram record (version 3):
//!
//! ```text
//! 0      format version (3)
//! 1      flags (0)
//! 2      channel index
//! 3      path length
//! 4..6   RSSI (i16)
//! 6      SNR (i8)
//! 7..11  timestamp (0)
//! 11     sender length (0)
//! 12     text length (0)
//! 13     reserved (0)
//! 14..16 data type (u16)
//! 16..   data
//! ```
//!
//! Version 1 text records (written before signed-message support) are still
//! decoded.

use heapless::Vec;

use crate::packets::grpdata::MAX_GROUP_DATA;

/// Largest possible encoded text record (key + name + text + signing prefix).
pub const MESSAGE_RECORD_MAX: usize = 240;
/// Smallest encoded text record (empty name and text, channel message).
pub const MESSAGE_RECORD_MIN: usize = HEADER_SIZE;
/// Largest encoded channel-data record.
pub const CHANNEL_DATA_RECORD_MAX: usize = HEADER_SIZE + 2 + MAX_GROUP_DATA;

const TEXT_VERSION_V1: u8 = 1;
const TEXT_VERSION: u8 = 2;
const CHANNEL_DATA_VERSION: u8 = 3;
const HEADER_SIZE: usize = 14;
const FLAG_DIRECT: u8 = 0x01;

const MAX_SENDER: usize = 31;
const MAX_TEXT: usize = 159;

/// MeshCore text type for signed messages.
pub const TXT_TYPE_SIGNED_PLAIN: u8 = 2;

/// A text message as stored on flash.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MessageRecord {
    pub channel_idx: u8,
    pub path_len: u8,
    pub is_direct: bool,
    /// Display name (max 31 bytes; not NUL-terminated on flash).
    pub sender: [u8; 32],
    /// Full sender public key; all zeroes for channel messages.
    pub sender_pub: [u8; 32],
    /// Text (max 159 bytes; not NUL-terminated on flash).
    pub text: [u8; 160],
    pub timestamp: u32,
    pub rssi: i16,
    pub snr: i8,
    /// MeshCore text type from the wire (0 = plain).
    pub txt_type: u8,
    /// Signing-key prefix for `TXT_TYPE_SIGNED_PLAIN`; zero otherwise.
    pub signed_prefix: [u8; 4],
}

impl Default for MessageRecord {
    fn default() -> Self {
        Self {
            channel_idx: 0,
            path_len: 0,
            is_direct: false,
            sender: [0; 32],
            sender_pub: [0; 32],
            text: [0; 160],
            timestamp: 0,
            rssi: 0,
            snr: 0,
            txt_type: 0,
            signed_prefix: [0; 4],
        }
    }
}

fn truncate_str(dst: &mut [u8], src: &str, max: usize) -> usize {
    let mut n = src.len().min(max).min(dst.len());
    // Never cut through a UTF-8 code point; the decoder would otherwise see
    // invalid UTF-8 and replace the whole field with an empty string.
    while n > 0 && !src.is_char_boundary(n) {
        n -= 1;
    }
    dst[..n].copy_from_slice(&src.as_bytes()[..n]);
    n
}

fn get_str(src: &[u8]) -> &str {
    let end = src.iter().position(|&b| b == 0).unwrap_or(src.len());
    core::str::from_utf8(&src[..end]).unwrap_or("")
}

impl MessageRecord {
    /// Build a record from text fields, truncating to the field limits.
    pub fn new(
        channel_idx: u8,
        path_len: u8,
        is_direct: bool,
        sender: &str,
        sender_pub: [u8; 32],
        text: &str,
        timestamp: u32,
        rssi: i16,
        snr: i8,
    ) -> Self {
        let mut rec = Self::default();
        rec.channel_idx = channel_idx;
        rec.path_len = path_len;
        rec.is_direct = is_direct;
        rec.sender_pub = sender_pub;
        rec.timestamp = timestamp;
        rec.rssi = rssi;
        rec.snr = snr;
        truncate_str(&mut rec.sender, sender, MAX_SENDER);
        truncate_str(&mut rec.text, text, MAX_TEXT);
        rec
    }

    pub fn sender_str(&self) -> &str {
        get_str(&self.sender)
    }

    pub fn text_str(&self) -> &str {
        get_str(&self.text)
    }

    fn sender_len(&self) -> usize {
        self.sender
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(MAX_SENDER)
            .min(MAX_SENDER)
    }

    fn text_len(&self) -> usize {
        self.text
            .iter()
            .position(|&b| b == 0)
            .unwrap_or(MAX_TEXT)
            .min(MAX_TEXT)
    }

    fn is_signed(&self) -> bool {
        self.txt_type == TXT_TYPE_SIGNED_PLAIN
    }

    /// Number of bytes the encoded form occupies.
    pub fn encoded_len(&self) -> usize {
        HEADER_SIZE
            + if self.is_direct { 32 } else { 0 }
            + self.sender_len()
            + self.text_len()
            + if self.is_signed() { 4 } else { 0 }
    }

    /// Serialize into `out`, returning the encoded length.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let len = self.encoded_len();
        if out.len() < len {
            return None;
        }
        let sender_len = self.sender_len();
        let text_len = self.text_len();
        out[0] = TEXT_VERSION;
        out[1] = if self.is_direct { FLAG_DIRECT } else { 0 };
        out[2] = self.channel_idx;
        out[3] = self.path_len;
        out[4..6].copy_from_slice(&self.rssi.to_le_bytes());
        out[6] = self.snr as u8;
        out[7..11].copy_from_slice(&self.timestamp.to_le_bytes());
        out[11] = sender_len as u8;
        out[12] = text_len as u8;
        out[13] = self.txt_type & 0x3F;
        let mut pos = HEADER_SIZE;
        if self.is_direct {
            out[pos..pos + 32].copy_from_slice(&self.sender_pub);
            pos += 32;
        }
        out[pos..pos + sender_len].copy_from_slice(&self.sender[..sender_len]);
        pos += sender_len;
        out[pos..pos + text_len].copy_from_slice(&self.text[..text_len]);
        pos += text_len;
        if self.is_signed() {
            out[pos..pos + 4].copy_from_slice(&self.signed_prefix);
            pos += 4;
        }
        Some(pos)
    }

    /// Parse a text record; returns `None` for a short or truncated record.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < HEADER_SIZE {
            return None;
        }
        let version = bytes[0];
        if version != TEXT_VERSION_V1 && version != TEXT_VERSION {
            return None;
        }
        let direct = bytes[1] & FLAG_DIRECT != 0;
        let sender_len = bytes[11] as usize;
        let text_len = bytes[12] as usize;
        if sender_len > MAX_SENDER || text_len > MAX_TEXT {
            return None;
        }
        let mut rec = Self::default();
        rec.is_direct = direct;
        rec.channel_idx = bytes[2];
        rec.path_len = bytes[3];
        rec.rssi = i16::from_le_bytes(bytes[4..6].try_into().ok()?);
        rec.snr = bytes[6] as i8;
        rec.timestamp = u32::from_le_bytes(bytes[7..11].try_into().ok()?);
        rec.txt_type = bytes[13] & 0x3F;
        let signed = version >= TEXT_VERSION && rec.is_signed();
        let expected = HEADER_SIZE
            + if direct { 32 } else { 0 }
            + sender_len
            + text_len
            + if signed { 4 } else { 0 };
        if bytes.len() != expected {
            return None;
        }
        let mut pos = HEADER_SIZE;
        if direct {
            rec.sender_pub.copy_from_slice(&bytes[pos..pos + 32]);
            pos += 32;
        }
        rec.sender[..sender_len].copy_from_slice(&bytes[pos..pos + sender_len]);
        pos += sender_len;
        rec.text[..text_len].copy_from_slice(&bytes[pos..pos + text_len]);
        pos += text_len;
        if signed {
            rec.signed_prefix.copy_from_slice(&bytes[pos..pos + 4]);
        }
        Some(rec)
    }
}

/// A channel datagram as stored on flash.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelDataRecord {
    pub channel_idx: u8,
    pub path_len: u8,
    pub snr: i8,
    pub data_type: u16,
    pub data: Vec<u8, MAX_GROUP_DATA>,
}

impl ChannelDataRecord {
    pub fn new(
        channel_idx: u8,
        path_len: u8,
        snr: i8,
        data_type: u16,
        data: &[u8],
    ) -> Option<Self> {
        if data.len() > MAX_GROUP_DATA {
            return None;
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(data).ok()?;
        Some(Self {
            channel_idx,
            path_len,
            snr,
            data_type,
            data: bytes,
        })
    }

    pub fn encoded_len(&self) -> usize {
        HEADER_SIZE + 2 + self.data.len()
    }

    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let len = self.encoded_len();
        if out.len() < len {
            return None;
        }
        out[..HEADER_SIZE].fill(0);
        out[0] = CHANNEL_DATA_VERSION;
        out[2] = self.channel_idx;
        out[3] = self.path_len;
        out[6] = self.snr as u8;
        out[HEADER_SIZE..HEADER_SIZE + 2].copy_from_slice(&self.data_type.to_le_bytes());
        out[HEADER_SIZE + 2..len].copy_from_slice(&self.data);
        Some(len)
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < HEADER_SIZE + 2 || bytes[0] != CHANNEL_DATA_VERSION {
            return None;
        }
        let data_len = bytes.len() - HEADER_SIZE - 2;
        if data_len > MAX_GROUP_DATA {
            return None;
        }
        let mut data = Vec::new();
        data.extend_from_slice(&bytes[HEADER_SIZE + 2..]).ok()?;
        Some(Self {
            channel_idx: bytes[2],
            path_len: bytes[3],
            snr: bytes[6] as i8,
            data_type: u16::from_le_bytes([bytes[HEADER_SIZE], bytes[HEADER_SIZE + 1]]),
            data,
        })
    }
}

const _: () = assert!(HEADER_SIZE + 32 + MAX_SENDER + MAX_TEXT + 4 == MESSAGE_RECORD_MAX);
const _: () = assert!(CHANNEL_DATA_RECORD_MAX < MESSAGE_RECORD_MAX);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_channel() {
        let record = MessageRecord::new(
            3,
            2,
            false,
            "alice",
            [0; 32],
            "hello world",
            1_700_000_000,
            -72,
            7,
        );
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        let n = record.encode(&mut buf).unwrap();
        assert_eq!(n, record.encoded_len());
        let decoded = MessageRecord::decode(&buf[..n]).unwrap();
        assert_eq!(decoded, record);
        assert_eq!(decoded.sender_str(), "alice");
        assert_eq!(decoded.text_str(), "hello world");
        assert_eq!(decoded.rssi, -72);
    }

    #[test]
    fn roundtrip_direct() {
        let record = MessageRecord::new(
            0,
            0,
            true,
            "a1b2c3d4e5f6",
            [9; 32],
            "ping",
            42,
            10,
            -3,
        );
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        let n = record.encode(&mut buf).unwrap();
        assert_eq!(n, HEADER_SIZE + 32 + 12 + 4);
        assert_eq!(MessageRecord::decode(&buf[..n]).unwrap(), record);
    }

    #[test]
    fn roundtrip_signed() {
        let mut record = MessageRecord::new(
            0,
            0,
            true,
            "room",
            [3; 32],
            "a post",
            1234,
            -60,
            5,
        );
        record.txt_type = TXT_TYPE_SIGNED_PLAIN;
        record.signed_prefix = [0xDE, 0xAD, 0xBE, 0xEF];
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        let n = record.encode(&mut buf).unwrap();
        let decoded = MessageRecord::decode(&buf[..n]).unwrap();
        assert_eq!(decoded, record);
        assert_eq!(decoded.signed_prefix, [0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn decodes_legacy_v1_text() {
        // A version-1 record has no signing prefix.
        let record = MessageRecord::new(0, 0, false, "x", [0; 32], "old", 1, 0, 0);
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        let n = record.encode(&mut buf).unwrap();
        buf[0] = TEXT_VERSION_V1;
        let decoded = MessageRecord::decode(&buf[..n]).unwrap();
        assert_eq!(decoded.text_str(), "old");
        assert_eq!(decoded.signed_prefix, [0; 4]);
    }

    #[test]
    fn short_messages_are_compact() {
        let record = MessageRecord::new(1, 1, false, "bob", [0; 32], "ok", 1, -50, 4);
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        let n = record.encode(&mut buf).unwrap();
        assert_eq!(n, HEADER_SIZE + 3 + 2);
        assert!(n < 24, "short message should be compact, got {n}");
    }

    #[test]
    fn truncates_long_fields() {
        let sender = "s".repeat(40);
        let text = "t".repeat(200);
        let record = MessageRecord::new(0, 0, false, &sender, [0; 32], &text, 1, 0, 0);
        assert_eq!(record.sender_str(), "s".repeat(MAX_SENDER));
        assert_eq!(record.text_str(), "t".repeat(MAX_TEXT));
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        let n = record.encode(&mut buf).unwrap();
        assert_eq!(n, HEADER_SIZE + MAX_SENDER + MAX_TEXT);
        assert_eq!(MessageRecord::decode(&buf[..n]).unwrap(), record);

        // A direct signed record with the same fields uses the full budget.
        let mut direct = MessageRecord::new(0, 0, true, &sender, [1; 32], &text, 1, 0, 0);
        direct.txt_type = TXT_TYPE_SIGNED_PLAIN;
        let n = direct.encode(&mut buf).unwrap();
        assert_eq!(n, MESSAGE_RECORD_MAX);
        assert_eq!(MessageRecord::decode(&buf[..n]).unwrap(), direct);
    }

    #[test]
    fn truncation_does_not_split_utf8() {
        // 15 two-byte characters (30 bytes) plus a three-byte character.
        // The 31-byte sender limit falls inside the last code point.
        let sender = "ééééééééééééééé€";
        let record = MessageRecord::new(0, 0, false, sender, [0; 32], "x", 0, 0, 0);
        assert_eq!(record.sender_str(), "ééééééééééééééé");
    }

    #[test]
    fn rejects_bad_records() {
        let record = MessageRecord::new(0, 0, false, "x", [0; 32], "y", 0, 0, 0);
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        let n = record.encode(&mut buf).unwrap();
        assert!(MessageRecord::decode(&buf[..n - 1]).is_none());
        buf[0] = 9;
        assert!(MessageRecord::decode(&buf[..n]).is_none());
        buf[0] = TEXT_VERSION;
        buf[12] = 200;
        assert!(MessageRecord::decode(&buf[..n]).is_none());
    }

    #[test]
    fn channel_data_roundtrip() {
        let payload = [1u8, 2, 3, 4, 5, 6];
        let record = ChannelDataRecord::new(2, 0xFF, -55, 0x1234, &payload).unwrap();
        let mut buf = [0u8; CHANNEL_DATA_RECORD_MAX];
        let n = record.encode(&mut buf).unwrap();
        let decoded = ChannelDataRecord::decode(&buf[..n]).unwrap();
        assert_eq!(decoded, record);
        assert_eq!(decoded.data_type, 0x1234);
        assert_eq!(&decoded.data[..], &payload);
    }

    #[test]
    fn channel_data_rejects_oversize() {
        let data = [0u8; MAX_GROUP_DATA + 1];
        assert!(ChannelDataRecord::new(0, 0, 0, 1, &data).is_none());
    }
}
