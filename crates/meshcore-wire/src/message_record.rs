//! Variable-length persisted message record.
//!
//! The flash queue stores one compact record per message. Only the fields that
//! are actually present are written: a short channel message costs a couple of
//! dozen bytes instead of a fixed 236-byte slot, which directly reduces the
//! bytes written and therefore the page erases over the region's lifetime.
//!
//! Layout (little endian):
//!
//! ```text
//! 0      format version (1)
//! 1      flags: bit 0 = direct message
//! 2      channel index
//! 3      path length
//! 4..6   RSSI (i16)
//! 6      SNR (i8)
//! 7..11  timestamp (u32)
//! 11     sender length
//! 12     text length
//! 13     reserved (0)
//! 14..   sender public key (32 bytes, direct messages only)
//!        sender display name
//!        text
//! ```

/// Largest possible encoded record (31-byte name + 159-byte text + key).
pub const MESSAGE_RECORD_MAX: usize = 236;
/// Smallest encoded record (empty name and text, channel message).
pub const MESSAGE_RECORD_MIN: usize = HEADER_SIZE;

const FORMAT_VERSION: u8 = 1;
const HEADER_SIZE: usize = 14;
const FLAG_DIRECT: u8 = 0x01;

const MAX_SENDER: usize = 31;
const MAX_TEXT: usize = 159;

/// A message as stored on flash.
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
        }
    }
}

fn truncate_str(dst: &mut [u8], src: &str, max: usize) -> usize {
    let bytes = src.as_bytes();
    let n = bytes.len().min(max).min(dst.len());
    dst[..n].copy_from_slice(&bytes[..n]);
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
        self.sender.iter().position(|&b| b == 0).unwrap_or(MAX_SENDER).min(MAX_SENDER)
    }

    fn text_len(&self) -> usize {
        self.text.iter().position(|&b| b == 0).unwrap_or(MAX_TEXT).min(MAX_TEXT)
    }

    /// Number of bytes the encoded form occupies.
    pub fn encoded_len(&self) -> usize {
        HEADER_SIZE
            + if self.is_direct { 32 } else { 0 }
            + self.sender_len()
            + self.text_len()
    }

    /// Serialize into `out`, returning the encoded length.
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let len = self.encoded_len();
        if out.len() < len {
            return None;
        }
        let sender_len = self.sender_len();
        let text_len = self.text_len();
        out[0] = FORMAT_VERSION;
        out[1] = if self.is_direct { FLAG_DIRECT } else { 0 };
        out[2] = self.channel_idx;
        out[3] = self.path_len;
        out[4..6].copy_from_slice(&self.rssi.to_le_bytes());
        out[6] = self.snr as u8;
        out[7..11].copy_from_slice(&self.timestamp.to_le_bytes());
        out[11] = sender_len as u8;
        out[12] = text_len as u8;
        out[13] = 0;
        let mut pos = HEADER_SIZE;
        if self.is_direct {
            out[pos..pos + 32].copy_from_slice(&self.sender_pub);
            pos += 32;
        }
        out[pos..pos + sender_len].copy_from_slice(&self.sender[..sender_len]);
        pos += sender_len;
        out[pos..pos + text_len].copy_from_slice(&self.text[..text_len]);
        Some(len)
    }

    /// Parse a record; returns `None` for a short, truncated or unknown record.
    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < HEADER_SIZE || bytes[0] != FORMAT_VERSION {
            return None;
        }
        let direct = bytes[1] & FLAG_DIRECT != 0;
        let sender_len = bytes[11] as usize;
        let text_len = bytes[12] as usize;
        if sender_len > MAX_SENDER || text_len > MAX_TEXT {
            return None;
        }
        let expected = HEADER_SIZE + if direct { 32 } else { 0 } + sender_len + text_len;
        if bytes.len() != expected {
            return None;
        }
        let mut rec = Self::default();
        rec.is_direct = direct;
        rec.channel_idx = bytes[2];
        rec.path_len = bytes[3];
        rec.rssi = i16::from_le_bytes(bytes[4..6].try_into().ok()?);
        rec.snr = bytes[6] as i8;
        rec.timestamp = u32::from_le_bytes(bytes[7..11].try_into().ok()?);
        let mut pos = HEADER_SIZE;
        if direct {
            rec.sender_pub.copy_from_slice(&bytes[pos..pos + 32]);
            pos += 32;
        }
        rec.sender[..sender_len].copy_from_slice(&bytes[pos..pos + sender_len]);
        pos += sender_len;
        rec.text[..text_len].copy_from_slice(&bytes[pos..pos + text_len]);
        Some(rec)
    }
}

const _: () = assert!(HEADER_SIZE + 32 + MAX_SENDER + MAX_TEXT == MESSAGE_RECORD_MAX);

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
        let decoded = MessageRecord::decode(&buf[..n]).unwrap();
        assert_eq!(decoded, record);
        assert_eq!(decoded.sender_pub, [9; 32]);
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
        // Channel records have no public key: header + 31-byte name + 159-byte text.
        assert_eq!(n, HEADER_SIZE + MAX_SENDER + MAX_TEXT);
        assert_eq!(MessageRecord::decode(&buf[..n]).unwrap(), record);

        // A direct record with the same fields uses the full 236-byte budget.
        let direct = MessageRecord::new(0, 0, true, &sender, [1; 32], &text, 1, 0, 0);
        let n = direct.encode(&mut buf).unwrap();
        assert_eq!(n, MESSAGE_RECORD_MAX);
        assert_eq!(MessageRecord::decode(&buf[..n]).unwrap(), direct);
    }

    #[test]
    fn rejects_bad_records() {
        assert!(MessageRecord::decode(&[0u8; 10]).is_none());
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        let record = MessageRecord::new(0, 0, false, "x", [0; 32], "y", 0, 0, 0);
        let n = record.encode(&mut buf).unwrap();
        // Truncated payload.
        assert!(MessageRecord::decode(&buf[..n - 1]).is_none());
        // Wrong version.
        buf[0] = 2;
        assert!(MessageRecord::decode(&buf[..n]).is_none());
        // Oversized length field.
        buf[0] = FORMAT_VERSION;
        buf[12] = 200;
        assert!(MessageRecord::decode(&buf[..n]).is_none());
    }

    #[test]
    fn padded_fields_are_trimmed() {
        let mut record = MessageRecord::default();
        record.text[0] = b'h';
        record.text[1] = b'i';
        assert_eq!(record.text_str(), "hi");
        assert_eq!(record.sender_str(), "");
    }
}
