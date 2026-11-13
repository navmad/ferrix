//! Fixed-size persisted message record.
//!
//! The flash queue stores one 236-byte record per message. This module owns
//! the byte layout so it can be round-trip tested on the host; the firmware
//! only converts its in-RAM `ReceivedMessage` to and from this record.

/// Bytes written to flash for a single message.
pub const MESSAGE_RECORD_SIZE: usize = 236;

const SENDER_OFFSET: usize = 4;
const SENDER_PUB_OFFSET: usize = SENDER_OFFSET + 32;
const TEXT_OFFSET: usize = SENDER_PUB_OFFSET + 32;
const TIMESTAMP_OFFSET: usize = TEXT_OFFSET + 160;
const RSSI_OFFSET: usize = TIMESTAMP_OFFSET + 4;
const SNR_OFFSET: usize = RSSI_OFFSET + 2;

/// A message as stored on flash.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MessageRecord {
    pub channel_idx: u8,
    pub path_len: u8,
    pub is_direct: bool,
    /// NUL-padded display name (max 31 bytes plus terminator).
    pub sender: [u8; 32],
    /// Full sender public key; all zeroes for channel messages.
    pub sender_pub: [u8; 32],
    /// NUL-padded text (max 159 bytes plus terminator).
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

fn put_str(dst: &mut [u8], src: &str) {
    let bytes = src.as_bytes();
    let n = bytes.len().min(dst.len() - 1);
    dst[..n].copy_from_slice(&bytes[..n]);
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
        put_str(&mut rec.sender, sender);
        put_str(&mut rec.text, text);
        rec
    }

    pub fn sender_str(&self) -> &str {
        get_str(&self.sender)
    }

    pub fn text_str(&self) -> &str {
        get_str(&self.text)
    }

    /// Serialize into the flash representation.
    pub fn to_bytes(&self) -> [u8; MESSAGE_RECORD_SIZE] {
        let mut out = [0u8; MESSAGE_RECORD_SIZE];
        out[0] = self.channel_idx;
        out[1] = self.path_len;
        out[2] = if self.is_direct { 1 } else { 0 };
        out[SENDER_OFFSET..SENDER_PUB_OFFSET].copy_from_slice(&self.sender);
        out[SENDER_PUB_OFFSET..TEXT_OFFSET].copy_from_slice(&self.sender_pub);
        out[TEXT_OFFSET..TIMESTAMP_OFFSET].copy_from_slice(&self.text);
        out[TIMESTAMP_OFFSET..RSSI_OFFSET].copy_from_slice(&self.timestamp.to_le_bytes());
        out[RSSI_OFFSET..SNR_OFFSET].copy_from_slice(&self.rssi.to_le_bytes());
        out[SNR_OFFSET] = self.snr as u8;
        out
    }

    /// Parse a record; returns `None` when the slice is too short.
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < MESSAGE_RECORD_SIZE {
            return None;
        }
        let mut rec = Self::default();
        rec.channel_idx = bytes[0];
        rec.path_len = bytes[1];
        rec.is_direct = bytes[2] != 0;
        rec.sender.copy_from_slice(&bytes[SENDER_OFFSET..SENDER_PUB_OFFSET]);
        rec.sender_pub.copy_from_slice(&bytes[SENDER_PUB_OFFSET..TEXT_OFFSET]);
        rec.text.copy_from_slice(&bytes[TEXT_OFFSET..TIMESTAMP_OFFSET]);
        rec.timestamp = u32::from_le_bytes(
            bytes[TIMESTAMP_OFFSET..RSSI_OFFSET].try_into().ok()?,
        );
        rec.rssi = i16::from_le_bytes(bytes[RSSI_OFFSET..SNR_OFFSET].try_into().ok()?);
        rec.snr = bytes[SNR_OFFSET] as i8;
        Some(rec)
    }
}

const _: () = assert!(MESSAGE_RECORD_SIZE == 236);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let record = MessageRecord::new(
            3,
            2,
            true,
            "alice",
            [7; 32],
            "hello world",
            1_700_000_000,
            -72,
            7,
        );
        let bytes = record.to_bytes();
        assert_eq!(bytes.len(), MESSAGE_RECORD_SIZE);
        let decoded = MessageRecord::from_bytes(&bytes).unwrap();
        assert_eq!(decoded, record);
        assert_eq!(decoded.sender_str(), "alice");
        assert_eq!(decoded.text_str(), "hello world");
        assert_eq!(decoded.rssi, -72);
    }

    #[test]
    fn truncates_long_fields() {
        let sender = "s".repeat(40);
        let text = "t".repeat(200);
        let record = MessageRecord::new(0, 0, false, &sender, [0; 32], &text, 1, 0, 0);
        assert_eq!(record.sender_str(), "s".repeat(31));
        assert_eq!(record.text_str(), "t".repeat(159));
    }

    #[test]
    fn rejects_short_slice() {
        assert!(MessageRecord::from_bytes(&[0u8; 10]).is_none());
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
