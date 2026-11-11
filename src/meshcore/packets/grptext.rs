//! Group / Channel Text Message Packet (`PAYLOAD_TYPE_GRP_TXT = 0x05`)
//!
//! Broadcasts group channel messages (prefixed with channel hash and MAC, AES-128-ECB encrypted).

use heapless::String;
use crate::meshcore::packet::{Packet, PAYLOAD_TYPE_GRP_TXT};

#[derive(Clone, Debug, PartialEq)]
pub struct GrpTextPacket {
    pub channel_hash: u8,
    pub mac: [u8; 2],
    pub timestamp: u32,
    pub txt_type: u8,
    pub attempt: u8,
    pub sender: String<32>,
    pub message: String<96>,
}

impl GrpTextPacket {

    /// Text type constants for GRP_TXT plaintext byte 4 (upper 6 bits)
    pub const TXT_TYPE_PLAIN: u8 = 0x00;
    pub const TXT_TYPE_CLI: u8 = 0x01;
    pub const TXT_TYPE_SIGNED: u8 = 0x02;
    pub const TXT_TYPE_CLI_CMD: u8 = 0x03;

    pub fn new(channel_hash: u8, sender: &str, message: &str, timestamp: u32, txt_type: u8, attempt: u8) -> Self {
        let mut s = String::new();
        let _ = s.push_str(sender);
        let mut m = String::new();
        let _ = m.push_str(message);

        Self {
            channel_hash,
            mac: [0, 0],
            timestamp,
            txt_type: txt_type & 0x3F,    // NEW: mask to upper 6 bits
            attempt: attempt & 0x03,      // NEW: mask to lower 2 bits
            sender: s,
            message: m,
        }
    }

    /// Encode into official MeshCore wire format:
    /// `[channel_hash: 1B] [mac: 2B] [AES128-ECB ciphertext: N*16B]`
    /// Plaintext inside is: `[timestamp: 4B le] [txt_type: 1B (0)] [<sender>: <message>\0]` zero-padded to 16B.
    pub fn encode_encrypted(&self, secret: &[u8], dest: &mut [u8]) -> Result<usize, ()> {
        if dest.len() < 1 + 2 + 16 {
            return Err(());
        }

        dest[0] = self.channel_hash;

        let mut pt = [0u8; 192];
        pt[0..4].copy_from_slice(&self.timestamp.to_le_bytes());
        pt[4] = (self.txt_type << 2) | self.attempt;

        let mut offset = 5;
        let s_bytes = self.sender.as_bytes();
        if offset + s_bytes.len() + 2 > pt.len() {
            return Err(());
        }
        pt[offset..offset + s_bytes.len()].copy_from_slice(s_bytes);
        offset += s_bytes.len();
        pt[offset] = b':';
        offset += 1;
        pt[offset] = b' ';
        offset += 1;

        let m_bytes = self.message.as_bytes();
        let avail = pt.len() - offset - 1;
        let take = m_bytes.len().min(avail);
        pt[offset..offset + take].copy_from_slice(&m_bytes[..take]);
        offset += take;
        pt[offset] = 0; // null terminator
        offset += 1;

        let enc_and_mac_len = crate::meshcore::crypto::meshcore_encrypt_then_mac(
            secret,
            &mut dest[1..],
            &pt[..offset],
        )?;

        Ok(1 + enc_and_mac_len)
    }

    /// Decode group text packet payload from official MeshCore encrypted format
    pub fn decode_encrypted(src: &[u8], secret: &[u8]) -> Result<Self, ()> {
        if src.len() < 1 + 2 + 16 {
            return Err(());
        }

        let channel_hash = src[0];
        let mut mac = [0u8; 2];
        mac.copy_from_slice(&src[1..3]);

        let mut pt = [0u8; 192];
        let pt_len = crate::meshcore::crypto::meshcore_mac_then_decrypt(secret, &mut pt, &src[1..])?;
        if pt_len < 5 {
            return Err(());
        }

        let timestamp = u32::from_le_bytes([pt[0], pt[1], pt[2], pt[3]]);
        let combined = pt[4];
        let txt_type = combined >> 2;
        let attempt = combined & 0x03;

        let text_slice = &pt[5..pt_len];
        let str_len = text_slice.iter().position(|&b| b == 0).unwrap_or(text_slice.len());
        let text_str = core::str::from_utf8(&text_slice[..str_len]).map_err(|_| ())?;

        let (sender_str, msg_str) = if let Some((s, m)) = text_str.split_once(": ") {
            (s, m)
        } else {
            ("Unknown", text_str)
        };

        let mut sender = String::new();
        let _ = sender.push_str(sender_str);
        let mut message = String::new();
        let _ = message.push_str(msg_str);

        Ok(Self {
            channel_hash,
            mac,
            timestamp,
            txt_type,
            attempt,
            sender,
            message,
        })
    }

    /// Pack into a complete MeshCore Packet
    pub fn to_packet(&self, secret: &[u8], route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_GRP_TXT);
        let mut buf = [0u8; 192];
        let len = self.encode_encrypted(secret, &mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
