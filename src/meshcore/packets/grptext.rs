//! Group / Channel Text Message Packet (`PAYLOAD_TYPE_GRP_TXT = 0x05`)
//!
//! Broadcasts group channel messages (prefixed with channel hash and MAC).

use heapless::String;
use crate::meshcore::packet::{Packet, PAYLOAD_TYPE_GRP_TXT};

#[derive(Clone, Debug, PartialEq)]
pub struct GrpTextPacket {
    pub channel_hash: u8,
    pub mac: [u8; 2],
    pub timestamp: u32,
    pub sender: String<32>,
    pub message: String<96>,
}

impl GrpTextPacket {
    pub fn new(channel_hash: u8, sender: &str, message: &str, timestamp: u32) -> Self {
        let mut s = String::new();
        let _ = s.push_str(sender);
        let mut m = String::new();
        let _ = m.push_str(message);

        // Simple checksum MAC over sender + message
        let mut chk = 0x55AAu16;
        for &b in sender.as_bytes() {
            chk = chk.wrapping_add(b as u16).rotate_left(3);
        }
        for &b in message.as_bytes() {
            chk = chk.wrapping_add(b as u16).rotate_left(3);
        }

        Self {
            channel_hash,
            mac: chk.to_le_bytes(),
            timestamp,
            sender: s,
            message: m,
        }
    }

    /// Encode into wire format:
    /// `[channel_hash: 1] [mac: 2] [timestamp: 4] [sender_len: 1] [sender...] [msg...]`
    pub fn encode(&self, dest: &mut [u8]) -> Result<usize, ()> {
        let s_bytes = self.sender.as_bytes();
        let m_bytes = self.message.as_bytes();
        let needed = 1 + 2 + 4 + 1 + s_bytes.len() + m_bytes.len();
        if dest.len() < needed {
            return Err(());
        }

        let mut offset = 0;
        dest[offset] = self.channel_hash;
        offset += 1;

        dest[offset..offset + 2].copy_from_slice(&self.mac);
        offset += 2;

        dest[offset..offset + 4].copy_from_slice(&self.timestamp.to_le_bytes());
        offset += 4;

        dest[offset] = s_bytes.len() as u8;
        offset += 1;

        dest[offset..offset + s_bytes.len()].copy_from_slice(s_bytes);
        offset += s_bytes.len();

        dest[offset..offset + m_bytes.len()].copy_from_slice(m_bytes);
        offset += m_bytes.len();

        Ok(offset)
    }

    /// Decode group text packet payload from bytes
    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.len() < 1 + 2 + 4 + 1 {
            return Err(());
        }

        let mut offset = 0;
        let channel_hash = src[offset];
        offset += 1;

        let mut mac = [0u8; 2];
        mac.copy_from_slice(&src[offset..offset + 2]);
        offset += 2;

        let timestamp = u32::from_le_bytes([
            src[offset],
            src[offset + 1],
            src[offset + 2],
            src[offset + 3],
        ]);
        offset += 4;

        let s_len = src[offset] as usize;
        offset += 1;

        if src.len() < offset + s_len {
            return Err(());
        }

        let s_str = core::str::from_utf8(&src[offset..offset + s_len]).map_err(|_| ())?;
        let mut sender = String::new();
        let _ = sender.push_str(s_str);
        offset += s_len;

        let m_str = core::str::from_utf8(&src[offset..]).map_err(|_| ())?;
        let mut message = String::new();
        let _ = message.push_str(m_str);

        Ok(Self {
            channel_hash,
            mac,
            timestamp,
            sender,
            message,
        })
    }

    /// Pack into a complete MeshCore Packet
    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_GRP_TXT);
        let mut buf = [0u8; 192];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
