//! Direct Text Message Packet (`PAYLOAD_TYPE_TXT_MSG = 0x02`)
//!
//! Point-to-point text message sent between specific node identities.

use heapless::String;
use crate::meshcore::packet::{Packet, PAYLOAD_TYPE_TXT_MSG};

#[derive(Clone, Debug, PartialEq)]
pub struct TxtMsgPacket {
    pub dest_hash: [u8; 4],
    pub src_hash: [u8; 4],
    pub timestamp: u32,
    pub mac: [u8; 2],
    pub text: String<128>,
}

impl TxtMsgPacket {
    pub fn new(dest_hash: [u8; 4], src_hash: [u8; 4], text: &str, timestamp: u32) -> Self {
        let mut t = String::new();
        let _ = t.push_str(text);

        let mut chk = 0xA55Au16;
        for &b in text.as_bytes() {
            chk = chk.wrapping_add(b as u16).rotate_left(2);
        }

        Self {
            dest_hash,
            src_hash,
            timestamp,
            mac: chk.to_le_bytes(),
            text: t,
        }
    }

    /// Encode into wire format:
    /// `[dest_hash: 4] [src_hash: 4] [timestamp: 4] [mac: 2] [text_len: 1] [text...]`
    pub fn encode(&self, dest: &mut [u8]) -> Result<usize, ()> {
        let text_bytes = self.text.as_bytes();
        let needed = 4 + 4 + 4 + 2 + 1 + text_bytes.len();
        if dest.len() < needed {
            return Err(());
        }

        dest[0..4].copy_from_slice(&self.dest_hash);
        dest[4..8].copy_from_slice(&self.src_hash);
        dest[8..12].copy_from_slice(&self.timestamp.to_le_bytes());
        dest[12..14].copy_from_slice(&self.mac);
        dest[14] = text_bytes.len() as u8;
        dest[15..15 + text_bytes.len()].copy_from_slice(text_bytes);

        Ok(15 + text_bytes.len())
    }

    /// Decode direct text message from bytes
    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.len() < 15 {
            return Err(());
        }

        let mut dest_hash = [0u8; 4];
        dest_hash.copy_from_slice(&src[0..4]);

        let mut src_hash = [0u8; 4];
        src_hash.copy_from_slice(&src[4..8]);

        let timestamp = u32::from_le_bytes([src[8], src[9], src[10], src[11]]);

        let mut mac = [0u8; 2];
        mac.copy_from_slice(&src[12..14]);

        let text_len = src[14] as usize;
        if src.len() < 15 + text_len {
            return Err(());
        }

        let text_str = core::str::from_utf8(&src[15..15 + text_len]).map_err(|_| ())?;
        let mut text = String::new();
        let _ = text.push_str(text_str);

        Ok(Self {
            dest_hash,
            src_hash,
            timestamp,
            mac,
            text,
        })
    }

    /// Pack into a complete MeshCore Packet
    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_TXT_MSG);
        let mut buf = [0u8; 192];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
