//! Acknowledgment (`PAYLOAD_TYPE_ACK = 0x03`).
//!
//! The payload is the 4-byte ACK CRC followed by an optional tail. Companions
//! send 6 bytes: `crc(4) ‖ extended_attempt ‖ random`. Servers send 4, and room
//! keep-alives send 5. Reception only requires the first 4 bytes.

use crate::packet::{Packet, PAYLOAD_TYPE_ACK};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AckPacket {
    pub crc: u32,
    /// Up to two trailing bytes (extended attempt, random).
    pub tail: [u8; 2],
    pub tail_len: u8,
}

impl AckPacket {
    /// Bare ACK: the CRC only (4 bytes).
    pub const fn new(crc: u32) -> Self {
        Self {
            crc,
            tail: [0, 0],
            tail_len: 0,
        }
    }

    /// Companion ACK: `crc ‖ extended_attempt ‖ random` (6 bytes).
    pub const fn with_extended(crc: u32, extended_attempt: u8, random: u8) -> Self {
        Self {
            crc,
            tail: [extended_attempt, random],
            tail_len: 2,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, ()> {
        let len = 4 + self.tail_len as usize;
        if self.tail_len > 2 || out.len() < len {
            return Err(());
        }
        out[0..4].copy_from_slice(&self.crc.to_le_bytes());
        out[4..len].copy_from_slice(&self.tail[..self.tail_len as usize]);
        Ok(len)
    }

    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.len() < 4 {
            return Err(());
        }
        let crc = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let tail_len = (src.len() - 4).min(2) as u8;
        let mut tail = [0u8; 2];
        tail[..tail_len as usize].copy_from_slice(&src[4..4 + tail_len as usize]);
        Ok(Self {
            crc,
            tail,
            tail_len,
        })
    }

    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_ACK);
        let mut buf = [0u8; 6];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
