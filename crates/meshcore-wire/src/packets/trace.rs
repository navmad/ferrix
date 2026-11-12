//! TRACE (`PAYLOAD_TYPE_TRACE = 0x09`).
//!
//! Plain payload: `tag u32 ‖ auth u32 ‖ flags u8 ‖ hashes…`. TRACE is direct
//! only and is not encrypted.

use heapless::Vec;

use crate::packet::{Packet, PAYLOAD_TYPE_TRACE};

pub const MAX_TRACE_HASHES: usize = 175;

#[derive(Clone, Debug, PartialEq)]
pub struct TracePacket {
    pub tag: u32,
    pub auth: u32,
    pub flags: u8,
    pub hashes: Vec<u8, MAX_TRACE_HASHES>,
}

impl TracePacket {
    pub fn new(tag: u32, auth: u32, flags: u8, hashes: &[u8]) -> Result<Self, ()> {
        if hashes.len() > MAX_TRACE_HASHES {
            return Err(());
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(hashes).map_err(|_| ())?;
        Ok(Self {
            tag,
            auth,
            flags,
            hashes: bytes,
        })
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, ()> {
        let needed = 9 + self.hashes.len();
        if out.len() < needed {
            return Err(());
        }
        out[0..4].copy_from_slice(&self.tag.to_le_bytes());
        out[4..8].copy_from_slice(&self.auth.to_le_bytes());
        out[8] = self.flags;
        out[9..needed].copy_from_slice(&self.hashes);
        Ok(needed)
    }

    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        // The C++ reads 9 bytes unconditionally; below that it over-reads.
        if src.len() < 9 {
            return Err(());
        }
        let tag = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let auth = u32::from_le_bytes([src[4], src[5], src[6], src[7]]);
        let flags = src[8];
        let mut hashes = Vec::new();
        hashes.extend_from_slice(&src[9..]).map_err(|_| ())?;
        Ok(Self {
            tag,
            auth,
            flags,
            hashes,
        })
    }

    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_TRACE);
        let mut buf = [0u8; 184];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
