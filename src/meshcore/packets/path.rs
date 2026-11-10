//! Route Discovery & Path Packet (`PAYLOAD_TYPE_PATH = 0x08`)
//!
//! Returns route paths discovered between mesh nodes.

use crate::meshcore::packet::{Packet, PAYLOAD_TYPE_PATH};

pub const MAX_PATH_HOPS: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathPacket {
    pub target_hash: [u8; 4],
    pub hop_count: u8,
    pub hops: [u8; MAX_PATH_HOPS],
}

impl PathPacket {
    pub fn new(target_hash: [u8; 4]) -> Self {
        Self {
            target_hash,
            hop_count: 0,
            hops: [0u8; MAX_PATH_HOPS],
        }
    }

    pub fn add_hop(&mut self, hop_hash: u8) -> bool {
        if (self.hop_count as usize) < MAX_PATH_HOPS {
            self.hops[self.hop_count as usize] = hop_hash;
            self.hop_count += 1;
            true
        } else {
            false
        }
    }

    /// Encode into wire format: `[target_hash: 4] [hop_count: 1] [hops...]`
    pub fn encode(&self, dest: &mut [u8]) -> Result<usize, ()> {
        let hop_bytes = self.hop_count as usize;
        if dest.len() < 4 + 1 + hop_bytes {
            return Err(());
        }

        dest[0..4].copy_from_slice(&self.target_hash);
        dest[4] = self.hop_count;
        dest[5..5 + hop_bytes].copy_from_slice(&self.hops[..hop_bytes]);

        Ok(5 + hop_bytes)
    }

    /// Decode path payload from bytes
    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.len() < 5 {
            return Err(());
        }

        let mut target_hash = [0u8; 4];
        target_hash.copy_from_slice(&src[0..4]);

        let hop_count = src[4];
        let hop_bytes = hop_count as usize;
        if hop_bytes > MAX_PATH_HOPS || src.len() < 5 + hop_bytes {
            return Err(());
        }

        let mut hops = [0u8; MAX_PATH_HOPS];
        hops[..hop_bytes].copy_from_slice(&src[5..5 + hop_bytes]);

        Ok(Self {
            target_hash,
            hop_count,
            hops,
        })
    }

    /// Pack into a complete MeshCore Packet
    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_PATH);
        let mut buf = [0u8; 64];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
