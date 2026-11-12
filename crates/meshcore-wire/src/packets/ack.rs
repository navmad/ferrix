//! Lightweight Acknowledgment Packet (`PAYLOAD_TYPE_ACK = 0x03`)
//!
//! Confirms receipt and end-to-end delivery of direct messages or requests.

use crate::packet::{Packet, PAYLOAD_TYPE_ACK};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AckPacket {
    pub dest_hash: [u8; 4],
    pub src_hash: [u8; 4],
    pub ack_packet_hash: [u8; 4],
    pub timestamp: u32,
}

impl AckPacket {
    pub fn new(dest_hash: [u8; 4], src_hash: [u8; 4], ack_packet_hash: [u8; 4], timestamp: u32) -> Self {
        Self {
            dest_hash,
            src_hash,
            ack_packet_hash,
            timestamp,
        }
    }

    /// Encode into wire format: `[dest_hash: 4] [src_hash: 4] [ack_hash: 4] [timestamp: 4]`
    pub fn encode(&self, dest: &mut [u8]) -> Result<usize, ()> {
        if dest.len() < 16 {
            return Err(());
        }

        dest[0..4].copy_from_slice(&self.dest_hash);
        dest[4..8].copy_from_slice(&self.src_hash);
        dest[8..12].copy_from_slice(&self.ack_packet_hash);
        dest[12..16].copy_from_slice(&self.timestamp.to_le_bytes());

        Ok(16)
    }

    /// Decode acknowledgment payload from bytes
    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.len() < 16 {
            return Err(());
        }

        let mut dest_hash = [0u8; 4];
        dest_hash.copy_from_slice(&src[0..4]);

        let mut src_hash = [0u8; 4];
        src_hash.copy_from_slice(&src[4..8]);

        let mut ack_packet_hash = [0u8; 4];
        ack_packet_hash.copy_from_slice(&src[8..12]);

        let timestamp = u32::from_le_bytes([src[12], src[13], src[14], src[15]]);

        Ok(Self {
            dest_hash,
            src_hash,
            ack_packet_hash,
            timestamp,
        })
    }

    /// Pack into a complete MeshCore Packet
    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_ACK);
        let mut buf = [0u8; 16];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
