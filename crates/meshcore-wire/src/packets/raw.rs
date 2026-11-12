//! RAW_CUSTOM (`PAYLOAD_TYPE_RAW_CUSTOM = 0x0F`).
//!
//! Opaque bytes, delivered only when direct with zero hops and never
//! flood-forwarded.

use heapless::Vec;

use crate::packet::{Packet, PAYLOAD_TYPE_RAW_CUSTOM};

pub const MAX_RAW_CUSTOM: usize = 184;

#[derive(Clone, Debug, PartialEq)]
pub struct RawPacket {
    pub data: Vec<u8, MAX_RAW_CUSTOM>,
}

impl RawPacket {
    pub fn new(data: &[u8]) -> Result<Self, ()> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(data).map_err(|_| ())?;
        Ok(Self { data: bytes })
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, ()> {
        if out.len() < self.data.len() {
            return Err(());
        }
        out[..self.data.len()].copy_from_slice(&self.data);
        Ok(self.data.len())
    }

    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        Self::new(src)
    }

    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_RAW_CUSTOM);
        pkt.set_payload(&self.data)?;
        Ok(pkt)
    }
}
