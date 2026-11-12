//! CONTROL (`PAYLOAD_TYPE_CONTROL = 0x0B`).
//!
//! Payload: `flags(1) ‖ data…`. The subtype is the high nibble. `flags & 0x80`
//! means zero-hop direct only.

use heapless::Vec;

use crate::packet::{Packet, PAYLOAD_TYPE_CONTROL};

pub const MAX_CONTROL_DATA: usize = 183;

#[derive(Clone, Debug, PartialEq)]
pub struct ControlPacket {
    pub flags: u8,
    pub data: Vec<u8, MAX_CONTROL_DATA>,
}

impl ControlPacket {
    pub fn new(flags: u8, data: &[u8]) -> Result<Self, ()> {
        if data.len() > MAX_CONTROL_DATA {
            return Err(());
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(data).map_err(|_| ())?;
        Ok(Self { flags, data: bytes })
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, ()> {
        let needed = 1 + self.data.len();
        if out.len() < needed {
            return Err(());
        }
        out[0] = self.flags;
        out[1..needed].copy_from_slice(&self.data);
        Ok(needed)
    }

    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.is_empty() {
            return Err(());
        }
        let mut data = Vec::new();
        data.extend_from_slice(&src[1..]).map_err(|_| ())?;
        Ok(Self {
            flags: src[0],
            data,
        })
    }

    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_CONTROL);
        let mut buf = [0u8; 184];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
