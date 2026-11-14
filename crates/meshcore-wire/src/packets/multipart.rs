//! MULTIPART (`PAYLOAD_TYPE_MULTIPART = 0x0A`).
//!
//! Payload: `(remaining << 4 | inner_type)(1) ‖ body…`. The only inner type in
//! v1.18 is ACK.

use heapless::Vec;

use crate::packet::{Packet, PAYLOAD_TYPE_MULTIPART};

pub const MAX_MULTIPART_BODY: usize = 183;

#[derive(Clone, Debug, PartialEq)]
pub struct MultipartPacket {
    pub remaining: u8,
    pub inner_type: u8,
    pub body: Vec<u8, MAX_MULTIPART_BODY>,
}

impl MultipartPacket {
    pub fn new(remaining: u8, inner_type: u8, body: &[u8]) -> Result<Self, ()> {
        if body.len() > MAX_MULTIPART_BODY || remaining > 15 {
            return Err(());
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(body).map_err(|_| ())?;
        Ok(Self {
            remaining,
            inner_type: inner_type & 0x0F,
            body: bytes,
        })
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, ()> {
        let needed = 1 + self.body.len();
        if out.len() < needed {
            return Err(());
        }
        out[0] = (self.remaining << 4) | (self.inner_type & 0x0F);
        out[1..needed].copy_from_slice(&self.body);
        Ok(needed)
    }

    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.len() < 1 {
            return Err(());
        }
        let mut body = Vec::new();
        body.extend_from_slice(&src[1..]).map_err(|_| ())?;
        Ok(Self {
            remaining: src[0] >> 4,
            inner_type: src[0] & 0x0F,
            body,
        })
    }

    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_MULTIPART);
        let mut buf = [0u8; 184];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let body = [0x11, 0x22, 0x33, 0x44, 0x55];
        let original = MultipartPacket::new(2, crate::packet::PAYLOAD_TYPE_ACK, &body).unwrap();
        let mut buf = [0u8; 184];
        let len = original.encode(&mut buf).unwrap();
        let decoded = MultipartPacket::decode(&buf[..len]).unwrap();
        assert_eq!(decoded, original);
        assert_eq!(decoded.remaining, 2);
        assert_eq!(decoded.inner_type, crate::packet::PAYLOAD_TYPE_ACK);
    }
}
