//! Generic addressed, encrypted datagram used by REQ and RESPONSE.
//!
//! Wire payload: `dest(1) ‖ src(1) ‖ seal(plain)`.

use heapless::Vec;

use crate::crypto::{meshcore_encrypt_then_mac, meshcore_mac_then_decrypt};
use crate::packet::Packet;

pub const MAX_ADDRESSED_PLAIN: usize = 167;

#[derive(Clone, Debug, PartialEq)]
pub struct AddressedPacket {
    pub dest: u8,
    pub src: u8,
    pub plain: Vec<u8, MAX_ADDRESSED_PLAIN>,
}

impl AddressedPacket {
    pub fn new(dest: u8, src: u8, plain: &[u8]) -> Result<Self, ()> {
        if plain.len() > MAX_ADDRESSED_PLAIN {
            return Err(());
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(plain).map_err(|_| ())?;
        Ok(Self {
            dest,
            src,
            plain: bytes,
        })
    }

    pub fn encode(&self, secret: &[u8; 32], out: &mut [u8]) -> Result<usize, ()> {
        if out.len() < 2 {
            return Err(());
        }
        out[0] = self.dest;
        out[1] = self.src;
        let sealed = meshcore_encrypt_then_mac(secret, &mut out[2..], &self.plain)?;
        Ok(2 + sealed)
    }

    pub fn decode(src: &[u8], secret: &[u8; 32]) -> Result<Self, ()> {
        if src.len() <= 4 {
            return Err(());
        }
        let mut plain = [0u8; MAX_ADDRESSED_PLAIN];
        let plen = meshcore_mac_then_decrypt(secret, &mut plain, &src[2..])?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&plain[..plen]).map_err(|_| ())?;
        Ok(Self {
            dest: src[0],
            src: src[1],
            plain: bytes,
        })
    }

    pub fn to_packet(
        &self,
        ptype: u8,
        route_type: u8,
        secret: &[u8; 32],
    ) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, ptype);
        let mut buf = [0u8; 184];
        let len = self.encode(secret, &mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
