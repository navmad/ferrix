//! Anonymous Request (`PAYLOAD_TYPE_ANON_REQ = 0x07`).
//!
//! Wire payload: `dest(1) ‖ sender_pub(32) ‖ seal(plain)`. The shared secret is
//! ECDH between the sender's static identity and the recipient.

use heapless::Vec;

use crate::crypto::{meshcore_encrypt_then_mac, meshcore_mac_then_decrypt};
use crate::packet::{Packet, PAYLOAD_TYPE_ANON_REQ};

pub const MAX_ANON_PLAIN: usize = 136;

#[derive(Clone, Debug, PartialEq)]
pub struct AnonReqPacket {
    pub dest: u8,
    pub sender_pub: [u8; 32],
    pub plain: Vec<u8, MAX_ANON_PLAIN>,
}

impl AnonReqPacket {
    pub fn new(dest: u8, sender_pub: [u8; 32], plain: &[u8]) -> Result<Self, ()> {
        if plain.len() > MAX_ANON_PLAIN {
            return Err(());
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(plain).map_err(|_| ())?;
        Ok(Self {
            dest,
            sender_pub,
            plain: bytes,
        })
    }

    pub fn encode(&self, secret: &[u8; 32], out: &mut [u8]) -> Result<usize, ()> {
        if out.len() < 1 + 32 + 2 + 16 {
            return Err(());
        }
        out[0] = self.dest;
        out[1..33].copy_from_slice(&self.sender_pub);
        let sealed = meshcore_encrypt_then_mac(secret, &mut out[33..], &self.plain)?;
        Ok(33 + sealed)
    }

    pub fn decode(src: &[u8], secret: &[u8; 32]) -> Result<Self, ()> {
        // The C++ requires more than 35 bytes.
        if src.len() <= 35 {
            return Err(());
        }
        let dest = src[0];
        let mut sender_pub = [0u8; 32];
        sender_pub.copy_from_slice(&src[1..33]);

        let mut plain = [0u8; MAX_ANON_PLAIN];
        let plen = meshcore_mac_then_decrypt(secret, &mut plain, &src[33..])?;
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&plain[..plen]).map_err(|_| ())?;

        Ok(Self {
            dest,
            sender_pub,
            plain: bytes,
        })
    }

    pub fn to_packet(&self, secret: &[u8; 32], route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_ANON_REQ);
        let mut buf = [0u8; 184];
        let len = self.encode(secret, &mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
