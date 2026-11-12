//! Group Datagram (`PAYLOAD_TYPE_GRP_DATA = 0x06`).
//!
//! Wire payload: `channel_hash(1) ‖ seal(data_type u16 ‖ length u8 ‖ data)`.

use heapless::Vec;

use crate::crypto::{meshcore_encrypt_then_mac, meshcore_mac_then_decrypt};
use crate::packet::{Packet, PAYLOAD_TYPE_GRP_DATA};

pub const MAX_GROUP_DATA: usize = 165;

#[derive(Clone, Debug, PartialEq)]
pub struct GrpDataPacket {
    pub channel_hash: u8,
    pub data_type: u16,
    pub data: Vec<u8, MAX_GROUP_DATA>,
}

impl GrpDataPacket {
    pub fn new(channel_hash: u8, data_type: u16, data: &[u8]) -> Result<Self, ()> {
        if data.len() > MAX_GROUP_DATA {
            return Err(());
        }
        let mut bytes = Vec::new();
        bytes.extend_from_slice(data).map_err(|_| ())?;
        Ok(Self {
            channel_hash,
            data_type,
            data: bytes,
        })
    }

    pub fn encode_encrypted(&self, secret: &[u8], out: &mut [u8]) -> Result<usize, ()> {
        if out.len() < 1 + 2 + 16 {
            return Err(());
        }
        let mut plain = [0u8; 3 + MAX_GROUP_DATA];
        plain[0..2].copy_from_slice(&self.data_type.to_le_bytes());
        plain[2] = self.data.len() as u8;
        plain[3..3 + self.data.len()].copy_from_slice(&self.data);

        out[0] = self.channel_hash;
        let sealed =
            meshcore_encrypt_then_mac(secret, &mut out[1..], &plain[..3 + self.data.len()])?;
        Ok(1 + sealed)
    }

    pub fn decode_encrypted(src: &[u8], secret: &[u8]) -> Result<Self, ()> {
        // The C++ requires more than 3 bytes before attempting decrypt.
        if src.len() <= 3 {
            return Err(());
        }
        let channel_hash = src[0];
        let mut plain = [0u8; 3 + MAX_GROUP_DATA];
        let plen = meshcore_mac_then_decrypt(secret, &mut plain, &src[1..])?;
        if plen < 3 {
            return Err(());
        }

        let data_type = u16::from_le_bytes([plain[0], plain[1]]);
        let len = plain[2] as usize;
        if 3 + len > plen {
            return Err(());
        }
        let mut data = Vec::new();
        data.extend_from_slice(&plain[3..3 + len]).map_err(|_| ())?;

        Ok(Self {
            channel_hash,
            data_type,
            data,
        })
    }

    pub fn to_packet(&self, secret: &[u8], route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_GRP_DATA);
        let mut buf = [0u8; 184];
        let len = self.encode_encrypted(secret, &mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
