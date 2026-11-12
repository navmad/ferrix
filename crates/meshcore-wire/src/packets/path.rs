//! Route Discovery / Path Return (`PAYLOAD_TYPE_PATH = 0x08`).
//!
//! Wire payload: `dest(1) ‖ src(1) ‖ seal(plain)`.
//! Plaintext: `path_len ‖ path ‖ extra_type ‖ extra`; without extra,
//! `0xFF ‖ random(4)`.

use heapless::Vec;

use crate::crypto::{meshcore_encrypt_then_mac, meshcore_mac_then_decrypt};
use crate::packet::{Packet, MAX_PATH_SIZE, PAYLOAD_TYPE_PATH};

pub const MAX_EXTRA: usize = 128;

#[derive(Clone, Debug, PartialEq)]
pub struct PathPacket {
    pub dest: u8,
    pub src: u8,
    /// Encoded path byte: hash size in bits 7-6, hop count in bits 5-0.
    pub path_len: u8,
    pub path: [u8; MAX_PATH_SIZE],
    /// Raw type byte after the path (0xFF for the no-extra form).
    pub extra_type: u8,
    pub extra: Vec<u8, MAX_EXTRA>,
}

impl PathPacket {
    pub fn new(
        dest: u8,
        src: u8,
        path_len: u8,
        path: &[u8],
        extra_type: u8,
        extra: &[u8],
    ) -> Result<Self, ()> {
        if !Packet::is_valid_path_len(path_len) {
            return Err(());
        }
        let expected = path_byte_len(path_len);
        if path.len() != expected {
            return Err(());
        }
        let mut p = [0u8; MAX_PATH_SIZE];
        p[..expected].copy_from_slice(path);
        let mut e = Vec::new();
        e.extend_from_slice(extra).map_err(|_| ())?;
        Ok(Self {
            dest,
            src,
            path_len,
            path: p,
            extra_type,
            extra: e,
        })
    }

    pub fn hop_count(&self) -> u8 {
        self.path_len & 63
    }

    pub fn hash_size(&self) -> u8 {
        (self.path_len >> 6) + 1
    }

    pub fn path_bytes(&self) -> &[u8] {
        &self.path[..path_byte_len(self.path_len)]
    }

    /// Encode the addressed, sealed payload. `random4` is used only for the
    /// no-extra form.
    pub fn encode(
        &self,
        secret: &[u8; 32],
        random4: &[u8; 4],
        out: &mut [u8],
    ) -> Result<usize, ()> {
        if out.len() < 2 {
            return Err(());
        }
        let path = self.path_bytes();
        let mut plain = [0u8; 184];
        let mut n = 0;

        plain[n] = self.path_len;
        n += 1;
        plain[n..n + path.len()].copy_from_slice(path);
        n += path.len();

        if self.extra_type == 0xFF || self.extra.is_empty() {
            plain[n] = 0xFF;
            n += 1;
            plain[n..n + 4].copy_from_slice(random4);
            n += 4;
        } else {
            plain[n] = self.extra_type;
            n += 1;
            plain[n..n + self.extra.len()].copy_from_slice(&self.extra);
            n += self.extra.len();
        }

        out[0] = self.dest;
        out[1] = self.src;
        let sealed = meshcore_encrypt_then_mac(secret, &mut out[2..], &plain[..n])?;
        Ok(2 + sealed)
    }

    pub fn decode(src: &[u8], secret: &[u8; 32]) -> Result<Self, ()> {
        // The C++ requires more than 4 bytes before attempting decrypt.
        if src.len() <= 4 {
            return Err(());
        }
        let mut plain = [0u8; 184];
        let plen = meshcore_mac_then_decrypt(secret, &mut plain, &src[2..])?;
        if plen < 2 {
            return Err(());
        }

        let path_len = plain[0];
        if !Packet::is_valid_path_len(path_len) {
            return Err(());
        }
        let expected = path_byte_len(path_len);
        if 1 + expected + 1 > plen {
            return Err(());
        }

        let mut path = [0u8; MAX_PATH_SIZE];
        path[..expected].copy_from_slice(&plain[1..1 + expected]);
        let extra_type = plain[1 + expected];
        let extra_start = 2 + expected;
        let extra_len = plen.saturating_sub(extra_start);
        let mut extra = Vec::new();
        let _ = extra.extend_from_slice(&plain[extra_start..extra_start + extra_len.min(MAX_EXTRA)]);

        Ok(Self {
            dest: src[0],
            src: src[1],
            path_len,
            path,
            extra_type,
            extra,
        })
    }

    pub fn to_packet(
        &self,
        route_type: u8,
        secret: &[u8; 32],
        random4: &[u8; 4],
    ) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_PATH);
        let mut buf = [0u8; 184];
        let len = self.encode(secret, random4, &mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}

fn path_byte_len(path_len: u8) -> usize {
    let count = (path_len & 63) as usize;
    let size = ((path_len >> 6) as usize) + 1;
    count * size
}
