//! MeshCore over-the-air wire protocol.
//!
//! This crate is the board- and HAL-independent part of the stack:
//! packet framing, cryptographic primitives, node identity, duplicate
//! detection and the concrete payload codecs. It has no dependency on
//! `embassy-nrf`, `esp-hal`, a radio driver or any other hardware crate,
//! so it builds and tests on the host.

#![cfg_attr(not(test), no_std)]

pub mod crypto;
pub mod identity;
pub mod packet;
pub mod packets;
pub mod queue;

pub use identity::{Identity, LocalIdentity};
pub use packet::{
    Packet, MAX_PACKET_PAYLOAD, MAX_PACKET_SIZE, MAX_PATH_SIZE, PAYLOAD_TYPE_ACK,
    PAYLOAD_TYPE_ADVERT, PAYLOAD_TYPE_GRP_TXT, PAYLOAD_TYPE_PATH, PAYLOAD_TYPE_TXT_MSG,
    ROUTE_TYPE_DIRECT, ROUTE_TYPE_FLOOD, ROUTE_TYPE_TRANSPORT_DIRECT, ROUTE_TYPE_TRANSPORT_FLOOD,
};
pub use packets::{
    AckPacket, AddressedPacket, AdvertPacket, AnonReqPacket, ControlPacket, GrpDataPacket,
    GrpTextPacket, MultipartPacket, PathPacket, RawPacket, TracePacket, TxtMsgPacket,
};
pub use queue::{PacketQueue, SeenPacketFilter};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packet_roundtrip() {
        let mut p = Packet::new(ROUTE_TYPE_FLOOD, PAYLOAD_TYPE_ADVERT);
        p.set_payload(b"hello").unwrap();

        let mut buf = [0u8; MAX_PACKET_SIZE];
        let n = p.write_to(&mut buf).unwrap();
        let q = Packet::read_from(&buf[..n]).unwrap();

        assert_eq!(p, q);
    }

    #[test]
    fn seal_open_roundtrip() {
        let secret = [7u8; 32];
        let plain = b"hello";

        let mut sealed = [0u8; 64];
        let sealed_len = crypto::meshcore_encrypt_then_mac(&secret, &mut sealed, plain).unwrap();

        let mut out = [0u8; 64];
        let out_len =
            crypto::meshcore_mac_then_decrypt(&secret, &mut out, &sealed[..sealed_len]).unwrap();

        assert_eq!(&out[..plain.len()], plain);
        assert!(out[plain.len()..out_len].iter().all(|&b| b == 0));
    }

    #[test]
    fn pop_front_hop_shifts_path() {
        let mut p = Packet::new(ROUTE_TYPE_DIRECT, PAYLOAD_TYPE_ADVERT);
        p.set_path_hash_size_and_count(2, 2);
        p.path[..4].copy_from_slice(&[1, 2, 3, 4]);

        assert!(p.pop_front_hop());
        assert_eq!(p.path_hash_size(), 2);
        assert_eq!(p.path_hash_count(), 1);
        assert_eq!(&p.path[..2], &[3, 4]);

        assert!(p.pop_front_hop());
        assert_eq!(p.path_hash_count(), 0);
        assert!(!p.pop_front_hop());
    }

    #[test]
    fn append_hop_stops_at_63() {
        let mut p = Packet::new(ROUTE_TYPE_FLOOD, PAYLOAD_TYPE_ADVERT);
        p.set_path_hash_size_and_count(1, 63);
        assert!(!p.append_hop(7));
        assert_eq!(p.path_hash_count(), 63);
    }

    #[test]
    fn channel_hash_is_stable() {
        let psk = [
            0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a, 0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15,
            0xcd, 0x72,
        ];
        assert_eq!(crypto::channel_hash(&psk), crate::crypto::sha256(&psk)[0]);
    }
}
