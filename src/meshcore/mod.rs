//! MeshCore Network Protocol Stack
//!
//! Provides the complete over-the-air (OTA) wire protocol for LoRa mesh networking:
//! - Packet serialization and bitfield header layout (`0bVVPPPPRR`).
//! - Cryptographic identity & key management (`Identity`, `LocalIdentity`).
//! - Cryptographic hashing (SHA-256) and block cipher (AES-128).
//! - Duplicate packet suppression (`SeenPacketFilter`) and transmission queues (`PacketQueue`).
//! - Mesh routing engine and repeater forwarding (`MeshEngine`).
//! - Concrete payload codecs: `AdvertPacket`, `GrpTextPacket`, `TxtMsgPacket`, `AckPacket`, `PathPacket`.

pub mod crypto;
pub mod identity;
pub mod mesh;
pub mod packet;
pub mod packets;
pub mod queue;

pub use identity::{Identity, LocalIdentity};
pub use mesh::{InboundAction, MeshEngine};
pub use packet::{
    Packet, MAX_PACKET_PAYLOAD, MAX_PACKET_SIZE, MAX_PATH_SIZE,
    PAYLOAD_TYPE_ACK, PAYLOAD_TYPE_ADVERT, PAYLOAD_TYPE_GRP_TXT,
    PAYLOAD_TYPE_PATH, PAYLOAD_TYPE_TXT_MSG, ROUTE_TYPE_DIRECT,
    ROUTE_TYPE_FLOOD, ROUTE_TYPE_TRANSPORT_DIRECT, ROUTE_TYPE_TRANSPORT_FLOOD,
};
pub use packets::{AckPacket, AdvertPacket, GrpTextPacket, PathPacket, TxtMsgPacket};
pub use queue::{PacketQueue, SeenPacketFilter};

