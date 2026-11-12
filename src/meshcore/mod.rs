//! MeshCore Network Protocol Stack
//!
//! The board- and HAL-independent wire protocol (packet framing, crypto,
//! identity, duplicate detection and payload codecs) lives in the
//! `meshcore-wire` crate and is re-exported here. The firmware-side routing
//! engine (`MeshEngine`) stays in this crate because it owns device state.

pub use meshcore_wire::{crypto, identity, packet, packets, queue};

pub use meshcore_wire::{
    AckPacket, AdvertPacket, GrpTextPacket, Identity, LocalIdentity, Packet, PacketQueue,
    PathPacket, SeenPacketFilter, TxtMsgPacket, MAX_PACKET_PAYLOAD, MAX_PACKET_SIZE, MAX_PATH_SIZE,
    PAYLOAD_TYPE_ACK, PAYLOAD_TYPE_ADVERT, PAYLOAD_TYPE_GRP_TXT, PAYLOAD_TYPE_PATH,
    PAYLOAD_TYPE_TXT_MSG, ROUTE_TYPE_DIRECT, ROUTE_TYPE_FLOOD, ROUTE_TYPE_TRANSPORT_DIRECT,
    ROUTE_TYPE_TRANSPORT_FLOOD,
};

pub mod mesh;

pub use mesh::{InboundAction, MeshEngine};
