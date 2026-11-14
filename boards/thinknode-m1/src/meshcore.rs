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

pub use ferrix_core::engine::{InboundAction, MeshEngine};

// Board implementation of the routing engine's host interface.
//
// The engine lives in `ferrix-core` and owns no globals. This module binds
// it to the board's companion state, flood-scope settings and TX scheduler.

use ferrix_core::engine::EngineContext;
use ferrix_core::types::{ChannelData, MAX_CHANNELS};

pub struct BoardEngineContext;

pub static BOARD_ENGINE_CONTEXT: BoardEngineContext = BoardEngineContext;

impl EngineContext for BoardEngineContext {
    fn path_hash_mode(&self) -> u8 {
        crate::storage::get_path_hash_mode()
    }

    fn peers_by_hash(&self, hash: u8) -> heapless::Vec<[u8; 32], 32> {
        let mut peers = heapless::Vec::new();
        crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
            for ct in c.borrow().iter() {
                if ct.pub_key[0] == hash {
                    let _ = peers.push(ct.pub_key);
                }
            }
        });
        peers
    }

    fn channels(&self) -> [ChannelData; MAX_CHANNELS] {
        crate::companion::usb::CHANNELS.lock(|chs| *chs.borrow())
    }

    fn peer_path(&self, pub_key: &[u8; 32]) -> (u8, [u8; 64]) {
        crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
            if let Some(ct) = c.borrow().iter().find(|ct| ct.pub_key == *pub_key) {
                (ct.out_path_len, ct.out_path)
            } else {
                (0xFF, [0u8; 64])
            }
        })
    }

    fn apply_flood_scope(&self, pkt: &mut Packet) {
        ferrix_core::state::apply_flood_scope(pkt);
    }

    fn apply_flood_scope_default(&self, pkt: &mut Packet) {
        ferrix_core::state::apply_flood_scope_default(pkt);
    }

    fn note_forward_scheduled(&self) {
        crate::radio::FORWARD_SCHEDULED.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }

    fn note_duplicate_delta(&self, ms: u64) {
        crate::radio::record_duplicate_delta(ms);
    }

    fn cancel_flood(&self, hash: [u8; 8]) {
        crate::radio::lora_phy::cancel_flood(hash);
    }
}
