//! MeshCore Routing Engine & Dispatcher
//!
//! Handles:
//! - Flood & direct packet routing decisions.
//! - Duplicate packet suppression via `SeenPacketFilter`.
//! - Repeater forwarding with path hash accumulation.
//! - High-level packet composition (Adverts, Group text, Direct messages, ACKs).

use crate::meshcore::identity::LocalIdentity;
use crate::meshcore::packet::{
    Packet, PAYLOAD_TYPE_ACK, PAYLOAD_TYPE_ADVERT, PAYLOAD_TYPE_GRP_TXT,
    PAYLOAD_TYPE_PATH, PAYLOAD_TYPE_TXT_MSG, ROUTE_TYPE_FLOOD,
};
use crate::meshcore::packets::{AckPacket, AdvertPacket, GrpTextPacket, PathPacket, TxtMsgPacket};
use crate::meshcore::queue::{PacketQueue, SeenPacketFilter};

pub const DEFAULT_MAX_HOPS: u8 = 7;

/// Discovered MeshCore Contact in range
#[derive(Clone, Debug)]
pub struct DiscoveredContact {
    pub pub_key: [u8; 32],
    pub callsign: heapless::String<32>,
    pub node_id: [u8; 4],
    pub last_seen_epoch: u32,
    pub rssi: i16,
    pub snr: i8,
    pub battery_percentage: u8,
    pub latitude: f32,
    pub longitude: f32,
}

/// Recently received message entry for message history / inbox
#[derive(Clone, Debug)]
pub struct ReceivedMessage {
    pub sender: heapless::String<32>,
    pub text: heapless::String<128>,
    pub timestamp: u32,
    pub rssi: i16,
    pub snr: i8,
    pub is_direct: bool,
}

/// Actions resulting from processing an inbound over-the-air packet
pub enum InboundAction {
    /// Dropped because it has already been processed recently
    DropDuplicate,
    /// Dropped because packet exceeded maximum hop limit
    DropHopLimitExceeded,
    /// Successfully decoded node advertisement
    ReceivedAdvert(AdvertPacket),
    /// Successfully decoded channel group broadcast
    ReceivedGroupText(GrpTextPacket),
    /// Successfully decoded direct message addressed to this node
    ReceivedDirectText(TxtMsgPacket),
    /// Received message delivery acknowledgment
    ReceivedAck(AckPacket),
    /// Received path discovery reply
    ReceivedPath(PathPacket),
    /// Unhandled or unknown packet payload
    Unhandled,
}

/// MeshCore Routing Engine
pub struct MeshEngine {
    pub identity: LocalIdentity,
    pub seen_filter: SeenPacketFilter,
    pub tx_queue: PacketQueue,
    pub is_repeater: bool,
    pub max_hops: u8,
}

impl MeshEngine {
    pub fn new(identity: LocalIdentity, is_repeater: bool) -> Self {
        Self {
            identity,
            seen_filter: SeenPacketFilter::new(),
            tx_queue: PacketQueue::new(),
            is_repeater,
            max_hops: DEFAULT_MAX_HOPS,
        }
    }

    /// Update the local identity callsign
    #[inline]
    pub fn set_callsign(&mut self, callsign: &str) {
        self.identity.set_callsign(callsign);
    }

    /// Update the local private key and derived public key
    #[inline]
    pub fn set_secret_key(&mut self, secret: &[u8; 32]) {
        self.identity.set_secret_key(secret);
    }

    /// Process a packet received over the LoRa radio interface
    pub fn handle_inbound_packet(&mut self, pkt: Packet) -> InboundAction {
        // 1. Check for duplicates; drop immediately if seen
        if self.seen_filter.check_and_mark(&pkt) {
            return InboundAction::DropDuplicate;
        }

        // 2. Check hop limits
        if pkt.path_len >= self.max_hops {
            return InboundAction::DropHopLimitExceeded;
        }

        // 3. If configured as a repeater and packet is flood-routed, re-broadcast
        if self.is_repeater && pkt.is_flood() {
            let mut repeat_pkt = pkt.clone();
            repeat_pkt.append_hop(self.identity.short_hash());
            let _ = self.tx_queue.push(repeat_pkt);
        }

        // 4. Dispatch based on payload type
        match pkt.payload_type() {
            PAYLOAD_TYPE_ADVERT => {
                if let Ok(adv) = AdvertPacket::decode(&pkt.payload[..pkt.payload_len]) {
                    InboundAction::ReceivedAdvert(adv)
                } else {
                    InboundAction::Unhandled
                }
            }
            PAYLOAD_TYPE_GRP_TXT => {
                if pkt.payload_len > 0 {
                    let ch_hash = pkt.payload[0];
                    let mut matched = None;
                    crate::companion::usb::CHANNELS.lock(|chs| {
                        let list = chs.borrow();
                        for ch in list.iter() {
                            if crate::meshcore::crypto::channel_hash(&ch.secret) == ch_hash {
                                if let Ok(grp) = GrpTextPacket::decode_encrypted(&pkt.payload[..pkt.payload_len], &ch.secret) {
                                    matched = Some(grp);
                                    break;
                                }
                            }
                        }
                    });
                    if let Some(grp) = matched {
                        InboundAction::ReceivedGroupText(grp)
                    } else {
                        InboundAction::Unhandled
                    }
                } else {
                    InboundAction::Unhandled
                }
            }
            PAYLOAD_TYPE_TXT_MSG => {
                if let Ok(msg) = TxtMsgPacket::decode(&pkt.payload[..pkt.payload_len]) {
                    // Check if addressed to us or broadcast
                    if self.identity.base.matches_hash(&msg.dest_hash) {
                        InboundAction::ReceivedDirectText(msg)
                    } else {
                        InboundAction::Unhandled
                    }
                } else {
                    InboundAction::Unhandled
                }
            }
            PAYLOAD_TYPE_ACK => {
                if let Ok(ack) = AckPacket::decode(&pkt.payload[..pkt.payload_len]) {
                    InboundAction::ReceivedAck(ack)
                } else {
                    InboundAction::Unhandled
                }
            }
            PAYLOAD_TYPE_PATH => {
                if let Ok(path) = PathPacket::decode(&pkt.payload[..pkt.payload_len]) {
                    InboundAction::ReceivedPath(path)
                } else {
                    InboundAction::Unhandled
                }
            }
            _ => InboundAction::Unhandled,
        }
    }

    /// Construct and queue an advertisement packet for transmission
    pub fn queue_advert(&mut self, timestamp: u32, lat: f32, lon: f32, bat: u8) -> Result<Packet, ()> {
        let mut adv = AdvertPacket::new(
            *self.identity.pub_key(),
            self.identity.callsign(),
            1, // ADV_TYPE_CHAT
            timestamp,
        );
        adv.latitude = lat;
        adv.longitude = lon;
        adv.battery_percentage = bat;

        let pkt = adv.to_packet(ROUTE_TYPE_FLOOD)?;
        let _ = self.tx_queue.push(pkt.clone());
        Ok(pkt)
    }

    /// Construct and queue an encrypted channel group text message
    pub fn queue_group_text(&mut self, channel_hash: u8, channel_secret: &[u8], message: &str, timestamp: u32) -> Result<Packet, ()> {
        let grp = GrpTextPacket::new(channel_hash, self.identity.callsign(), message, timestamp);
        let pkt = grp.to_packet(channel_secret, ROUTE_TYPE_FLOOD)?;
        let _ = self.tx_queue.push(pkt.clone());
        Ok(pkt)
    }

    /// Construct and queue a direct text message
    pub fn queue_direct_text(&mut self, dest_hash: [u8; 4], text: &str, timestamp: u32) -> Result<Packet, ()> {
        let src_hash = self.identity.hash_u32().to_le_bytes();
        let msg = TxtMsgPacket::new(dest_hash, src_hash, text, timestamp);
        let pkt = msg.to_packet(ROUTE_TYPE_FLOOD)?;
        let _ = self.tx_queue.push(pkt.clone());
        Ok(pkt)
    }

    /// Construct and queue an acknowledgment packet
    pub fn queue_ack(&mut self, dest_hash: [u8; 4], ack_packet_hash: [u8; 4], timestamp: u32) -> Result<Packet, ()> {
        let src_hash = self.identity.hash_u32().to_le_bytes();
        let ack = AckPacket::new(dest_hash, src_hash, ack_packet_hash, timestamp);
        let pkt = ack.to_packet(ROUTE_TYPE_FLOOD)?;
        let _ = self.tx_queue.push(pkt.clone());
        Ok(pkt)
    }
}
