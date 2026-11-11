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
    pub latitude: i32,       // Changed from f32 to i32 (degrees * 1,000,000)
    pub longitude: i32,      // Changed from f32 to i32 (degrees * 1,000,000)
    pub flags: u8,           // New: advert flags bitmask
}

/// Recently received message entry for message history / inbox
#[derive(Clone, Debug)]
pub struct ReceivedMessage {
    pub channel_idx: u8,
    pub path_len: u8,
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
    ReceivedGroupText {
        channel_idx: u8,
        path_len: u8,
        packet: GrpTextPacket,
    },
    /// Received repeater echo of our own transmitted group message
    ReceivedRepeaterEcho {
        pkt_hash: [u8; 4],
        hops: u8,
    },
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
    pub path_hash_mode: u8,
}

impl MeshEngine {
    pub fn new(identity: LocalIdentity, is_repeater: bool) -> Self {
        Self {
            identity,
            seen_filter: SeenPacketFilter::new(),
            tx_queue: PacketQueue::new(),
            is_repeater,
            max_hops: DEFAULT_MAX_HOPS,
            path_hash_mode: crate::storage::get_path_hash_mode(),
        }
    }

    /// Update the local identity callsign
    #[inline]
    pub fn set_callsign(&mut self, callsign: &str) {
        self.identity.set_callsign(callsign);
    }

    /// Set path hash mode (0: 1-byte, 1: 2-byte, 2: 3-byte)
    #[inline]
    pub fn set_path_hash_mode(&mut self, mode: u8) {
        if mode <= 2 {
            self.path_hash_mode = mode;
        }
    }

    /// Update the local private key and derived public key
    #[inline]
    pub fn set_secret_key(&mut self, secret: &[u8; 32]) {
        self.identity.set_secret_key(secret);
    }

    /// Process a packet received over the LoRa radio interface
    pub fn handle_inbound_packet(&mut self, pkt: Packet) -> InboundAction {
        // 1. Check hop limits
        if pkt.path_hash_count() >= self.max_hops as usize {
            return InboundAction::DropHopLimitExceeded;
        }

        let is_dupe = self.seen_filter.check_and_mark(&pkt);
        let already_in_path = pkt.contains_hop(self.identity.pub_key());

        // 2. Dispatch based on payload type
        match pkt.payload_type() {
            PAYLOAD_TYPE_ADVERT => {
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                if let Ok(adv) = AdvertPacket::decode(&pkt.payload[..pkt.payload_len]) {
                    // Verify Ed25519 signature before trusting this advert
                    if !adv.verify_signature() {
                        return InboundAction::DropDuplicate; // or a new DropInvalidSignature variant
                    }
                    // Only repeat adverts from other nodes
                    if self.is_repeater && pkt.is_flood() && !already_in_path && adv.pub_key != *self.identity.pub_key() {
                        let mut repeat_pkt = pkt.clone();
                        repeat_pkt.append_hop_bytes(self.identity.pub_key());
                        let _ = self.tx_queue.push(repeat_pkt);
                    }
                    InboundAction::ReceivedAdvert(adv)
                } else {
                    InboundAction::Unhandled
                }
            }
            PAYLOAD_TYPE_GRP_TXT => {
                // Look up matching channel by channel_hash (pkt.payload[0])
                if pkt.payload_len > 0 {
                    let ch_hash = pkt.payload[0];
                    // Copy the channel array out of the lock before doing any crypto.
                    // CHANNELS.lock() is a CriticalSectionRawMutex — it disables all interrupts
                    // for the duration of the callback. Running SHA-256 hashing and AES-128
                    // decryption with interrupts disabled will cause the Nordic BLE SoftDevice
                    // (S140) to fault and freeze the device solid.
                    //
                    // The array ([ChannelData; 8] = 320 bytes) is copied by value here,
                    // which is cheap compared to the crypto operations below.
                    let channels = crate::companion::usb::CHANNELS.lock(|chs| *chs.borrow());
                    let mut matched = None;
                    for (idx, ch) in channels.iter().enumerate() {
                        if crate::meshcore::crypto::channel_hash(&ch.secret) == ch_hash {
                            if let Ok(grp) = GrpTextPacket::decode_encrypted(&pkt.payload[..pkt.payload_len], &ch.secret) {
                                matched = Some((idx as u8, grp));
                                break;
                            }
                        }
                    }

                    if let Some((ch_idx, grp)) = matched {
                        if grp.sender.as_str() == self.identity.callsign() {
                            // Our own message echoed by repeater! NEVER re-broadcast our own message
                            if pkt.path_hash_count() >= 1 {
                                InboundAction::ReceivedRepeaterEcho {
                                    pkt_hash: pkt.calculate_packet_hash(),
                                    hops: pkt.path_hash_count() as u8,
                                }
                            } else {
                                InboundAction::DropDuplicate
                            }
                        } else {
                            // Message from other node: repeat if configured as repeater
                            if self.is_repeater && pkt.is_flood() && !is_dupe && !already_in_path {
                                let mut repeat_pkt = pkt.clone();
                                repeat_pkt.append_hop_bytes(self.identity.pub_key());
                                let _ = self.tx_queue.push(repeat_pkt);
                            }

                            if is_dupe {
                                InboundAction::DropDuplicate
                            } else {
                                InboundAction::ReceivedGroupText {
                                    channel_idx: ch_idx,
                                    path_len: pkt.path_len,
                                    packet: grp,
                                }
                            }
                        }
                    } else if is_dupe {
                        InboundAction::DropDuplicate
                    } else {
                        InboundAction::Unhandled
                    }
                } else {
                    InboundAction::Unhandled
                }
            }
            PAYLOAD_TYPE_TXT_MSG => {
                if self.is_repeater && pkt.is_flood() && !is_dupe && !already_in_path {
                    let mut repeat_pkt = pkt.clone();
                    repeat_pkt.append_hop_bytes(self.identity.pub_key());
                    let _ = self.tx_queue.push(repeat_pkt);
                }
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                if let Ok(msg) = TxtMsgPacket::decode(&pkt.payload[..pkt.payload_len]) {
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
    /// Construct, sign, and queue an advertisement packet for transmission
    pub fn queue_advert(&mut self, timestamp: u32, lat_f32: f32, lon_f32: f32) -> Result<Packet, ()> {
        // Convert f32 GPS to integer (degrees * 1,000,000)
        let lat_int = (lat_f32 * 1_000_000.0) as i32;
        let lon_int = (lon_f32 * 1_000_000.0) as i32;

        // Build flags: this node is a chat node + has location + has name
        let mut flags = AdvertPacket::ADV_FLAG_IS_CHAT;
        if self.is_repeater {
            flags |= AdvertPacket::ADV_FLAG_IS_REPEATER;
        }
        if lat_f32 != 0.0 || lon_f32 != 0.0 {
            flags |= AdvertPacket::ADV_FLAG_HAS_LOCATION;
        }
        flags |= AdvertPacket::ADV_FLAG_HAS_NAME;

        let mut adv = AdvertPacket::new(
            *self.identity.pub_key(),
            timestamp,
            flags,
            lat_int,
            lon_int,
            self.identity.callsign(),
        );

        // Sign the advert with our private key
        adv.sign(&self.identity);

        let mut pkt = adv.to_packet(ROUTE_TYPE_FLOOD)?;
        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        self.seen_filter.mark_seen(pkt.calculate_packet_hash());
        let _ = self.tx_queue.push(pkt.clone());
        Ok(pkt)
    }

    /// Construct and queue an encrypted channel group text message
    pub fn queue_group_text(&mut self, channel_hash: u8, channel_secret: &[u8], message: &str, timestamp: u32) -> Result<Packet, ()> {
        let grp = GrpTextPacket::new(channel_hash, self.identity.callsign(), message, timestamp, GrpTextPacket::TXT_TYPE_PLAIN, 0);
        let mut pkt = grp.to_packet(channel_secret, ROUTE_TYPE_FLOOD)?;
        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        self.seen_filter.mark_seen(pkt.calculate_packet_hash());
        let _ = self.tx_queue.push(pkt.clone());
        Ok(pkt)
    }

    /// Construct and queue a direct text message
    pub fn queue_direct_text(&mut self, dest_hash: [u8; 4], text: &str, timestamp: u32) -> Result<Packet, ()> {
        let src_hash = self.identity.hash_u32().to_le_bytes();
        let msg = TxtMsgPacket::new(dest_hash, src_hash, text, timestamp);
        let mut pkt = msg.to_packet(ROUTE_TYPE_FLOOD)?;
        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        self.seen_filter.mark_seen(pkt.calculate_packet_hash());
        let _ = self.tx_queue.push(pkt.clone());
        Ok(pkt)
    }

    /// Construct and queue an acknowledgment packet
    pub fn queue_ack(&mut self, dest_hash: [u8; 4], ack_packet_hash: [u8; 4], timestamp: u32) -> Result<Packet, ()> {
        let src_hash = self.identity.hash_u32().to_le_bytes();
        let ack = AckPacket::new(dest_hash, src_hash, ack_packet_hash, timestamp);
        let mut pkt = ack.to_packet(ROUTE_TYPE_FLOOD)?;
        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        self.seen_filter.mark_seen(pkt.calculate_packet_hash());
        let _ = self.tx_queue.push(pkt.clone());
        Ok(pkt)
    }
}
