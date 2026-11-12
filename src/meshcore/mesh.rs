//! MeshCore Routing Engine & Dispatcher
//!
//! Handles:
//! - Flood & direct packet routing decisions.
//! - Duplicate packet suppression via `SeenPacketFilter`.
//! - Repeater forwarding with path hash accumulation.
//! - High-level packet composition (Adverts, Group text, Direct messages, ACKs).

use crate::meshcore::identity::LocalIdentity;
use crate::meshcore::packet::{
    Packet, PAYLOAD_TYPE_ACK, PAYLOAD_TYPE_ADVERT, PAYLOAD_TYPE_ANON_REQ, PAYLOAD_TYPE_GRP_DATA,
    PAYLOAD_TYPE_GRP_TXT, PAYLOAD_TYPE_PATH, PAYLOAD_TYPE_TXT_MSG, ROUTE_TYPE_DIRECT,
    ROUTE_TYPE_FLOOD,
};
use crate::meshcore::packets::advert::{ADV_TYPE_CHAT, ADV_TYPE_REPEATER};
use crate::meshcore::packets::{
    AckPacket, AdvertPacket, AnonReqPacket, GrpDataPacket, GrpTextPacket, PathPacket, TxtMsgPacket,
};
use crate::meshcore::queue::{PacketQueue, SeenPacketFilter};

pub const DEFAULT_MAX_HOPS: u8 = 64;

/// Direct when the contact has a valid encoded path; flood otherwise.
fn route_for(out_path_len: u8) -> (u8, u8) {
    if out_path_len != 0xFF && Packet::is_valid_path_len(out_path_len) {
        (ROUTE_TYPE_DIRECT, out_path_len)
    } else {
        (ROUTE_TYPE_FLOOD, 0)
    }
}

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
    pub flags: u8,           // raw advert app_data[0] (kind nibble + masks)
    /// Learned return path (encoded path byte in `out_path_len`).
    pub out_path: [u8; 64],
    /// `0xFF` means unknown; otherwise a validated encoded path byte.
    pub out_path_len: u8,
    /// Timestamp from the last accepted advert (anti-replay).
    pub last_advert_ts: u32,
}

/// Recently received message entry for message history / inbox
#[derive(Clone, Debug)]
pub struct ReceivedMessage {
    pub channel_idx: u8,
    pub path_len: u8,
    pub sender: heapless::String<32>,
    /// Full sender public key for direct messages; zero for channel messages.
    pub sender_pub: [u8; 32],
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
        pkt_hash: [u8; 8],
        hops: u8,
    },
    /// Successfully decoded direct message addressed to this node
    ReceivedDirectText {
        msg: TxtMsgPacket,
        sender_pub: [u8; 32],
    },
    /// Received message delivery acknowledgment
    ReceivedAck(AckPacket),
    /// Received path discovery reply
    ReceivedPath {
        packet: PathPacket,
        sender_pub: [u8; 32],
    },
    /// Received channel datagram
    ReceivedGroupData {
        channel_idx: u8,
        packet: GrpDataPacket,
    },
    /// Received anonymous request
    ReceivedAnonReq(AnonReqPacket),
    /// Retransmitted a direct packet addressed to the next hop
    Forwarded,
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

    /// Update the local private key and derived public key.
    /// Returns false if the key derives a reserved public key.
    #[inline]
    pub fn set_private_key(&mut self, prv: &[u8; 64]) -> bool {
        self.identity.set_private_key(prv)
    }

    /// Update from a 32-byte seed (kept for convenience).
    #[inline]
    pub fn set_secret_key(&mut self, seed: &[u8; 32]) {
        self.identity.set_secret_key(seed);
    }

    /// Contact public keys whose first byte matches `hash`.
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

    /// Process a packet received over the LoRa radio interface
    pub fn handle_inbound_packet(&mut self, pkt: Packet) -> InboundAction {
        // 1. Check hop limits
        if pkt.path_hash_count() >= self.max_hops as usize {
            return InboundAction::DropHopLimitExceeded;
        }

        let is_dupe = self.seen_filter.check_and_mark(&pkt);
        let already_in_path = pkt.contains_hop(self.identity.pub_key());

        // 2. Direct forwarding: if the first hop is us, pop it and requeue.
        if pkt.route_type() == crate::meshcore::packet::ROUTE_TYPE_DIRECT {
            if pkt.path_hash_count() > 0 {
                let size = pkt.path_hash_size();
                if pkt.path[..size] == self.identity.pub_key()[..size] {
                    if self.is_repeater {
                        let mut fwd = pkt.clone();
                        if fwd.pop_front_hop() {
                            let ack = if fwd.payload_type() == PAYLOAD_TYPE_ACK {
                                AckPacket::decode(&fwd.payload[..fwd.payload_len]).ok()
                            } else {
                                None
                            };
                            let _ = self.tx_queue.push(fwd);
                            if let Some(ack) = ack {
                                return InboundAction::ReceivedAck(ack);
                            }
                            return InboundAction::Forwarded;
                        }
                    }
                }
                // A direct packet with a non-empty path is not for us unless we
                // are its next hop.
                return InboundAction::Unhandled;
            }
        }

        // 3. Dispatch based on payload type
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
                    // Forward flood packets BEFORE trying to decrypt, matching the reference firmware behavior.
                    // This ensures repeater nodes propagate group texts even when they can't
                    // decrypt them (e.g., unknown channel hash). The packet continues through
                    // the mesh regardless of whether this node can read its contents.
                    if self.is_repeater && pkt.is_flood() && !is_dupe && !already_in_path {
                        let mut repeat_pkt = pkt.clone();
                        repeat_pkt.append_hop_bytes(self.identity.pub_key());
                        let _ = self.tx_queue.push(repeat_pkt);
                    }

                    // Copy the channel array out of the lock before doing any crypto.
                    // Running SHA-256 hashing and AES-128 decryption inside a lock callback
                    // can block the BLE SoftDevice (S140) and freeze the device.
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
                let payload = &pkt.payload[..pkt.payload_len];
                // Addressed payload: dest(1) ‖ src(1) ‖ sealed.
                if payload.len() <= 4 {
                    return InboundAction::Unhandled;
                }
                let dest = payload[0];
                let src = payload[1];

                // Only forward flood packets that are not addressed to us.
                if dest != self.identity.pub_key()[0]
                    && self.is_repeater
                    && pkt.is_flood()
                    && !is_dupe
                    && !already_in_path
                {
                    let mut repeat_pkt = pkt.clone();
                    repeat_pkt.append_hop_bytes(self.identity.pub_key());
                    let _ = self.tx_queue.push(repeat_pkt);
                }
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                if dest != self.identity.pub_key()[0] {
                    return InboundAction::Unhandled;
                }

                // Collect candidate peer keys outside the lock, then try each
                // shared secret until one authenticates the MAC.
                let peers = self.peers_by_hash(src);
                for peer in peers.iter() {
                    let secret = match self.identity.shared_secret(peer) {
                        Some(secret) => secret,
                        None => continue,
                    };
                    if let Ok(msg) = TxtMsgPacket::decode(payload, &secret, peer) {
                        if msg.txt_type == crate::meshcore::packets::txtmsg::TXT_TYPE_PLAIN {
                            let ack = AckPacket::with_extended(
                                msg.ack_crc,
                                msg.extended_attempt,
                                0,
                            );
                            if pkt.is_flood() {
                                // Flooded DM: return the arrival path with the ACK
                                // as extra, so the sender learns a direct route.
                                let mut ack_bytes = [0u8; 8];
                                if let Ok(alen) = ack.encode(&mut ack_bytes) {
                                    let _ = self.queue_path_return(
                                        peer[0],
                                        self.identity.pub_key()[0],
                                        &secret,
                                        &pkt,
                                        &ack_bytes[..alen],
                                    );
                                }
                            } else {
                                let (out_path_len, out_path) = self.peer_path(peer);
                                let _ = self.queue_ack_packet(ack, out_path_len, &out_path);
                            }
                        }
                        return InboundAction::ReceivedDirectText {
                            msg,
                            sender_pub: *peer,
                        };
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_ACK => {
                if let Ok(ack) = AckPacket::decode(&pkt.payload[..pkt.payload_len]) {
                    InboundAction::ReceivedAck(ack)
                } else {
                    InboundAction::Unhandled
                }
            }
            PAYLOAD_TYPE_PATH => {
                let payload = &pkt.payload[..pkt.payload_len];
                if payload.len() <= 4 || payload[0] != self.identity.pub_key()[0] {
                    return InboundAction::Unhandled;
                }
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                for peer in self.peers_by_hash(payload[1]).iter() {
                    if let Some(secret) = self.identity.shared_secret(peer) {
                        if let Ok(path) = PathPacket::decode(payload, &secret) {
                            return InboundAction::ReceivedPath {
                                packet: path,
                                sender_pub: *peer,
                            };
                        }
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_GRP_DATA => {
                if pkt.payload_len < 4 {
                    return InboundAction::Unhandled;
                }
                let ch_hash = pkt.payload[0];
                let channels = crate::companion::usb::CHANNELS.lock(|chs| *chs.borrow());
                for (idx, ch) in channels.iter().enumerate() {
                    if crate::meshcore::crypto::channel_hash(&ch.secret) == ch_hash {
                        if let Ok(grp) =
                            GrpDataPacket::decode_encrypted(&pkt.payload[..pkt.payload_len], &ch.secret)
                        {
                            return InboundAction::ReceivedGroupData {
                                channel_idx: idx as u8,
                                packet: grp,
                            };
                        }
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_ANON_REQ => {
                let payload = &pkt.payload[..pkt.payload_len];
                if payload.len() <= 35 || payload[0] != self.identity.pub_key()[0] {
                    return InboundAction::Unhandled;
                }
                let mut sender_pub = [0u8; 32];
                sender_pub.copy_from_slice(&payload[1..33]);
                if let Some(secret) = self.identity.shared_secret(&sender_pub) {
                    if let Ok(req) = AnonReqPacket::decode(payload, &secret) {
                        return InboundAction::ReceivedAnonReq(req);
                    }
                }
                InboundAction::Unhandled
            }
            _ => InboundAction::Unhandled,
        }
    }

    /// Construct and queue an advertisement packet for transmission
    /// Construct, sign, and queue an advertisement packet for transmission
    pub fn queue_advert(
        &mut self,
        timestamp: u32,
        lat_f32: f32,
        lon_f32: f32,
        flood: bool,
    ) -> Result<Packet, ()> {
        // Convert f32 GPS to integer (degrees * 1,000,000)
        let lat_int = (lat_f32 * 1_000_000.0) as i32;
        let lon_int = (lon_f32 * 1_000_000.0) as i32;

        let kind = if self.is_repeater {
            ADV_TYPE_REPEATER
        } else {
            ADV_TYPE_CHAT
        };

        let mut adv = AdvertPacket::new(
            *self.identity.pub_key(),
            timestamp,
            kind,
            lat_int,
            lon_int,
            self.identity.callsign(),
        );

        // Sign the advert with our private key
        adv.sign(&self.identity);

        let route = if flood {
            ROUTE_TYPE_FLOOD
        } else {
            ROUTE_TYPE_DIRECT
        };
        let mut pkt = adv.to_packet(route)?;
        if flood {
            pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        } else {
            // Zero-hop direct: only neighbours hear it.
            pkt.path_len = 0;
        }
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

    /// Construct and queue a direct text message. Returns the ACK CRC the
    /// sender should expect.
    pub fn queue_direct_text(
        &mut self,
        dest_pub: &[u8; 32],
        text: &str,
        timestamp: u32,
        txt_type: u8,
        attempt: u8,
        out_path_len: u8,
        out_path: &[u8; 64],
    ) -> Result<u32, ()> {
        let secret = self.identity.shared_secret(dest_pub).ok_or(())?;
        let msg = TxtMsgPacket::new(
            dest_pub[0],
            self.identity.pub_key()[0],
            timestamp,
            txt_type,
            attempt,
            text,
            self.identity.pub_key(),
        )?;
        let pkt = msg.to_packet(ROUTE_TYPE_FLOOD, &secret)?;
        let (route, path_len) = route_for(out_path_len);
        let _ = self.finish_outbound(pkt, route, path_len, out_path);
        Ok(msg.ack_crc)
    }

    /// Construct and queue an acknowledgment packet. Returns the packet.
    pub fn queue_ack(
        &mut self,
        crc: u32,
        extended_attempt: u8,
        random: u8,
    ) -> Result<Packet, ()> {
        self.queue_ack_packet(
            AckPacket::with_extended(crc, extended_attempt, random),
            0xFF,
            &[0u8; 64],
        )
    }

    fn queue_ack_packet(
        &mut self,
        ack: AckPacket,
        out_path_len: u8,
        out_path: &[u8; 64],
    ) -> Result<Packet, ()> {
        let pkt = ack.to_packet(ROUTE_TYPE_FLOOD)?;
        let (route, path_len) = route_for(out_path_len);
        Ok(self.finish_outbound(pkt, route, path_len, out_path))
    }

    fn queue_path_return(
        &mut self,
        dest: u8,
        src: u8,
        secret: &[u8; 32],
        arrival: &Packet,
        ack: &[u8],
    ) -> Result<Packet, ()> {
        let plen = arrival.path_byte_len();
        let mut path = [0u8; 64];
        path[..plen].copy_from_slice(&arrival.path[..plen]);
        let pp = PathPacket::new(dest, src, arrival.path_len, &path[..plen], 3, ack)?;
        let pkt = pp.to_packet(ROUTE_TYPE_FLOOD, secret, &[0u8; 4])?;
        Ok(self.finish_outbound(pkt, ROUTE_TYPE_FLOOD, 0, &[0u8; 64]))
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

    fn finish_outbound(
        &mut self,
        mut pkt: Packet,
        route: u8,
        path_len: u8,
        path: &[u8; 64],
    ) -> Packet {
        pkt.header = (pkt.header & 0xFC) | (route & 0x03);
        if route == ROUTE_TYPE_DIRECT && Packet::is_valid_path_len(path_len) {
            pkt.path_len = path_len;
            let n = pkt.path_byte_len();
            pkt.path[..n].copy_from_slice(&path[..n]);
        } else {
            pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        }
        self.seen_filter.mark_seen(pkt.calculate_packet_hash());
        let _ = self.tx_queue.push(pkt.clone());
        pkt
    }
}
