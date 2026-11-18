//! MeshCore Routing Engine & Dispatcher
//!
//! Handles:
//! - Flood & direct packet routing decisions.
//! - Duplicate packet suppression via `SeenPacketFilter`.
//! - Repeater forwarding with path hash accumulation.
//! - High-level packet composition (Adverts, Group text, Direct messages, ACKs).

use meshcore_wire::identity::LocalIdentity;
use meshcore_wire::packet::{
    Packet, PAYLOAD_TYPE_ACK, PAYLOAD_TYPE_ADVERT, PAYLOAD_TYPE_ANON_REQ,
    PAYLOAD_TYPE_CONTROL, PAYLOAD_TYPE_GRP_DATA, PAYLOAD_TYPE_GRP_TXT, PAYLOAD_TYPE_MULTIPART,
    PAYLOAD_TYPE_PATH, PAYLOAD_TYPE_RAW_CUSTOM, PAYLOAD_TYPE_REQ, PAYLOAD_TYPE_RESPONSE,
    PAYLOAD_TYPE_TRACE, PAYLOAD_TYPE_TXT_MSG, ROUTE_TYPE_DIRECT, ROUTE_TYPE_FLOOD,
};
use meshcore_wire::packets::{
    AckPacket, AddressedPacket, AdvertPacket, AnonReqPacket, GrpDataPacket, GrpTextPacket,
    MultipartPacket, PathPacket, TxtMsgPacket,
};

use meshcore_wire::queue::{PacketQueue, SeenPacketFilter};
use meshcore_wire::tx::TxLane;

use crate::scheduler::MULTI_ACK_SPACING_MS;
use crate::state::default_channels;
use crate::types::{ChannelData, MAX_CHANNELS};

/// Host services the routing engine needs from the firmware around it.
///
/// The engine owns no globals: contacts, channels, flood scoping and the
/// transmit scheduler are reached through this trait, so the same engine can
/// run on a board, in the runtime, or in host tests.
pub trait EngineContext: Send + Sync {
    /// Current path hash width mode (0 = 1 byte, 1 = 2 bytes, 2 = 3 bytes).
    fn path_hash_mode(&self) -> u8 { 0 }

    /// Extra `PAYLOAD_TYPE_MULTIPART` ACK copies to send before a direct ACK.
    fn extra_ack_count(&self) -> u8 { 0 }

    /// Contact public keys whose first byte matches `hash`.
    fn peers_by_hash(&self, _hash: u8) -> heapless::Vec<[u8; 32], 32> {
        heapless::Vec::new()
    }

    /// Pending anonymous/binary request target matching `hash` when not in contacts.
    fn anon_peer_key(&self, _hash: u8) -> Option<[u8; 32]> { None }

    /// Snapshot of the group channel table.
    fn channels(&self) -> [ChannelData; MAX_CHANNELS] { default_channels() }

    /// Learned return path for a contact, or `(0xFF, zeroes)` when unknown.
    fn peer_path(&self, _pub_key: &[u8; 32]) -> (u8, [u8; 64]) {
        (0xFF, [0u8; 64])
    }

    /// Attach the current send scope to a flood packet.
    fn apply_flood_scope(&self, _pkt: &mut Packet) {}

    /// Attach the persisted default scope to a self advert.
    fn apply_flood_scope_default(&self, _pkt: &mut Packet) {}

    /// Account for one packet queued for forwarding.
    fn note_forward_scheduled(&self) {}

    /// Record how late a duplicate of a pending forward arrived.
    fn note_duplicate_delta(&self, _ms: u64) {}

    /// Ask the scheduler to drop a scheduled flood.
    fn cancel_flood(&self, _hash: [u8; 8]) {}
}

/// Context for host tests and nodes that do not forward or scope floods.
pub struct NoopContext;

impl EngineContext for NoopContext {}


pub const DEFAULT_MAX_HOPS: u8 = 64;

struct PendingForward {
    hash: [u8; 8],
    queued_at_ms: u64,
}

/// Monotonic milliseconds for transmit eligibility.
fn now_ms() -> u64 {
    embassy_time::Instant::now().as_millis() as u64
}




/// Direct when the contact has a valid encoded path; flood otherwise.
fn route_for(out_path_len: u8) -> (u8, u8) {
    if out_path_len != 0xFF && Packet::is_valid_path_len(out_path_len) {
        (ROUTE_TYPE_DIRECT, out_path_len)
    } else {
        (ROUTE_TYPE_FLOOD, 0)
    }
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
        /// Encoded flood arrival path length, or `0xFF` when received direct.
        path_len: u8,
    },
    /// Received message delivery acknowledgment
    ReceivedAck {
        ack: AckPacket,
        is_flood: bool,
        arrival_path_len: u8,
        arrival_path: heapless::Vec<u8, 64>,
    },
    /// Received path discovery reply
    ReceivedPath {
        packet: PathPacket,
        sender_pub: [u8; 32],
        is_flood: bool,
        /// Path the reply travelled to reach us (packed length + bytes).
        arrival_path_len: u8,
        arrival_path: heapless::Vec<u8, 64>,
    },
    /// Received custom raw payload (`PAYLOAD_TYPE_RAW_CUSTOM`).
    ReceivedRawCustom {
        data: heapless::Vec<u8, 184>,
    },
    /// Received channel datagram
    ReceivedGroupData {
        channel_idx: u8,
        path_len: u8,
        packet: GrpDataPacket,
    },
    /// Received anonymous request
    ReceivedAnonReq(AnonReqPacket),
    /// Received addressed request (REQ) for this node
    ReceivedRequest {
        plain: heapless::Vec<u8, 167>,
        sender_pub: [u8; 32],
        is_flood: bool,
        arrival_path_len: u8,
        arrival_path: heapless::Vec<u8, 64>,
    },
    /// Received addressed response (RESPONSE) for this node
    ReceivedResponse {
        plain: heapless::Vec<u8, 167>,
        sender_pub: [u8; 32],
    },
    /// Zero-hop control data addressed to the companion (discovery, etc.).
    ReceivedControl {
        data: heapless::Vec<u8, 184>,
        path_len: u8,
    },
    /// TRACE reached the end of its path.
    ReceivedTrace {
        tag: u32,
        auth: u32,
        flags: u8,
        hashes: heapless::Vec<u8, 175>,
        snrs: heapless::Vec<u8, 64>,
        final_snr: i8,
    },
    /// Retransmitted a direct packet addressed to the next hop
    Forwarded,
    /// Unhandled or unknown packet payload
    Unhandled,
}

/// MeshCore Routing Engine
pub struct MeshEngine<'a> {
    pub identity: LocalIdentity,
    pub seen_filter: SeenPacketFilter,
    pub tx_queue: PacketQueue,
    pending_forwards: heapless::Vec<PendingForward, 8>,
    /// Role policy: whether this node may forward packets.
    pub allow_forward: bool,
    /// Node type used in adverts (CHAT for companions, REPEATER for repeaters).
    pub advert_type: u8,
    pub max_hops: u8,
    pub path_hash_mode: u8,
    context: &'a dyn EngineContext,
}

impl<'a> MeshEngine<'a> {
    pub fn new(
        identity: LocalIdentity,
        advert_type: u8,
        context: &'a dyn EngineContext,
    ) -> Self {
        Self {
            identity,
            seen_filter: SeenPacketFilter::new(),
            tx_queue: PacketQueue::new(),
            pending_forwards: heapless::Vec::new(),
            allow_forward: false,
            advert_type,
            max_hops: DEFAULT_MAX_HOPS,
            path_hash_mode: context.path_hash_mode(),
            context,
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
        self.context.peers_by_hash(hash)
    }

    /// Queue a flood packet for forwarding.
    fn queue_forward(&mut self, pkt: Packet) {
        let now = now_ms();
        let hash = pkt.calculate_packet_hash();
        if self.pending_forwards.is_full() {
            self.pending_forwards.remove(0);
        }
        let _ = self.pending_forwards.push(PendingForward {
            hash,
            queued_at_ms: now,
        });
        let _ = self.tx_queue.push_flood(pkt, now);
        self.context.note_forward_scheduled();
    }

    /// When a duplicate arrives, record how late the other copy was and stop
    /// tracking the pending forward.
    fn note_duplicate_of_pending(&mut self, pkt: &Packet) {
        let now = now_ms();
        self.pending_forwards
            .retain(|entry| entry.queued_at_ms.saturating_add(8000) >= now);
        let hash = pkt.calculate_packet_hash();
        if let Some(index) = self
            .pending_forwards
            .iter()
            .position(|entry| entry.hash == hash)
        {
            let entry = self.pending_forwards.remove(index);
            self.context.note_duplicate_delta(now.saturating_sub(entry.queued_at_ms));
            let _ = self.context.cancel_flood(entry.hash);
        }
    }

    /// Drop every queued flood and ask the radio task to clear scheduled ones.
    /// Used when forwarding is turned off.
    pub fn cancel_pending_forwards(&mut self) {
        let mut hashes: heapless::Vec<[u8; 8], 8> = heapless::Vec::new();
        self.tx_queue.retain(|queued| {
            if queued.lane == TxLane::Flood {
                let _ = hashes.push(queued.packet.calculate_packet_hash());
                false
            } else {
                true
            }
        });
        for hash in hashes.iter() {
            self.context.cancel_flood(*hash);
        }
        for entry in self.pending_forwards.iter() {
            self.context.cancel_flood(entry.hash);
        }
        self.pending_forwards.clear();
    }

    /// Process a packet received over the LoRa radio interface
    pub fn handle_inbound_packet(&mut self, pkt: Packet, snr: i8) -> InboundAction {
        // Zero-hop control data is delivered to the app (discovery, etc.).
        if pkt.route_type() == ROUTE_TYPE_DIRECT
            && pkt.payload_type() == PAYLOAD_TYPE_CONTROL
            && pkt.path_hash_count() == 0
            && pkt.payload_len > 0
            && (pkt.payload[0] & 0x80) != 0
        {
            let mut data = heapless::Vec::<u8, 184>::new();
            let _ = data.extend_from_slice(&pkt.payload[..pkt.payload_len]);
            return InboundAction::ReceivedControl {
                data,
                path_len: pkt.path_len,
            };
        }

        // TRACE is direct but its path field carries SNR values, not hops, so
        // it must be handled before the normal direct-forwarding logic.
        if pkt.route_type() == meshcore_wire::packet::ROUTE_TYPE_DIRECT
            && pkt.payload_type() == PAYLOAD_TYPE_TRACE
        {
            if pkt.payload_len < 9 {
                return InboundAction::Unhandled;
            }
            let flags = pkt.payload[8];
            let hash_size = 1usize << (flags & 0x03);
            let len = pkt.payload_len as usize - 9;
            let offset = (pkt.path_len as usize) << (flags & 0x03);

            if offset >= len {
                let mut hashes = heapless::Vec::<u8, 175>::new();
                let _ = hashes.extend_from_slice(&pkt.payload[9..9 + len]);
                let mut snrs = heapless::Vec::<u8, 64>::new();
                let _ = snrs.extend_from_slice(&pkt.path[..pkt.path_len as usize]);
                return InboundAction::ReceivedTrace {
                    tag: u32::from_le_bytes([
                        pkt.payload[0],
                        pkt.payload[1],
                        pkt.payload[2],
                        pkt.payload[3],
                    ]),
                    auth: u32::from_le_bytes([
                        pkt.payload[4],
                        pkt.payload[5],
                        pkt.payload[6],
                        pkt.payload[7],
                    ]),
                    flags,
                    hashes,
                    snrs,
                    final_snr: snr,
                };
            }

            if offset + hash_size <= len
                && self.identity.pub_key()[..hash_size]
                    == pkt.payload[9 + offset..9 + offset + hash_size]
            {
                // The companion role must not relay TRACE unless it is acting
                // as a forwarder (off-grid client repeat).
                if !self.allow_forward {
                    return InboundAction::Unhandled;
                }
                if self.seen_filter.check_and_mark(&pkt) {
                    return InboundAction::DropDuplicate;
                }
                let mut relay = pkt.clone();
                if relay.path_len < 64 {
                    relay.path[relay.path_len as usize] = (snr.saturating_mul(4)) as u8;
                    relay.path_len += 1;
                    let _ = self.tx_queue.push_control(relay, now_ms());
                }
                return InboundAction::Forwarded;
            }
            return InboundAction::Unhandled;
        }

        // 1. Check hop limits
        if pkt.path_hash_count() >= self.max_hops as usize {
            return InboundAction::DropHopLimitExceeded;
        }

        let is_dupe = self.seen_filter.check_and_mark(&pkt);
        let already_in_path = pkt.contains_hop(self.identity.pub_key());
        if is_dupe {
            self.note_duplicate_of_pending(&pkt);
        }

        // 2. Direct forwarding: if the first hop is us, pop it and requeue.
        if pkt.route_type() == meshcore_wire::packet::ROUTE_TYPE_DIRECT {
            if pkt.path_hash_count() > 0 {
                let size = pkt.path_hash_size();
                if pkt.path[..size] == self.identity.pub_key()[..size] {
                    if self.allow_forward {
                        match pkt.payload_type() {
                            PAYLOAD_TYPE_ACK => {
                                if is_dupe {
                                    return InboundAction::DropDuplicate;
                                }
                                let mut fwd = pkt.clone();
                                if fwd.pop_front_hop() {
                                    if let Ok(ack) =
                                        AckPacket::decode(&fwd.payload[..fwd.payload_len])
                                    {
                                        let _ = self.emit_direct_ack_burst(
                                            &fwd.payload[..fwd.payload_len],
                                            fwd.path_len,
                                            &fwd.path,
                                            0,
                                        );
                                        return InboundAction::ReceivedAck {
                                            ack,
                                            is_flood: false,
                                            arrival_path_len: 0,
                                            arrival_path: heapless::Vec::new(),
                                        };
                                    }
                                    self.queue_forward(fwd);
                                    return InboundAction::Forwarded;
                                }
                            }
                            PAYLOAD_TYPE_MULTIPART => {
                                if let Ok(mp) =
                                    MultipartPacket::decode(&pkt.payload[..pkt.payload_len])
                                {
                                    if mp.inner_type == PAYLOAD_TYPE_ACK && mp.body.len() >= 4 {
                                        if let Ok(ack) = AckPacket::decode(&mp.body) {
                                            let mut inner_pkt = Packet::new(
                                                ROUTE_TYPE_DIRECT,
                                                PAYLOAD_TYPE_ACK,
                                            );
                                            if inner_pkt.set_payload(&mp.body).is_ok() {
                                                let inner_dupe =
                                                    self.seen_filter.check_and_mark(&inner_pkt);
                                                if is_dupe || inner_dupe {
                                                    return InboundAction::DropDuplicate;
                                                }
                                                let mut fwd = pkt.clone();
                                                if fwd.pop_front_hop() {
                                                    let delay_ms = (mp.remaining as u64 + 1)
                                                        * MULTI_ACK_SPACING_MS;
                                                    let _ = self.emit_direct_ack_burst(
                                                        &mp.body,
                                                        fwd.path_len,
                                                        &fwd.path,
                                                        delay_ms,
                                                    );
                                                    return InboundAction::ReceivedAck {
                                                        ack,
                                                        is_flood: false,
                                                        arrival_path_len: 0,
                                                        arrival_path: heapless::Vec::new(),
                                                    };
                                                }
                                            }
                                        }
                                        return InboundAction::Unhandled;
                                    }
                                }
                                if is_dupe {
                                    return InboundAction::DropDuplicate;
                                }
                                let mut fwd = pkt.clone();
                                if fwd.pop_front_hop() {
                                    self.queue_forward(fwd);
                                    return InboundAction::Forwarded;
                                }
                            }
                            _ => {
                                let mut fwd = pkt.clone();
                                if fwd.pop_front_hop() {
                                    self.queue_forward(fwd);
                                    return InboundAction::Forwarded;
                                }
                            }
                        }
                    }
                }
                // A direct packet with a non-empty path is not for us unless we
                // are its next hop.
                return InboundAction::Unhandled;
            }
        }

        // 3. Dispatch based on payload type, tracking whether this node
        // consumed the packet so it is not retransmitted afterward.
        let mut consumed = false;
        let action = (|| {
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
                    if self.allow_forward && pkt.is_flood() && !already_in_path && adv.pub_key != *self.identity.pub_key() {
                        let mut repeat_pkt = pkt.clone();
                        repeat_pkt.append_hop_bytes(self.identity.pub_key());
                        self.queue_forward(repeat_pkt);
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
                    // Running SHA-256 hashing and AES-128 decryption inside a lock callback
                    // can block the BLE SoftDevice (S140) and freeze the device.
                    //
                    // The array ([ChannelData; 40] = 1920 bytes) is copied by value here,
                    // which is cheap compared to the crypto operations below.
                    let channels = self.context.channels();
                    let mut matched = None;
                    for (idx, ch) in channels.iter().enumerate() {
                        if meshcore_wire::crypto::channel_hash(&ch.secret) == ch_hash {
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
                        let plain = msg.txt_type
                            == meshcore_wire::packets::txtmsg::TXT_TYPE_PLAIN;
                        let signed = msg.txt_type
                            == meshcore_wire::packets::txtmsg::TXT_TYPE_SIGNED_PLAIN;
                        if plain || signed {
                            // Signed messages carry a 4-byte ACK; companions
                            // use the extended 6-byte form.
                            let ack = if signed {
                                AckPacket::new(msg.ack_crc)
                            } else {
                                AckPacket::with_extended(
                                    msg.ack_crc,
                                    msg.extended_attempt,
                                    0,
                                )
                            };
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
                        consumed = true;
                        return InboundAction::ReceivedDirectText {
                            msg,
                            sender_pub: *peer,
                            path_len: if pkt.is_flood() { pkt.path_len } else { 0xFF },
                        };
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_ACK => {
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                if let Ok(ack) = AckPacket::decode(&pkt.payload[..pkt.payload_len]) {
                    let mut arrival_path = heapless::Vec::<u8, 64>::new();
                    let an = pkt.path_byte_len();
                    let _ = arrival_path.extend_from_slice(&pkt.path[..an]);
                    InboundAction::ReceivedAck {
                        ack,
                        is_flood: pkt.is_flood(),
                        arrival_path_len: pkt.path_len,
                        arrival_path,
                    }
                } else {
                    InboundAction::Unhandled
                }
            }
            PAYLOAD_TYPE_PATH => {
                let payload = &pkt.payload[..pkt.payload_len];
                if payload.len() <= 4 {
                    return InboundAction::Unhandled;
                }
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                if payload[0] != self.identity.pub_key()[0] {
                    return InboundAction::Unhandled;
                }
                for peer in self.peers_by_hash(payload[1]).iter() {
                    if let Some(secret) = self.identity.shared_secret(peer) {
                        if let Ok(path) = PathPacket::decode(payload, &secret) {
                            let mut arrival_path = heapless::Vec::<u8, 64>::new();
                            let an = pkt.path_byte_len();
                            let _ = arrival_path.extend_from_slice(&pkt.path[..an]);
                            consumed = true;
                            return InboundAction::ReceivedPath {
                                packet: path,
                                sender_pub: *peer,
                                is_flood: pkt.is_flood(),
                                arrival_path_len: pkt.path_len,
                                arrival_path,
                            };
                        }
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_MULTIPART => {
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                // v1.18 uses MULTIPART for batched ACKs: the first byte is
                // (remaining << 4 | inner_type), followed by the inner payload.
                // Check and mark the unwrapped ACK hash so subsequent copies
                // and the trailing ACK deduplicate cleanly.
                if let Ok(mp) = MultipartPacket::decode(&pkt.payload[..pkt.payload_len]) {
                    if mp.inner_type == PAYLOAD_TYPE_ACK && mp.body.len() >= 4 {
                        if let Ok(ack) = AckPacket::decode(&mp.body) {
                            let mut inner_pkt =
                                Packet::new(pkt.route_type(), PAYLOAD_TYPE_ACK);
                            if inner_pkt.set_payload(&mp.body).is_ok()
                                && self.seen_filter.check_and_mark(&inner_pkt)
                            {
                                return InboundAction::DropDuplicate;
                            }
                            let mut arrival_path = heapless::Vec::<u8, 64>::new();
                            let an = pkt.path_byte_len();
                            let _ = arrival_path.extend_from_slice(&pkt.path[..an]);
                            return InboundAction::ReceivedAck {
                                ack,
                                is_flood: pkt.is_flood(),
                                arrival_path_len: pkt.path_len,
                                arrival_path,
                            };
                        }
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_GRP_DATA => {
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                if pkt.payload_len < 4 {
                    return InboundAction::Unhandled;
                }
                let ch_hash = pkt.payload[0];
                let channels = self.context.channels();
                for (idx, ch) in channels.iter().enumerate() {
                    if meshcore_wire::crypto::channel_hash(&ch.secret) == ch_hash {
                        if let Ok(grp) =
                            GrpDataPacket::decode_encrypted(&pkt.payload[..pkt.payload_len], &ch.secret)
                        {
                            return InboundAction::ReceivedGroupData {
                                channel_idx: idx as u8,
                                path_len: if pkt.is_flood() { pkt.path_len } else { 0xFF },
                                packet: grp,
                            };
                        }
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_REQ | PAYLOAD_TYPE_RESPONSE => {
                let payload = &pkt.payload[..pkt.payload_len];
                if payload.len() <= 4 {
                    return InboundAction::Unhandled;
                }
                let dest = payload[0];
                let src = payload[1];

                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                if dest != self.identity.pub_key()[0] {
                    return InboundAction::Unhandled;
                }

                for peer in self.peers_by_hash(src).iter() {
                    if let Some(secret) = self.identity.shared_secret(peer) {
                        if let Ok(ap) = AddressedPacket::decode(payload, &secret) {
                            consumed = true;
                            return if pkt.payload_type() == PAYLOAD_TYPE_RESPONSE {
                                InboundAction::ReceivedResponse {
                                    plain: ap.plain,
                                    sender_pub: *peer,
                                }
                            } else {
                                let mut arrival_path = heapless::Vec::<u8, 64>::new();
                                let an = pkt.path_byte_len();
                                let _ = arrival_path.extend_from_slice(&pkt.path[..an]);
                                InboundAction::ReceivedRequest {
                                    plain: ap.plain,
                                    sender_pub: *peer,
                                    is_flood: pkt.is_flood(),
                                    arrival_path_len: pkt.path_len,
                                    arrival_path,
                                }
                            };
                        }
                    }
                }
                if pkt.payload_type() == PAYLOAD_TYPE_RESPONSE {
                    if let Some(peer) = self.context.anon_peer_key(src) {
                        if let Some(secret) = self.identity.shared_secret(&peer) {
                            if let Ok(ap) = AddressedPacket::decode(payload, &secret) {
                                consumed = true;
                                return InboundAction::ReceivedResponse {
                                    plain: ap.plain,
                                    sender_pub: peer,
                                };
                            }
                        }
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_ANON_REQ => {
                let payload = &pkt.payload[..pkt.payload_len];
                if payload.len() <= 35 {
                    return InboundAction::Unhandled;
                }
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                if payload[0] != self.identity.pub_key()[0] {
                    return InboundAction::Unhandled;
                }
                let mut sender_pub = [0u8; 32];
                sender_pub.copy_from_slice(&payload[1..33]);
                if let Some(secret) = self.identity.shared_secret(&sender_pub) {
                    if let Ok(req) = AnonReqPacket::decode(payload, &secret) {
                        consumed = true;
                        return InboundAction::ReceivedAnonReq(req);
                    }
                }
                InboundAction::Unhandled
            }
            PAYLOAD_TYPE_RAW_CUSTOM => {
                if is_dupe {
                    return InboundAction::DropDuplicate;
                }
                let mut data = heapless::Vec::<u8, 184>::new();
                let _ = data.extend_from_slice(&pkt.payload[..pkt.payload_len]);
                InboundAction::ReceivedRawCustom { data }
            }
            _ => InboundAction::Unhandled,
            }
        })();

        // 4. Flood forwarding is payload-agnostic, but a flooded unicast that
        // this node decrypted and consumed must not be relayed. Adverts are
        // excluded because they forward only after signature verification;
        // TRACE has its own direct handling above.
        if self.allow_forward
            && pkt.is_flood()
            && !is_dupe
            && !already_in_path
            && !consumed
            && pkt.payload_type() != PAYLOAD_TYPE_ADVERT
            && pkt.payload_type() != PAYLOAD_TYPE_TRACE
        {
            let mut repeat_pkt = pkt.clone();
            repeat_pkt.append_hop_bytes(self.identity.pub_key());
            self.queue_forward(repeat_pkt);
        }

        action
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

        let kind = self.advert_type;

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
            // Self adverts always use the persisted default scope, even when
            // the session scope override is force-unscoped.
            self.context.apply_flood_scope_default(&mut pkt);
        } else {
            // Zero-hop direct: only neighbours hear it.
            pkt.path_len = 0;
        }
        self.seen_filter.mark_seen(pkt.calculate_packet_hash());
        let _ = self.tx_queue.push_background(pkt.clone(), now_ms());
        Ok(pkt)
    }

    /// Construct and queue an encrypted channel group text message
    pub fn queue_group_text(&mut self, channel_hash: u8, channel_secret: &[u8], message: &str, timestamp: u32) -> Result<Packet, ()> {
        let grp = GrpTextPacket::new(channel_hash, self.identity.callsign(), message, timestamp, GrpTextPacket::TXT_TYPE_PLAIN, 0);
        let mut pkt = grp.to_packet(channel_secret, ROUTE_TYPE_FLOOD)?;
        pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        self.context.apply_flood_scope(&mut pkt);
        self.seen_filter.mark_seen(pkt.calculate_packet_hash());
        let _ = self.tx_queue.push_control(pkt.clone(), now_ms());
        Ok(pkt)
    }

    /// Construct and queue a channel datagram. `path_len == 0xFF` floods the
    /// packet; any other value sends it direct over the supplied path.
    pub fn queue_group_data(
        &mut self,
        channel_hash: u8,
        channel_secret: &[u8],
        data_type: u16,
        data: &[u8],
        path_len: u8,
        path: &[u8; 64],
    ) -> Result<Packet, ()> {
        let grp = GrpDataPacket::new(channel_hash, data_type, data)?;
        if path_len == 0xFF {
            let mut pkt = grp.to_packet(channel_secret, ROUTE_TYPE_FLOOD)?;
            pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
            self.context.apply_flood_scope(&mut pkt);
            self.seen_filter.mark_seen(pkt.calculate_packet_hash());
            let _ = self.tx_queue.push_control(pkt.clone(), now_ms());
            Ok(pkt)
        } else {
            let mut pkt = grp.to_packet(channel_secret, ROUTE_TYPE_DIRECT)?;
            pkt.path_len = path_len;
            let n = pkt.path_byte_len();
            pkt.path[..n].copy_from_slice(&path[..n]);
            self.seen_filter.mark_seen(pkt.calculate_packet_hash());
            let _ = self.tx_queue.push_control(pkt.clone(), now_ms());
            Ok(pkt)
        }
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
        let (route, path_len) = route_for(out_path_len);
        if route == ROUTE_TYPE_DIRECT {
            let mut ack_bytes = [0u8; 6];
            let alen = ack.encode(&mut ack_bytes)?;
            self.emit_direct_ack_burst(&ack_bytes[..alen], path_len, out_path, 0)
        } else {
            let pkt = ack.to_packet(ROUTE_TYPE_FLOOD)?;
            Ok(self.finish_outbound(pkt, route, path_len, out_path))
        }
    }

    fn emit_direct_ack_burst(
        &mut self,
        ack_bytes: &[u8],
        out_path_len: u8,
        out_path: &[u8; 64],
        initial_delay_ms: u64,
    ) -> Result<Packet, ()> {
        let extra = self.context.extra_ack_count().min(15);
        let now = now_ms();

        let mut ack_pkt = Packet::new(ROUTE_TYPE_DIRECT, PAYLOAD_TYPE_ACK);
        ack_pkt.set_payload(ack_bytes)?;
        if Packet::is_valid_path_len(out_path_len) {
            ack_pkt.path_len = out_path_len;
            let n = ack_pkt.path_byte_len();
            ack_pkt.path[..n].copy_from_slice(&out_path[..n]);
        } else {
            ack_pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        }
        self.seen_filter.mark_seen(ack_pkt.calculate_packet_hash());

        let mut offset = initial_delay_ms;
        for rem in (1..=extra).rev() {
            let mp = MultipartPacket::new(rem, PAYLOAD_TYPE_ACK, ack_bytes)?;
            let mut mp_pkt = mp.to_packet(ROUTE_TYPE_DIRECT)?;
            mp_pkt.path_len = ack_pkt.path_len;
            let n = mp_pkt.path_byte_len();
            mp_pkt.path[..n].copy_from_slice(&ack_pkt.path[..n]);
            self.seen_filter.mark_seen(mp_pkt.calculate_packet_hash());
            let _ = self
                .tx_queue
                .push_control_at(mp_pkt, now, now.saturating_add(offset));
            offset = offset.saturating_add(MULTI_ACK_SPACING_MS);
        }

        let _ = self
            .tx_queue
            .push_control_at(ack_pkt.clone(), now, now.saturating_add(offset));
        Ok(ack_pkt)
    }

    fn queue_path_return(
        &mut self,
        dest: u8,
        src: u8,
        secret: &[u8; 32],
        arrival: &Packet,
        ack: &[u8],
    ) -> Result<Packet, ()> {
        const TXT_ACK_DELAY_MS: u64 = 200;
        let plen = arrival.path_byte_len();
        let mut path = [0u8; 64];
        path[..plen].copy_from_slice(&arrival.path[..plen]);
        let pp = PathPacket::new(dest, src, arrival.path_len, &path[..plen], 3, ack)?;
        let pkt = pp.to_packet(ROUTE_TYPE_FLOOD, secret, &[0u8; 4])?;
        Ok(self.finish_outbound_at(pkt, ROUTE_TYPE_FLOOD, 0, &[0u8; 64], TXT_ACK_DELAY_MS))
    }

    /// Queue a standalone reciprocal `PAYLOAD_TYPE_PATH` (no extra payload)
    /// telling `dest_pub` the return path it should use to reach us.
    pub fn queue_path_return_only(
        &mut self,
        dest_pub: &[u8; 32],
        send_path_len: u8,
        send_path: &[u8; 64],
        returned_path_len: u8,
        returned_path: &[u8],
        delay_ms: u64,
    ) -> Result<Packet, ()> {
        let secret = self.identity.shared_secret(dest_pub).ok_or(())?;
        let dest = dest_pub[0];
        let src = self.identity.pub_key()[0];
        let mut rand4 = [0u8; 4];
        rand4.copy_from_slice(&now_ms().to_le_bytes()[..4]);
        let pp = PathPacket::new(dest, src, returned_path_len, returned_path, 0xFF, &[])?;
        let (route, path_len) = route_for(send_path_len);
        let pkt = pp.to_packet(route, &secret, &rand4)?;
        Ok(self.finish_outbound_at(pkt, route, path_len, send_path, delay_ms))
    }

    /// Construct and queue a response to an inbound `PAYLOAD_TYPE_REQ`.
    ///
    /// - When the request arrived via flood (`is_flood`), return the arrival
    ///   path in a flooded `PAYLOAD_TYPE_PATH` packet with
    ///   `extra_type = PAYLOAD_TYPE_RESPONSE` so the requester learns the
    ///   direct route to us (including for Path Discovery) and receives the
    ///   response payload in a single packet.
    /// - When the request arrived direct, send a `PAYLOAD_TYPE_RESPONSE`
    ///   datagram (direct along the contact's known `out_path`, or flooded if
    ///   unknown).
    pub fn queue_request_reply(
        &mut self,
        sender_pub: &[u8; 32],
        is_flood: bool,
        arrival_path_len: u8,
        arrival_path: &[u8],
        reply_plain: &[u8],
    ) -> Result<Packet, ()> {
        let secret = self.identity.shared_secret(sender_pub).ok_or(())?;
        let dest = sender_pub[0];
        let src = self.identity.pub_key()[0];
        if is_flood {
            let pp = PathPacket::new(
                dest,
                src,
                arrival_path_len,
                arrival_path,
                PAYLOAD_TYPE_RESPONSE,
                reply_plain,
            )?;
            let pkt = pp.to_packet(ROUTE_TYPE_FLOOD, &secret, &[0u8; 4])?;
            Ok(self.finish_outbound(pkt, ROUTE_TYPE_FLOOD, 0, &[0u8; 64]))
        } else {
            let (out_path_len, out_path) = self.peer_path(sender_pub);
            let (route, path_len) = route_for(out_path_len);
            let addressed = AddressedPacket::new(dest, src, reply_plain)?;
            let pkt = addressed.to_packet(PAYLOAD_TYPE_RESPONSE, route, &secret)?;
            Ok(self.finish_outbound(pkt, route, path_len, &out_path))
        }
    }

    fn peer_path(&self, pub_key: &[u8; 32]) -> (u8, [u8; 64]) {
        self.context.peer_path(pub_key)
    }

    fn finish_outbound(
        &mut self,
        pkt: Packet,
        route: u8,
        path_len: u8,
        path: &[u8; 64],
    ) -> Packet {
        self.finish_outbound_at(pkt, route, path_len, path, 0)
    }

    fn finish_outbound_at(
        &mut self,
        mut pkt: Packet,
        route: u8,
        path_len: u8,
        path: &[u8; 64],
        delay_ms: u64,
    ) -> Packet {
        pkt.header = (pkt.header & 0xFC) | (route & 0x03);
        if route == ROUTE_TYPE_FLOOD {
            self.context.apply_flood_scope(&mut pkt);
        }
        if route == ROUTE_TYPE_DIRECT && Packet::is_valid_path_len(path_len) {
            pkt.path_len = path_len;
            let n = pkt.path_byte_len();
            pkt.path[..n].copy_from_slice(&path[..n]);
        } else {
            pkt.set_path_hash_size_and_count(self.path_hash_mode + 1, 0);
        }
        self.seen_filter.mark_seen(pkt.calculate_packet_hash());
        let now = now_ms();
        let _ = self
            .tx_queue
            .push_control_at(pkt.clone(), now, now.saturating_add(delay_ms));
        pkt
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use meshcore_wire::packets::advert::ADV_TYPE_REPEATER;

    struct TestContext;

    impl EngineContext for TestContext {
        fn path_hash_mode(&self) -> u8 {
            2
        }

        fn peers_by_hash(&self, hash: u8) -> heapless::Vec<[u8; 32], 32> {
            let mut peers = heapless::Vec::new();
            let _ = peers.push([hash; 32]);
            peers
        }
    }

    fn identity() -> LocalIdentity {
        LocalIdentity::from_seed("test", &[7u8; 32])
    }

    #[test]
    fn engine_reads_the_host_context() {
        let mut engine = MeshEngine::new(identity(), ADV_TYPE_REPEATER, &TestContext);
        engine.allow_forward = true;
        assert_eq!(engine.path_hash_mode, 2);
        assert!(engine.allow_forward);
        assert_eq!(engine.advert_type, ADV_TYPE_REPEATER);
        assert_eq!(engine.peers_by_hash(9)[0], [9u8; 32]);
    }

    #[test]
    fn noop_context_provides_public_channel_defaults() {
        let channels = NoopContext.channels();
        assert_eq!(&channels[0].name[..6], b"Public");
        assert!(NoopContext.peers_by_hash(0).is_empty());
        assert_eq!(NoopContext.peer_path(&[0u8; 32]).0, 0xFF);
    }

    fn flood(mut pkt: Packet) -> Packet {
        pkt.set_path_hash_size_and_count(2, 0);
        pkt
    }

    #[test]
    fn flooded_anon_request_for_us_is_not_forwarded() {
        let id = identity();
        let peer = LocalIdentity::from_seed("peer", &[9u8; 32]);
        let sender = *peer.pub_key();
        let secret = id.shared_secret(&sender).unwrap();
        let request = AnonReqPacket::new(id.pub_key()[0], sender, b"ping").unwrap();
        let pkt = flood(request.to_packet(&secret, ROUTE_TYPE_FLOOD).unwrap());

        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &TestContext);
        engine.allow_forward = true;

        assert!(matches!(
            engine.handle_inbound_packet(pkt, 6),
            InboundAction::ReceivedAnonReq(_)
        ));
        assert!(
            engine.tx_queue.is_empty(),
            "a consumed anonymous request must not be relayed"
        );
    }

    #[test]
    fn flooded_anon_request_for_another_node_is_forwarded() {
        let id = identity();
        let peer = LocalIdentity::from_seed("peer", &[9u8; 32]);
        let sender = *peer.pub_key();
        let secret = id.shared_secret(&sender).unwrap();
        let dest = id.pub_key()[0] ^ 0xFF;
        let request = AnonReqPacket::new(dest, sender, b"ping").unwrap();
        let pkt = flood(request.to_packet(&secret, ROUTE_TYPE_FLOOD).unwrap());

        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &TestContext);
        engine.allow_forward = true;

        assert!(matches!(
            engine.handle_inbound_packet(pkt, 6),
            InboundAction::Unhandled
        ));
        assert!(engine
            .tx_queue
            .iter()
            .any(|q| q.lane == TxLane::Flood
                && q.packet.payload_type() == PAYLOAD_TYPE_ANON_REQ));
    }

    #[test]
    fn flooded_direct_text_for_us_is_not_forwarded() {
        let id = identity();
        let src = 0x33u8;
        let peer = [src; 32];
        let secret = id.shared_secret(&peer).unwrap();
        let msg = TxtMsgPacket::new(
            id.pub_key()[0],
            src,
            1234,
            meshcore_wire::packets::txtmsg::TXT_TYPE_PLAIN,
            0,
            "hello",
            &peer,
        )
        .unwrap();
        let pkt = flood(msg.to_packet(ROUTE_TYPE_FLOOD, &secret).unwrap());

        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &TestContext);
        engine.allow_forward = true;

        assert!(matches!(
            engine.handle_inbound_packet(pkt, 6),
            InboundAction::ReceivedDirectText { .. }
        ));
        // The flooded DM queues an ACK/path return, but the original text must
        // not be retransmitted.
        assert!(!engine
            .tx_queue
            .iter()
            .any(|q| q.packet.payload_type() == PAYLOAD_TYPE_TXT_MSG));
    }

    #[test]
    fn flooded_direct_text_for_another_node_is_forwarded() {
        let id = identity();
        let src = 0x33u8;
        let peer = [src; 32];
        let secret = id.shared_secret(&peer).unwrap();
        let dest = id.pub_key()[0] ^ 0xFF;
        let msg = TxtMsgPacket::new(
            dest,
            src,
            1234,
            meshcore_wire::packets::txtmsg::TXT_TYPE_PLAIN,
            0,
            "hello",
            &peer,
        )
        .unwrap();
        let pkt = flood(msg.to_packet(ROUTE_TYPE_FLOOD, &secret).unwrap());

        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &TestContext);
        engine.allow_forward = true;

        assert!(matches!(
            engine.handle_inbound_packet(pkt, 6),
            InboundAction::Unhandled
        ));
        assert!(engine
            .tx_queue
            .iter()
            .any(|q| q.lane == TxLane::Flood
                && q.packet.payload_type() == PAYLOAD_TYPE_TXT_MSG));
    }

    #[test]
    fn flooded_group_text_is_still_forwarded() {
        let id = identity();
        let channels = TestContext.channels();
        let channel = &channels[0];
        let channel_hash = meshcore_wire::crypto::channel_hash(&channel.secret);
        let group = GrpTextPacket::new(
            channel_hash,
            "other",
            "hello",
            1234,
            GrpTextPacket::TXT_TYPE_PLAIN,
            0,
        );
        let pkt = flood(group.to_packet(&channel.secret, ROUTE_TYPE_FLOOD).unwrap());

        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &TestContext);
        engine.allow_forward = true;

        assert!(matches!(
            engine.handle_inbound_packet(pkt, 6),
            InboundAction::ReceivedGroupText { .. }
        ));
        assert!(engine
            .tx_queue
            .iter()
            .any(|q| q.lane == TxLane::Flood
                && q.packet.payload_type() == PAYLOAD_TYPE_GRP_TXT));
    }

    struct MultiAckContext {
        extra_acks: u8,
        out_path_len: u8,
        out_path: [u8; 64],
    }

    impl EngineContext for MultiAckContext {
        fn extra_ack_count(&self) -> u8 {
            self.extra_acks
        }

        fn peers_by_hash(&self, hash: u8) -> heapless::Vec<[u8; 32], 32> {
            let mut peers = heapless::Vec::new();
            let _ = peers.push([hash; 32]);
            peers
        }

        fn peer_path(&self, _pub_key: &[u8; 32]) -> (u8, [u8; 64]) {
            (self.out_path_len, self.out_path)
        }
    }

    fn make_direct_txt_to(id: &LocalIdentity, src: u8, text: &str) -> Packet {
        let peer = [src; 32];
        let secret = id.shared_secret(&peer).unwrap();
        let msg = TxtMsgPacket::new(
            id.pub_key()[0],
            src,
            1234,
            meshcore_wire::packets::txtmsg::TXT_TYPE_PLAIN,
            0,
            text,
            &peer,
        )
        .unwrap();
        let mut pkt = msg.to_packet(ROUTE_TYPE_DIRECT, &secret).unwrap();
        pkt.set_path_hash_size_and_count(1, 0);
        pkt
    }

    #[test]
    fn direct_dm_ack_without_multi_acks_emits_single_ack() {
        let id = identity();
        let pkt = make_direct_txt_to(&id, 0x33, "hi");
        let mut out_path = [0u8; 64];
        out_path[0] = 0xAA;
        let ctx = MultiAckContext {
            extra_acks: 0,
            out_path_len: 1,
            out_path,
        };
        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &ctx);

        assert!(matches!(
            engine.handle_inbound_packet(pkt, 6),
            InboundAction::ReceivedDirectText { .. }
        ));
        assert_eq!(engine.tx_queue.len(), 1);
        let q0 = engine.tx_queue.pop().unwrap();
        assert_eq!(q0.lane, TxLane::Control);
        assert_eq!(q0.packet.route_type(), ROUTE_TYPE_DIRECT);
        assert_eq!(q0.packet.payload_type(), PAYLOAD_TYPE_ACK);
        assert_eq!(q0.packet.path_len, 1);
        assert_eq!(q0.packet.path[0], 0xAA);
        assert_eq!(q0.due_at_ms, q0.queued_at_ms);
    }

    #[test]
    fn direct_dm_ack_with_multi_acks_emits_multipart_and_spaced_ack() {
        let id = identity();
        let pkt = make_direct_txt_to(&id, 0x33, "hi");
        let mut out_path = [0u8; 64];
        out_path[0] = 0xAA;
        out_path[1] = 0xBB;
        let ctx = MultiAckContext {
            extra_acks: 1,
            out_path_len: 0x41, // 2-byte hash, 1 hop
            out_path,
        };
        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &ctx);

        assert!(matches!(
            engine.handle_inbound_packet(pkt, 6),
            InboundAction::ReceivedDirectText { .. }
        ));
        assert_eq!(engine.tx_queue.len(), 2);

        let q0 = engine.tx_queue.pop().unwrap();
        assert_eq!(q0.lane, TxLane::Control);
        assert_eq!(q0.packet.route_type(), ROUTE_TYPE_DIRECT);
        assert_eq!(q0.packet.payload_type(), PAYLOAD_TYPE_MULTIPART);
        assert_eq!(q0.packet.path_len, 0x41);
        assert_eq!(&q0.packet.path[..2], &[0xAA, 0xBB]);
        assert_eq!(q0.due_at_ms, q0.queued_at_ms);
        let mp = MultipartPacket::decode(&q0.packet.payload[..q0.packet.payload_len]).unwrap();
        assert_eq!(mp.remaining, 1);
        assert_eq!(mp.inner_type, PAYLOAD_TYPE_ACK);

        let q1 = engine.tx_queue.pop().unwrap();
        assert_eq!(q1.lane, TxLane::Control);
        assert_eq!(q1.packet.route_type(), ROUTE_TYPE_DIRECT);
        assert_eq!(q1.packet.payload_type(), PAYLOAD_TYPE_ACK);
        assert_eq!(q1.packet.path_len, 0x41);
        assert_eq!(&q1.packet.path[..2], &[0xAA, 0xBB]);
        assert_eq!(q1.due_at_ms, q1.queued_at_ms + MULTI_ACK_SPACING_MS);
        assert_eq!(&q1.packet.payload[..q1.packet.payload_len], &mp.body[..]);
    }

    #[test]
    fn flooded_dm_still_emits_path_return_even_with_multi_acks() {
        let id = identity();
        let src = 0x33u8;
        let peer = [src; 32];
        let secret = id.shared_secret(&peer).unwrap();
        let msg = TxtMsgPacket::new(
            id.pub_key()[0],
            src,
            1234,
            meshcore_wire::packets::txtmsg::TXT_TYPE_PLAIN,
            0,
            "flooded",
            &peer,
        )
        .unwrap();
        let pkt = flood(msg.to_packet(ROUTE_TYPE_FLOOD, &secret).unwrap());
        let ctx = MultiAckContext {
            extra_acks: 1,
            out_path_len: 1,
            out_path: [0xAA; 64],
        };
        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &ctx);

        assert!(matches!(
            engine.handle_inbound_packet(pkt, 6),
            InboundAction::ReceivedDirectText { .. }
        ));
        assert_eq!(engine.tx_queue.len(), 1);
        let q0 = engine.tx_queue.pop().unwrap();
        assert_eq!(q0.packet.payload_type(), PAYLOAD_TYPE_PATH);
    }

    #[test]
    fn inbound_multipart_ack_and_trailing_ack_deduplicate() {
        let id = identity();
        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &TestContext);

        let ack = AckPacket::with_extended(0xDEADBEEF, 1, 2);
        let mut ack_bytes = [0u8; 6];
        let alen = ack.encode(&mut ack_bytes).unwrap();
        let mp = MultipartPacket::new(1, PAYLOAD_TYPE_ACK, &ack_bytes[..alen]).unwrap();
        let mut mp_pkt = mp.to_packet(ROUTE_TYPE_DIRECT).unwrap();
        mp_pkt.set_path_hash_size_and_count(1, 0);

        let mut ack_pkt = ack.to_packet(ROUTE_TYPE_DIRECT).unwrap();
        ack_pkt.set_path_hash_size_and_count(1, 0);

        match engine.handle_inbound_packet(mp_pkt, 5) {
            InboundAction::ReceivedAck { ack: got, .. } => assert_eq!(got.crc, 0xDEADBEEF),
            _ => panic!("expected ReceivedAck on first MULTIPART copy"),
        }
        assert!(matches!(
            engine.handle_inbound_packet(ack_pkt, 5),
            InboundAction::DropDuplicate
        ));
    }

    #[test]
    fn direct_forward_multipart_ack_delays_and_deduplicates() {
        let id = identity();
        let self_hash = id.pub_key()[0];
        let ctx = MultiAckContext {
            extra_acks: 1,
            out_path_len: 0xFF,
            out_path: [0u8; 64],
        };
        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &ctx);
        engine.allow_forward = true;

        let ack = AckPacket::with_extended(0xCAFEBABE, 0, 1);
        let mut ack_bytes = [0u8; 6];
        let alen = ack.encode(&mut ack_bytes).unwrap();
        let mp = MultipartPacket::new(1, PAYLOAD_TYPE_ACK, &ack_bytes[..alen]).unwrap();
        let mut mp_pkt = mp.to_packet(ROUTE_TYPE_DIRECT).unwrap();
        mp_pkt.set_path_hash_size_and_count(1, 2);
        mp_pkt.path[0] = self_hash;
        mp_pkt.path[1] = 0x55;

        match engine.handle_inbound_packet(mp_pkt, 5) {
            InboundAction::ReceivedAck { ack: got, .. } => assert_eq!(got.crc, 0xCAFEBABE),
            _ => panic!("expected ReceivedAck when forwarding direct MULTIPART ACK"),
        }
        assert_eq!(engine.tx_queue.len(), 2);

        let q0 = engine.tx_queue.pop().unwrap();
        assert_eq!(q0.lane, TxLane::Control);
        assert_eq!(q0.packet.payload_type(), PAYLOAD_TYPE_MULTIPART);
        assert_eq!(q0.packet.path_hash_count(), 1);
        assert_eq!(q0.packet.path[0], 0x55);
        // remaining = 1 => initial delay = (1 + 1) * 300ms = 600ms
        assert_eq!(q0.due_at_ms, q0.queued_at_ms + 2 * MULTI_ACK_SPACING_MS);

        let q1 = engine.tx_queue.pop().unwrap();
        assert_eq!(q1.lane, TxLane::Control);
        assert_eq!(q1.packet.payload_type(), PAYLOAD_TYPE_ACK);
        assert_eq!(q1.packet.path_hash_count(), 1);
        assert_eq!(q1.packet.path[0], 0x55);
        assert_eq!(q1.due_at_ms, q1.queued_at_ms + 3 * MULTI_ACK_SPACING_MS);

        // Trailing standard ACK from previous hop must not be re-forwarded.
        let mut ack_pkt = ack.to_packet(ROUTE_TYPE_DIRECT).unwrap();
        ack_pkt.set_path_hash_size_and_count(1, 2);
        ack_pkt.path[0] = self_hash;
        ack_pkt.path[1] = 0x55;
        assert!(matches!(
            engine.handle_inbound_packet(ack_pkt, 5),
            InboundAction::DropDuplicate
        ));
        assert!(engine.tx_queue.is_empty());
    }

    #[test]
    fn flooded_req_returns_path_packet_with_response_extra() {
        let id = identity();
        let src = 0x33u8;
        let peer = [src; 32];
        let secret = id.shared_secret(&peer).unwrap();

        let req_plain = [0x01, 0x02, 0x03, 0x04, 0x03, 0x00];
        let req = AddressedPacket::new(id.pub_key()[0], src, &req_plain).unwrap();
        let mut pkt = req.to_packet(PAYLOAD_TYPE_REQ, ROUTE_TYPE_FLOOD, &secret).unwrap();
        pkt.set_path_hash_size_and_count(1, 2);
        pkt.path[0] = 0xAA;
        pkt.path[1] = 0xBB;

        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &TestContext);
        let (sender_pub, is_flood, arrival_path_len, arrival_path) =
            match engine.handle_inbound_packet(pkt, 5) {
                InboundAction::ReceivedRequest {
                    plain,
                    sender_pub,
                    is_flood,
                    arrival_path_len,
                    arrival_path,
                } => {
                    assert_eq!(&plain[..req_plain.len()], &req_plain);
                    assert!(plain[req_plain.len()..].iter().all(|&b| b == 0));
                    (sender_pub, is_flood, arrival_path_len, arrival_path)
                }
                _ => panic!("expected ReceivedRequest"),
            };
        assert!(is_flood);
        assert_eq!(arrival_path_len, 2);
        assert_eq!(arrival_path.as_slice(), &[0xAA, 0xBB]);

        let reply_plain = [0x01, 0x02, 0x03, 0x04, 0x01, 0x74, 0x01, 0x90];
        engine
            .queue_request_reply(
                &sender_pub,
                is_flood,
                arrival_path_len,
                &arrival_path,
                &reply_plain,
            )
            .unwrap();

        let queued = engine.tx_queue.pop().unwrap();
        assert_eq!(queued.packet.route_type(), ROUTE_TYPE_FLOOD);
        assert_eq!(queued.packet.payload_type(), PAYLOAD_TYPE_PATH);
        let decoded = PathPacket::decode(
            &queued.packet.payload[..queued.packet.payload_len],
            &secret,
        )
        .unwrap();
        assert_eq!(decoded.dest, src);
        assert_eq!(decoded.path_len, 2);
        assert_eq!(&decoded.path[..2], &[0xAA, 0xBB]);
        assert_eq!(decoded.extra_type & 0x0F, PAYLOAD_TYPE_RESPONSE);
        assert_eq!(&decoded.extra[..reply_plain.len()], &reply_plain);
        assert!(decoded.extra[reply_plain.len()..].iter().all(|&b| b == 0));
    }

    #[test]
    fn direct_req_returns_addressed_response_along_peer_path() {
        let id = identity();
        let src = 0x33u8;
        let peer = [src; 32];
        let secret = id.shared_secret(&peer).unwrap();

        let req_plain = [0x01, 0x02, 0x03, 0x04, 0x03, 0x00];
        let req = AddressedPacket::new(id.pub_key()[0], src, &req_plain).unwrap();
        let mut pkt = req.to_packet(PAYLOAD_TYPE_REQ, ROUTE_TYPE_DIRECT, &secret).unwrap();
        pkt.set_path_hash_size_and_count(1, 0);

        let mut out_path = [0u8; 64];
        out_path[0] = 0xCC;
        let ctx = MultiAckContext {
            extra_acks: 0,
            out_path_len: 1,
            out_path,
        };
        let mut engine = MeshEngine::new(id, ADV_TYPE_REPEATER, &ctx);
        let (sender_pub, is_flood, arrival_path_len, arrival_path) =
            match engine.handle_inbound_packet(pkt, 5) {
                InboundAction::ReceivedRequest {
                    sender_pub,
                    is_flood,
                    arrival_path_len,
                    arrival_path,
                    ..
                } => (sender_pub, is_flood, arrival_path_len, arrival_path),
                _ => panic!("expected ReceivedRequest"),
            };
        assert!(!is_flood);

        let reply_plain = [0x01, 0x02, 0x03, 0x04, 0x01, 0x74, 0x01, 0x90];
        engine
            .queue_request_reply(
                &sender_pub,
                is_flood,
                arrival_path_len,
                &arrival_path,
                &reply_plain,
            )
            .unwrap();

        let queued = engine.tx_queue.pop().unwrap();
        assert_eq!(queued.packet.route_type(), ROUTE_TYPE_DIRECT);
        assert_eq!(queued.packet.payload_type(), PAYLOAD_TYPE_RESPONSE);
        assert_eq!(queued.packet.path_len, 1);
        assert_eq!(queued.packet.path[0], 0xCC);
        let decoded = AddressedPacket::decode(
            &queued.packet.payload[..queued.packet.payload_len],
            &secret,
        )
        .unwrap();
        assert_eq!(&decoded.plain[..reply_plain.len()], &reply_plain);
        assert!(decoded.plain[reply_plain.len()..].iter().all(|&b| b == 0));
    }
}
