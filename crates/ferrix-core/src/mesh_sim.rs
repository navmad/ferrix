//! Multi-node DM/ACK round-trip simulation (host tests only).
//!
//! Real `MeshEngine`s with real identities exchange real encoded packets over a
//! deterministic in-memory "air". The single-engine tests in `engine.rs` check
//! one hop of one node at a time against fake peer keys; this harness checks
//! that the pieces still agree end to end: the CRC the sender expects is the
//! CRC the receiver returns, the ACK survives relays and duplicate
//! suppression, and a flooded DM teaches the sender a path.
//!
//! Notes on the model:
//! - The embassy-time mock clock is a process-wide global and stays at zero, so
//!   the harness keeps its own virtual clock. After every engine call the
//!   node's `tx_queue` is drained and each packet is scheduled at
//!   `now + (due_at_ms - queued_at_ms)`, i.e. the delay the engine asked for.
//! - Lane scheduling, duty-cycle budgets and flood-calendar slot jitter belong
//!   to the radio task and are not modelled: forwarded packets go out as soon
//!   as they are queued.
//! - The "app" side (pending-ACK table, path learning, the reciprocal PATH
//!   after a flooded PATH) mirrors `ferrix-runtime/src/app.rs`.

use std::sync::Mutex;

use meshcore_wire::crypto::ack_crc;
use meshcore_wire::identity::LocalIdentity;
use meshcore_wire::packet::{
    Packet, PAYLOAD_TYPE_ACK, PAYLOAD_TYPE_MULTIPART, PAYLOAD_TYPE_PATH, PAYLOAD_TYPE_TXT_MSG,
    ROUTE_TYPE_DIRECT, ROUTE_TYPE_FLOOD,
};
use meshcore_wire::packets::advert::ADV_TYPE_CHAT;
use meshcore_wire::packets::txtmsg::TXT_TYPE_PLAIN;
use meshcore_wire::packets::AckPacket;

use crate::engine::{EngineContext, InboundAction, MeshEngine, TXT_ACK_DELAY_MS};
use crate::scheduler::MULTI_ACK_SPACING_MS;

const NO_PATH: u8 = 0xFF;
const MAX_STEPS: usize = 500;
/// Delay `app.rs` gives the reciprocal PATH it sends after a flooded PATH.
const RECIPROCAL_PATH_DELAY_MS: u64 = 500;

#[derive(Clone)]
struct Contact {
    pub_key: [u8; 32],
    out_path_len: u8,
    out_path: [u8; 64],
}

/// Per-node contact table behind the engine's `EngineContext`.
struct SimContext {
    extra_acks: u8,
    contacts: Mutex<Vec<Contact>>,
    /// ACK CRCs this node's "app" is still waiting for (`PENDING_MESSAGE_ACKS`).
    pending: Mutex<Vec<u32>>,
}

impl SimContext {
    fn add_contact(&self, pub_key: [u8; 32]) {
        self.contacts.lock().unwrap().push(Contact {
            pub_key,
            out_path_len: NO_PATH,
            out_path: [0; 64],
        });
    }

    fn set_path(&self, pub_key: &[u8; 32], len: u8, path: &[u8; 64]) {
        let mut contacts = self.contacts.lock().unwrap();
        let contact = contacts
            .iter_mut()
            .find(|c| &c.pub_key == pub_key)
            .expect("unknown contact");
        contact.out_path_len = len;
        contact.out_path = *path;
    }

    fn path(&self, pub_key: &[u8; 32]) -> (u8, [u8; 64]) {
        self.contacts
            .lock()
            .unwrap()
            .iter()
            .find(|c| &c.pub_key == pub_key)
            .map(|c| (c.out_path_len, c.out_path))
            .unwrap_or((NO_PATH, [0; 64]))
    }
}

impl EngineContext for SimContext {
    fn extra_ack_count(&self) -> u8 {
        self.extra_acks
    }

    fn peers_by_hash(&self, hash: u8) -> heapless::Vec<[u8; 32], 32> {
        let mut peers = heapless::Vec::new();
        for c in self.contacts.lock().unwrap().iter() {
            if c.pub_key[0] == hash {
                let _ = peers.push(c.pub_key);
            }
        }
        peers
    }

    fn peer_path(&self, pub_key: &[u8; 32]) -> (u8, [u8; 64]) {
        self.path(pub_key)
    }

    fn is_pending_ack(&self, crc: u32) -> bool {
        self.pending.lock().unwrap().contains(&crc)
    }
}

struct Node {
    name: &'static str,
    pub_key: [u8; 32],
    ctx: &'static SimContext,
    engine: MeshEngine<'static>,
}

/// What a node's engine reported for one received packet.
#[derive(Clone, Debug, PartialEq)]
enum Ev {
    Ack { node: usize, crc: u32, is_flood: bool },
    Text { node: usize, text: String, flood: bool },
    Path { node: usize, is_flood: bool, ack: Option<u32> },
    Forwarded { node: usize },
    Duplicate { node: usize, ptype: u8 },
    Ignored { node: usize },
}

/// One transmission on the air.
#[derive(Clone, Debug)]
struct TxRec {
    at: u64,
    from: usize,
    ptype: u8,
    route: u8,
    hops: usize,
}

struct InFlight {
    deliver_at: u64,
    seq: u64,
    from: usize,
    pkt: Packet,
}

type DropFn = Box<dyn FnMut(usize, usize, &Packet) -> bool>;

struct Sim {
    nodes: Vec<Node>,
    links: Vec<(usize, usize)>,
    queue: Vec<InFlight>,
    seq: u64,
    now: u64,
    ts: u32,
    drop_fn: Option<DropFn>,
    tx_log: Vec<TxRec>,
    events: Vec<Ev>,
    /// `(node, crc)` for every ACK that matched a pending entry.
    confirmed: Vec<(usize, u32)>,
}

impl Sim {
    fn new() -> Self {
        Self {
            nodes: Vec::new(),
            links: Vec::new(),
            queue: Vec::new(),
            seq: 0,
            now: 0,
            ts: 1_700_000_000,
            drop_fn: None,
            tx_log: Vec::new(),
            events: Vec::new(),
            confirmed: Vec::new(),
        }
    }

    /// Add a node. Every node knows every other node's key (a full contact
    /// book) but starts with no path to any of them.
    fn add_node(&mut self, name: &'static str, seed: u8, forward: bool, extra_acks: u8) -> usize {
        let identity = LocalIdentity::from_seed(name, &[seed; 32]);
        let pub_key = identity.pub_key_bytes();
        assert!(
            self.nodes.iter().all(|n| n.pub_key[0] != pub_key[0]),
            "seed {seed} gives {name} the same 1-byte hash as another node; pick another seed"
        );
        let ctx: &'static SimContext = Box::leak(Box::new(SimContext {
            extra_acks,
            contacts: Mutex::new(Vec::new()),
            pending: Mutex::new(Vec::new()),
        }));
        for other in &self.nodes {
            ctx.add_contact(other.pub_key);
            other.ctx.add_contact(pub_key);
        }
        let mut engine = MeshEngine::new(identity, ADV_TYPE_CHAT, ctx);
        engine.allow_forward = forward;
        self.nodes.push(Node {
            name,
            pub_key,
            ctx,
            engine,
        });
        self.nodes.len() - 1
    }

    fn link(&mut self, a: usize, b: usize) {
        self.links.push((a, b));
    }

    /// Give `from` a direct route to `to` through `hops` (1-byte hashes).
    fn set_path(&mut self, from: usize, to: usize, hops: &[usize]) {
        let mut path = [0u8; 64];
        for (i, hop) in hops.iter().enumerate() {
            path[i] = self.nodes[*hop].pub_key[0];
        }
        let to_key = self.nodes[to].pub_key;
        self.nodes[from]
            .ctx
            .set_path(&to_key, hops.len() as u8, &path);
    }

    fn path_len(&self, from: usize, to: usize) -> u8 {
        self.nodes[from].ctx.path(&self.nodes[to].pub_key).0
    }

    fn set_drop(&mut self, f: impl FnMut(usize, usize, &Packet) -> bool + 'static) {
        self.drop_fn = Some(Box::new(f));
    }

    fn clear_drop(&mut self) {
        self.drop_fn = None;
    }

    fn fresh_ts(&mut self) -> u32 {
        self.ts += 1;
        self.ts
    }

    /// Queue a DM the way the companion does and register its expected ACK.
    fn send_dm(&mut self, from: usize, to: usize, text: &str, attempt: u8) -> u32 {
        let ts = self.fresh_ts();
        self.send_dm_at(from, to, ts, text, attempt)
    }

    /// Like [`Self::send_dm`] with an explicit timestamp, so a retry can reuse
    /// the original one as the companion apps do.
    fn send_dm_at(&mut self, from: usize, to: usize, ts: u32, text: &str, attempt: u8) -> u32 {
        let (path_len, path) = self.nodes[from].ctx.path(&self.nodes[to].pub_key);
        let to_key = self.nodes[to].pub_key;
        let from_key = self.nodes[from].pub_key;
        let crc = self.nodes[from]
            .engine
            .queue_direct_text(&to_key, text, ts, TXT_TYPE_PLAIN, attempt, path_len, &path)
            .expect("queue_direct_text");

        // The companion computes the CRC it reports in RESP_CODE_SENT with the
        // same formula (protocol.rs); it must match what the engine uses.
        let flags = ((TXT_TYPE_PLAIN & 0x3F) << 2) | (attempt & 0x03);
        assert_eq!(
            crc,
            ack_crc(ts, flags, text.as_bytes(), &from_key),
            "engine ACK CRC differs from the companion's expected_ack formula"
        );

        self.nodes[from].ctx.pending.lock().unwrap().push(crc);
        self.drain(from);
        crc
    }

    /// Move everything a node's engine queued onto the virtual air.
    fn drain(&mut self, node: usize) {
        while let Some(q) = self.nodes[node].engine.tx_queue.pop() {
            let delay = q.due_at_ms.saturating_sub(q.queued_at_ms);
            self.seq += 1;
            self.queue.push(InFlight {
                deliver_at: self.now + delay,
                seq: self.seq,
                from: node,
                pkt: q.packet,
            });
        }
    }

    /// Run until nothing is left to transmit.
    fn run(&mut self) {
        for _ in 0..MAX_STEPS {
            let Some(idx) = (0..self.queue.len())
                .min_by_key(|&i| (self.queue[i].deliver_at, self.queue[i].seq))
            else {
                return;
            };
            let flight = self.queue.remove(idx);
            self.now = self.now.max(flight.deliver_at);
            self.transmit(flight);
        }
        panic!("simulation did not settle after {MAX_STEPS} transmissions");
    }

    fn transmit(&mut self, flight: InFlight) {
        let mut buf = [0u8; 256];
        let len = flight.pkt.write_to(&mut buf).expect("serialise packet");
        self.tx_log.push(TxRec {
            at: self.now,
            from: flight.from,
            ptype: flight.pkt.payload_type(),
            route: flight.pkt.route_type(),
            hops: flight.pkt.path_hash_count(),
        });

        let neighbours: Vec<usize> = self
            .links
            .iter()
            .filter_map(|&(a, b)| {
                if a == flight.from {
                    Some(b)
                } else if b == flight.from {
                    Some(a)
                } else {
                    None
                }
            })
            .collect();

        for to in neighbours {
            if let Some(f) = self.drop_fn.as_mut() {
                if f(flight.from, to, &flight.pkt) {
                    continue;
                }
            }
            let rx = Packet::read_from(&buf[..len]).expect("parse packet");
            let ptype = rx.payload_type();
            let action = self.nodes[to].engine.handle_inbound_packet(rx, 0);
            self.apply(to, ptype, action);
            self.drain(to);
        }
    }

    /// The part of `app.rs` that reacts to engine actions.
    fn apply(&mut self, node: usize, ptype: u8, action: InboundAction) {
        match action {
            InboundAction::DropDuplicate => self.events.push(Ev::Duplicate { node, ptype }),
            InboundAction::Forwarded => self.events.push(Ev::Forwarded { node }),
            InboundAction::ReceivedAck { ack, is_flood, .. } => {
                self.events.push(Ev::Ack {
                    node,
                    crc: ack.crc,
                    is_flood,
                });
                self.confirm(node, ack.crc);
            }
            InboundAction::ReceivedDirectText { msg, path_len, .. } => {
                self.events.push(Ev::Text {
                    node,
                    text: msg.text.as_str().to_string(),
                    flood: path_len != NO_PATH,
                });
            }
            InboundAction::ReceivedPath {
                packet: path,
                sender_pub,
                is_flood,
                arrival_path_len,
                arrival_path,
            } => {
                let ack = if path.extra_type & 0x0F == 3 && path.extra.len() >= 4 {
                    AckPacket::decode(path.extra.as_slice()).ok().map(|a| a.crc)
                } else {
                    None
                };
                self.events.push(Ev::Path {
                    node,
                    is_flood,
                    ack,
                });
                self.nodes[node]
                    .ctx
                    .set_path(&sender_pub, path.path_len, &path.path);
                if is_flood {
                    let _ = self.nodes[node].engine.queue_path_return_only(
                        &sender_pub,
                        path.path_len,
                        &path.path,
                        arrival_path_len,
                        &arrival_path,
                        RECIPROCAL_PATH_DELAY_MS,
                    );
                }
                if let Some(crc) = ack {
                    self.confirm(node, crc);
                }
            }
            _ => self.events.push(Ev::Ignored { node }),
        }
    }

    /// `handle_direct_message_confirmed`: the first match consumes the entry.
    fn confirm(&mut self, node: usize, crc: u32) {
        let mut pending = self.nodes[node].ctx.pending.lock().unwrap();
        if let Some(pos) = pending.iter().position(|&c| c == crc) {
            pending.remove(pos);
            drop(pending);
            self.confirmed.push((node, crc));
        }
    }

    // ---- assertions helpers -------------------------------------------------

    fn confirmations(&self, node: usize, crc: u32) -> usize {
        self.confirmed
            .iter()
            .filter(|&&(n, c)| n == node && c == crc)
            .count()
    }

    fn tx_by(&self, node: usize, ptype: u8) -> Vec<&TxRec> {
        self.tx_log
            .iter()
            .filter(|t| t.from == node && t.ptype == ptype)
            .collect()
    }

    fn total_tx_by(&self, node: usize) -> usize {
        self.tx_log.iter().filter(|t| t.from == node).count()
    }

    fn texts_at(&self, node: usize) -> Vec<String> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Ev::Text { node: n, text, .. } if *n == node => Some(text.clone()),
                _ => None,
            })
            .collect()
    }

    fn ack_crcs_at(&self, node: usize) -> Vec<u32> {
        self.events
            .iter()
            .filter_map(|e| match e {
                Ev::Ack { node: n, crc, .. } if *n == node => Some(*crc),
                _ => None,
            })
            .collect()
    }

    fn paths_at(&self, node: usize) -> Vec<&Ev> {
        self.events
            .iter()
            .filter(|e| matches!(e, Ev::Path { node: n, .. } if *n == node))
            .collect()
    }

    /// Duplicates of one payload type dropped at `node`. Nodes also hear their
    /// own ACK/PATH echoed back by a repeater and drop it, so tests ask about
    /// the packet type they care about.
    fn duplicates_at(&self, node: usize, ptype: u8) -> usize {
        self.events
            .iter()
            .filter(|e| matches!(e, Ev::Duplicate { node: n, ptype: p } if *n == node && *p == ptype))
            .count()
    }

    fn name(&self, node: usize) -> &'static str {
        self.nodes[node].name
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 1. A direct DM over a known one-hop route is ACKed once, with the CRC
    ///    the sender computed up front.
    #[test]
    fn direct_dm_is_acked_with_the_expected_crc() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let b = s.add_node("B", 2, false, 0);
        s.link(a, b);
        s.set_path(a, b, &[]);
        s.set_path(b, a, &[]);

        let crc = s.send_dm(a, b, "hello", 0);
        s.run();

        assert_eq!(s.texts_at(b), ["hello"]);
        let acks = s.tx_by(b, PAYLOAD_TYPE_ACK);
        assert_eq!(acks.len(), 1, "{} should send exactly one ACK", s.name(b));
        assert_eq!(acks[0].route, ROUTE_TYPE_DIRECT);
        // Upstream TXT_ACK_DELAY (BaseChatMesh.cpp): pinned as a literal so the
        // test cannot drift together with the constant.
        assert_eq!(acks[0].at, 200);
        assert_eq!(TXT_ACK_DELAY_MS, 200);
        assert_eq!(s.ack_crcs_at(a), [crc]);
        assert_eq!(s.confirmations(a, crc), 1);
    }

    /// 2. With no known path the DM floods, the receiver answers with a PATH
    ///    that carries the ACK, the sender learns the route (and tells the
    ///    receiver its own), and the next DM goes direct and is ACKed direct.
    #[test]
    fn flooded_dm_learns_a_path_then_goes_direct() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let b = s.add_node("B", 2, false, 0);
        s.link(a, b);
        assert_eq!(s.path_len(a, b), NO_PATH);

        let crc1 = s.send_dm(a, b, "first", 0);
        s.run();

        let dm = s.tx_by(a, PAYLOAD_TYPE_TXT_MSG);
        assert_eq!(dm[0].route, ROUTE_TYPE_FLOOD);
        assert_eq!(s.texts_at(b), ["first"]);
        // The receiver answers with PATH+ACK, not a bare ACK.
        assert!(s.tx_by(b, PAYLOAD_TYPE_ACK).is_empty());
        assert_eq!(s.tx_by(b, PAYLOAD_TYPE_PATH).len(), 1);
        assert_eq!(s.confirmations(a, crc1), 1);
        assert!(
            s.events.contains(&Ev::Path {
                node: a,
                is_flood: true,
                ack: Some(crc1)
            }),
            "sender should see a flooded PATH carrying the ACK"
        );
        // Both sides now have a (zero-hop) route to each other.
        assert_eq!(s.path_len(a, b), 0);
        assert_eq!(s.path_len(b, a), 0, "reciprocal PATH should teach the receiver");

        let crc2 = s.send_dm(a, b, "second", 0);
        s.run();

        let dm = s.tx_by(a, PAYLOAD_TYPE_TXT_MSG);
        assert_eq!(dm.len(), 2);
        assert_eq!(dm[1].route, ROUTE_TYPE_DIRECT);
        let acks = s.tx_by(b, PAYLOAD_TYPE_ACK);
        assert_eq!(acks.len(), 1);
        assert_eq!(acks[0].route, ROUTE_TYPE_DIRECT);
        assert_eq!(s.confirmations(a, crc2), 1);
    }

    /// 3. A -- R -- B with a repeater in the middle: the DM is relayed, and the
    ///    ACK comes back through the same repeater.
    #[test]
    fn direct_dm_and_ack_cross_a_repeater() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let r = s.add_node("R", 3, true, 0);
        let b = s.add_node("B", 2, false, 0);
        s.link(a, r);
        s.link(r, b);
        s.set_path(a, b, &[r]);
        s.set_path(b, a, &[r]);

        let crc = s.send_dm(a, b, "via R", 0);
        s.run();

        assert_eq!(s.texts_at(b), ["via R"]);
        assert_eq!(s.tx_by(r, PAYLOAD_TYPE_TXT_MSG).len(), 1, "R relays the DM once");
        assert_eq!(s.tx_by(r, PAYLOAD_TYPE_TXT_MSG)[0].hops, 0, "R strips its own hop");
        assert_eq!(s.tx_by(b, PAYLOAD_TYPE_ACK).len(), 1);
        assert_eq!(s.tx_by(b, PAYLOAD_TYPE_ACK)[0].hops, 1, "B addresses the ACK to R");
        assert_eq!(s.tx_by(r, PAYLOAD_TYPE_ACK).len(), 1, "R relays the ACK once");
        assert_eq!(s.confirmations(a, crc), 1);
        assert_eq!(s.total_tx_by(a), 1, "A only sends the original DM");
    }

    /// 4. The ACK is lost, the sender retries with attempt + 1. The retry has a
    ///    different ciphertext and CRC, so the receiver's duplicate filter lets
    ///    it through and its ACK is the one that confirms.
    #[test]
    fn retry_after_lost_ack_is_not_dropped_as_a_duplicate() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let b = s.add_node("B", 2, false, 0);
        s.link(a, b);
        s.set_path(a, b, &[]);
        s.set_path(b, a, &[]);
        s.set_drop(move |from, _to, pkt| from == b && pkt.payload_type() == PAYLOAD_TYPE_ACK);

        let ts = s.fresh_ts();
        let crc0 = s.send_dm_at(a, b, ts, "retry me", 0);
        s.run();
        assert_eq!(s.texts_at(b), ["retry me"]);
        assert_eq!(s.confirmations(a, crc0), 0, "the first ACK was lost");

        s.clear_drop();
        let crc1 = s.send_dm_at(a, b, ts, "retry me", 1);
        assert_ne!(crc0, crc1, "attempt is part of the ACK CRC");
        s.run();

        assert_eq!(s.texts_at(b), ["retry me", "retry me"]);
        assert_eq!(
            s.duplicates_at(b, PAYLOAD_TYPE_TXT_MSG),
            0,
            "the retry must not look like a duplicate"
        );
        assert_eq!(s.confirmations(a, crc1), 1);
        assert_eq!(s.confirmations(a, crc0), 0, "the lost ACK never shows up");
    }

    /// 5a. multi_acks: when the trailing ACK is lost the MULTIPART copy alone
    ///     confirms delivery.
    #[test]
    fn multipart_ack_confirms_when_the_trailing_ack_is_lost() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let b = s.add_node("B", 2, false, 1);
        s.link(a, b);
        s.set_path(a, b, &[]);
        s.set_path(b, a, &[]);
        s.set_drop(move |from, _to, pkt| from == b && pkt.payload_type() == PAYLOAD_TYPE_ACK);

        let crc = s.send_dm(a, b, "multi", 0);
        s.run();

        assert_eq!(s.tx_by(b, PAYLOAD_TYPE_MULTIPART).len(), 1);
        assert_eq!(s.tx_by(b, PAYLOAD_TYPE_ACK).len(), 1, "sent, but lost on the air");
        assert_eq!(s.ack_crcs_at(a), [crc]);
        assert_eq!(s.confirmations(a, crc), 1);
    }

    /// 5b. multi_acks without loss: MULTIPART first, the ACK 300 ms later, and
    ///     the sender treats the second as a duplicate of the first.
    #[test]
    fn multipart_and_trailing_ack_deduplicate_at_the_sender() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let b = s.add_node("B", 2, false, 1);
        s.link(a, b);
        s.set_path(a, b, &[]);
        s.set_path(b, a, &[]);

        let crc = s.send_dm(a, b, "multi", 0);
        s.run();

        let mp = s.tx_by(b, PAYLOAD_TYPE_MULTIPART);
        let ack = s.tx_by(b, PAYLOAD_TYPE_ACK);
        assert_eq!((mp.len(), ack.len()), (1, 1));
        // Upstream sendAckTo: copy at +200 ms, trailing ACK 300 ms later.
        assert_eq!(mp[0].at, 200);
        assert_eq!(ack[0].at, 500);
        assert_eq!(MULTI_ACK_SPACING_MS, 300);
        assert_eq!(s.ack_crcs_at(a), [crc], "only one copy reaches the app");
        assert_eq!(s.duplicates_at(a, PAYLOAD_TYPE_ACK), 1);
        assert_eq!(s.confirmations(a, crc), 1);
    }

    /// 6. A can hear B directly as well as through R. B overhears nothing useful
    ///    from A's first transmission (R is the next hop), so it must still
    ///    accept the relayed copy; A hears B's ACK early and must not relay it.
    #[test]
    fn overheard_traffic_neither_blocks_delivery_nor_is_forwarded() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let r = s.add_node("R", 3, true, 0);
        let b = s.add_node("B", 2, false, 0);
        s.link(a, r);
        s.link(r, b);
        s.link(a, b);
        s.set_path(a, b, &[r]);
        s.set_path(b, a, &[r]);

        let crc = s.send_dm(a, b, "overheard", 0);
        s.run();

        assert_eq!(s.texts_at(b), ["overheard"], "B must accept the relayed copy once");
        assert_eq!(s.duplicates_at(b, PAYLOAD_TYPE_TXT_MSG), 0);
        // The early copy (straight from B) and the relayed copy both reach A,
        // but the app only confirms once.
        let acks = s.ack_crcs_at(a);
        assert!(!acks.is_empty() && acks.iter().all(|&c| c == crc));
        assert_eq!(s.confirmations(a, crc), 1);
        assert_eq!(s.total_tx_by(a), 1, "A never forwards what it overhears");
    }

    /// 8. A repeater that hears a direct DM and its ACK but is not on the path
    ///    stays silent. Relaying traffic it was not addressed on would waste
    ///    airtime and put extra copies of the ACK on the air.
    #[test]
    fn bystander_repeater_does_not_relay_direct_traffic() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let r1 = s.add_node("R1", 3, true, 0);
        let r2 = s.add_node("R2", 4, true, 0);
        let b = s.add_node("B", 2, false, 0);
        s.link(a, r1);
        s.link(r1, b);
        // R2 hears everyone but the route goes through R1.
        s.link(a, r2);
        s.link(r2, b);
        s.link(r1, r2);
        s.set_path(a, b, &[r1]);
        s.set_path(b, a, &[r1]);

        let crc = s.send_dm(a, b, "not via R2", 0);
        s.run();

        assert_eq!(s.texts_at(b), ["not via R2"]);
        assert_eq!(s.confirmations(a, crc), 1);
        assert_eq!(
            s.total_tx_by(r2),
            0,
            "R2 is not on the path and must not transmit"
        );
        assert_eq!(s.tx_by(r1, PAYLOAD_TYPE_TXT_MSG).len(), 1);
        assert_eq!(s.tx_by(r1, PAYLOAD_TYPE_ACK).len(), 1);
    }

    /// 7. Two repeaters in parallel: the flooded DM arrives twice and both PATH
    ///    replies come back, but each side dedupes, so the receiver replies once
    ///    and the sender confirms once.
    #[test]
    fn flood_over_two_repeaters_is_deduplicated_end_to_end() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, false, 0);
        let r1 = s.add_node("R1", 3, true, 0);
        let r2 = s.add_node("R2", 4, true, 0);
        let b = s.add_node("B", 2, false, 0);
        s.link(a, r1);
        s.link(a, r2);
        s.link(r1, b);
        s.link(r2, b);

        let crc = s.send_dm(a, b, "diamond", 0);
        s.run();

        assert_eq!(s.texts_at(b), ["diamond"], "B delivers the DM once");
        assert_eq!(
            s.duplicates_at(b, PAYLOAD_TYPE_TXT_MSG),
            1,
            "the second flooded copy is a duplicate"
        );
        assert_eq!(s.tx_by(b, PAYLOAD_TYPE_PATH).len(), 1, "B answers once");
        assert_eq!(s.paths_at(a).len(), 1, "the second PATH is a duplicate at A");
        assert_eq!(s.confirmations(a, crc), 1);
        assert_eq!(s.path_len(a, b), 1, "A learned a one-hop route");
    }

    /// 10. Upstream `markDoNotRetransmit`: a flooded ACK that confirms our own
    ///     send is consumed. B has no route back to A, so it floods the ACK.
    ///     A (which also forwards) must not re-flood it, while an unrelated
    ///     forwarding node R still relays it.
    #[test]
    fn matched_flood_ack_is_not_relayed_but_others_are() {
        let mut s = Sim::new();
        let a = s.add_node("A", 1, true, 0);
        let b = s.add_node("B", 2, false, 0);
        let r = s.add_node("R", 3, true, 0);
        s.link(a, b);
        s.link(b, r);
        s.set_path(a, b, &[]); // A -> B direct; B has no path back to A

        let crc = s.send_dm(a, b, "ping", 0);
        s.run();

        assert_eq!(s.texts_at(b), ["ping"]);
        let acks = s.tx_by(b, PAYLOAD_TYPE_ACK);
        assert_eq!(acks.len(), 1);
        assert_eq!(acks[0].route, ROUTE_TYPE_FLOOD, "B has no path, so it floods");
        assert_eq!(s.confirmations(a, crc), 1);
        assert_eq!(
            s.tx_by(a, PAYLOAD_TYPE_ACK).len(),
            0,
            "A consumed its own ACK and must not re-flood it"
        );
        assert_eq!(
            s.tx_by(r, PAYLOAD_TYPE_ACK).len(),
            1,
            "a node with no matching send keeps relaying ACKs"
        );
    }
}
