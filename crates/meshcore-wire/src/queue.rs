//! Packet Transmission Queue & Duplicate Packet Detection
//!
//! Provides:
//! 1. `SeenPacketFilter`: Circular cache of recently observed packet hashes to prevent
//!    broadcast loops and packet floods.
//! 2. `PacketQueue`: Outbound packet FIFO buffer.

use crate::packet::Packet;
use crate::tx::TxLane;
use heapless::Deque;

/// Matches the C++ `SimpleMeshTables` ring: 160 eight-byte hashes.
pub const SEEN_CACHE_SIZE: usize = 160;
pub const OUTBOUND_QUEUE_SIZE: usize = 8;

/// Fast duplicate packet detector preventing infinite mesh re-broadcasts
pub struct SeenPacketFilter {
    cache: [[u8; 8]; SEEN_CACHE_SIZE],
    index: usize,
    count: usize,
}

impl SeenPacketFilter {
    pub const fn new() -> Self {
        Self {
            cache: [[0u8; 8]; SEEN_CACHE_SIZE],
            index: 0,
            count: 0,
        }
    }

    /// Checks if this packet hash has already been received recently
    pub fn has_seen(&self, hash: &[u8; 8]) -> bool {
        let check_len = self.count.min(SEEN_CACHE_SIZE);
        for i in 0..check_len {
            if &self.cache[i] == hash {
                return true;
            }
        }
        false
    }

    /// Marks a packet hash as seen
    pub fn mark_seen(&mut self, hash: [u8; 8]) {
        self.cache[self.index] = hash;
        self.index = (self.index + 1) % SEEN_CACHE_SIZE;
        if self.count < SEEN_CACHE_SIZE {
            self.count += 1;
        }
    }

    /// Convenience: check and mark seen in one call. Returns true if duplicate.
    pub fn check_and_mark(&mut self, pkt: &Packet) -> bool {
        let h = pkt.calculate_packet_hash();
        if self.has_seen(&h) {
            true
        } else {
            self.mark_seen(h);
            false
        }
    }
}

/// An outbound packet with its lane and enqueue time.
#[derive(Clone, Debug)]
pub struct QueuedPacket {
    pub packet: Packet,
    pub lane: TxLane,
    pub queued_at_ms: u64,
}

/// Outbound Transmit FIFO Queue
pub struct PacketQueue {
    queue: Deque<QueuedPacket, OUTBOUND_QUEUE_SIZE>,
}

impl PacketQueue {
    pub const fn new() -> Self {
        Self {
            queue: Deque::new(),
        }
    }

    pub fn push(&mut self, pkt: Packet, now_ms: u64, lane: TxLane) -> Result<(), QueuedPacket> {
        self.queue.push_back(QueuedPacket {
            packet: pkt,
            lane,
            queued_at_ms: now_ms,
        })
    }

    /// ACKs, path returns, admin replies and locally originated sends.
    pub fn push_control(&mut self, pkt: Packet, now_ms: u64) -> Result<(), QueuedPacket> {
        self.push(pkt, now_ms, TxLane::Control)
    }

    /// Forwarded floods, scheduled by the radio's flood calendar.
    pub fn push_flood(&mut self, pkt: Packet, now_ms: u64) -> Result<(), QueuedPacket> {
        self.push(pkt, now_ms, TxLane::Flood)
    }

    /// Periodic adverts, sent when control and flood work is idle.
    pub fn push_background(&mut self, pkt: Packet, now_ms: u64) -> Result<(), QueuedPacket> {
        self.push(pkt, now_ms, TxLane::Background)
    }

    pub fn clear(&mut self) {
        self.queue.clear();
    }

    /// Keep only the entries for which `keep` returns true.
    pub fn retain(&mut self, mut keep: impl FnMut(&QueuedPacket) -> bool) {
        let mut kept: Deque<QueuedPacket, OUTBOUND_QUEUE_SIZE> = Deque::new();
        while let Some(item) = self.queue.pop_front() {
            if keep(&item) {
                let _ = kept.push_back(item);
            }
        }
        self.queue = kept;
    }

    pub fn pop(&mut self) -> Option<QueuedPacket> {
        self.queue.pop_front()
    }

    pub fn front(&self) -> Option<&QueuedPacket> {
        self.queue.front()
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }

    pub fn iter(&self) -> impl Iterator<Item = &QueuedPacket> {
        self.queue.iter()
    }
}
