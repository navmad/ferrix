//! Packet Transmission Queue & Duplicate Packet Detection
//!
//! Provides:
//! 1. `SeenPacketFilter`: Circular cache of recently observed packet hashes to prevent
//!    broadcast loops and packet floods.
//! 2. `PacketQueue`: Outbound packet FIFO buffer.

use crate::packet::Packet;
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

/// An outbound packet with the timing metadata used by the transmit
/// scheduler. `eligible_at_ms` is on the same monotonic millisecond clock as
/// the radio task; `enqueued_at_ms` is used for wait-time telemetry.
#[derive(Clone, Debug)]
pub struct QueuedPacket {
    pub packet: Packet,
    pub eligible_at_ms: u64,
    pub enqueued_at_ms: u64,
    pub is_forward: bool,
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

    /// Queue a locally produced packet for immediate transmission.
    pub fn push(&mut self, pkt: Packet, now_ms: u64) -> Result<(), QueuedPacket> {
        self.push_at(pkt, now_ms, now_ms, false)
    }

    /// Queue a forwarded flood packet with deterministic spreading delay.
    pub fn push_forward(
        &mut self,
        pkt: Packet,
        now_ms: u64,
        node_hash: &[u8],
    ) -> Result<(), QueuedPacket> {
        let delay = crate::tx::forward_delay_ms(&pkt.calculate_packet_hash(), node_hash);
        self.push_at(pkt, now_ms.saturating_add(delay), now_ms, true)
    }

    pub fn push_at(
        &mut self,
        pkt: Packet,
        eligible_at_ms: u64,
        enqueued_at_ms: u64,
        is_forward: bool,
    ) -> Result<(), QueuedPacket> {
        self.queue.push_back(QueuedPacket {
            packet: pkt,
            eligible_at_ms,
            enqueued_at_ms,
            is_forward,
        })
    }

    pub fn pop(&mut self) -> Option<QueuedPacket> {
        self.queue.pop_front()
    }

    /// Peek at the next packet without removing it.
    pub fn front(&self) -> Option<&QueuedPacket> {
        self.queue.front()
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }
}

