//! Packet Transmission Queue & Duplicate Packet Detection
//!
//! Provides:
//! 1. `SeenPacketFilter`: Circular cache of recently observed packet hashes to prevent
//!    broadcast loops and packet floods.
//! 2. `PacketQueue`: Outbound packet FIFO buffer.

use crate::meshcore::packet::Packet;
use heapless::Deque;

pub const SEEN_CACHE_SIZE: usize = 64;
pub const OUTBOUND_QUEUE_SIZE: usize = 8;

/// Fast duplicate packet detector preventing infinite mesh re-broadcasts
pub struct SeenPacketFilter {
    cache: [[u8; 4]; SEEN_CACHE_SIZE],
    index: usize,
    count: usize,
}

impl SeenPacketFilter {
    pub const fn new() -> Self {
        Self {
            cache: [[0u8; 4]; SEEN_CACHE_SIZE],
            index: 0,
            count: 0,
        }
    }

    /// Checks if this packet hash has already been received recently
    pub fn has_seen(&self, hash: &[u8; 4]) -> bool {
        let check_len = self.count.min(SEEN_CACHE_SIZE);
        for i in 0..check_len {
            if &self.cache[i] == hash {
                return true;
            }
        }
        false
    }

    /// Marks a packet hash as seen
    pub fn mark_seen(&mut self, hash: [u8; 4]) {
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

/// Outbound Transmit FIFO Queue
pub struct PacketQueue {
    queue: Deque<Packet, OUTBOUND_QUEUE_SIZE>,
}

impl PacketQueue {
    pub const fn new() -> Self {
        Self {
            queue: Deque::new(),
        }
    }

    pub fn push(&mut self, pkt: Packet) -> Result<(), Packet> {
        self.queue.push_back(pkt)
    }

    pub fn pop(&mut self) -> Option<Packet> {
        self.queue.pop_front()
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }
}

