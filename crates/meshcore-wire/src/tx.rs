//! Transmit scheduling helpers.
//!
//! Forwarded flood packets are delayed by a deterministic per-packet, per-node
//! amount so several repeaters do not retransmit in lockstep. The same
//! eligibility clock is used by the radio task to decide when to transmit.

use heapless::Vec;


/// Minimum delay before a flood packet is forwarded.
pub const FLOOD_SPREAD_BASE_MS: u64 = 40;
/// Extra spread added deterministically per packet and node.
pub const FLOOD_SPREAD_WINDOW_MS: u64 = 120;

const FNV_INIT: u32 = 0x811C_9DC5;
const FNV_PRIME: u32 = 0x0100_0193;

/// Deterministic forwarding delay for a packet on this node.
pub fn flood_spread_ms(
    packet_hash: &[u8; 8],
    node_hash: &[u8],
    base_ms: u64,
    window_ms: u64,
) -> u64 {
    if window_ms == 0 {
        return base_ms;
    }
    let mut h = FNV_INIT;
    for &b in packet_hash.iter().chain(node_hash.iter()) {
        h ^= b as u32;
        h = h.wrapping_mul(FNV_PRIME);
    }
    base_ms + (h as u64 % window_ms)
}

/// One scheduled transmission: an opaque payload plus timing metadata.
#[derive(Clone, Debug)]
pub struct PendingTx<T> {
    pub payload: T,
    pub due_at_ms: u64,
    pub queued_at_ms: u64,
    pub is_forward: bool,
    /// Transmission attempts so far (radio-side retries).
    pub attempts: u8,
}

/// Fixed-capacity, unsorted transmit scheduler.
///
/// The radio task keeps requests here until their eligibility time. The queue
/// is small, so scanning for the next due item is cheaper than maintaining an
/// ordered structure across inserts and retries.
pub struct PendingTxQueue<T, const N: usize> {
    items: Vec<PendingTx<T>, N>,
}

impl<T, const N: usize> Default for PendingTxQueue<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> PendingTxQueue<T, N> {
    pub const fn new() -> Self {
        Self { items: Vec::new() }
    }

    pub fn push(&mut self, item: PendingTx<T>) -> Result<(), PendingTx<T>> {
        self.items.push(item)
    }

    pub fn len(&self) -> usize {
        self.items.len()
    }

    pub fn get(&self, index: usize) -> Option<&PendingTx<T>> {
        self.items.get(index)
    }

    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// Index of the earliest request whose eligibility time has passed.
    pub fn next_due_index(&self, now_ms: u64) -> Option<usize> {
        self.items
            .iter()
            .enumerate()
            .filter(|(_, item)| item.due_at_ms <= now_ms)
            .min_by_key(|(_, item)| item.due_at_ms)
            .map(|(index, _)| index)
    }

    /// Earliest eligibility time still in the future.
    pub fn next_due_at(&self, now_ms: u64) -> Option<u64> {
        self.items
            .iter()
            .map(|item| item.due_at_ms)
            .filter(|at| *at > now_ms)
            .min()
    }

    pub fn remove(&mut self, index: usize) -> PendingTx<T> {
        self.items.remove(index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn item(eligible: u64) -> PendingTx<u32> {
        PendingTx {
            payload: 0,
            due_at_ms: eligible,
            queued_at_ms: 0,
            is_forward: true,
            attempts: 0,
        }
    }

    #[test]
    fn flood_spread_is_deterministic_and_bounded() {
        let hash = [1u8; 8];
        let node = [2u8; 4];
        let base = 40;
        let window = 120;
        let first = flood_spread_ms(&hash, &node, base, window);
        assert_eq!(first, flood_spread_ms(&hash, &node, base, window));
        assert!((base..base + window).contains(&first));
        // A different node spreads differently.
        assert_ne!(first, flood_spread_ms(&hash, &[3u8; 4], base, window).max(first));
        // A zero window means a fixed delay.
        assert_eq!(flood_spread_ms(&hash, &node, 25, 0), 25);
    }

    #[test]
    fn scheduler_picks_earliest_due_and_next_due_at() {
        let mut schedule = PendingTxQueue::<u32, 4>::new();
        schedule.push(item(300)).unwrap();
        schedule.push(item(100)).unwrap();
        schedule.push(item(200)).unwrap();

        assert_eq!(schedule.next_due_index(50), None);
        assert_eq!(schedule.next_due_at(50), Some(100));
        assert_eq!(schedule.next_due_index(150), Some(1));
        let due = schedule.remove(1);
        assert_eq!(due.due_at_ms, 100);
        assert_eq!(schedule.next_due_index(150), None);
        assert_eq!(schedule.next_due_at(150), Some(200));
    }

    #[test]
    fn scheduler_is_bounded() {
        let mut schedule = PendingTxQueue::<u32, 1>::new();
        schedule.push(item(0)).unwrap();
        assert!(schedule.push(item(0)).is_err());
    }
}
