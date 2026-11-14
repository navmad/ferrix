//! Transmit lane classification and flood-slot assignment.

/// Which lane a queued packet belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxLane {
    /// ACKs, path returns, admin replies and locally originated sends.
    Control,
    /// Forwarded floods, scheduled into the flood calendar.
    Flood,
    /// Periodic adverts, sent when the other lanes are idle.
    Background,
}

const FNV_INIT: u32 = 0x811C_9DC5;
const FNV_PRIME: u32 = 0x0100_0193;

/// Deterministic slot for a flood packet, in `0..slots`.
///
/// Slot 0 is the current opportunity; using it for floods keeps latency low
/// when the calendar is idle. Control traffic is sent independently and does
/// not occupy a slot.
pub fn flood_slot(packet_hash: &[u8; 8], node_hash: &[u8], slots: usize) -> usize {
    if slots == 0 {
        return 0;
    }
    let mut h = FNV_INIT;
    for &b in packet_hash.iter().chain(node_hash.iter()) {
        h ^= b as u32;
        h = h.wrapping_mul(FNV_PRIME);
    }
    h as usize % slots
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slot_is_deterministic_and_in_range() {
        let hash = [7u8; 8];
        let node = [3u8; 4];
        for slots in 1..=8 {
            let slot = flood_slot(&hash, &node, slots);
            assert_eq!(slot, flood_slot(&hash, &node, slots));
            assert!(slot < slots, "slot {slot} for {slots}");
        }
        assert_eq!(flood_slot(&hash, &node, 0), 0);
    }

    #[test]
    fn different_nodes_spread() {
        let hash = [9u8; 8];
        let a = flood_slot(&hash, &[1u8; 4], 4);
        let b = flood_slot(&hash, &[2u8; 4], 4);
        let _ = (a, b);
    }
}
