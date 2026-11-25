//! Bounded LRU cache for X25519 ECDH shared secrets.
//!
//! Each entry maps a peer's 32-byte Ed25519 public key to the 32-byte X25519
//! shared secret derived against the active local private scalar (`prv[..32]`).
//! Keeping this cache separate from [`crate::types::DiscoveredContact`] avoids
//! reserving 32 bytes for every passively discovered contact slot while still
//! covering active DMs, path returns, requests, responses, and anonymous peers.
//!
//! X25519 scalar multiplication always runs **outside** the critical section
//! so radio and BLE interrupts are never masked during crypto work.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

/// Maximum number of peer shared secrets retained concurrently (~4.2 KiB RAM).
pub const CACHE_CAPACITY: usize = 64;

const _: () = assert!(CACHE_CAPACITY > 0 && CACHE_CAPACITY <= 255);

#[derive(Clone, Copy)]
struct CacheEntry {
    peer_pub: [u8; 32],
    secret: [u8; 32],
}

impl CacheEntry {
    const EMPTY: Self = Self {
        peer_pub: [0u8; 32],
        secret: [0u8; 32],
    };
}

struct SharedSecretCache {
    initialized: bool,
    order_initialized: bool,
    owner_scalar: [u8; 32],
    entries: [CacheEntry; CACHE_CAPACITY],
    /// Permutation of `0..CACHE_CAPACITY` once `order_initialized` is true:
    /// - `order[0..len]` are active entries from MRU (`0`) to LRU (`len - 1`).
    /// - `order[len..CACHE_CAPACITY]` are free slots.
    order: [u8; CACHE_CAPACITY],
    len: usize,
}

impl SharedSecretCache {
    const fn new() -> Self {
        Self {
            initialized: false,
            order_initialized: false,
            owner_scalar: [0u8; 32],
            entries: [CacheEntry::EMPTY; CACHE_CAPACITY],
            order: [0u8; CACHE_CAPACITY],
            len: 0,
        }
    }

    fn clear_active(&mut self) {
        if self.order_initialized {
            for &slot in &self.order[..self.len] {
                self.entries[slot as usize] = CacheEntry::EMPTY;
            }
        }
        self.len = 0;
        self.owner_scalar = [0u8; 32];
        self.initialized = false;
    }

    fn lookup(&mut self, scalar: &[u8; 32], peer: &[u8; 32]) -> Option<[u8; 32]> {
        if !self.initialized || self.owner_scalar != *scalar {
            return None;
        }
        for pos in 0..self.len {
            let slot = self.order[pos];
            let entry = &self.entries[slot as usize];
            if entry.peer_pub[0] == peer[0] && entry.peer_pub == *peer {
                let secret = entry.secret;
                if pos > 0 {
                    self.order.copy_within(0..pos, 1);
                    self.order[0] = slot;
                }
                return Some(secret);
            }
        }
        None
    }

    fn insert(&mut self, scalar: &[u8; 32], peer: &[u8; 32], secret: [u8; 32]) {
        if !self.order_initialized {
            for (i, slot) in self.order.iter_mut().enumerate() {
                *slot = i as u8;
            }
            self.order_initialized = true;
        }
        if !self.initialized || self.owner_scalar != *scalar {
            self.clear_active();
            self.owner_scalar = *scalar;
            self.initialized = true;
        } else {
            // Avoid duplicates if two callers raced on the same peer.
            for pos in 0..self.len {
                let slot = self.order[pos];
                if self.entries[slot as usize].peer_pub == *peer {
                    self.entries[slot as usize].secret = secret;
                    if pos > 0 {
                        self.order.copy_within(0..pos, 1);
                        self.order[0] = slot;
                    }
                    return;
                }
            }
        }

        let slot = if self.len < CACHE_CAPACITY {
            let slot = self.order[self.len];
            self.order.copy_within(0..self.len, 1);
            self.order[0] = slot;
            self.len += 1;
            slot
        } else {
            let slot = self.order[CACHE_CAPACITY - 1];
            self.order.copy_within(0..CACHE_CAPACITY - 1, 1);
            self.order[0] = slot;
            slot
        };

        self.entries[slot as usize] = CacheEntry {
            peer_pub: *peer,
            secret,
        };
    }

    fn evict_peer(&mut self, peer: &[u8; 32]) {
        if !self.initialized {
            return;
        }
        for pos in 0..self.len {
            let slot = self.order[pos];
            if self.entries[slot as usize].peer_pub == *peer {
                self.entries[slot as usize] = CacheEntry::EMPTY;
                self.order.copy_within(pos + 1..self.len, pos);
                self.len -= 1;
                self.order[self.len] = slot;
                return;
            }
        }
    }
}

static CACHE: Mutex<CriticalSectionRawMutex, RefCell<SharedSecretCache>> =
    Mutex::new(RefCell::new(SharedSecretCache::new()));

static CACHE_HITS: AtomicU32 = AtomicU32::new(0);
static CACHE_MISSES: AtomicU32 = AtomicU32::new(0);

/// Return the cached X25519 shared secret for `(prv, peer)`, computing and
/// caching it on miss.
///
/// Automatically resets the cache if `prv[..32]` differs from the scalar that
/// populated existing entries. Returns `None` if `peer` is not a valid
/// compressed Ed25519 public key.
pub fn shared_secret(prv: &[u8; 64], peer: &[u8; 32]) -> Option<[u8; 32]> {
    let mut scalar = [0u8; 32];
    scalar.copy_from_slice(&prv[..32]);

    if let Some(secret) = CACHE.lock(|c| c.borrow_mut().lookup(&scalar, peer)) {
        CACHE_HITS.fetch_add(1, Ordering::Relaxed);
        return Some(secret);
    }

    // Run X25519 scalar multiplication outside the critical section.
    let secret = meshcore_wire::identity::compute_shared_secret(prv, peer)?;

    CACHE_MISSES.fetch_add(1, Ordering::Relaxed);
    CACHE.lock(|c| c.borrow_mut().insert(&scalar, peer, secret));
    Some(secret)
}

/// Clear and zero every cached shared secret.
///
/// Called when the local private key changes or all contacts are cleared.
pub fn invalidate() {
    CACHE.lock(|c| c.borrow_mut().clear_active());
}

/// Remove and zero the cached shared secret for a single peer, if present.
pub fn evict_peer(peer: &[u8; 32]) {
    CACHE.lock(|c| c.borrow_mut().evict_peer(peer));
}

/// Cumulative `(hits, misses)` since boot.
pub fn stats() -> (u32, u32) {
    (
        CACHE_HITS.load(Ordering::Relaxed),
        CACHE_MISSES.load(Ordering::Relaxed),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::TEST_LOCK;
    use meshcore_wire::identity::LocalIdentity;

    fn reset_all() {
        invalidate();
        CACHE_HITS.store(0, Ordering::Relaxed);
        CACHE_MISSES.store(0, Ordering::Relaxed);
    }

    fn peer_identity(seed: u16) -> LocalIdentity {
        let mut buf = [0u8; 32];
        buf[..2].copy_from_slice(&seed.to_le_bytes());
        buf[2] = 0xA5;
        LocalIdentity::from_seed("peer", &buf)
    }

    #[test]
    fn caches_hits_and_matches_direct_ecdh() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_all();

        let local = LocalIdentity::from_seed("local", &[0x11; 32]);
        let peer = peer_identity(1);
        let expected = local.shared_secret(peer.pub_key()).unwrap();

        let s1 = shared_secret(local.private_key(), peer.pub_key()).unwrap();
        assert_eq!(s1, expected);
        assert_eq!(stats(), (0, 1));

        let s2 = shared_secret(local.private_key(), peer.pub_key()).unwrap();
        assert_eq!(s2, expected);
        assert_eq!(stats(), (1, 1));
    }

    #[test]
    fn invalidates_automatically_on_private_key_change() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_all();

        let local_a = LocalIdentity::from_seed("a", &[0x11; 32]);
        let local_b = LocalIdentity::from_seed("b", &[0x22; 32]);
        let peer = peer_identity(2);

        let sa = shared_secret(local_a.private_key(), peer.pub_key()).unwrap();
        assert_eq!(sa, local_a.shared_secret(peer.pub_key()).unwrap());
        assert_eq!(stats(), (0, 1));

        // Different local private key must miss, recompute, and return the new secret.
        let sb = shared_secret(local_b.private_key(), peer.pub_key()).unwrap();
        assert_eq!(sb, local_b.shared_secret(peer.pub_key()).unwrap());
        assert_ne!(sa, sb);
        assert_eq!(stats(), (0, 2));

        // Second lookup with local_b hits.
        assert_eq!(shared_secret(local_b.private_key(), peer.pub_key()).unwrap(), sb);
        assert_eq!(stats(), (1, 2));
    }

    #[test]
    fn explicit_invalidate_and_evict_peer_clear_entries() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_all();

        let local = LocalIdentity::from_seed("local", &[0x33; 32]);
        let p1 = peer_identity(10);
        let p2 = peer_identity(11);

        let _ = shared_secret(local.private_key(), p1.pub_key()).unwrap();
        let _ = shared_secret(local.private_key(), p2.pub_key()).unwrap();
        assert_eq!(stats(), (0, 2));

        // Evicting p1 leaves p2 cached.
        evict_peer(p1.pub_key());
        let _ = shared_secret(local.private_key(), p2.pub_key()).unwrap();
        assert_eq!(stats(), (1, 2));
        let _ = shared_secret(local.private_key(), p1.pub_key()).unwrap();
        assert_eq!(stats(), (1, 3));

        // Full invalidate clears both and preserves slot permutation invariant.
        invalidate();
        let _ = shared_secret(local.private_key(), p1.pub_key()).unwrap();
        let _ = shared_secret(local.private_key(), p2.pub_key()).unwrap();
        assert_eq!(stats(), (1, 5));
    }

    #[test]
    fn lru_evicts_least_recently_used_entry() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_all();

        let local = LocalIdentity::from_seed("local", &[0x44; 32]);
        let peers: std::vec::Vec<LocalIdentity> =
            (0..=CACHE_CAPACITY as u16).map(peer_identity).collect();

        // Fill cache with peers[0..CACHE_CAPACITY].
        for p in &peers[..CACHE_CAPACITY] {
            let _ = shared_secret(local.private_key(), p.pub_key()).unwrap();
        }
        assert_eq!(stats(), (0, CACHE_CAPACITY as u32));

        // Touch peers[0] so peers[1] becomes the LRU entry.
        let _ = shared_secret(local.private_key(), peers[0].pub_key()).unwrap();
        assert_eq!(stats(), (1, CACHE_CAPACITY as u32));

        // Insert one more peer -> evicts peers[1], keeps peers[0].
        let _ = shared_secret(local.private_key(), peers[CACHE_CAPACITY].pub_key()).unwrap();
        assert_eq!(stats(), (1, (CACHE_CAPACITY + 1) as u32));

        // peers[0] must still hit; peers[1] must miss.
        let _ = shared_secret(local.private_key(), peers[0].pub_key()).unwrap();
        assert_eq!(stats(), (2, (CACHE_CAPACITY + 1) as u32));

        let _ = shared_secret(local.private_key(), peers[1].pub_key()).unwrap();
        assert_eq!(stats(), (2, (CACHE_CAPACITY + 2) as u32));
    }

    #[test]
    fn invalid_peer_key_returns_none_without_polluting_cache() {
        let _g = TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_all();

        let local = LocalIdentity::from_seed("local", &[0x55; 32]);
        let bad_peer = (0u8..=255)
            .map(|b| {
                let mut k = [0u8; 32];
                k[0] = b;
                k
            })
            .find(|k| meshcore_wire::identity::VerifyingKey::from_bytes(k).is_err())
            .expect("some 32-byte values are not valid Ed25519 points");
        assert!(shared_secret(local.private_key(), &bad_peer).is_none());
        assert_eq!(stats(), (0, 0));
    }
}
