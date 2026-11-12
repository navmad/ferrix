//! MeshCore Cryptographic Node Identity & Key Management
//!
//! MeshCore stores an orlp-style **64-byte expanded private key**:
//! `scalar[32] ‖ nonce_prefix[32]`, derived from a 32-byte seed with SHA-512
//! and clamping. The seed is not retained. Signing and ECDH both use the
//! expanded form.

use curve25519_dalek::scalar::Scalar;
use ed25519_dalek::hazmat::{raw_sign, ExpandedSecretKey};
use heapless::String;
use sha2::{Digest, Sha512};

pub use ed25519_dalek::{Signature, Verifier, VerifyingKey};

pub const PUB_KEY_SIZE: usize = 32;
pub const PRV_KEY_SIZE: usize = 64;
pub const SIGNATURE_SIZE: usize = 64;

/// Remote Node Identity (Public Key + Callsign)
#[derive(Clone, Debug, PartialEq)]
pub struct Identity {
    pub pub_key: [u8; PUB_KEY_SIZE],
    pub callsign: String<32>,
}

impl Identity {
    pub fn new(pub_key: [u8; PUB_KEY_SIZE], callsign: &str) -> Self {
        let mut cs = String::new();
        let _ = cs.push_str(callsign);
        Self { pub_key, callsign: cs }
    }

    /// Single-byte hash prefix used in path hop routing
    #[inline]
    pub fn short_hash(&self) -> u8 {
        self.pub_key[0]
    }

    /// 4-byte hash prefix used by the legacy request/response addressing
    #[inline]
    pub fn hash_u32(&self) -> u32 {
        u32::from_le_bytes([
            self.pub_key[0],
            self.pub_key[1],
            self.pub_key[2],
            self.pub_key[3],
        ])
    }

    /// Check if a byte slice matches this identity's hash prefix
    pub fn matches_hash(&self, hash_slice: &[u8]) -> bool {
        if hash_slice.is_empty() || hash_slice.len() > PUB_KEY_SIZE {
            false
        } else {
            &self.pub_key[..hash_slice.len()] == hash_slice
        }
    }

    /// Verifies an Ed25519 digital signature against a message
    pub fn verify(&self, message: &[u8], signature_bytes: &[u8; SIGNATURE_SIZE]) -> bool {
        if let Ok(vk) = VerifyingKey::from_bytes(&self.pub_key) {
            let sig = Signature::from_bytes(signature_bytes);
            vk.verify(message, &sig).is_ok()
        } else {
            false
        }
    }
}

/// Expand a 32-byte seed into MeshCore's 64-byte expanded private key:
/// `clamp(SHA512(seed)[0..32]) ‖ SHA512(seed)[32..64]`.
pub fn expand_seed(seed: &[u8; 32]) -> [u8; PRV_KEY_SIZE] {
    let digest = Sha512::digest(seed);
    let mut prv = [0u8; PRV_KEY_SIZE];
    prv.copy_from_slice(&digest);
    prv[0] &= 248;
    prv[31] &= 63;
    prv[31] |= 64;
    prv
}

fn expanded(prv: &[u8; PRV_KEY_SIZE]) -> ExpandedSecretKey {
    let mut scalar = [0u8; 32];
    scalar.copy_from_slice(&prv[..32]);
    let mut hash_prefix = [0u8; 32];
    hash_prefix.copy_from_slice(&prv[32..]);
    ExpandedSecretKey {
        scalar: Scalar::from_bytes_mod_order(scalar),
        hash_prefix,
    }
}

/// Local Node Identity containing the 64-byte expanded signing key.
pub struct LocalIdentity {
    pub base: Identity,
    prv: [u8; PRV_KEY_SIZE],
}

impl LocalIdentity {
    /// Initialize from a 64-byte expanded key. Returns `None` if the derived
    /// public key is reserved (`pub[0] == 0x00` or `0xFF`), as MeshCore
    /// rejects such identities.
    pub fn from_private(callsign: &str, prv: &[u8; PRV_KEY_SIZE]) -> Option<Self> {
        let esk = expanded(prv);
        let pub_key = VerifyingKey::from(&esk).to_bytes();
        if pub_key[0] == 0x00 || pub_key[0] == 0xFF {
            return None;
        }
        Some(Self {
            base: Identity::new(pub_key, callsign),
            prv: *prv,
        })
    }

    /// Initialize from a 32-byte seed. In the rare reserved-key case, the
    /// seed is re-hashed deterministically, matching MeshCore's "regenerate"
    /// behaviour.
    pub fn from_seed(callsign: &str, seed: &[u8; 32]) -> Self {
        let mut current = *seed;
        loop {
            let prv = expand_seed(&current);
            if let Some(id) = Self::from_private(callsign, &prv) {
                return id;
            }
            current = crate::crypto::sha256(&current);
        }
    }

    /// Backward-compatible alias for [`Self::from_seed`].
    pub fn from_secret_key(callsign: &str, seed: &[u8; 32]) -> Self {
        Self::from_seed(callsign, seed)
    }

    /// Initialize with deterministic Ed25519 key generation from a text seed.
    pub fn new(callsign: &str, seed_str: &str) -> Self {
        let seed = crate::crypto::sha256(seed_str.as_bytes());
        Self::from_seed(callsign, &seed)
    }

    /// Replace the key with a 64-byte expanded key. Returns false if the
    /// derived public key is reserved.
    pub fn set_private_key(&mut self, prv: &[u8; PRV_KEY_SIZE]) -> bool {
        match Self::from_private(self.base.callsign.as_str(), prv) {
            Some(id) => {
                self.prv = id.prv;
                self.base.pub_key = id.base.pub_key;
                true
            }
            None => false,
        }
    }

    /// Replace the key from a 32-byte seed.
    pub fn set_secret_key(&mut self, seed: &[u8; 32]) {
        let id = Self::from_seed(self.base.callsign.as_str(), seed);
        self.prv = id.prv;
        self.base.pub_key = id.base.pub_key;
    }

    /// 64-byte expanded private key (`scalar ‖ nonce_prefix`).
    #[inline]
    pub fn private_key(&self) -> &[u8; PRV_KEY_SIZE] {
        &self.prv
    }

    /// 64-byte expanded private key, by value.
    #[inline]
    pub fn private_key_bytes(&self) -> [u8; PRV_KEY_SIZE] {
        self.prv
    }

    /// 32-byte public key.
    #[inline]
    pub fn pub_key_bytes(&self) -> [u8; PUB_KEY_SIZE] {
        self.base.pub_key
    }

    #[inline]
    pub fn pub_key(&self) -> &[u8; PUB_KEY_SIZE] {
        &self.base.pub_key
    }

    #[inline]
    pub fn callsign(&self) -> &str {
        &self.base.callsign
    }

    /// Update the local callsign name
    pub fn set_callsign(&mut self, callsign: &str) {
        self.base.callsign.clear();
        let _ = self.base.callsign.push_str(callsign);
    }

    #[inline]
    pub fn short_hash(&self) -> u8 {
        self.base.short_hash()
    }

    #[inline]
    pub fn hash_u32(&self) -> u32 {
        self.base.hash_u32()
    }

    /// The public key starts with 0x00 or 0xFF, which MeshCore rejects.
    #[inline]
    pub fn is_reserved(&self) -> bool {
        self.base.pub_key[0] == 0x00 || self.base.pub_key[0] == 0xFF
    }

    /// Sign a message with the expanded key.
    pub fn sign(&self, message: &[u8]) -> [u8; SIGNATURE_SIZE] {
        let esk = expanded(&self.prv);
        let vk = VerifyingKey::from(&esk);
        raw_sign::<Sha512>(&esk, message, &vk).to_bytes()
    }

    /// MeshCore ECDH: X25519 with the clamped scalar and the peer's Ed25519
    /// public key mapped to Montgomery form. Returns `None` if the peer key
    /// does not decompress.
    pub fn shared_secret(&self, peer: &[u8; PUB_KEY_SIZE]) -> Option<[u8; 32]> {
        let vk = VerifyingKey::from_bytes(peer).ok()?;
        let point = vk.to_montgomery();
        let mut scalar = [0u8; 32];
        scalar.copy_from_slice(&self.prv[..32]);
        Some(point.mul_clamped(scalar).to_bytes())
    }
}
