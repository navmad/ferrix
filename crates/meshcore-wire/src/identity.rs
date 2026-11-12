//! MeshCore Cryptographic Node Identity & Key Management
//!
//! Powered by `ed25519-dalek` for constant-time cryptographic signatures & verification.

use heapless::String;
use crate::crypto::{sha256, Signature, Signer, SigningKey, Verifier, VerifyingKey};

pub const PUB_KEY_SIZE: usize = 32;
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

    /// 4-byte hash prefix used in request/response addressing
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

/// Local Node Identity containing Ed25519 signing key
pub struct LocalIdentity {
    pub base: Identity,
    signing_key: SigningKey,
}

impl LocalIdentity {
    /// Initialize local node identity from a 32-byte secret key / seed
    pub fn from_secret_key(callsign: &str, secret: &[u8; 32]) -> Self {
        let signing_key = SigningKey::from_bytes(secret);
        let pub_key = signing_key.verifying_key().to_bytes();

        Self {
            base: Identity::new(pub_key, callsign),
            signing_key,
        }
    }

    /// Initialize local node identity with deterministic Ed25519 key generation from seed/hardware ID
    pub fn new(callsign: &str, seed_str: &str) -> Self {
        let seed = sha256(seed_str.as_bytes());
        Self::from_secret_key(callsign, &seed)
    }

    /// Set a new 32-byte secret key, updating both private key and derived public key
    pub fn set_secret_key(&mut self, secret: &[u8; 32]) {
        self.signing_key = SigningKey::from_bytes(secret);
        self.base.pub_key = self.signing_key.verifying_key().to_bytes();
    }

    /// Get the 32-byte private signing key bytes
    #[inline]
    pub fn signing_key_bytes(&self) -> [u8; 32] {
        self.signing_key.to_bytes()
    }

    /// Get the 32-byte public key bytes
    #[inline]
    pub fn pub_key_bytes(&self) -> [u8; 32] {
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

    /// Sign a message using the local Ed25519 private key
    pub fn sign(&self, message: &[u8]) -> [u8; SIGNATURE_SIZE] {
        self.signing_key.sign(message).to_bytes()
    }
}
