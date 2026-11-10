//! MeshCore Cryptographic Primitives backed by official RustCrypto crates
//!
//! - SHA-256 via `sha2` (audited, standard NIST SHA-256)
//! - AES-128-CBC via `aes` and `cbc` (with PKCS#7 padding)
//! - Ed25519 via `ed25519-dalek` (constant-time, audited signatures & verification)

use aes::Aes128;
use cbc::cipher::{BlockDecryptMut, BlockEncryptMut, KeyIvInit};
use cbc::cipher::block_padding::Pkcs7;
use sha2::{Digest, Sha256};

pub use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};

type Aes128CbcEnc = cbc::Encryptor<Aes128>;
type Aes128CbcDec = cbc::Decryptor<Aes128>;

/// Computes SHA-256 hash using the standard `sha2` RustCrypto crate
pub fn sha256(data: &[u8]) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(data);
    hasher.finalize().into()
}

/// Derives the 1-byte channel hash prefix from a channel secret key
pub fn channel_hash(key: &[u8]) -> u8 {
    sha256(key)[0]
}

/// Encrypts data in place using AES-128-CBC with PKCS#7 padding.
/// `buf` must have enough capacity for padding (typically data_len + 16 bytes).
/// Returns the total ciphertext length.
pub fn aes128_cbc_encrypt(
    key: &[u8; 16],
    iv: &[u8; 16],
    buf: &mut [u8],
    data_len: usize,
) -> Result<usize, ()> {
    let enc = Aes128CbcEnc::new(key.into(), iv.into());
    let ct = enc.encrypt_padded_mut::<Pkcs7>(buf, data_len).map_err(|_| ())?;
    Ok(ct.len())
}

/// Decrypts data in place using AES-128-CBC with PKCS#7 padding.
/// Returns the decrypted plaintext slice.
pub fn aes128_cbc_decrypt<'a>(
    key: &[u8; 16],
    iv: &[u8; 16],
    buf: &'a mut [u8],
) -> Result<&'a [u8], ()> {
    let dec = Aes128CbcDec::new(key.into(), iv.into());
    dec.decrypt_padded_mut::<Pkcs7>(buf).map_err(|_| ())
}
