//! MeshCore Cryptographic Primitives backed by official RustCrypto crates
//!
//! - SHA-256 via `sha2` (audited, standard NIST SHA-256)
//! - AES-128-CBC via `aes` and `cbc` (with PKCS#7 padding)
//! - Ed25519 via `ed25519-dalek` (constant-time, audited signatures & verification)

use aes::Aes128;
use aes::cipher::{BlockDecrypt, BlockEncrypt, KeyInit};
use sha2::{Digest, Sha256};

pub use ed25519_dalek::{Signature, Signer, SigningKey, Verifier, VerifyingKey};

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

/// Convert a channel secret (16 or 32 bytes) into a 32-byte array with trailing zeros
pub fn channel_secret_32(secret: &[u8]) -> [u8; 32] {
    let mut s32 = [0u8; 32];
    let len = secret.len().min(32);
    s32[..len].copy_from_slice(&secret[..len]);
    s32
}

/// Standard HMAC-SHA256 computation
pub fn hmac_sha256(key: &[u8], data: &[u8]) -> [u8; 32] {
    let mut k_pad = [0u8; 64];
    if key.len() > 64 {
        let hash = sha256(key);
        k_pad[..32].copy_from_slice(&hash);
    } else {
        k_pad[..key.len()].copy_from_slice(key);
    }

    let mut ipad = [0x36u8; 64];
    let mut opad = [0x5cu8; 64];
    for i in 0..64 {
        ipad[i] ^= k_pad[i];
        opad[i] ^= k_pad[i];
    }

    let mut inner = Sha256::new();
    inner.update(&ipad);
    inner.update(data);
    let inner_hash = inner.finalize();

    let mut outer = Sha256::new();
    outer.update(&opad);
    outer.update(&inner_hash);
    outer.finalize().into()
}

/// Constant-time comparison of two short MAC tags.
fn ct_eq_2(a: &[u8; 2], b: &[u8; 2]) -> bool {
    let diff = (a[0] ^ b[0]) | (a[1] ^ b[1]);
    diff == 0
}

/// Compute truncated 2-byte MAC for MeshCore packets using HMAC-SHA256 with 32-byte key
pub fn compute_mac_2b(key_32: &[u8; 32], ciphertext: &[u8]) -> [u8; 2] {
    let full = hmac_sha256(key_32, ciphertext);
    [full[0], full[1]]
}

/// MeshCore AES-128-ECB zero-padded encryption
/// Encrypts `src` into `dest` with zero-padding to 16-byte block boundaries.
/// Returns total ciphertext length in bytes (multiple of 16).
pub fn meshcore_encrypt(key_16: &[u8; 16], dest: &mut [u8], src: &[u8]) -> Result<usize, ()> {
    let cipher = Aes128::new(key_16.into());
    let mut rem = src.len();
    let mut sp = 0;
    let mut dp = 0;

    while rem >= 16 {
        if dest.len() < dp + 16 {
            return Err(());
        }
        let mut block = aes::Block::clone_from_slice(&src[sp..sp + 16]);
        cipher.encrypt_block(&mut block);
        dest[dp..dp + 16].copy_from_slice(&block);
        sp += 16;
        dp += 16;
        rem -= 16;
    }

    if rem > 0 {
        if dest.len() < dp + 16 {
            return Err(());
        }
        let mut block = [0u8; 16];
        block[..rem].copy_from_slice(&src[sp..sp + rem]);
        let mut b = aes::Block::clone_from_slice(&block);
        cipher.encrypt_block(&mut b);
        dest[dp..dp + 16].copy_from_slice(&b);
        dp += 16;
    }

    Ok(dp)
}

/// MeshCore AES-128-ECB decryption
/// Decrypts `src` (must be multiple of 16) into `dest`.
/// Returns total decrypted bytes in `dest`.
pub fn meshcore_decrypt(key_16: &[u8; 16], dest: &mut [u8], src: &[u8]) -> Result<usize, ()> {
    if src.len() % 16 != 0 || src.is_empty() {
        return Err(());
    }
    if dest.len() < src.len() {
        return Err(());
    }
    let cipher = Aes128::new(key_16.into());
    let mut offset = 0;
    while offset < src.len() {
        let mut block = aes::Block::clone_from_slice(&src[offset..offset + 16]);
        cipher.decrypt_block(&mut block);
        dest[offset..offset + 16].copy_from_slice(&block);
        offset += 16;
    }
    Ok(src.len())
}

/// MeshCore Encrypt-Then-MAC:
/// Writes `[MAC: 2 bytes] [Ciphertext: N*16 bytes]` into `dest`.
/// Returns total bytes written (2 + enc_len).
pub fn meshcore_encrypt_then_mac(
    secret: &[u8],
    dest: &mut [u8],
    src: &[u8],
) -> Result<usize, ()> {
    if dest.len() < 2 {
        return Err(());
    }
    let key_32 = channel_secret_32(secret);
    let mut key_16 = [0u8; 16];
    key_16.copy_from_slice(&key_32[..16]);

    let enc_len = meshcore_encrypt(&key_16, &mut dest[2..], src)?;
    let mac = compute_mac_2b(&key_32, &dest[2..2 + enc_len]);
    dest[0] = mac[0];
    dest[1] = mac[1];
    Ok(2 + enc_len)
}

/// MeshCore MAC-Then-Decrypt:
/// Verifies 2-byte MAC on `src[2..]`, then decrypts `src[2..]` into `dest`.
/// Returns number of decrypted bytes.
pub fn meshcore_mac_then_decrypt(
    secret: &[u8],
    dest: &mut [u8],
    src: &[u8],
) -> Result<usize, ()> {
    if src.len() <= 2 {
        return Err(());
    }
    let ct = &src[2..];
    if ct.len() % 16 != 0 {
        return Err(());
    }
    let key_32 = channel_secret_32(secret);
    let mut key_16 = [0u8; 16];
    key_16.copy_from_slice(&key_32[..16]);

    let expected_mac = compute_mac_2b(&key_32, ct);
    if !ct_eq_2(&[src[0], src[1]], &expected_mac) {
        return Err(());
    }
    meshcore_decrypt(&key_16, dest, ct)
}

/// MeshCore ACK CRC: `SHA256(ts u32 LE ‖ flags ‖ text ‖ sender_pub)[0..4]`
/// read as a little-endian u32.
///
/// `flags` is the full plaintext byte (`txt_type << 2 | attempt & 3`).
pub fn ack_crc(ts: u32, flags: u8, text: &[u8], sender_pub: &[u8; 32]) -> u32 {
    let mut hasher = Sha256::new();
    hasher.update(ts.to_le_bytes());
    hasher.update([flags]);
    hasher.update(text);
    hasher.update(sender_pub);
    let full: [u8; 32] = hasher.finalize().into();
    u32::from_le_bytes([full[0], full[1], full[2], full[3]])
}

/// ACK CRC for a signed message: the signing-key prefix is part of the
/// authenticated plaintext, between the flags and the text.
pub fn ack_crc_signed(
    ts: u32,
    flags: u8,
    prefix: &[u8; 4],
    text: &[u8],
    sender_pub: &[u8; 32],
) -> u32 {
    let mut hasher = Sha256::new();
    hasher.update(ts.to_le_bytes());
    hasher.update([flags]);
    hasher.update(prefix);
    hasher.update(text);
    hasher.update(sender_pub);
    let full: [u8; 32] = hasher.finalize().into();
    u32::from_le_bytes([full[0], full[1], full[2], full[3]])
}

/// Region key: `SHA256("#" ‖ name)[0..16]`. "fr" and "#fr" are equivalent.
pub fn region_key(name: &str) -> [u8; 16] {
    let name = name.strip_prefix('#').unwrap_or(name);
    let mut hasher = Sha256::new();
    hasher.update(b"#");
    hasher.update(name.as_bytes());
    let full: [u8; 32] = hasher.finalize().into();
    let mut key = [0u8; 16];
    key.copy_from_slice(&full[..16]);
    key
}

/// Transport (scope) code: `HMAC-SHA256(key16, type ‖ payload)[0..2]` as a
/// little-endian u16. 0 becomes 1 and 0xFFFF becomes 0xFFFE.
pub fn transport_code(key: &[u8; 16], ptype: u8, payload: &[u8]) -> u16 {
    let mut input = [0u8; 1 + 184];
    if payload.len() > 184 {
        return 0;
    }
    input[0] = ptype & 0x0F;
    input[1..1 + payload.len()].copy_from_slice(payload);
    let full = hmac_sha256(key, &input[..1 + payload.len()]);
    match u16::from_le_bytes([full[0], full[1]]) {
        0 => 1,
        0xFFFF => 0xFFFE,
        code => code,
    }
}
