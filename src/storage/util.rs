//! Small helpers shared by the storage modules.

/// FNV-1a initial value.
pub const FNV_INIT: u32 = 0x811C_9DC5;
const FNV_PRIME: u32 = 0x0100_0193;

pub fn fnv_bytes(h: &mut u32, bytes: &[u8]) {
    for &b in bytes {
        *h ^= b as u32;
        *h = h.wrapping_mul(FNV_PRIME);
    }
}

/// Copy a string into a fixed-size, NUL-padded field.
pub fn fixed_str<const N: usize>(s: &str) -> [u8; N] {
    let mut out = [0u8; N];
    let bytes = s.as_bytes();
    let n = bytes.len().min(N);
    out[..n].copy_from_slice(&bytes[..n]);
    out
}

/// Read a NUL-padded fixed-size string (or the whole field when full).
pub fn str_from_fixed(bytes: &[u8]) -> &str {
    let n = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    core::str::from_utf8(&bytes[..n]).unwrap_or("")
}
