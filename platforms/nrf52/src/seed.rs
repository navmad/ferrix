//! nRF52840 hardware identity.

/// Read the factory-fused hardware unique ID (FICR) and derive a 32-byte seed.
pub fn hardware_seed() -> [u8; 32] {
    let id0 = unsafe { (0x1000_0060 as *const u32).read_volatile() };
    let id1 = unsafe { (0x1000_0064 as *const u32).read_volatile() };
    let addr0 = unsafe { (0x1000_00A4 as *const u32).read_volatile() };
    let addr1 = unsafe { (0x1000_00A8 as *const u32).read_volatile() };

    let mut buf = [0u8; 16];
    buf[0..4].copy_from_slice(&id0.to_le_bytes());
    buf[4..8].copy_from_slice(&id1.to_le_bytes());
    buf[8..12].copy_from_slice(&addr0.to_le_bytes());
    buf[12..16].copy_from_slice(&addr1.to_le_bytes());

    meshcore_wire::crypto::sha256(&buf)
}
