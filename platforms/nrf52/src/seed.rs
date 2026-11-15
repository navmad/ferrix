//! nRF52840 hardware identity and entropy.

/// Read the factory-fused hardware unique ID (FICR) and derive a 32-byte seed.
///
/// This value is deterministic and publicly derivable; use it only as a last
/// resort. New identities are generated with [`random_seed`] instead.
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

/// Generate 32 bytes from the nRF52840 true random number generator.
///
/// Must be called before the SoftDevice is enabled. Falls back to the
/// deterministic FICR seed only if the RNG never signals a value, which should
/// not happen on working hardware.
pub fn random_seed() -> [u8; 32] {
    const RNG_BASE: usize = 0x4000_D000;
    const TASKS_START: usize = 0x000;
    const TASKS_STOP: usize = 0x004;
    const EVENTS_VALRDY: usize = 0x100;
    const CONFIG: usize = 0x504;
    const VALUE: usize = 0x508;

    let mut out = [0u8; 32];
    unsafe {
        let base = RNG_BASE as *mut u32;
        // Digital error correction, then start the generator.
        base.add(CONFIG / 4).write_volatile(1);
        base.add(TASKS_START / 4).write_volatile(1);

        for chunk in out.chunks_mut(4) {
            let mut ready = false;
            for _ in 0..1_000_000 {
                if base.add(EVENTS_VALRDY / 4).read_volatile() != 0 {
                    ready = true;
                    break;
                }
            }
            if !ready {
                base.add(TASKS_STOP / 4).write_volatile(1);
                return hardware_seed();
            }
            let value = base.add(VALUE / 4).read_volatile();
            base.add(EVENTS_VALRDY / 4).write_volatile(0);
            chunk.copy_from_slice(&value.to_le_bytes()[..chunk.len()]);
        }

        base.add(TASKS_STOP / 4).write_volatile(1);
    }
    out
}
