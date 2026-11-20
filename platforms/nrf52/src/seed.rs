//! nRF52840 hardware identity and entropy.

use nrf_pac as pac;

/// Read the factory-fused hardware unique ID (FICR) and derive a 32-byte seed.
///
/// This value is deterministic and publicly derivable; use it only as a last
/// resort. New identities are generated with [`random_seed`] instead.
pub fn hardware_seed() -> [u8; 32] {
    let ficr = pac::FICR;
    let id0 = ficr.deviceid(0).read();
    let id1 = ficr.deviceid(1).read();
    let addr0 = ficr.deviceaddr(0).read();
    let addr1 = ficr.deviceaddr(1).read();

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
    let rng = pac::RNG;
    let mut out = [0u8; 32];

    // Enable digital bias correction, then start the generator.
    rng.config().write(|w| w.set_dercen(true));
    rng.events_valrdy().write_value(0);
    rng.tasks_start().write_value(1);

    for byte in out.iter_mut() {
        let mut ready = false;
        for _ in 0..1_000_000 {
            if rng.events_valrdy().read() != 0 {
                ready = true;
                break;
            }
        }
        if !ready {
            rng.tasks_stop().write_value(1);
            return hardware_seed();
        }
        *byte = rng.value().read().value();
        rng.events_valrdy().write_value(0);
    }

    rng.tasks_stop().write_value(1);
    out
}
