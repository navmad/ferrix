//! LoRa modem math shared by the radio scheduler and the companion protocol.
//!
//! These helpers are platform independent and allocation free.

/// Convert the compact kHz setting to the exact bandwidth in Hz.
pub fn bw_khz_to_hz(bw_khz: u32) -> u32 {
    match bw_khz {
        7 => 7_800,
        10 => 10_400,
        15 => 15_600,
        20 => 20_800,
        31 => 31_250,
        41 => 41_670,
        62 => 62_500,
        other => other * 1000,
    }
}

/// Estimate LoRa airtime in milliseconds (preamble + header + payload + CRC).
pub fn estimate_airtime_ms(len: usize, sf: u8, bw_khz: u32, cr: u8, preamble: u16) -> u32 {
    let bw_hz = bw_khz_to_hz(bw_khz) as u64;
    if bw_hz == 0 {
        return 0;
    }
    let sym_ns = ((1u64 << sf) * 1_000_000_000) / bw_hz;
    let de = if sym_ns > 16_380_000 { 1u64 } else { 0 };
    let num = (8u64 * len as u64).saturating_sub(4 * sf as u64) + 44;
    let den = (4 * (sf as u64 - 2 * de)).max(4);
    let nibbles = (num + den - 1) / den;
    let payload_symbols = 8 + nibbles * cr as u64;
    let preamble_ns = (preamble as u64 * 4 + 17) * sym_ns / 4;
    let total_ns = preamble_ns + payload_symbols * sym_ns;
    ((total_ns + 999_999) / 1_000_000) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_bandwidths_are_exact() {
        assert_eq!(bw_khz_to_hz(62), 62_500);
        assert_eq!(bw_khz_to_hz(125), 125_000);
        assert_eq!(bw_khz_to_hz(500), 500_000);
    }

    #[test]
    fn airtime_grows_with_payload_and_spreading_factor() {
        let short = estimate_airtime_ms(16, 7, 125, 5, 16);
        let long = estimate_airtime_ms(200, 7, 125, 5, 16);
        let slow = estimate_airtime_ms(16, 12, 125, 5, 16);
        assert!(long > short);
        assert!(slow > short);
        assert_eq!(estimate_airtime_ms(16, 7, 0, 5, 16), 0);
    }
}
