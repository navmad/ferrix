//! Flash layout: storage regions and the MCU/QSPI bounds they must respect.
//!
//! `memory.x` is the source of truth for the application window; the tests
//! parse it so the two cannot drift apart.

use ferrix_core::storage::StorageRegions;

/// Application image start (after MBR + S140 v7.3.0).
pub const APP_START: u32 = 0x0002_7000;
/// First byte after the application image == start of the config journal.
pub const APP_END: u32 = 0x000A_0000;
/// Start of the bootloader region; internal stores must end at or before it.
pub const BOOTLOADER_START: u32 = 0x000E_D000;
/// Size of the external P25Q16H QSPI flash.
pub const QSPI_SIZE: u32 = 2 * 1024 * 1024;
/// RAM origin: first 24 KB belong to the SoftDevice/bootloader.
pub const RAM_START: u32 = 0x2000_6000;

/// Default profile: config/bonds internal, contacts/adverts and messages on
/// the external QSPI chip (offsets into the 2 MB chip).
pub static EXTERNAL_REGIONS: StorageRegions = StorageRegions {
    config: 0x000A_0000..0x000B_0000,
    bonds: 0x000B_0000..0x000B_4000,
    contacts: 0x0000_0000..0x0004_0000,
    messages: 0x0004_0000..0x0014_0000,
    factory_reset: 0x000A_0000..0x000B_4000,
    factory_reset_external: 0x0000_0000..0x0014_0000,
};

/// Internal-only profile: 128 KB contacts/adverts and 96 KB messages from
/// the unused internal window between the bond map and the bootloader.
pub static INTERNAL_REGIONS: StorageRegions = StorageRegions {
    config: 0x000A_0000..0x000B_0000,
    bonds: 0x000B_0000..0x000B_4000,
    contacts: 0x000B_4000..0x000D_4000,
    messages: 0x000D_4000..0x000E_C000,
    factory_reset: 0x000A_0000..0x000E_D000,
    factory_reset_external: 0..0,
};

const _: () = assert!(EXTERNAL_REGIONS.validate_external());
const _: () = assert!(INTERNAL_REGIONS.validate_internal());

#[cfg(test)]
mod tests {
    use super::*;
    use core::ops::Range;

    const MEMORY_X: &str = include_str!("../../memory.x");

    /// Parse `ORIGIN = 0x…, LENGTH = 0x… - 0x…` (or a single literal) for a
    /// region line such as `FLASH (rx)  : ORIGIN = …`.
    fn region(name: &str) -> Range<u32> {
        let line = MEMORY_X
            .lines()
            .find(|l| l.trim_start().starts_with(name) && l.contains("ORIGIN"))
            .unwrap_or_else(|| panic!("no {name} region in memory.x"));
        let origin = hex_after(line, "ORIGIN");
        let after_len = &line[line.find("LENGTH").expect("LENGTH") + "LENGTH".len()..];
        let length = match after_len.split_once('-') {
            Some((hi, lo)) => parse_hex(hi) - parse_hex(lo),
            None => parse_hex(after_len),
        };
        origin..origin + length
    }

    fn hex_after(line: &str, key: &str) -> u32 {
        parse_hex(&line[line.find(key).unwrap() + key.len()..])
    }

    fn parse_hex(s: &str) -> u32 {
        let s = s.split(',').next().unwrap();
        let i = s.find("0x").expect("hex literal");
        let digits: std::string::String = s[i + 2..]
            .chars()
            .take_while(|c| c.is_ascii_hexdigit())
            .collect();
        u32::from_str_radix(&digits, 16).unwrap()
    }

    #[test]
    fn memory_x_flash_matches_the_layout_constants() {
        let flash = region("FLASH");
        assert_eq!(flash.start, APP_START);
        assert_eq!(flash.end, APP_END);
    }

    #[test]
    fn memory_x_ram_matches_the_layout_constants() {
        let ram = region("RAM");
        assert_eq!(ram.start, RAM_START);
        assert_eq!(ram.end, 0x2004_0000);
    }

    #[test]
    fn app_window_is_sector_aligned() {
        assert_eq!(APP_START % 4096, 0);
        assert_eq!(APP_END % 4096, 0);
    }

    #[test]
    fn config_journal_starts_where_the_app_ends() {
        assert_eq!(EXTERNAL_REGIONS.config.start, APP_END);
        assert_eq!(INTERNAL_REGIONS.config.start, APP_END);
    }

    #[test]
    fn external_profile_stays_inside_the_qspi_chip() {
        let r = &EXTERNAL_REGIONS;
        assert!(r.contacts.end <= QSPI_SIZE);
        assert!(r.messages.end <= QSPI_SIZE);
        assert!(r.factory_reset_external.end <= QSPI_SIZE);
    }

    #[test]
    fn external_profile_keeps_config_and_bonds_inside_the_mcu_window() {
        let r = &EXTERNAL_REGIONS;
        for range in [&r.config, &r.bonds, &r.factory_reset] {
            assert!(range.start >= APP_END, "{range:?} overlaps the app");
            assert!(range.end <= BOOTLOADER_START, "{range:?} overlaps the bootloader");
        }
    }

    #[test]
    fn internal_profile_stays_between_the_app_and_the_bootloader() {
        let r = &INTERNAL_REGIONS;
        for range in [&r.config, &r.bonds, &r.contacts, &r.messages, &r.factory_reset] {
            assert!(range.start >= APP_END, "{range:?} overlaps the app");
            assert!(range.end <= BOOTLOADER_START, "{range:?} overlaps the bootloader");
        }
        assert_eq!(r.factory_reset_external, 0..0);
    }

    #[test]
    fn internal_profile_sizes_match_the_feature_guide() {
        let r = &INTERNAL_REGIONS;
        assert_eq!(r.contacts.end - r.contacts.start, 128 * 1024);
        assert_eq!(r.messages.end - r.messages.start, 96 * 1024);
        assert_eq!(r.config.end - r.config.start, 64 * 1024);
        assert_eq!(r.bonds.end - r.bonds.start, 16 * 1024);
    }

    #[test]
    fn external_profile_sizes_match_the_feature_guide() {
        let r = &EXTERNAL_REGIONS;
        assert_eq!(r.contacts.end - r.contacts.start, 256 * 1024);
        assert_eq!(r.messages.end - r.messages.start, 1024 * 1024);
    }

    #[test]
    fn factory_reset_covers_every_store() {
        for r in [&EXTERNAL_REGIONS, &INTERNAL_REGIONS] {
            assert!(r.factory_reset.start <= r.config.start && r.config.end <= r.factory_reset.end);
            assert!(r.factory_reset.start <= r.bonds.start && r.bonds.end <= r.factory_reset.end);
        }
        let i = &INTERNAL_REGIONS;
        assert!(i.factory_reset.end >= i.messages.end);
        let e = &EXTERNAL_REGIONS;
        assert!(e.factory_reset_external.end >= e.messages.end);
    }
}
