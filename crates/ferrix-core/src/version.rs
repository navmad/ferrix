//! Compile-time firmware version and build metadata.
//!
//! Defaults to the workspace version in root `Cargo.toml` (`[workspace.package]`)
//! and can be overridden at build time via `FERRIX_VERSION` (e.g. from a Git tag)
//! and `FERRIX_BUILD_DATE` / `SOURCE_DATE_EPOCH`.

/// Clean semver string without leading `v` or `-ferrix` suffix (e.g. `"1.18.0"`).
pub const SEMVER: &str = env!("FERRIX_SEMVER");

/// Full Ferrix firmware version shown on the display and USB CLI `ver` (e.g. `"1.18.0-ferrix"`).
pub const FIRMWARE_VERSION: &str = env!("FERRIX_FW_VERSION");

/// BLE Device Information Service firmware revision string (e.g. `"Ferrix 1.18.0"`).
pub const BLE_FW_REVISION: &str = env!("FERRIX_BLE_FW_REVISION");

/// MeshCore companion protocol semver reported in `RESP_CODE_DEVICE_INFO` (e.g. `"v1.18.0"`).
pub const PROTOCOL_SEMVER: &str = env!("FERRIX_PROTOCOL_SEMVER");

/// Build date string reported in `RESP_CODE_DEVICE_INFO` (e.g. `"Nov 15 2026"`).
pub const BUILD_DATE: &str = env!("FERRIX_BUILD_DATE");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_strings_match_protocol_bounds() {
        assert!(!SEMVER.is_empty());
        assert!(FIRMWARE_VERSION.ends_with("-ferrix"));
        assert!(BLE_FW_REVISION.starts_with("Ferrix "));
        assert!(PROTOCOL_SEMVER.starts_with('v'));
        assert!(PROTOCOL_SEMVER.len() <= 20);
        assert!(!BUILD_DATE.is_empty());
        assert!(BUILD_DATE.len() <= 12);
    }
}
