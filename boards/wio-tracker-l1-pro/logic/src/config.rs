//! Board identity and fixed hardware constants.

use ferrix_l76k::L76kConfig;

/// Hardware model reported to MeshCore companion apps.
pub const NAME: &str = "Wio Tracker L1 Pro";

/// USB (vendor, product) IDs the board enumerates with.
pub const USB_IDS: (u16, u16) = (0x2886, 0x1668);

/// Wio Tracker L1 Pro battery ADC: AIN7 (P0.31), 2:1 divider enabled by P0.04,
/// SAADC gain 1/6 with the internal 0.6 V reference (3.6 V full scale).
///
/// `7200 / 4096` = 2 (divider) x 3600 mV, matching MeshCore's
/// `ADC_MULTIPLIER` / `AREF_VOLTAGE`.
pub const BATTERY_ADC_SCALE: u32 = 7200;

/// Fatal-error LED blink counts (status LED P1.01).
pub const PANIC_BLINKS: u32 = 3;
pub const HARDFAULT_BLINKS: u32 = 6;

/// No reset line: skip the reset pulse. Enable is active-high (MeshCore's
/// `PIN_GPS_EN`); confirm on hardware if the module never produces NMEA.
pub const L76K_CONFIG: L76kConfig = L76kConfig {
    standby_active_high: true,
    reset_active_low: true,
    reset_pulse_ms: 0,
};
