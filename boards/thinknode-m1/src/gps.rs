//! Quectel L76K GNSS / GPS Subsystem for ThinkNode M1
//!
//! Parses NMEA sentences over UARTE using the `nmea` crate.

use core::sync::atomic::{AtomicBool, Ordering};

/// GPS module power state controlled from the dashboard.
pub static GPS_ENABLED: AtomicBool = AtomicBool::new(false);

pub fn gps_enabled() -> bool {
    GPS_ENABLED.load(Ordering::Relaxed)
}

pub fn set_gps_enabled(enabled: bool) {
    GPS_ENABLED.store(enabled, Ordering::Relaxed);
}
