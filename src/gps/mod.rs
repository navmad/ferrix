//! Quectel L76K GNSS / GPS Subsystem for ThinkNode M1
//!
//! Parses NMEA sentences over UARTE using the `nmea` crate.

use core::sync::atomic::{AtomicBool, Ordering};
use nmea::Nmea;

/// GPS module power state controlled from the dashboard.
pub static GPS_ENABLED: AtomicBool = AtomicBool::new(false);

pub fn gps_enabled() -> bool {
    GPS_ENABLED.load(Ordering::Relaxed)
}

pub fn set_gps_enabled(enabled: bool) {
    GPS_ENABLED.store(enabled, Ordering::Relaxed);
}

/// Parsed GPS Position Snapshot
#[derive(Copy, Clone, Debug, Default)]
pub struct GpsFix {
    pub has_fix: bool,
    pub latitude: f32,
    pub longitude: f32,
    pub altitude_m: f32,
    pub satellites_visible: u32,
}

/// Helper to parse incoming NMEA chunks into GPS fixes
pub struct GpsParser {
    nmea: Nmea,
    line_buf: [u8; 128],
    line_pos: usize,
}

impl GpsParser {
    pub fn new() -> Self {
        Self {
            nmea: Nmea::default(),
            line_buf: [0u8; 128],
            line_pos: 0,
        }
    }

    /// Feed incoming bytes from UARTE and parse full NMEA sentences
    pub fn push_byte(&mut self, byte: u8) -> Option<GpsFix> {
        if byte == b'\n' || byte == b'\r' {
            if self.line_pos > 0 {
                let sentence = &self.line_buf[..self.line_pos];
                if let Ok(s) = core::str::from_utf8(sentence) {
                    if let Ok(_parsed) = self.nmea.parse(s) {
                        if let (Some(lat), Some(lon)) = (self.nmea.latitude, self.nmea.longitude) {
                            let fix = GpsFix {
                                has_fix: self.nmea.fix_type.is_some(),
                                latitude: lat as f32,
                                longitude: lon as f32,
                                altitude_m: self.nmea.altitude.unwrap_or(0.0) as f32,
                                satellites_visible: self.nmea.satellites().len() as u32,
                            };
                            self.line_pos = 0;
                            return Some(fix);
                        }
                    }
                }
                self.line_pos = 0;
            }
            None
        } else {
            if self.line_pos < self.line_buf.len() {
                self.line_buf[self.line_pos] = byte;
                self.line_pos += 1;
            } else {
                self.line_pos = 0;
            }
            None
        }
    }
}
