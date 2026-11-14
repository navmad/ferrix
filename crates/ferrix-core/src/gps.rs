//! NMEA parsing for the GNSS receiver.
//!
//! The board task reads UART lines and calls [`parse_line`]. Keeping the
//! parser here makes it host-testable and free of board dependencies.

use crate::state::{gps_state, update_gps_state};

#[cfg(test)]
fn now_ms() -> u64 {
    1
}

#[cfg(not(test))]
fn now_ms() -> u64 {
    embassy_time::Instant::now().as_millis() as u64
}

/// Parse one NMEA sentence and update the shared GPS state.
pub fn parse_line(line: &[u8]) {
    if line.first() != Some(&b'$') || !checksum_ok(line) {
        return;
    }
    let Ok(text) = core::str::from_utf8(line) else {
        return;
    };

    let mut fields = [""; 20];
    let mut n = 0;
    for field in text.split(',') {
        if n < fields.len() {
            fields[n] = field;
            n += 1;
        }
    }
    if n == 0 {
        return;
    }

    let tag = fields[0];
    if tag.ends_with("GGA") && n >= 10 {
        let fix_quality: u8 = fields[6].parse().unwrap_or(0);
        let satellites: u8 = fields[7].parse().unwrap_or(0);
        let hdop: f32 = fields[8].parse().unwrap_or(0.0);
        let altitude: f32 = fields[9].parse().unwrap_or(0.0);

        let mut state = gps_state();
        state.fix = fix_quality > 0;
        state.satellites = satellites;
        state.hdop = hdop;
        state.altitude_m = altitude;
        if state.fix {
            if let (Some(lat), Some(lon)) = (
                parse_coordinate(fields[2], fields[3]),
                parse_coordinate(fields[4], fields[5]),
            ) {
                state.latitude = lat;
                state.longitude = lon;
            }
        }
        state.last_update_ms = now_ms();
        update_gps_state(state);
    } else if tag.ends_with("RMC") && n >= 9 {
        let valid = fields[2].starts_with('A');
        let mut state = gps_state();
        state.fix = valid;
        state.speed_knots = fields[7].parse().unwrap_or(0.0);
        state.course_deg = fields[8].parse().unwrap_or(0.0);
        if valid {
            if let (Some(lat), Some(lon)) = (
                parse_coordinate(fields[3], fields[4]),
                parse_coordinate(fields[5], fields[6]),
            ) {
                state.latitude = lat;
                state.longitude = lon;
            }
        }
        state.last_update_ms = now_ms();
        update_gps_state(state);
    }
}

fn checksum_ok(line: &[u8]) -> bool {
    let Some(star) = line.iter().position(|&b| b == b'*') else {
        return false;
    };
    if star + 3 > line.len() {
        return false;
    }
    let mut sum = 0u8;
    for &b in &line[1..star] {
        sum ^= b;
    }
    let Some(hi) = hex(line[star + 1]) else {
        return false;
    };
    let Some(lo) = hex(line[star + 2]) else {
        return false;
    };
    sum == (hi << 4) | lo
}

fn hex(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Parse `ddmm.mmmm` / `dddmm.mmmm` plus the N/S/E/W hemisphere.
fn parse_coordinate(value: &str, hemisphere: &str) -> Option<f32> {
    if value.is_empty() || hemisphere.is_empty() {
        return None;
    }
    let dot = value.find('.')?;
    if dot < 3 {
        return None;
    }
    let deg_len = dot - 2;
    let degrees: f32 = value[..deg_len].parse().ok()?;
    let minutes: f32 = value[deg_len..].parse().ok()?;
    if !(0.0..60.0).contains(&minutes) {
        return None;
    }
    let mut coord = degrees + minutes / 60.0;
    match hemisphere.as_bytes().first()? {
        b'S' | b'W' => coord = -coord,
        b'N' | b'E' => {}
        _ => return None,
    }
    Some(coord)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::state::clear_gps_state;

    #[test]
    fn parses_fix_from_gga() {
        clear_gps_state();
        parse_line(b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
        let state = gps_state();
        assert!(state.fix);
        assert_eq!(state.satellites, 8);
        assert!((state.latitude - 48.1173).abs() < 0.0001);
        assert!((state.longitude - 11.516_667).abs() < 0.0001);
        assert!((state.altitude_m - 545.4).abs() < 0.1);
        assert!((state.hdop - 0.9).abs() < 0.01);
    }

    #[test]
    fn parses_no_fix_gga() {
        clear_gps_state();
        // A valid no-fix GGA: quality 0, no satellites, HDOP 25.5.
        parse_line(b"$GPGGA,123519,,,,,0,00,25.5,,M,,M,,*6A");
        let state = gps_state();
        assert!(!state.fix);
        assert_eq!(state.satellites, 0);
    }

    #[test]
    fn parses_rmc_speed_and_position() {
        clear_gps_state();
        parse_line(b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A");
        let state = gps_state();
        assert!(state.fix);
        assert!((state.latitude - 48.1173).abs() < 0.0001);
        assert!((state.longitude - 11.516_667).abs() < 0.0001);
        assert!((state.speed_knots - 22.4).abs() < 0.1);
        assert!((state.course_deg - 84.4).abs() < 0.1);
    }

    #[test]
    fn rejects_bad_checksum() {
        clear_gps_state();
        parse_line(b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*00");
        assert!(!gps_state().fix);
        assert_eq!(gps_state().last_update_ms, 0);
    }
}
