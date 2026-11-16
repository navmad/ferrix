use std::env;
use std::time::{SystemTime, UNIX_EPOCH};

fn format_utc_date(secs: i64) -> String {
    const MONTHS: [&str; 12] = [
        "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec",
    ];
    let days = secs.div_euclid(86_400);
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = mp + if mp < 10 { 3 } else { -9 };
    let y = y + if m <= 2 { 1 } else { 0 };
    format!("{} {:2} {:04}", MONTHS[(m - 1) as usize], d, y)
}

fn main() {
    println!("cargo:rerun-if-env-changed=FERRIX_VERSION");
    println!("cargo:rerun-if-env-changed=FERRIX_BUILD_DATE");
    println!("cargo:rerun-if-env-changed=SOURCE_DATE_EPOCH");

    let raw = env::var("FERRIX_VERSION")
        .ok()
        .filter(|s| !s.trim().is_empty())
        .unwrap_or_else(|| env::var("CARGO_PKG_VERSION").expect("CARGO_PKG_VERSION"));

    let trimmed = raw.trim();
    let no_v = trimmed.strip_prefix('v').or_else(|| trimmed.strip_prefix('V')).unwrap_or(trimmed);
    let semver = no_v.strip_suffix("-ferrix").unwrap_or(no_v);

    let fw_version = format!("{semver}-ferrix");
    let ble_fw_revision = format!("Ferrix {semver}");

    let base_semver = semver.split(['-', '+']).next().unwrap_or(semver);
    let mut protocol_semver = format!("v{base_semver}");
    protocol_semver.truncate(20);

    let mut build_date = env::var("FERRIX_BUILD_DATE")
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| {
            let secs = env::var("SOURCE_DATE_EPOCH")
                .ok()
                .and_then(|s| s.trim().parse::<i64>().ok())
                .unwrap_or_else(|| {
                    SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .map(|d| d.as_secs() as i64)
                        .unwrap_or(0)
                });
            format_utc_date(secs)
        });
    build_date.truncate(12);

    println!("cargo:rustc-env=FERRIX_SEMVER={semver}");
    println!("cargo:rustc-env=FERRIX_FW_VERSION={fw_version}");
    println!("cargo:rustc-env=FERRIX_BLE_FW_REVISION={ble_fw_revision}");
    println!("cargo:rustc-env=FERRIX_PROTOCOL_SEMVER={protocol_semver}");
    println!("cargo:rustc-env=FERRIX_BUILD_DATE={build_date}");
}
