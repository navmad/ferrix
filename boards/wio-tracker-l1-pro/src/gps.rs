//! Quectel L76K GNSS subsystem for the Wio Tracker L1 Pro.
//!
//! The module is on UARTE0 at 9600 baud with a standby/enable line and no
//! reset line. The task body lives in `ferrix_nrf52::gnss`.

use embassy_nrf::peripherals;
use ferrix_l76k::L76kConfig;

use crate::pins::GpsPins;

pub use ferrix_nrf52::gnss::{gps_enabled, set_gps_enabled};

/// No reset line: skip the reset pulse. Enable is active-high (MeshCore's
/// `PIN_GPS_EN`); confirm on hardware if the module never produces NMEA.
const CONFIG: L76kConfig = L76kConfig {
    standby_active_high: true,
    reset_active_low: true,
    reset_pulse_ms: 0,
};

/// Read NMEA from the L76K and publish the latest solution.
#[embassy_executor::task]
pub async fn gps_task(uarte: peripherals::UARTE0, pins: GpsPins) -> ! {
    ferrix_nrf52::gnss::run(uarte, pins, CONFIG).await
}
