//! Quectel L76K GNSS subsystem for the Wio Tracker L1 Pro.
//!
//! The module is on UARTE0 at 9600 baud with a standby/enable line and no
//! reset line. The task body lives in `ferrix_nrf52::gnss`.

use embassy_nrf::peripherals;
use ferrix_wio_tracker_l1_pro_logic::config::L76K_CONFIG;

use crate::pins::GpsPins;

pub use ferrix_nrf52::gnss::{gps_enabled, set_gps_enabled};

/// Read NMEA from the L76K and publish the latest solution.
#[embassy_executor::task]
pub async fn gps_task(uarte: peripherals::UARTE0, pins: GpsPins) -> ! {
    ferrix_nrf52::gnss::run(uarte, pins, L76K_CONFIG).await
}
