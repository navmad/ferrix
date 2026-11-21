//! Quectel L76K GNSS subsystem for ThinkNode M1.
//!
//! The module is on UARTE0 at 9600 baud. The task body lives in
//! `ferrix_nrf52::gnss`; this module owns only the board's task wrapper.

use embassy_nrf::gpio::Output;
use embassy_nrf::peripherals;
use ferrix_nrf52::gnss::GnssPins;
use ferrix_thinknode_m1_logic::config::L76K_CONFIG;

pub use ferrix_nrf52::gnss::{gps_enabled, set_gps_enabled};

/// Read NMEA from the L76K and publish the latest solution.
#[embassy_executor::task]
pub async fn gps_task(
    uarte: peripherals::UARTE0,
    pins: GnssPins<Output<'static>, Output<'static>>,
) -> ! {
    ferrix_nrf52::gnss::run(uarte, pins, L76K_CONFIG).await
}
