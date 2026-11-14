//! Quectel L76K GNSS / GPS subsystem for ThinkNode M1.
//!
//! The module is on UARTE0 at 9600 baud. GPS power is controlled from the
//! dashboard; the task owns the standby/reset pins so it can pulse the L76K
//! reset when the user enables it.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_nrf::peripherals;
use embassy_nrf::uarte::{self, Uarte};
use embassy_time::Timer;

use ferrix_core::state::clear_gps_state;

use crate::board::GpsPins;

/// GPS module power state controlled from the dashboard.
pub static GPS_ENABLED: AtomicBool = AtomicBool::new(false);

pub fn gps_enabled() -> bool {
    GPS_ENABLED.load(Ordering::Relaxed)
}

pub fn set_gps_enabled(enabled: bool) {
    GPS_ENABLED.store(enabled, Ordering::Relaxed);
}

embassy_nrf::bind_interrupts!(struct GpsIrqs {
    UARTE0 => embassy_nrf::uarte::InterruptHandler<peripherals::UARTE0>;
});

/// Read NMEA from the L76K and publish the latest solution.
#[embassy_executor::task]
pub async fn gps_task(uarte: peripherals::UARTE0, mut pins: GpsPins) -> ! {
    let mut config = uarte::Config::default();
    config.baudrate = uarte::Baudrate::BAUD9600;
    // Uarte::new takes (rxd, txd).
    let mut uart = Uarte::new(uarte, GpsIrqs, pins.rx, pins.tx, config);

    // Start asleep and held in reset; the dashboard enables the module.
    pins.standby.set_low();
    pins.reinit.set_low();
    let mut enabled = false;
    let mut line = [0u8; 128];
    let mut len = 0usize;

    loop {
        let want = gps_enabled();
        if want != enabled {
            enabled = want;
            if enabled {
                pins.standby.set_high();
                pins.reinit.set_low();
                Timer::after_millis(120).await;
                pins.reinit.set_high();
                clear_gps_state();
                info!("GPS: powered on");
            } else {
                pins.reinit.set_low();
                pins.standby.set_low();
                clear_gps_state();
                info!("GPS: powered off");
            }
        }

        if !enabled {
            Timer::after_millis(200).await;
            continue;
        }

        let mut byte = [0u8; 1];
        match select(uart.read(&mut byte), Timer::after_millis(500)).await {
            Either::First(Ok(())) => {
                let b = byte[0];
                if b == b'\n' {
                    if len > 0 {
                        ferrix_core::gps::parse_line(&line[..len]);
                        len = 0;
                    }
                } else if b != b'\r' {
                    if len < line.len() {
                        line[len] = b;
                        len += 1;
                    } else {
                        // Drop an overlong sentence and resynchronise.
                        len = 0;
                    }
                }
            }
            Either::First(Err(_)) => {
                warn!("GPS: UART read error");
                Timer::after_millis(100).await;
            }
            Either::Second(_) => {}
        }
    }
}

