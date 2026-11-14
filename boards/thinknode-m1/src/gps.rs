//! Quectel L76K GNSS subsystem for ThinkNode M1.
//!
//! The module is on UARTE0 at 9600 baud. Power/reset sequencing and UART reads
//! come from `ferrix-l76k`; NMEA parsing and the published fix stay in
//! `ferrix-core`.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_nrf::peripherals;
use embassy_nrf::uarte::{self, Uarte};
use embassy_time::{Delay, Timer};

use ferrix_core::state::clear_gps_state;
use ferrix_l76k::{L76k, L76kConfig};

use crate::pins::GpsPins;

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

/// Adapter that exposes the nRF UART as an `embedded-io-async` reader.
///
/// `embassy-nrf`'s `Uarte` has an inherent async read but does not implement
/// the trait, so the board provides the small bridge instead of leaking the
/// nRF type into the driver.
struct GpsUart(Uarte<'static, peripherals::UARTE0>);

impl embedded_io_async::ErrorType for GpsUart {
    type Error = embedded_io_async::ErrorKind;
}

impl embedded_io_async::Read for GpsUart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.0
            .read(buf)
            .await
            .map_err(|_| embedded_io_async::ErrorKind::Other)?;
        Ok(buf.len())
    }
}

/// Read NMEA from the L76K and publish the latest solution.
#[embassy_executor::task]
pub async fn gps_task(uarte: peripherals::UARTE0, pins: GpsPins) -> ! {
    let mut config = uarte::Config::default();
    config.baudrate = uarte::Baudrate::BAUD9600;
    // Uarte::new takes (rxd, txd).
    let uart = Uarte::new(uarte, GpsIrqs, pins.rx, pins.tx, config);

    // Start asleep and held in reset; the dashboard enables the module.
    let mut gps = L76k::new(
        GpsUart(uart),
        pins.standby,
        pins.reinit,
        Delay,
        L76kConfig::default(),
    )
    .expect("L76K control pins");

    let mut enabled = false;
    let mut line = [0u8; 128];
    let mut len = 0usize;

    loop {
        let want = gps_enabled();
        if want != enabled {
            enabled = want;
            if enabled {
                if gps.power_on().await.is_err() {
                    warn!("GPS: power-on pin error");
                }
                clear_gps_state();
                use core::sync::atomic::Ordering;
                ferrix_core::state::GPS_BYTES.store(0, Ordering::Relaxed);
                ferrix_core::state::GPS_SENTENCES.store(0, Ordering::Relaxed);
                info!("GPS: powered on");
            } else {
                if gps.power_off().await.is_err() {
                    warn!("GPS: power-off pin error");
                }
                clear_gps_state();
                info!("GPS: powered off");
            }
        }

        if !enabled {
            Timer::after_millis(200).await;
            continue;
        }

        let mut byte = [0u8; 1];
        match select(gps.read_byte(&mut byte), Timer::after_millis(500)).await {
            Either::First(Ok(())) => {
                use core::sync::atomic::Ordering;
                ferrix_core::state::GPS_BYTES.fetch_add(1, Ordering::Relaxed);
                let b = byte[0];
                if b == b'\n' {
                    if len > 0 {
                        ferrix_core::gps::parse_line(&line[..len]);
                        use core::sync::atomic::Ordering;
                        ferrix_core::state::GPS_SENTENCES
                            .fetch_add(1, Ordering::Relaxed);
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
