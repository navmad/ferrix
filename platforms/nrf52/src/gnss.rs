//! L76K GNSS task body shared by nRF52 boards.
//!
//! The module sits on UARTE0 at 9600 baud. Power/reset sequencing and UART
//! reads come from `ferrix-l76k`; NMEA parsing and the published fix stay in
//! `ferrix-core`. Boards wrap [`run`] in their own `#[embassy_executor::task]`.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::AnyPin;
use embassy_nrf::peripherals;
use embassy_nrf::uarte::{self, Uarte, UarteRxWithIdle, UarteTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Timer};
use embedded_hal::digital::OutputPin;

use ferrix_core::state::clear_gps_state;
use ferrix_l76k::{L76k, L76kConfig};

/// GPS module power state controlled from the dashboard.
pub static GPS_ENABLED: AtomicBool = AtomicBool::new(false);

pub fn gps_enabled() -> bool {
    GPS_ENABLED.load(Ordering::Relaxed)
}

static GPS_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub fn set_gps_enabled(enabled: bool) {
    GPS_ENABLED.store(enabled, Ordering::Relaxed);
    GPS_WAKE.signal(());
}

embassy_nrf::bind_interrupts!(struct GpsIrqs {
    UARTE0 => embassy_nrf::uarte::InterruptHandler<peripherals::UARTE0>;
});

/// Wiring for the GNSS module. `reset` may be [`ferrix_l76k::NoPin`] on
/// boards that do not route the module's reset line.
pub struct GnssPins<Standby, Reset> {
    /// MCU receive pin (the module's TX).
    pub rx: AnyPin,
    /// MCU transmit pin (the module's RX).
    pub tx: AnyPin,
    pub standby: Standby,
    pub reset: Reset,
    pub timer1: peripherals::TIMER1,
    pub ppi_ch0: peripherals::PPI_CH0,
    pub ppi_ch1: peripherals::PPI_CH1,
}

/// Adapter that exposes the nRF idle-delimited UART receiver as an
/// `embedded-io-async` reader.
///
/// `embassy-nrf`'s `UarteRxWithIdle` uses `TIMER1` + PPI to complete
/// `read_until_idle` once the line goes quiet after an NMEA burst, avoiding
/// per-byte wakeups.
struct GpsUart {
    _tx: UarteTx<'static, peripherals::UARTE0>,
    rx: UarteRxWithIdle<'static, peripherals::UARTE0, peripherals::TIMER1>,
}

impl embedded_io_async::ErrorType for GpsUart {
    type Error = embedded_io_async::ErrorKind;
}

impl embedded_io_async::Read for GpsUart {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.rx
            .read_until_idle(buf)
            .await
            .map_err(|_| embedded_io_async::ErrorKind::Other)
    }
}

/// Read NMEA from the L76K and publish the latest solution.
pub async fn run<Standby, Reset>(
    uarte: peripherals::UARTE0,
    pins: GnssPins<Standby, Reset>,
    config: L76kConfig,
) -> !
where
    Standby: OutputPin,
    Reset: OutputPin,
{
    let mut uart_config = uarte::Config::default();
    uart_config.baudrate = uarte::Baudrate::BAUD9600;
    // Uarte::new takes (rxd, txd).
    let uart = Uarte::new(uarte, GpsIrqs, pins.rx, pins.tx, uart_config);
    let (tx, rx) = uart.split_with_idle(pins.timer1, pins.ppi_ch0, pins.ppi_ch1);

    // Start asleep and held in reset; the dashboard enables the module.
    let mut gps = L76k::new(GpsUart { _tx: tx, rx }, pins.standby, pins.reset, Delay, config)
        .expect("L76K control pins");

    let mut enabled = false;
    let mut line = [0u8; 128];
    let mut len = 0usize;
    let mut buf = [0u8; 128];

    loop {
        let want = gps_enabled();
        if want != enabled {
            enabled = want;
            if enabled {
                if gps.power_on().await.is_err() {
                    warn!("GPS: power-on pin error");
                }
                clear_gps_state();
                ferrix_core::state::GPS_BYTES.store(0, Ordering::Relaxed);
                ferrix_core::state::GPS_SENTENCES.store(0, Ordering::Relaxed);
                len = 0;
                info!("GPS: powered on");
            } else {
                if gps.power_off().await.is_err() {
                    warn!("GPS: power-off pin error");
                }
                clear_gps_state();
                len = 0;
                info!("GPS: powered off");
            }
        }

        if !enabled {
            // Sleep indefinitely until set_gps_enabled(true) is called.
            GPS_WAKE.wait().await;
            continue;
        }

        match select(gps.read(&mut buf), GPS_WAKE.wait()).await {
            Either::First(Ok(n)) => {
                if n > 0 {
                    ferrix_core::state::GPS_BYTES.fetch_add(n as u32, Ordering::Relaxed);
                    for &b in &buf[..n] {
                        if b == b'\n' {
                            if len > 0 {
                                ferrix_core::gps::parse_line(&line[..len]);
                                ferrix_core::state::GPS_SENTENCES.fetch_add(1, Ordering::Relaxed);
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
