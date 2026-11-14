//! Hardware watchdog and stall detection.
//!
//! The nRF52840 watchdog resets the device if the supervisor stops petting it.
//! The supervisor only pets while both the application loop and the storage
//! loop are making progress, so a hung task or a saturated executor results in
//! an automatic restart instead of a manual power cycle.

use core::sync::atomic::Ordering;

use defmt::warn;
use embassy_nrf::peripherals;
use embassy_nrf::wdt::{Config, HaltConfig, SleepConfig, Watchdog, WatchdogHandle};
use embassy_time::Timer;

use ferrix_core::state::{APP_HEARTBEAT, STORAGE_HEARTBEAT, WDT_PETS};

/// 30 seconds at 32768 Hz.
const TIMEOUT_TICKS: u32 = 30 * 32768;
/// Stop feeding after this many seconds without a heartbeat.
const STALE_SECS: u32 = 10;

/// Start the watchdog and return the handle used by the supervisor task.
pub fn start(wdt: peripherals::WDT) -> WatchdogHandle {
    let mut config = Config::default();
    config.timeout_ticks = TIMEOUT_TICKS;
    config.action_during_sleep = SleepConfig::RUN;
    config.action_during_debug_halt = HaltConfig::RUN;

    let (watchdog, [handle]) = match Watchdog::try_new::<1>(wdt, config) {
        Ok(value) => value,
        Err(_) => panic!("watchdog already active"),
    };
    // The driver object only owns the peripheral; the watchdog keeps running.
    core::mem::forget(watchdog);
    handle
}

#[embassy_executor::task]
pub async fn watchdog_task(mut handle: WatchdogHandle) -> ! {
    let mut last_app = APP_HEARTBEAT.load(Ordering::Relaxed);
    let mut last_storage = STORAGE_HEARTBEAT.load(Ordering::Relaxed);
    let mut app_stale = 0u32;
    let mut storage_stale = 0u32;
    let mut warned = false;

    loop {
        Timer::after_secs(1).await;

        let app = APP_HEARTBEAT.load(Ordering::Relaxed);
        let storage = STORAGE_HEARTBEAT.load(Ordering::Relaxed);

        if app != last_app {
            last_app = app;
            app_stale = 0;
        } else {
            app_stale = app_stale.saturating_add(1);
        }
        if storage != last_storage {
            last_storage = storage;
            storage_stale = 0;
        } else {
            storage_stale = storage_stale.saturating_add(1);
        }

        if app_stale < STALE_SECS && storage_stale < STALE_SECS {
            handle.pet();
            WDT_PETS.fetch_add(1, Ordering::Relaxed);
            warned = false;
        } else if !warned {
            warn!(
                "Watchdog: heartbeat stalled (app {}s, storage {}s)",
                app_stale, storage_stale
            );
            warned = true;
        }
    }
}
