//! Storage task bodies shared by nRF52 boards.
//!
//! The board selects the profile (and so which function it calls) at compile
//! time, which keeps the unused profile's generic loop out of the image:
//!
//! - [`run_external`]: config/bonds internal, contacts/adverts and messages on
//!   the external QSPI chip;
//! - [`run_internal`]: all four stores in internal flash, no QSPI.
//!
//! Boards wrap these in their own `#[embassy_executor::task]` and spawn it
//! once the SoftDevice has been enabled.

use core::cell::RefCell;

use embassy_nrf::peripherals;
use embassy_nrf::qspi::Qspi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use ferrix_core::storage::StorageRegions;
use ferrix_runtime::storage::{storage_loop_external, storage_loop_internal, StorageExit};
use nrf_softdevice::Softdevice;
use static_cell::StaticCell;

use crate::qspi::QspiPins;

type QspiFlash = Qspi<'static, peripherals::QSPI>;

static QSPI: StaticCell<QspiFlash> = StaticCell::new();

/// QSPI pins are parked here between board startup and the storage task.
static QSPI_PINS: Mutex<CriticalSectionRawMutex, RefCell<Option<QspiPins>>> =
    Mutex::new(RefCell::new(None));

/// Hand the optional external flash pins to the storage task.
pub fn set_qspi(pins: Option<QspiPins>) {
    QSPI_PINS.lock(|c| *c.borrow_mut() = pins);
}

/// Take the parked QSPI pins (once, when spawning the storage task).
pub fn take_qspi() -> Option<QspiPins> {
    QSPI_PINS.lock(|c| c.borrow_mut().take())
}

fn finish(exit: StorageExit) {
    if exit == StorageExit::FactoryReset {
        crate::reset::system_reset();
    }
}

/// Default profile: internal config/bonds plus external contacts and messages.
///
/// `accepted_ids` lists the JEDEC IDs of the flash chips the board may carry.
pub async fn run_external(
    sd: &'static Softdevice,
    qspi_pins: Option<QspiPins>,
    regions: &'static StorageRegions,
    accepted_ids: &'static [[u8; 3]],
) -> ! {
    let mut flash = crate::flash::take(sd);

    let mut qspi = if let Some(pins) = qspi_pins {
        // The external flash may be on a switched rail. Give it time and let
        // the JEDEC probe retry while the rail settles.
        embassy_time::Timer::after_millis(10).await;
        Some(QSPI.init(crate::qspi::init(pins)))
    } else {
        None
    };

    let ready = match qspi.as_mut() {
        Some(qspi) => crate::qspi::probe(qspi, accepted_ids).await,
        None => false,
    };

    let external: Option<&mut QspiFlash> = if ready {
        defmt::info!("Storage: internal + external flash");
        qspi
    } else {
        // No fallback: external stores stay unavailable for this session.
        ferrix_core::storage::mark_external_unavailable();
        None
    };

    finish(storage_loop_external(&mut flash, external, regions).await);
    core::future::pending().await
}

/// `internal-storage` profile: every store in internal flash.
pub async fn run_internal(sd: &'static Softdevice, regions: &'static StorageRegions) -> ! {
    let mut flash = crate::flash::take(sd);
    defmt::info!("Storage: internal flash only");
    finish(storage_loop_internal(&mut flash, regions).await);
    core::future::pending().await
}
