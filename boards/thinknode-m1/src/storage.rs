//! Board binding for the storage formats owned by `ferrix-core`.
//!
//! The board selects one compile-time storage profile:
//!
//! - default: config/bonds internal, contacts/adverts and messages on the
//!   external QSPI chip;
//! - `internal-storage`: all four stores in internal flash, no QSPI.
//!
//! The storage loop itself lives in `ferrix-runtime`.

pub use ferrix_core::storage::{
    adverts, config, contacts, counted_flash, get_callsign, get_path_hash_mode, messages,
    request_contacts, request_factory_reset, request_save_config, set_callsign,
    set_custom_priv_key, set_path_hash_mode, util, MappedFlash, MessageCommand, StorageRegions,
};

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
#[cfg(not(feature = "internal-storage"))]
use ferrix_runtime::storage::storage_loop_external;
#[cfg(feature = "internal-storage")]
use ferrix_runtime::storage::storage_loop_internal;
use ferrix_runtime::storage::StorageExit;
use heapless::String;

/// Default profile: config/bonds internal, contacts/adverts and messages on
/// the external QSPI chip (offsets into the 2 MB chip).
pub static EXTERNAL_REGIONS: StorageRegions = StorageRegions {
    config: 0x000A_0000..0x000B_0000,
    bonds: 0x000B_0000..0x000B_4000,
    contacts: 0x0000_0000..0x0004_0000,
    messages: 0x0004_0000..0x0014_0000,
    factory_reset: 0x000A_0000..0x000B_4000,
    factory_reset_external: 0x0000_0000..0x0014_0000,
};

/// Internal-only profile: 128 KB contacts/adverts and 96 KB messages from
/// the unused internal window between the bond map and the bootloader.
pub static INTERNAL_REGIONS: StorageRegions = StorageRegions {
    config: 0x000A_0000..0x000B_0000,
    bonds: 0x000B_0000..0x000B_4000,
    contacts: 0x000B_4000..0x000D_4000,
    messages: 0x000D_4000..0x000E_C000,
    factory_reset: 0x000A_0000..0x000E_D000,
    factory_reset_external: 0..0,
};

const _: () = assert!(EXTERNAL_REGIONS.validate_external());
const _: () = assert!(INTERNAL_REGIONS.validate_internal());

#[cfg(not(feature = "internal-storage"))]
pub use EXTERNAL_REGIONS as REGIONS;
#[cfg(feature = "internal-storage")]
pub use INTERNAL_REGIONS as REGIONS;

/// Initialize configuration from the journal, or defaults.
pub fn init_config() -> (String<32>, Option<[u8; 64]>) {
    ferrix_core::storage::config::init_config(
        &ferrix_nrf52::flash::MAPPED,
        REGIONS.config.clone(),
        crate::NAME,
    )
}

/// Load the BLE bond map into RAM. Called once at boot, before BLE starts.
pub fn load() {
    ferrix_core::storage::bonds::init(
        &ferrix_nrf52::flash::MAPPED,
        REGIONS.bonds.clone(),
    );
}

/// QSPI pins are parked here between board startup and the storage task.
static QSPI_PINS: Mutex<CriticalSectionRawMutex, RefCell<Option<ferrix_nrf52::qspi::QspiPins>>> =
    Mutex::new(RefCell::new(None));

/// Hand the optional external flash pins to the storage task.
pub fn set_qspi(pins: Option<ferrix_nrf52::qspi::QspiPins>) {
    QSPI_PINS.lock(|c| *c.borrow_mut() = pins);
}

/// Spawn the storage task once the SoftDevice has been enabled.
pub fn spawn_storage(
    spawner: embassy_executor::Spawner,
    sd: &'static nrf_softdevice::Softdevice,
) {
    let pins = QSPI_PINS.lock(|c| c.borrow_mut().take());
    let _ = spawner.spawn(storage_task(sd, pins));
}

#[cfg(not(feature = "internal-storage"))]
type QspiFlash = embassy_nrf::qspi::Qspi<'static, embassy_nrf::peripherals::QSPI>;

#[cfg(not(feature = "internal-storage"))]
static QSPI: static_cell::StaticCell<QspiFlash> = static_cell::StaticCell::new();

/// Background storage task that serializes all flash writes.
#[embassy_executor::task]
pub async fn storage_task(
    sd: &'static nrf_softdevice::Softdevice,
    qspi_pins: Option<ferrix_nrf52::qspi::QspiPins>,
) -> ! {
    let mut flash = ferrix_nrf52::flash::take(sd);

    #[cfg(not(feature = "internal-storage"))]
    let exit = {
        let mut qspi = if let Some(pins) = qspi_pins {
            // The external flash is on a switched rail. Give it time and let
            // the JEDEC probe retry while the rail settles.
            embassy_time::Timer::after_millis(10).await;
            Some(QSPI.init(ferrix_nrf52::qspi::init(pins)))
        } else {
            None
        };

        let ready = match qspi.as_mut() {
            Some(qspi) => ferrix_nrf52::qspi::probe(qspi).await,
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

        storage_loop_external(&mut flash, external, &REGIONS).await
    };

    #[cfg(feature = "internal-storage")]
    let exit = {
        let _ = qspi_pins;
        defmt::info!("Storage: internal flash only");
        storage_loop_internal(&mut flash, &REGIONS).await
    };

    if exit == StorageExit::FactoryReset {
        ferrix_nrf52::reset::system_reset();
    }
    core::future::pending().await
}
