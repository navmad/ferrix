//! Board binding for the storage formats owned by `ferrix-core`.
//!
//! The ThinkNode layout reserves the region above the application image:
//! adverts, contacts, messages and one configuration page. The storage loop
//! itself lives in `ferrix-runtime`.

pub use ferrix_core::storage::{
    adverts, config, contacts, counted_flash, get_callsign, get_path_hash_mode, messages,
    request_contacts, request_factory_reset, request_save_config, set_callsign,
    set_custom_priv_key, set_path_hash_mode, util, MappedFlash, MessageCommand, StorageRegions,
};

use core::cell::RefCell;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use ferrix_runtime::storage::{storage_loop, StorageExit};
use heapless::String;

/// Flash layout for ThinkNode M1 (nRF52840, 1 MB internal + 2 MB external).
///
/// The external QSPI regions are offsets into the 2 MB chip.
pub static REGIONS: StorageRegions = StorageRegions {
    // Internal on every board: config journal and bond map.
    config: 0x000A_0000..0x000B_0000,
    bonds: 0x000B_0000..0x000B_4000,
    // External QSPI when present: contacts/advert blobs and messages.
    contacts: 0x0000_0000..0x0004_0000,
    messages: 0x0004_0000..0x0014_0000,
    contacts_external: true,
    messages_external: true,
    factory_reset: 0x000A_0000..0x000B_4000,
    factory_reset_external: 0x0000_0000..0x0014_0000,
};

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

type QspiFlash = embassy_nrf::qspi::Qspi<'static, embassy_nrf::peripherals::QSPI>;

static QSPI: static_cell::StaticCell<QspiFlash> = static_cell::StaticCell::new();

/// Background storage task that serializes all flash writes.
#[embassy_executor::task]
pub async fn storage_task(
    sd: &'static nrf_softdevice::Softdevice,
    qspi_pins: Option<ferrix_nrf52::qspi::QspiPins>,
) -> ! {
    let mut flash = ferrix_nrf52::flash::take(sd);

    let exit = if let Some(pins) = qspi_pins {
        // The external flash is on a switched rail. Bring it up here rather
        // than at boot, and let the self-test retry if the rail is settling.
        embassy_time::Timer::after_millis(10).await;
        let qspi = QSPI.init(ferrix_nrf52::qspi::init(pins));
        ferrix_nrf52::qspi::self_test(qspi).await;
        defmt::info!("Storage: internal + external flash");
        storage_loop(&mut flash, Some(qspi), &REGIONS).await
    } else {
        defmt::info!("Storage: internal flash only");
        storage_loop(&mut flash, None::<&mut QspiFlash>, &REGIONS).await
    };

    if exit == StorageExit::FactoryReset {
        ferrix_nrf52::reset::system_reset();
    }
    core::future::pending().await
}
