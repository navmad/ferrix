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

use ferrix_runtime::storage::{storage_loop, StorageExit};
use heapless::String;

/// Flash layout for ThinkNode M1 (nRF52840, 1 MB internal + 2 MB external).
///
/// The external QSPI regions are offsets into the 2 MB chip.
pub static REGIONS: StorageRegions = StorageRegions {
    // Internal: config and bonds stay here for now.
    config: 0x000E_C000..0x000E_D000,
    bonds: 0x000A_F000..0x000B_0000,
    // External QSPI.
    contacts: 0x0000_0000..0x0002_C000,
    adverts: 0x0002_C000..0x0004_0000,
    messages: 0x0004_0000..0x0014_0000,
    factory_reset: 0x000A_0000..0x000E_D000,
    factory_reset_external: 0x0000_0000..0x0014_0000,
};

/// Initialize configuration from the reserved page, or defaults.
pub fn init_config() -> (String<32>, Option<[u8; 64]>) {
    let len = (REGIONS.config.end - REGIONS.config.start) as usize;
    // SAFETY: the config page is memory-mapped flash.
    let page = unsafe { core::slice::from_raw_parts(REGIONS.config.start as *const u8, len) };
    ferrix_core::storage::config::init_config(page, crate::board::NAME)
}

/// Load the advert log into RAM. Called once at boot, before the tasks start.
pub fn load() {
    ferrix_core::storage::bonds::init(
        &ferrix_nrf52::flash::MAPPED,
        REGIONS.bonds.clone(),
    );
}

/// Spawn the storage task once the SoftDevice has been enabled.
pub fn spawn_storage(
    spawner: embassy_executor::Spawner,
    sd: &'static nrf_softdevice::Softdevice,
    qspi: ferrix_nrf52::qspi::QspiPins,
) {
    let _ = spawner.spawn(storage_task(sd, qspi));
}

static QSPI: static_cell::StaticCell<
    embassy_nrf::qspi::Qspi<'static, embassy_nrf::peripherals::QSPI>,
> = static_cell::StaticCell::new();

/// Background storage task that serializes all flash writes.
#[embassy_executor::task]
pub async fn storage_task(
    sd: &'static nrf_softdevice::Softdevice,
    qspi_pins: ferrix_nrf52::qspi::QspiPins,
) -> ! {
    let mut flash = ferrix_nrf52::flash::take(sd);

    // The external flash is on a switched rail. Bring it up here rather than
    // at boot, and let the self-test retry if the rail is still settling.
    embassy_time::Timer::after_millis(10).await;
    let qspi = QSPI.init(ferrix_nrf52::qspi::init(qspi_pins));
    ferrix_nrf52::qspi::self_test(qspi).await;
    ferrix_core::storage::adverts::init(&mut *qspi, REGIONS.adverts.clone()).await;

    defmt::info!("Storage: Flash persistence task started");
    if storage_loop(&mut flash, qspi, &REGIONS).await == StorageExit::FactoryReset {
        ferrix_nrf52::reset::system_reset();
    }
    core::future::pending().await
}
