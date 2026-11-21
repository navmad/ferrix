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

pub use ferrix_nrf52::flash::{capture_softdevice_vectors, softdevice_vectors};
use heapless::String;

pub use ferrix_thinknode_m1_logic::layout::{EXTERNAL_REGIONS, INTERNAL_REGIONS};

#[cfg(not(feature = "internal-storage"))]
pub use EXTERNAL_REGIONS as REGIONS;
#[cfg(feature = "internal-storage")]
pub use INTERNAL_REGIONS as REGIONS;

/// Initialize configuration from the journal, or defaults.
pub fn init_config(reader: &dyn MappedFlash) -> (String<32>, Option<[u8; 64]>) {
    ferrix_core::storage::config::init_config(
        reader,
        REGIONS.config.clone(),
        crate::NAME,
    )
}

/// Load the BLE bond map into RAM. Called once at boot, before BLE starts.
pub fn load(reader: &dyn MappedFlash) {
    ferrix_core::storage::bonds::init(
        reader,
        REGIONS.bonds.clone(),
    );
}

/// JEDEC IDs of the external flash fitted to the ThinkNode M1.
#[cfg(not(feature = "internal-storage"))]
static EXTERNAL_FLASH_IDS: [[u8; 3]; 1] = [ferrix_nrf52::qspi::MX25R1635F_ID];

/// Spawn the storage task once the SoftDevice has been enabled.
pub fn spawn_storage(
    spawner: embassy_executor::Spawner,
    sd: &'static nrf_softdevice::Softdevice,
) {
    let pins = ferrix_nrf52::storage::take_qspi();
    let _ = spawner.spawn(storage_task(sd, pins));
}

/// Background storage task that serializes all flash writes.
#[embassy_executor::task]
pub async fn storage_task(
    sd: &'static nrf_softdevice::Softdevice,
    qspi_pins: Option<ferrix_nrf52::qspi::QspiPins>,
) -> ! {
    #[cfg(not(feature = "internal-storage"))]
    {
        ferrix_nrf52::storage::run_external(sd, qspi_pins, &REGIONS, &EXTERNAL_FLASH_IDS).await
    }

    #[cfg(feature = "internal-storage")]
    {
        let _ = qspi_pins;
        ferrix_nrf52::storage::run_internal(sd, &REGIONS).await
    }
}
