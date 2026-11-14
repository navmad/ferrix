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

use crate::boards::{Board, CurrentBoard};
use ferrix_runtime::storage::{storage_loop, StorageExit};
use heapless::String;

/// Flash layout for ThinkNode M1 (nRF52840, 1 MB).
pub static REGIONS: StorageRegions = StorageRegions {
    config: 0x000E_C000..0x000E_D000,
    contacts: 0x000B_0000..0x000C_C000,
    messages: 0x000C_C000..0x000E_C000,
    adverts: 0x000A_0000..0x000B_0000,
    factory_reset: 0x000A_0000..0x000E_D000,
};

/// Memory-mapped flash reader for the advert log.
struct NrfMappedFlash;

impl MappedFlash for NrfMappedFlash {
    fn read(&self, offset: u32, out: &mut [u8]) {
        // SAFETY: offsets come from `REGIONS`, which is inside the reserved,
        // memory-mapped flash region above the application image.
        unsafe {
            core::ptr::copy_nonoverlapping(offset as *const u8, out.as_mut_ptr(), out.len());
        }
    }
}

static MAPPED_FLASH: NrfMappedFlash = NrfMappedFlash;

/// Initialize configuration from the reserved page, or defaults.
pub fn init_config() -> (String<32>, Option<[u8; 64]>) {
    let len = (REGIONS.config.end - REGIONS.config.start) as usize;
    // SAFETY: the config page is memory-mapped flash.
    let page = unsafe { core::slice::from_raw_parts(REGIONS.config.start as *const u8, len) };
    ferrix_core::storage::config::init_config(page, CurrentBoard::NAME)
}

/// Load the advert log into RAM. Called once at boot, before the tasks start.
pub fn load() {
    adverts::init(&MAPPED_FLASH, REGIONS.adverts.clone());
}

/// Background storage task that serializes all flash writes.
#[embassy_executor::task]
pub async fn storage_task(sd: &'static nrf_softdevice::Softdevice) -> ! {
    let mut flash = nrf_softdevice::Flash::take(sd);
    defmt::info!("Storage: Flash persistence task started");
    if storage_loop(&mut flash, &REGIONS).await == StorageExit::FactoryReset {
        cortex_m::peripheral::SCB::sys_reset();
    }
    core::future::pending().await
}
