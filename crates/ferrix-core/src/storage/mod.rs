//! Persistent storage formats and handlers, generic over the flash driver.
//!
//! Flash regions are supplied by the board; the storage loop lives in
//! `ferrix-runtime`. All handlers are `no_std` and allocation free.

pub mod adverts;
pub mod bonds;
pub mod config;
pub mod contact_map;
pub mod contacts;
pub mod counted_flash;
pub mod messages;
pub mod util;

use core::ops::Range;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

pub use config::{
    get_callsign, get_path_hash_mode, request_save_config, set_callsign, set_custom_priv_key,
    set_path_hash_mode,
};
pub use contacts::request as request_contacts;
pub use messages::MessageCommand;

/// Flash regions reserved by the board for each store.
#[derive(Clone, Debug)]
pub struct StorageRegions {
    /// Configuration page (one erase sector).
    pub config: Range<u32>,
    /// Contact key-value map (external flash).
    pub contacts: Range<u32>,
    /// Message FIFO (external flash).
    pub messages: Range<u32>,
    /// BLE bond table.
    pub bonds: Range<u32>,
    /// Internal data regions erased by a factory reset.
    pub factory_reset: Range<u32>,
    /// External flash regions erased by a factory reset.
    pub factory_reset_external: Range<u32>,
}

/// Synchronous reader over a memory-mapped flash region.
///
/// Advert blobs are looked up from protocol handlers that cannot await, so the
/// board exposes its mapped flash through this trait.
pub trait MappedFlash: Sync {
    fn read(&self, offset: u32, out: &mut [u8]);
}

/// Request a factory reset: erase all data pages, then reboot.
static RESET_CHANNEL: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();

pub fn request_factory_reset() {
    let _ = RESET_CHANNEL.try_send(());
}

/// Wait for a factory-reset request.
pub async fn wait_factory_reset() {
    RESET_CHANNEL.receive().await;
}

/// Request an immediate flush of pending contact/blob/config/bond writes.
static FLUSH_REQUEST: Channel<CriticalSectionRawMutex, (), 1> = Channel::new();
static FLUSH_DONE: AtomicBool = AtomicBool::new(false);

/// Wait for a flush request from a reboot/DFU path.
pub async fn wait_flush_request() {
    FLUSH_REQUEST.receive().await;
}

/// Mark a flush request as complete.
pub fn complete_flush() {
    FLUSH_DONE.store(true, Ordering::Release);
}

/// Flush pending writes and wait for the storage task to finish.
///
/// Used before reboot/DFU so a just-changed contact or advert is not lost.
pub async fn flush_pending() -> bool {
    #[cfg(test)]
    {
        true
    }
    #[cfg(not(test))]
    {
        FLUSH_DONE.store(false, Ordering::Release);
        // A request may already be queued; that request will complete and
        // set FLUSH_DONE, so a full channel is not an error.
        let _ = FLUSH_REQUEST.try_send(());
        for _ in 0..600 {
            if FLUSH_DONE.load(Ordering::Acquire) {
                return true;
            }
            embassy_time::Timer::after_millis(5).await;
        }
        false
    }
}
