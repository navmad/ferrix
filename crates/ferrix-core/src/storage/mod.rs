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
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

pub use config::{
    get_callsign, get_path_hash_mode, request_save_config, set_callsign, set_custom_priv_key,
    set_path_hash_mode,
};
pub use contacts::request as request_contacts;
pub use messages::MessageCommand;

/// Flash regions reserved by the board for each store.
///
/// The board also chooses a compile-time profile: internal contacts/messages
/// (all four stores on the MCU flash) or external contacts/messages (those two
/// on the QSPI flash). The profile selects the storage-loop entry point; the
/// ranges below are interpreted against that flash.
#[derive(Clone, Debug)]
pub struct StorageRegions {
    /// Configuration journal (always internal).
    pub config: Range<u32>,
    /// BLE bond map (always internal).
    pub bonds: Range<u32>,
    /// Contact/advert key-value map.
    pub contacts: Range<u32>,
    /// Message FIFO.
    pub messages: Range<u32>,
    /// Internal data regions erased by a factory reset.
    pub factory_reset: Range<u32>,
    /// External flash regions erased by a factory reset.
    pub factory_reset_external: Range<u32>,
}

impl StorageRegions {
    /// Validate a profile whose contacts/adverts and messages are external.
    pub const fn validate_external(&self) -> bool {
        range_valid(&self.config)
            && range_valid(&self.bonds)
            && range_valid(&self.contacts)
            && range_valid(&self.messages)
            && !overlaps(&self.contacts, &self.messages)
            && contains(&self.factory_reset, &self.config)
            && contains(&self.factory_reset, &self.bonds)
            && contains(&self.factory_reset_external, &self.contacts)
            && contains(&self.factory_reset_external, &self.messages)
    }

    /// Validate a profile whose contacts/adverts and messages are internal.
    pub const fn validate_internal(&self) -> bool {
        range_valid(&self.config)
            && range_valid(&self.bonds)
            && range_valid(&self.contacts)
            && range_valid(&self.messages)
            && !overlaps(&self.contacts, &self.messages)
            && !overlaps(&self.config, &self.contacts)
            && !overlaps(&self.config, &self.messages)
            && !overlaps(&self.bonds, &self.contacts)
            && !overlaps(&self.bonds, &self.messages)
            && contains(&self.factory_reset, &self.config)
            && contains(&self.factory_reset, &self.bonds)
            && contains(&self.factory_reset, &self.contacts)
            && contains(&self.factory_reset, &self.messages)
    }
}

const fn range_valid(range: &Range<u32>) -> bool {
    range.start < range.end && range.start % 4096 == 0 && range.end % 4096 == 0
}

const fn overlaps(a: &Range<u32>, b: &Range<u32>) -> bool {
    a.start < b.end && b.start < a.end
}

const fn contains(outer: &Range<u32>, inner: &Range<u32>) -> bool {
    outer.start <= inner.start && inner.end <= outer.end
}

/// True when the board expects external storage but the QSPI flash is not
/// usable. External stores are skipped; they are never redirected to internal.
pub static EXTERNAL_UNAVAILABLE: AtomicBool = AtomicBool::new(false);
/// External-store operations skipped because the flash is unavailable.
pub static EXTERNAL_DROPS: AtomicU32 = AtomicU32::new(0);

/// Mark external storage unavailable. Logs once, not per skipped operation.
pub fn mark_external_unavailable() {
    if !EXTERNAL_UNAVAILABLE.swap(true, Ordering::Relaxed) {
        #[cfg(not(test))]
        defmt::error!("Storage: external flash unavailable");
    }
}

/// Count an external-store operation that could not run.
pub fn note_external_skip() {
    mark_external_unavailable();
    EXTERNAL_DROPS.fetch_add(1, Ordering::Relaxed);
}

pub fn external_unavailable() -> bool {
    EXTERNAL_UNAVAILABLE.load(Ordering::Relaxed)
}

pub fn external_drops() -> u32 {
    EXTERNAL_DROPS.load(Ordering::Relaxed)
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
