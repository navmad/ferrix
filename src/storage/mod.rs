//! Persistent storage for the companion.
//!
//! - `config`   — identity, radio settings, channels (one 4 KB page).
//! - `contacts` — two-bank contact snapshots, written only on structural change.
//! - `messages` — wear-levelled persistent message queue.
//! - `snapshot` — shared two-bank snapshot format and save/load logic.

pub mod adverts;
pub mod config;
pub mod contacts;
pub mod messages;
pub mod snapshot;

use defmt::{error, info};
use embedded_storage_async::nor_flash::NorFlash;
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_time::Timer;

pub use config::{
    get_callsign, get_path_hash_mode, init_config, request_save_config, set_callsign,
    set_custom_priv_key, set_path_hash_mode,
};
pub use contacts::request as request_contacts;
pub use messages::MessageCommand;

/// Request a factory reset: erase all data pages, then reboot.
static RESET_CHANNEL: embassy_sync::channel::Channel<
    embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex,
    (),
    1,
> = embassy_sync::channel::Channel::new();

pub fn request_factory_reset() {
    let _ = RESET_CHANNEL.try_send(());
}

/// Load the contact snapshot and advert log into RAM. Called once at boot,
/// before the tasks start. Messages are loaded by the storage task itself
/// because the queue owns the flash instance.
pub fn load() {
    contacts::load();
    adverts::init();
}

/// Background storage task that serializes all flash writes.
#[embassy_executor::task]
pub async fn storage_task(sd: &'static nrf_softdevice::Softdevice) -> ! {
    let mut flash = nrf_softdevice::Flash::take(sd);
    info!("Storage: Flash persistence task started");
    messages::load(&mut flash).await;

    loop {
        let select4_fut = select4(
            config::SAVE_REQUEST_CHANNEL.receive(),
            contacts::SAVE_CHANNEL.receive(),
            messages::COMMAND_CHANNEL.receive(),
            adverts::SAVE_CHANNEL.receive(),
        );
        match select(select4_fut, RESET_CHANNEL.receive()).await {
            Either::Second(_) => {
                info!("Storage: factory reset - erasing data pages");
                if flash.erase(0x000A_0000, 0x000E_D000).await.is_err() {
                    error!("Storage: factory reset erase failed");
                }
                cortex_m::peripheral::SCB::sys_reset();
            }
            Either::First(selected) => match selected {
            Either4::First(_) => {
                // Debounce in case several settings are saved together.
                Timer::after_millis(200).await;
                while config::SAVE_REQUEST_CHANNEL.try_receive().is_ok() {}
                config::save_config(&mut flash).await;
            }
            Either4::Second(_) => {
                // Advert/app bursts arrive together; coalesce them.
                Timer::after_millis(1000).await;
                while contacts::SAVE_CHANNEL.try_receive().is_ok() {}
                contacts::save(&mut flash).await;
            }
            Either4::Third(command) => match command {
                // One record per message; the FIFO erases a page only when
                // it wraps, so no batching is needed or wanted.
                messages::MessageCommand::Append(msg) => {
                    messages::append(&mut flash, &msg).await
                }
                messages::MessageCommand::Consume => {
                    messages::consume(&mut flash).await
                }
                messages::MessageCommand::Clear => messages::clear(&mut flash).await,
            },
            Either4::Fourth(_) => {
                // New contacts are rare; batch a discovery burst first.
                Timer::after_millis(1000).await;
                while adverts::SAVE_CHANNEL.try_receive().is_ok() {}
                adverts::save(&mut flash).await;
            }
            },
        }
    }
}
