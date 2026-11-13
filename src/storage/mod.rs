//! Persistent storage for the companion.
//!
//! - `config`   — identity, radio settings, channels (one 4 KB page).
//! - `contacts` — two-bank contact snapshots, written only on structural change.
//! - `messages` — two-bank message-log snapshots.
//! - `snapshot` — shared two-bank snapshot format and save/load logic.

pub mod adverts;
pub mod config;
pub mod contacts;
pub mod messages;
pub mod snapshot;

use defmt::info;
use embassy_futures::select::{select4, Either4};
use embassy_time::Timer;

pub use config::{
    get_callsign, get_path_hash_mode, init_config, request_save_config, set_callsign,
    set_custom_priv_key, set_path_hash_mode,
};
pub use contacts::request as request_contacts;
pub use messages::request as request_messages;

/// Load the contact and message snapshots into RAM. Called once at boot,
/// before the tasks start.
pub fn load() {
    contacts::load();
    messages::load();
    adverts::init();
}

/// Background storage task that serializes all flash writes.
#[embassy_executor::task]
pub async fn storage_task(sd: &'static nrf_softdevice::Softdevice) -> ! {
    let mut flash = nrf_softdevice::Flash::take(sd);
    info!("Storage: Flash persistence task started");

    loop {
        match select4(
            config::SAVE_REQUEST_CHANNEL.receive(),
            contacts::SAVE_CHANNEL.receive(),
            messages::SAVE_CHANNEL.receive(),
            adverts::SAVE_CHANNEL.receive(),
        )
        .await
        {
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
            Either4::Third(_) => {
                // Coalesce bursts: a 256-message snapshot writes ~60 KB, so
                // don't do it once per received message.
                Timer::after_millis(5000).await;
                while messages::SAVE_CHANNEL.try_receive().is_ok() {}
                messages::save(&mut flash).await;
            }
            Either4::Fourth(_) => {
                // New contacts are rare; batch a discovery burst first.
                Timer::after_millis(1000).await;
                while adverts::SAVE_CHANNEL.try_receive().is_ok() {}
                adverts::save(&mut flash).await;
            }
        }
    }
}
