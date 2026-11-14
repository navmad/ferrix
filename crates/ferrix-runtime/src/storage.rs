//! Serialized storage loop.
//!
//! The board owns the flash driver because it is usually `!Send`; this loop
//! owns the policy: coalescing saves, servicing message/contact mutations and
//! performing the factory-reset erase.

use defmt::{error, info};
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_time::Timer;
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use ferrix_core::storage::contacts::ContactOp;
use ferrix_core::storage::{self, adverts, config, contacts, messages, StorageRegions};

/// Why the storage loop exited.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageExit {
    /// Data pages were erased; the caller must reset the node.
    FactoryReset,
}

/// Run the storage loop until a factory reset completes.
pub async fn storage_loop<F>(flash: &mut F, regions: &'static StorageRegions) -> StorageExit
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    contacts::load(flash, regions).await;
    messages::load(flash, regions).await;

    loop {
        let select4_fut = select4(
            config::wait_save_request(),
            contacts::wait_save_request(),
            messages::next_command(),
            adverts::wait_save_request(),
        );
        // Contact removals are out-of-band: they must not wait for the
        // coalescing delay of a save request.
        let op_fut = contacts::wait_operation();

        match select(select(select4_fut, storage::wait_factory_reset()), op_fut).await {
            Either::Second(op) => match op {
                ContactOp::Remove(pub_key) => contacts::remove(flash, regions, pub_key).await,
                ContactOp::Clear => contacts::clear(flash, regions).await,
            },
            Either::First(inner) => match inner {
                Either::Second(_) => {
                    info!("Storage: factory reset - erasing data pages");
                    if flash
                        .erase(regions.factory_reset.start, regions.factory_reset.end)
                        .await
                        .is_err()
                    {
                        error!("Storage: factory reset erase failed");
                    }
                    return StorageExit::FactoryReset;
                }
                Either::First(selected) => match selected {
                    Either4::First(_) => {
                        // Debounce in case several settings are saved together.
                        Timer::after_millis(200).await;
                        config::drain_save_requests();
                        config::save_config(flash, &regions.config).await;
                    }
                    Either4::Second(_) => {
                        // Advert/app bursts arrive together; coalesce them.
                        Timer::after_millis(1000).await;
                        contacts::drain_save_requests();
                        contacts::save(flash, regions).await;
                    }
                    Either4::Third(command) => match command {
                        // One record per message; the FIFO erases a page only
                        // when it wraps, so no batching is needed or wanted.
                        messages::MessageCommand::Append(msg) => {
                            messages::append(flash, regions, &msg).await
                        }
                        messages::MessageCommand::AppendChannelData(data) => {
                            messages::append_channel_data(flash, regions, &data).await
                        }
                        messages::MessageCommand::Consume => {
                            messages::consume(flash, regions).await
                        }
                        messages::MessageCommand::Clear => messages::clear(flash, regions).await,
                    },
                    Either4::Fourth(_) => {
                        // New contacts are rare; batch a discovery burst first.
                        Timer::after_millis(1000).await;
                        adverts::drain_save_requests();
                        adverts::save(flash).await;
                    }
                },
            },
        }
    }
}
