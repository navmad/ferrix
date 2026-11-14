//! Serialized storage loop.
//!
//! The board owns the flash drivers because they are usually `!Send`; this
//! loop owns the policy: coalescing saves, servicing message/contact mutations
//! and performing the factory-reset erase.
//!
//! Internal flash holds config, bonds and (for now) the advert blob log.
//! External QSPI flash holds contacts and messages.

use defmt::{error, info};
use embassy_futures::select::{select, select5, Either, Either5};
use embassy_time::Timer;
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use ferrix_core::storage::contacts::ContactOp;
use ferrix_core::storage::{self, adverts, bonds, config, contacts, messages, StorageRegions};

/// Why the storage loop exited.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageExit {
    /// Data pages were erased; the caller must reset the node.
    FactoryReset,
}

/// Run the storage loop until a factory reset completes.
pub async fn storage_loop<IF, EF>(
    internal: &mut IF,
    external: &mut EF,
    regions: &'static StorageRegions,
) -> StorageExit
where
    IF: NorFlash + MultiwriteNorFlash,
    EF: NorFlash + MultiwriteNorFlash,
    IF::Error: defmt::Format,
    EF::Error: defmt::Format,
{
    contacts::load(external, &regions.contacts).await;
    messages::load(external, &regions.messages).await;

    loop {
        // The advert lane handles both coalesced saves and synchronous
        // protocol fetches, which are served from the external flash.
        let advert_fut = select(
            adverts::wait_save_request(),
            adverts::wait_fetch_request(),
        );
        let select5_fut = select5(
            config::wait_save_request(),
            contacts::wait_save_request(),
            messages::next_command(),
            advert_fut,
            bonds::wait_save_request(),
        );
        // Contact removals are out-of-band: they must not wait for the
        // coalescing delay of a save request.
        let op_fut = contacts::wait_operation();

        match select(select(select5_fut, storage::wait_factory_reset()), op_fut).await {
            Either::Second(op) => match op {
                ContactOp::Remove(pub_key) => {
                    contacts::remove(external, &regions.contacts, pub_key).await
                }
                ContactOp::Clear => contacts::clear(external, &regions.contacts).await,
            },
            Either::First(inner) => match inner {
                Either::Second(_) => {
                    info!("Storage: factory reset - erasing data pages");
                    if internal
                        .erase(regions.factory_reset.start, regions.factory_reset.end)
                        .await
                        .is_err()
                    {
                        error!("Storage: internal factory reset erase failed");
                    }
                    if external
                        .erase(
                            regions.factory_reset_external.start,
                            regions.factory_reset_external.end,
                        )
                        .await
                        .is_err()
                    {
                        error!("Storage: external factory reset erase failed");
                    }
                    return StorageExit::FactoryReset;
                }
                Either::First(selected) => match selected {
                    Either5::First(seq) => {
                        // A caller waiting for durability (for example a BLE
                        // PIN change, which the app applies by rebooting) must
                        // not be delayed. Ordinary setting changes still
                        // coalesce for 200 ms.
                        if seq == 0 {
                            Timer::after_millis(200).await;
                        }
                        let ack = seq.max(config::drain_save_requests());
                        let before =
                            config::CONFIG_SAVES.load(core::sync::atomic::Ordering::Relaxed);
                        config::save_config(internal, &regions.config).await;
                        if config::CONFIG_SAVES.load(core::sync::atomic::Ordering::Relaxed) != before
                        {
                            config::notify_save_complete(ack);
                        }
                    }
                    Either5::Second(_) => {
                        // Advert/app bursts arrive together; coalesce them.
                        Timer::after_millis(1000).await;
                        contacts::drain_save_requests();
                        contacts::save(external, &regions.contacts).await;
                    }
                    Either5::Third(command) => match command {
                        // One record per message; the FIFO erases a page only
                        // when it wraps, so no batching is needed or wanted.
                        messages::MessageCommand::Append(msg) => {
                            messages::append(external, &regions.messages, &msg).await
                        }
                        messages::MessageCommand::AppendChannelData(data) => {
                            messages::append_channel_data(external, &regions.messages, &data).await
                        }
                        messages::MessageCommand::Consume => {
                            messages::consume(external, &regions.messages).await
                        }
                        messages::MessageCommand::Clear => {
                            messages::clear(external, &regions.messages).await
                        }
                    },
                    Either5::Fourth(event) => match event {
                        Either::First(_) => {
                            // New contacts are rare; batch a discovery burst first.
                            Timer::after_millis(1000).await;
                            adverts::drain_save_requests();
                            adverts::save(external).await;
                        }
                        Either::Second(req) => adverts::serve_fetch(external, req).await,
                    },
                    Either5::Fifth(_) => {
                        bonds::save(internal, &regions.bonds).await;
                    }
                },
            },
        }
    }
}
