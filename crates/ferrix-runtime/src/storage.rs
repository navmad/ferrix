//! Serialized storage loop.
//!
//! The board owns the flash drivers because they are usually `!Send`; this
//! loop owns the policy: coalescing saves, servicing message/contact mutations
//! and performing the factory-reset erase.
//!
//! Config and BLE bonds always live in internal flash. Contacts, advert blobs
//! and messages use external QSPI when the board has it (see
//! `StorageRegions::contacts_external` / `messages_external`) and internal
//! ranges otherwise.

use defmt::{error, info};
use embassy_futures::select::{select, select3, select5, Either, Either3, Either5};
use embassy_time::{Duration, Ticker, Timer};
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use ferrix_core::state::bump_storage_heartbeat;
use ferrix_core::storage::contacts::ContactOp;
use ferrix_core::storage::{self, adverts, bonds, config, contacts, messages, StorageRegions};

/// Why the storage loop exited.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageExit {
    /// Data pages were erased; the caller must reset the node.
    FactoryReset,
}

/// Dispatch a contacts/adverts operation to the configured flash.
macro_rules! contacts_flash {
    ($external:expr, $internal:expr, $is_external:expr, $call:expr) => {{
        if $is_external {
            match $external.as_deref_mut() {
                Some(ext) => $call(ext).await,
                None => error!("Storage: external flash required for contacts/adverts"),
            }
        } else {
            $call($internal).await
        }
    }};
}

/// Dispatch a messages operation to the configured flash.
macro_rules! messages_flash {
    ($external:expr, $internal:expr, $is_external:expr, $call:expr) => {{
        if $is_external {
            match $external.as_deref_mut() {
                Some(ext) => $call(ext).await,
                None => error!("Storage: external flash required for messages"),
            }
        } else {
            $call($internal).await
        }
    }};
}

/// Run the storage loop until a factory reset completes.
pub async fn storage_loop<IF, EF>(
    internal: &mut IF,
    mut external: Option<&mut EF>,
    regions: &'static StorageRegions,
) -> StorageExit
where
    IF: NorFlash + MultiwriteNorFlash,
    EF: NorFlash + MultiwriteNorFlash,
    IF::Error: defmt::Format,
    EF::Error: defmt::Format,
{
    // A bond region left by an older layout cannot be mounted, so reclaim it
    // before the first pairing write.
    bonds::format_if_needed(internal, &regions.bonds).await;
    contacts_flash!(external, internal, regions.contacts_external, |flash| contacts::load(
        flash,
        &regions.contacts
    ));
    messages_flash!(external, internal, regions.messages_external, |flash| messages::load(
        flash,
        &regions.messages
    ));
    let mut heartbeat = Ticker::every(Duration::from_secs(5));

    loop {
        bump_storage_heartbeat();
        // Drain consume requests even if their wake-up message was dropped.
        let consumes = messages::take_pending_consumes();
        for _ in 0..consumes {
            messages_flash!(external, internal, regions.messages_external, |flash| messages::consume(
                flash,
                &regions.messages
            ));
        }
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

        let events_fut = select(select5_fut, storage::wait_factory_reset());
        let side_fut = select(op_fut, storage::wait_flush_request());

        match select3(events_fut, side_fut, heartbeat.next()).await {
            Either3::First(inner) => match inner {
                Either::Second(_) => {
                    info!("Storage: factory reset - erasing data pages");
                    if internal
                        .erase(regions.factory_reset.start, regions.factory_reset.end)
                        .await
                        .is_err()
                    {
                        error!("Storage: internal factory reset erase failed");
                    }
                    if regions.contacts_external || regions.messages_external {
                        if let Some(ext) = external.as_deref_mut() {
                            if ext
                                .erase(
                                    regions.factory_reset_external.start,
                                    regions.factory_reset_external.end,
                                )
                                .await
                                .is_err()
                            {
                                error!("Storage: external factory reset erase failed");
                            }
                        }
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
                        contacts_flash!(external, internal, regions.contacts_external, |flash| contacts::save(
                            flash,
                            &regions.contacts
                        ));
                    }
                    Either5::Third(command) => match command {
                        // One record per message; the FIFO erases a page only
                        // when it wraps, so no batching is needed or wanted.
                        messages::MessageCommand::Append(msg) => {
                            messages_flash!(external, internal, regions.messages_external, |flash| messages::append(
                                flash,
                                &regions.messages,
                                &msg
                            ))
                        }
                        messages::MessageCommand::AppendChannelData(data) => {
                            messages_flash!(external, internal, regions.messages_external, |flash| messages::append_channel_data(
                                flash,
                                &regions.messages,
                                &data
                            ))
                        }
                        messages::MessageCommand::Consume => {
                            let consumes = messages::take_pending_consumes();
                            for _ in 0..consumes {
                                messages_flash!(external, internal, regions.messages_external, |flash| messages::consume(
                                    flash,
                                    &regions.messages
                                ));
                            }
                        }
                        messages::MessageCommand::Clear => {
                            messages_flash!(external, internal, regions.messages_external, |flash| messages::clear(
                                flash,
                                &regions.messages
                            ))
                        }
                    },
                    Either5::Fourth(event) => match event {
                        Either::First(_) => {
                            // New contacts are rare; batch a discovery burst first.
                            Timer::after_millis(1000).await;
                            adverts::drain_save_requests();
                            contacts_flash!(external, internal, regions.contacts_external, |flash| adverts::save(
                                flash,
                                &regions.contacts
                            ));
                        }
                        Either::Second(req) => {
                            contacts_flash!(external, internal, regions.contacts_external, |flash| adverts::serve_fetch(
                                flash,
                                &regions.contacts,
                                req
                            ))
                        }
                    },
                    Either5::Fifth(_) => {
                        bonds::save(internal, &regions.bonds).await;
                    }
                },
            },
            Either3::Second(side) => match side {
                Either::First(op) => match op {
                    ContactOp::Remove(pub_key) => {
                        contacts_flash!(external, internal, regions.contacts_external, |flash| contacts::remove(
                            flash,
                            &regions.contacts,
                            pub_key
                        ))
                    }
                    ContactOp::Clear => contacts_flash!(external, internal, regions.contacts_external, |flash| contacts::clear(
                            flash,
                            &regions.contacts
                        )),
                },
                Either::Second(_) => {
                    // Flush pending writes before reboot/DFU. The blob map
                    // shares the contact region, so one save covers both.
                    let pending_config = config::has_pending_save();
                    let seq = config::drain_save_requests();
                    if pending_config {
                        let before =
                            config::CONFIG_SAVES.load(core::sync::atomic::Ordering::Relaxed);
                        config::save_config(internal, &regions.config).await;
                        if config::CONFIG_SAVES.load(core::sync::atomic::Ordering::Relaxed) != before
                        {
                            config::notify_save_complete(seq);
                        }
                    }
                    contacts::drain_save_requests();
                    contacts_flash!(external, internal, regions.contacts_external, |flash| contacts::save(
                            flash,
                            &regions.contacts
                        ));
                    adverts::drain_save_requests();
                    contacts_flash!(external, internal, regions.contacts_external, |flash| adverts::save(
                                flash,
                                &regions.contacts
                            ));
                    bonds::save(internal, &regions.bonds).await;
                    storage::complete_flush();
                }
            },
            Either3::Third(_) => bump_storage_heartbeat(),
        }
    }
}
