//! Serialized storage loop.
//!
//! The board owns the flash drivers because they are usually `!Send`; this
//! loop owns the policy: coalescing saves, servicing message/contact mutations
//! and performing the factory-reset erase.
//!
//! Config and BLE bonds always live in internal flash. The board chooses a
//! compile-time profile for the other two stores: external QSPI or internal
//! flash. Each profile is a distinct [`Stores`] implementation, so only the
//! selected flash type and its storage state machines are instantiated.

use core::ops::Range;
use core::sync::atomic::Ordering;

use defmt::{error, info};
use embassy_futures::select::{select, select3, select5, Either, Either3, Either5};
use embassy_time::{Duration, Ticker, Timer};
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use ferrix_core::state::bump_storage_heartbeat;
use ferrix_core::storage::adverts::FetchRequest;
use ferrix_core::storage::contacts::ContactOp;
use ferrix_core::storage::{
    self, adverts, bonds, config, contacts, messages, MessageCommand, StorageRegions,
};

/// Why the storage loop exited.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StorageExit {
    /// Data pages were erased; the caller must reset the node.
    FactoryReset,
}

/// Per-profile operations used by the generic loop.
///
/// Implementations are statically dispatched, so the loop contains no flash
/// type branch and a build only generates the profile it links.
trait Stores {
    async fn init(&mut self);
    async fn save_config(&mut self, seq: u32);
    async fn save_contacts(&mut self);
    async fn save_adverts(&mut self);
    async fn save_bonds(&mut self);
    async fn handle_contact_op(&mut self, op: ContactOp);
    async fn handle_message(&mut self, command: MessageCommand);
    async fn consume_messages(&mut self, count: u32);
    async fn serve_advert(&mut self, request: FetchRequest);
    async fn flush(&mut self);
    async fn factory_reset(&mut self);
}

/// Save config and acknowledge the highest drained request.
async fn save_config_now<F>(flash: &mut F, region: &Range<u32>, seq: u32)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let ack = seq.max(config::drain_save_requests());
    let before = config::CONFIG_SAVES.load(Ordering::Relaxed);
    config::save_config(flash, region).await;
    if config::CONFIG_SAVES.load(Ordering::Relaxed) != before {
        config::notify_save_complete(ack);
    }
}

/// External-QSPI profile: contacts/adverts and messages on the external chip.
struct ExternalStores<'a, IF, EF> {
    internal: &'a mut IF,
    external: Option<&'a mut EF>,
    regions: &'static StorageRegions,
}

impl<IF, EF> Stores for ExternalStores<'_, IF, EF>
where
    IF: NorFlash + MultiwriteNorFlash,
    EF: NorFlash + MultiwriteNorFlash,
    IF::Error: defmt::Format,
    EF::Error: defmt::Format,
{
    async fn init(&mut self) {
        bonds::format_if_needed(self.internal, &self.regions.bonds).await;
        match self.external.as_deref_mut() {
            Some(flash) => {
                contacts::load(flash, &self.regions.contacts).await;
                messages::load(flash, &self.regions.messages).await;
            }
            None => storage::note_external_skip(),
        }
    }

    async fn save_config(&mut self, seq: u32) {
        if seq == 0 {
            Timer::after_millis(200).await;
        }
        save_config_now(self.internal, &self.regions.config, seq).await;
    }

    async fn save_contacts(&mut self) {
        match self.external.as_deref_mut() {
            Some(flash) => contacts::save(flash, &self.regions.contacts).await,
            None => storage::note_external_skip(),
        }
    }

    async fn save_adverts(&mut self) {
        match self.external.as_deref_mut() {
            Some(flash) => adverts::save(flash, &self.regions.contacts).await,
            None => storage::note_external_skip(),
        }
    }

    async fn save_bonds(&mut self) {
        bonds::save(self.internal, &self.regions.bonds).await;
    }

    async fn handle_contact_op(&mut self, op: ContactOp) {
        match op {
            ContactOp::Remove(pub_key) => match self.external.as_deref_mut() {
                Some(flash) => contacts::remove(flash, &self.regions.contacts, pub_key).await,
                None => storage::note_external_skip(),
            },
            ContactOp::Clear => match self.external.as_deref_mut() {
                Some(flash) => contacts::clear(flash, &self.regions.contacts).await,
                None => storage::note_external_skip(),
            },
        }
    }

    async fn handle_message(&mut self, command: MessageCommand) {
        match command {
            MessageCommand::Append(msg) => match self.external.as_deref_mut() {
                Some(flash) => messages::append(flash, &self.regions.messages, &msg).await,
                None => storage::note_external_skip(),
            },
            MessageCommand::AppendChannelData(data) => match self.external.as_deref_mut() {
                Some(flash) => {
                    messages::append_channel_data(flash, &self.regions.messages, &data).await
                }
                None => storage::note_external_skip(),
            },
            MessageCommand::Consume => {
                let count = messages::take_pending_consumes();
                self.consume_messages(count).await;
            }
            MessageCommand::Clear => match self.external.as_deref_mut() {
                Some(flash) => messages::clear(flash, &self.regions.messages).await,
                None => storage::note_external_skip(),
            },
        }
    }

    async fn consume_messages(&mut self, count: u32) {
        for _ in 0..count {
            match self.external.as_deref_mut() {
                Some(flash) => messages::consume(flash, &self.regions.messages).await,
                None => {
                    storage::note_external_skip();
                    return;
                }
            }
        }
    }

    async fn serve_advert(&mut self, request: FetchRequest) {
        match self.external.as_deref_mut() {
            Some(flash) => adverts::serve_fetch(flash, &self.regions.contacts, request).await,
            None => storage::note_external_skip(),
        }
    }

    async fn flush(&mut self) {
        let pending_config = config::has_pending_save();
        let seq = config::drain_save_requests();
        if pending_config {
            save_config_now(self.internal, &self.regions.config, seq).await;
        }
        contacts::drain_save_requests();
        self.save_contacts().await;
        adverts::drain_save_requests();
        self.save_adverts().await;
        self.save_bonds().await;
        storage::complete_flush();
    }

    async fn factory_reset(&mut self) {
        if self
            .internal
            .erase(self.regions.factory_reset.start, self.regions.factory_reset.end)
            .await
            .is_err()
        {
            error!("Storage: internal factory reset erase failed");
        }
        match self.external.as_deref_mut() {
            Some(flash) => {
                if flash
                    .erase(
                        self.regions.factory_reset_external.start,
                        self.regions.factory_reset_external.end,
                    )
                    .await
                    .is_err()
                {
                    error!("Storage: external factory reset erase failed");
                }
            }
            None => storage::note_external_skip(),
        }
    }
}

/// Internal-flash profile: every store lives on the MCU flash.
struct InternalStores<'a, F> {
    flash: &'a mut F,
    regions: &'static StorageRegions,
}

impl<F> Stores for InternalStores<'_, F>
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    async fn init(&mut self) {
        bonds::format_if_needed(self.flash, &self.regions.bonds).await;
        contacts::load(self.flash, &self.regions.contacts).await;
        messages::load(self.flash, &self.regions.messages).await;
    }

    async fn save_config(&mut self, seq: u32) {
        if seq == 0 {
            Timer::after_millis(200).await;
        }
        save_config_now(self.flash, &self.regions.config, seq).await;
    }

    async fn save_contacts(&mut self) {
        contacts::save(self.flash, &self.regions.contacts).await;
    }

    async fn save_adverts(&mut self) {
        adverts::save(self.flash, &self.regions.contacts).await;
    }

    async fn save_bonds(&mut self) {
        bonds::save(self.flash, &self.regions.bonds).await;
    }

    async fn handle_contact_op(&mut self, op: ContactOp) {
        match op {
            ContactOp::Remove(pub_key) => {
                contacts::remove(self.flash, &self.regions.contacts, pub_key).await
            }
            ContactOp::Clear => contacts::clear(self.flash, &self.regions.contacts).await,
        }
    }

    async fn handle_message(&mut self, command: MessageCommand) {
        match command {
            MessageCommand::Append(msg) => {
                messages::append(self.flash, &self.regions.messages, &msg).await
            }
            MessageCommand::AppendChannelData(data) => {
                messages::append_channel_data(self.flash, &self.regions.messages, &data).await
            }
            MessageCommand::Consume => {
                let count = messages::take_pending_consumes();
                self.consume_messages(count).await;
            }
            MessageCommand::Clear => messages::clear(self.flash, &self.regions.messages).await,
        }
    }

    async fn consume_messages(&mut self, count: u32) {
        for _ in 0..count {
            messages::consume(self.flash, &self.regions.messages).await;
        }
    }

    async fn serve_advert(&mut self, request: FetchRequest) {
        adverts::serve_fetch(self.flash, &self.regions.contacts, request).await;
    }

    async fn flush(&mut self) {
        let pending_config = config::has_pending_save();
        let seq = config::drain_save_requests();
        if pending_config {
            save_config_now(self.flash, &self.regions.config, seq).await;
        }
        contacts::drain_save_requests();
        self.save_contacts().await;
        adverts::drain_save_requests();
        self.save_adverts().await;
        self.save_bonds().await;
        storage::complete_flush();
    }

    async fn factory_reset(&mut self) {
        if self
            .flash
            .erase(self.regions.factory_reset.start, self.regions.factory_reset.end)
            .await
            .is_err()
        {
            error!("Storage: internal factory reset erase failed");
        }
    }
}

/// Run the storage loop for a board whose contacts/adverts and messages are
/// external. `external` may be `None` when the QSPI probe failed; those stores
/// are then skipped and reported, never redirected to internal flash.
pub async fn storage_loop_external<IF, EF>(
    internal: &mut IF,
    external: Option<&mut EF>,
    regions: &'static StorageRegions,
) -> StorageExit
where
    IF: NorFlash + MultiwriteNorFlash,
    EF: NorFlash + MultiwriteNorFlash,
    IF::Error: defmt::Format,
    EF::Error: defmt::Format,
{
    let mut stores = ExternalStores {
        internal,
        external,
        regions,
    };
    run_loop(&mut stores).await
}

/// Run the storage loop for a board whose contacts/adverts and messages are
/// internal.
pub async fn storage_loop_internal<F>(
    flash: &mut F,
    regions: &'static StorageRegions,
) -> StorageExit
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut stores = InternalStores { flash, regions };
    run_loop(&mut stores).await
}

/// Profile-independent event loop.
async fn run_loop<S: Stores>(stores: &mut S) -> StorageExit {
    stores.init().await;
    let mut heartbeat = Ticker::every(Duration::from_secs(5));

    loop {
        bump_storage_heartbeat();
        // Drain consume requests even if their wake-up message was dropped.
        let consumes = messages::take_pending_consumes();
        if consumes > 0 {
            stores.consume_messages(consumes).await;
        }

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
                    stores.factory_reset().await;
                    return StorageExit::FactoryReset;
                }
                Either::First(selected) => match selected {
                    Either5::First(seq) => stores.save_config(seq).await,
                    Either5::Second(_) => {
                        // Advert/app bursts arrive together; coalesce them.
                        Timer::after_millis(1000).await;
                        contacts::drain_save_requests();
                        stores.save_contacts().await;
                    }
                    Either5::Third(command) => stores.handle_message(command).await,
                    Either5::Fourth(event) => match event {
                        Either::First(_) => {
                            // New contacts are rare; batch a discovery burst.
                            Timer::after_millis(1000).await;
                            adverts::drain_save_requests();
                            stores.save_adverts().await;
                        }
                        Either::Second(request) => stores.serve_advert(request).await,
                    },
                    Either5::Fifth(_) => stores.save_bonds().await,
                },
            },
            Either3::Second(side) => match side {
                Either::First(op) => stores.handle_contact_op(op).await,
                Either::Second(_) => stores.flush().await,
            },
            Either3::Third(_) => bump_storage_heartbeat(),
        }
    }
}
