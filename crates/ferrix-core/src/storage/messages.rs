//! Persistent message queue.
//!
//! Messages live in a wear-levelled FIFO (`sequential-storage`) over the
//! 128 KB message region. Each message costs one compact record write; a
//! 4 KB page is erased only when the FIFO wraps into it. The previous design
//! rewrote a ~60 KB snapshot (with a page erase) for every message.
//!
//! The queue is the source of truth: a message is popped from flash when the
//! app consumes it, so a reboot never resurrects already-synced messages.

use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::String;
use meshcore_wire::message_record::{ChannelDataRecord, MessageRecord, MESSAGE_RECORD_MAX};
use sequential_storage::cache::{Cache, Uncached};
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use sequential_storage::queue::{QueueConfig, QueueStorage};

use super::counted_flash::CountedFlash;
use crate::state::MESSAGE_LOG;
use crate::types::{InboxItem, ReceivedChannelData, ReceivedMessage, MAX_MESSAGES};

/// Completed record writes and page erases since boot.
pub static RECORD_SAVES: AtomicU32 = AtomicU32::new(0);
pub static PAGE_ERASES: AtomicU32 = AtomicU32::new(0);
/// Records currently held by the flash queue.
pub static QUEUED: AtomicU32 = AtomicU32::new(0);
/// Consumes requested but not yet applied to the flash queue.
pub static PENDING_CONSUMES: AtomicU32 = AtomicU32::new(0);

fn open<'a, F>(flash: &'a mut F, region: &Range<u32>) -> MessageQueue<'a, F>
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    QueueStorage::new(
        CountedFlash::new(flash, &PAGE_ERASES),
        QueueConfig::new(region.clone()),
        Cache::new_uncached(),
    )
}

type MessageCache = Cache<Uncached, Uncached, Uncached>;
type MessageQueue<'a, F> = QueueStorage<CountedFlash<'a, F>, MessageCache>;


/// Work sent to the storage task. The storage task owns the flash, so message
/// mutations are serialized through this channel.
pub enum MessageCommand {
    Append(ReceivedMessage),
    AppendChannelData(ReceivedChannelData),
    Consume,
    Clear,
}

pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, MessageCommand, 32> =
    Channel::new();

/// Wait for the next message mutation.
pub async fn next_command() -> MessageCommand {
    COMMAND_CHANNEL.receive().await
}

pub async fn queue_append(msg: ReceivedMessage) {
    let _ = COMMAND_CHANNEL.send(MessageCommand::Append(msg)).await;
}

/// Non-blocking append for the radio RX loop so storage backpressure never
/// stalls packet handling or ACK transmission.
#[inline(never)]
pub fn try_queue_append(msg: ReceivedMessage) -> bool {
    COMMAND_CHANNEL.try_send(MessageCommand::Append(msg)).is_ok()
}

pub async fn queue_append_channel_data(data: ReceivedChannelData) {
    let _ = COMMAND_CHANNEL
        .send(MessageCommand::AppendChannelData(data))
        .await;
}

/// Non-blocking channel-data append for the radio RX loop.
#[inline(never)]
pub fn try_queue_append_channel_data(data: ReceivedChannelData) -> bool {
    COMMAND_CHANNEL
        .try_send(MessageCommand::AppendChannelData(data))
        .is_ok()
}

pub async fn queue_consume() {
    PENDING_CONSUMES.fetch_add(1, Ordering::Relaxed);
    let _ = COMMAND_CHANNEL.send(MessageCommand::Consume).await;
}

/// Non-blocking variant for command handlers that cannot await.
///
/// The counter is the source of truth; the channel message only wakes the
/// storage task. If the channel is full, the next drain still sees the count.
pub fn queue_consume_now() {
    PENDING_CONSUMES.fetch_add(1, Ordering::Relaxed);
    let _ = COMMAND_CHANNEL.try_send(MessageCommand::Consume);
}

/// Take all pending consume requests.
pub fn take_pending_consumes() -> u32 {
    PENDING_CONSUMES.swap(0, Ordering::AcqRel)
}

pub async fn queue_clear() {
    let _ = COMMAND_CHANNEL.send(MessageCommand::Clear).await;
}

fn message_to_record(m: &ReceivedMessage) -> MessageRecord {
    let mut rec = MessageRecord::new(
        m.channel_idx,
        m.path_len,
        m.is_direct,
        m.sender.as_str(),
        m.sender_pub,
        m.text.as_str(),
        m.timestamp,
        m.rssi,
        m.snr,
    );
    rec.txt_type = m.txt_type;
    rec.signed_prefix = m.signed_prefix;
    rec
}

fn record_to_message(r: &MessageRecord) -> ReceivedMessage {
    let mut sender = String::<32>::new();
    let _ = sender.push_str(r.sender_str());
    let mut text = String::<160>::new();
    let _ = text.push_str(r.text_str());
    ReceivedMessage {
        channel_idx: r.channel_idx,
        path_len: r.path_len,
        sender,
        sender_pub: r.sender_pub,
        text,
        timestamp: r.timestamp,
        rssi: r.rssi,
        snr: r.snr,
        is_direct: r.is_direct,
        txt_type: r.txt_type,
        signed_prefix: r.signed_prefix,
    }
}

async fn has_magic<F>(flash: &mut F, addr: u32, magic: u32) -> bool
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut buf = super::util::AlignedBuf::<4>::new();
    flash.read(addr, &mut buf.0).await.is_ok() && u32::from_le_bytes(buf.0) == magic
}

/// Replace `MESSAGE_LOG` with the unconsumed messages from flash.
pub async fn load<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    MESSAGE_LOG.lock(|m| m.borrow_mut().clear());

    // The previous firmware stored two whole-log snapshots in this region.
    // Their record format is not compatible with the FIFO, so clear the legacy
    // banks once instead of making the queue repair arbitrary data.
    const LEGACY_SNAPSHOT_MAGIC: u32 = 0x4D53_474D;
    let legacy = has_magic(flash, region.start, LEGACY_SNAPSHOT_MAGIC).await
        || has_magic(flash, region.start + 0x1_0000, LEGACY_SNAPSHOT_MAGIC).await;

    let mut queue = open(flash, region);
    if legacy {
        info!("Persistence: clearing legacy message snapshot banks");
        if let Err(e) = queue.erase_all().await {
            error!("Persistence: legacy message erase failed: {:?}", e);
        }
    }

    let mut buf = super::util::AlignedBuf::<MESSAGE_RECORD_MAX>::new();
    let mut count = 0usize;
    {
        match queue.iter().await {
            Ok(mut iter) => {
                while let Ok(Some(entry)) = iter.next(&mut buf.0).await {
                    let bytes = entry.into_buf();
                    let version = bytes.first().copied().unwrap_or(0);
                    let item = if version == 1 || version == 2 {
                        MessageRecord::decode(bytes)
                            .map(|rec| InboxItem::Text(record_to_message(&rec)))
                    } else if version == 3 {
                        ChannelDataRecord::decode(bytes).map(|rec| {
                            InboxItem::ChannelData(ReceivedChannelData {
                                channel_idx: rec.channel_idx,
                                path_len: rec.path_len,
                                snr: rec.snr,
                                data_type: rec.data_type,
                                data: rec.data,
                            })
                        })
                    } else {
                        None
                    };
                    let Some(item) = item else {
                        continue;
                    };
                    MESSAGE_LOG.lock(|m| {
                        let mut list = m.borrow_mut();
                        if list.len() >= MAX_MESSAGES {
                            list.remove(0);
                        }
                        let _ = list.push(item);
                    });
                    count += 1;
                }
            }
            Err(e) => error!("Persistence: message queue load failed: {:?}", e),
        }
    }

    // Keep the queue and the RAM log aligned if a previous session left more
    // records than the companion window (for example after a lost consume).
    if count > MAX_MESSAGES {
        for _ in 0..(count - MAX_MESSAGES) {
            if queue.pop(&mut buf.0).await.is_err() {
                break;
            }
        }
    }
    QUEUED.store(count.min(MAX_MESSAGES) as u32, Ordering::Relaxed);
    info!("Persistence: loaded {} messages from queue", count);
}

/// Append one message, dropping the oldest record when the region is full.
pub async fn append<F>(flash: &mut F, region: &Range<u32>, msg: &ReceivedMessage)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    // The SoftDevice flash driver requires a word-aligned write buffer.
    #[repr(C, align(4))]
    struct Aligned([u8; MESSAGE_RECORD_MAX]);

    let encoded = message_to_record(msg);
    let mut aligned = Aligned([0u8; MESSAGE_RECORD_MAX]);
    let Some(len) = encoded.encode(&mut aligned.0) else {
        error!("Persistence: message record too large");
        return;
    };
    let mut queue = open(flash, region);
    match queue.push(&aligned.0[..len], true).await {
        Ok(()) => {
            RECORD_SAVES.fetch_add(1, Ordering::Relaxed);
            QUEUED.fetch_add(1, Ordering::Relaxed);
        }
        Err(e) => error!("Persistence: message queue push failed: {:?}", e),
    }
}

/// Append one channel datagram.
pub async fn append_channel_data<F>(flash: &mut F, region: &Range<u32>, data: &ReceivedChannelData)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    #[repr(C, align(4))]
    struct Aligned([u8; MESSAGE_RECORD_MAX]);

    let Some(rec) = ChannelDataRecord::new(
        data.channel_idx,
        data.path_len,
        data.snr,
        data.data_type,
        &data.data,
    ) else {
        error!("Persistence: channel data too large");
        return;
    };
    let mut aligned = Aligned([0u8; MESSAGE_RECORD_MAX]);
    let Some(len) = rec.encode(&mut aligned.0) else {
        error!("Persistence: channel data record too large");
        return;
    };
    let mut queue = open(flash, region);
    match queue.push(&aligned.0[..len], true).await {
        Ok(()) => {
            RECORD_SAVES.fetch_add(1, Ordering::Relaxed);
            QUEUED.fetch_add(1, Ordering::Relaxed);
        }
        Err(e) => error!("Persistence: channel data push failed: {:?}", e),
    }
}

/// Drop the oldest message (the app synced it).
pub async fn consume<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut buf = super::util::AlignedBuf::<MESSAGE_RECORD_MAX>::new();
    let mut queue = open(flash, region);
    match queue.pop(&mut buf.0).await {
        Ok(Some(_)) => {
            let _ = QUEUED.try_update(Ordering::Relaxed, Ordering::Relaxed, |n| {
                Some(n.saturating_sub(1))
            });
        }
        Ok(None) => {}
        Err(e) => error!("Persistence: message queue pop failed: {:?}", e),
    }
}

/// Erase every stored message.
pub async fn clear<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let mut queue = open(flash, region);
    if let Err(e) = queue.erase_all().await {
        error!("Persistence: message queue clear failed: {:?}", e);
    } else {
        QUEUED.store(0, Ordering::Relaxed);
    }
}
