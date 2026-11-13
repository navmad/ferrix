//! Persistent message queue.
//!
//! Messages live in a wear-levelled FIFO (`sequential-storage`) over the
//! 128 KB message region. Each message costs one fixed-size record write; a
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
use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, ReadNorFlash,
};
use heapless::String;
use meshcore_wire::message_record::{MessageRecord, MESSAGE_RECORD_SIZE};
use nrf_softdevice::Flash;
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::queue::{QueueConfig, QueueStorage};

use crate::companion::usb::{MESSAGE_LOG, MAX_MESSAGES};
use crate::meshcore::mesh::ReceivedMessage;

/// 0xCC000..0xEC000: 32 pages of 4 KB.
const REGION: Range<u32> = 0x000C_C000..0x000E_C000;

/// Completed record writes and page erases since boot.
pub static RECORD_SAVES: AtomicU32 = AtomicU32::new(0);
pub static PAGE_ERASES: AtomicU32 = AtomicU32::new(0);

/// Flash wrapper that counts pages erased by the queue.
struct CountingFlash<'a>(&'a mut Flash);

impl ErrorType for CountingFlash<'_> {
    type Error = <Flash as ErrorType>::Error;
}

impl ReadNorFlash for CountingFlash<'_> {
    const READ_SIZE: usize = <Flash as ReadNorFlash>::READ_SIZE;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(offset, bytes).await
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl NorFlash for CountingFlash<'_> {
    const WRITE_SIZE: usize = <Flash as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <Flash as NorFlash>::ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let result = self.0.erase(from, to).await;
        if result.is_ok() {
            let pages = (to.saturating_sub(from) / Self::ERASE_SIZE as u32) as u32;
            PAGE_ERASES.fetch_add(pages.max(1), Ordering::Relaxed);
        }
        result
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.write(offset, bytes).await
    }
}

impl MultiwriteNorFlash for CountingFlash<'_> {}

type MessageCache = Cache<Uncached, Uncached, Uncached>;
type MessageQueue<'a> = QueueStorage<CountingFlash<'a>, MessageCache>;

fn open(flash: &mut Flash) -> MessageQueue<'_> {
    QueueStorage::new(
        CountingFlash(flash),
        QueueConfig::new(REGION),
        Cache::new_uncached(),
    )
}

/// Work sent to the storage task. The storage task owns the flash, so message
/// mutations are serialized through this channel.
pub enum MessageCommand {
    Append(ReceivedMessage),
    Consume,
    Clear,
}

pub static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, MessageCommand, 32> =
    Channel::new();

pub async fn queue_append(msg: ReceivedMessage) {
    let _ = COMMAND_CHANNEL.send(MessageCommand::Append(msg)).await;
}

pub async fn queue_consume() {
    let _ = COMMAND_CHANNEL.send(MessageCommand::Consume).await;
}

/// Non-blocking variant for command handlers that cannot await.
pub fn queue_consume_now() {
    let _ = COMMAND_CHANNEL.try_send(MessageCommand::Consume);
}

pub async fn queue_clear() {
    let _ = COMMAND_CHANNEL.send(MessageCommand::Clear).await;
}

fn message_to_record(m: &ReceivedMessage) -> MessageRecord {
    MessageRecord::new(
        m.channel_idx,
        m.path_len,
        m.is_direct,
        m.sender.as_str(),
        m.sender_pub,
        m.text.as_str(),
        m.timestamp,
        m.rssi,
        m.snr,
    )
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
    }
}

/// Replace `MESSAGE_LOG` with the unconsumed messages from flash.
pub async fn load(flash: &mut Flash) {
    MESSAGE_LOG.lock(|m| m.borrow_mut().clear());

    let mut queue = open(flash);

    // The previous firmware stored two whole-log snapshots in this region.
    // Their record format is not compatible with the FIFO, so clear the legacy
    // banks once instead of making the queue repair arbitrary data.
    const LEGACY_SNAPSHOT_MAGIC: u32 = 0x4D53_474D;
    let legacy = |addr: u32| unsafe {
        (addr as *const u32).read_unaligned() == LEGACY_SNAPSHOT_MAGIC
    };
    if legacy(REGION.start) || legacy(REGION.start + 0x1_0000) {
        info!("Persistence: clearing legacy message snapshot banks");
        if let Err(e) = queue.erase_all().await {
            error!("Persistence: legacy message erase failed: {:?}", e);
        }
    }

    let mut buf = [0u8; MESSAGE_RECORD_SIZE];
    let mut count = 0usize;
    {
        match queue.iter().await {
            Ok(mut iter) => {
                while let Ok(Some(entry)) = iter.next(&mut buf).await {
                    let Some(rec) = MessageRecord::from_bytes(entry.into_buf()) else {
                        continue;
                    };
                    let msg = record_to_message(&rec);
                    MESSAGE_LOG.lock(|m| {
                        let mut list = m.borrow_mut();
                        if list.len() >= MAX_MESSAGES {
                            list.remove(0);
                        }
                        let _ = list.push(msg);
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
            if queue.pop(&mut buf).await.is_err() {
                break;
            }
        }
    }
    info!("Persistence: loaded {} messages from queue", count);
}

/// Append one message, dropping the oldest record when the region is full.
pub async fn append(flash: &mut Flash, msg: &ReceivedMessage) {
    // The SoftDevice flash driver requires a word-aligned write buffer.
    #[repr(C, align(4))]
    struct Aligned([u8; MESSAGE_RECORD_SIZE]);

    let aligned = Aligned(message_to_record(msg).to_bytes());
    let mut queue = open(flash);
    match queue.push(&aligned.0, true).await {
        Ok(()) => {
            RECORD_SAVES.fetch_add(1, Ordering::Relaxed);
        }
        Err(e) => error!("Persistence: message queue push failed: {:?}", e),
    }
}

/// Drop the oldest message (the app synced it).
pub async fn consume(flash: &mut Flash) {
    let mut buf = [0u8; MESSAGE_RECORD_SIZE];
    let mut queue = open(flash);
    if let Err(e) = queue.pop(&mut buf).await {
        error!("Persistence: message queue pop failed: {:?}", e);
    }
}

/// Erase every stored message.
pub async fn clear(flash: &mut Flash) {
    let mut queue = open(flash);
    if let Err(e) = queue.erase_all().await {
        error!("Persistence: message queue clear failed: {:?}", e);
    }
}
