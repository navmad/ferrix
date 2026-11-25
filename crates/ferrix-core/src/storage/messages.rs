//! Optional persistent message queue.
//!
//! Offline inbox messages live in [`MESSAGE_LOG`] (RAM). Flash persistence is
//! opt-in (`PERSIST_MESSAGES`, default OFF) and skipped while the BLE
//! companion app is connected; any unread messages remaining in RAM are
//! reconciled to flash on BLE disconnect (or before a clean reboot).
//!
//! When enabled, messages are stored in a wear-levelled FIFO
//! (`sequential-storage`) synced directly from [`MESSAGE_LOG`] using monotonic
//! head/tail sequence numbers so no second copy is buffered in RAM.

use core::cell::RefCell;
use core::ops::Range;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt::{error, info};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use heapless::String;
use meshcore_wire::message_record::{ChannelDataRecord, MessageRecord, MESSAGE_RECORD_MAX};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::queue::{QueueConfig, QueueStorage};

use super::counted_flash::CountedFlash;
use crate::state::{BLE_CONNECTED, MESSAGE_LOG};
use crate::types::{InboxItem, ReceivedChannelData, ReceivedMessage, MAX_MESSAGES};

/// Completed record writes and page erases since boot.
pub static RECORD_SAVES: AtomicU32 = AtomicU32::new(0);
pub static PAGE_ERASES: AtomicU32 = AtomicU32::new(0);
/// Records currently held by the flash queue.
pub static QUEUED: AtomicU32 = AtomicU32::new(0);

/// Tracks the RAM inbox window `[ram_head, ram_tail)` against the flash FIFO
/// window `[flash_head, flash_tail)`.
///
/// Lock order whenever both are held: `MESSAGE_LOG` first, then `SYNC_STATE`.
/// `flash_head` and `flash_tail` are mutated exclusively by the storage task
/// (`load` and `sync`).
#[derive(Clone, Copy)]
struct SyncState {
    ram_head: u64,
    ram_tail: u64,
    flash_head: u64,
    flash_tail: u64,
    force_clear: bool,
    clear_if_nonempty: bool,
}

impl SyncState {
    const fn new() -> Self {
        Self {
            ram_head: 0,
            ram_tail: 0,
            flash_head: 0,
            flash_tail: 0,
            force_clear: false,
            clear_if_nonempty: false,
        }
    }
}

static SYNC_STATE: Mutex<CriticalSectionRawMutex, RefCell<SyncState>> =
    Mutex::new(RefCell::new(SyncState::new()));
static SYNC_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

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

/// Wait for the next message-persistence reconciliation request.
pub async fn wait_sync() {
    SYNC_SIGNAL.wait().await;
}

/// Push an inbound item into the RAM inbox, evicting the oldest if full, and
/// wake flash persistence when enabled and no BLE app is connected.
#[inline(never)]
pub fn push_inbox(item: InboxItem) {
    MESSAGE_LOG.lock(|m| {
        let mut list = m.borrow_mut();
        let evicted = list.is_full();
        if evicted {
            list.remove(0);
        }
        let _ = list.push(item);
        SYNC_STATE.lock(|s| {
            let mut st = s.borrow_mut();
            if evicted {
                st.ram_head += 1;
            }
            st.ram_tail = st.ram_head + list.len() as u64;
        });
    });

    if crate::state::persist_messages() && !BLE_CONNECTED.load(Ordering::Acquire) {
        SYNC_SIGNAL.signal(());
    }
}

/// Pop the oldest item from the RAM inbox for companion sync.
///
/// The caller should follow up with [`queue_consume_now`] (via the host trait)
/// to wake flash reconciliation when appropriate.
pub fn pop_inbox() -> Option<InboxItem> {
    MESSAGE_LOG.lock(|m| {
        let mut list = m.borrow_mut();
        if list.is_empty() {
            return None;
        }
        let item = list.remove(0);
        SYNC_STATE.lock(|s| {
            let mut st = s.borrow_mut();
            st.ram_head += 1;
            // Maintain `ram_tail - ram_head == list.len()` even if a test
            // pushed directly to `MESSAGE_LOG`.
            st.ram_tail = st.ram_head + list.len() as u64;
        });
        Some(item)
    })
}

/// Wake the storage task after a companion consume if flash needs to pop a
/// persisted record and writes are currently allowed.
pub fn queue_consume_now() {
    if !crate::state::persist_messages() || BLE_CONNECTED.load(Ordering::Acquire) {
        return;
    }
    let has_pop = SYNC_STATE.lock(|s| {
        let st = s.borrow();
        st.flash_head < st.flash_tail && st.flash_head < st.ram_head
    });
    if has_pop {
        SYNC_SIGNAL.signal(());
    }
}

/// Clear the RAM inbox and, if flash may hold records, schedule a flash clear.
pub fn clear_inbox() {
    let should_signal = MESSAGE_LOG.lock(|m| {
        m.borrow_mut().clear();
        SYNC_STATE.lock(|s| {
            let mut st = s.borrow_mut();
            st.ram_head = st.ram_tail;
            if crate::state::persist_messages()
                || st.flash_head < st.flash_tail
                || QUEUED.load(Ordering::Relaxed) > 0
            {
                st.clear_if_nonempty = true;
                true
            } else {
                false
            }
        })
    });
    if should_signal {
        SYNC_SIGNAL.signal(());
    }
}

/// Enable or disable offline message persistence to flash.
///
/// Returns `true` if the setting changed. Any transition erases the flash
/// message region so stale records from an earlier session or firmware never
/// mix with live RAM state or resurrect on reboot. When turning ON while
/// disconnected, unread messages currently in RAM are then written to flash.
pub fn set_persist_messages(enabled: bool) -> bool {
    if !crate::state::set_persist_messages(enabled) {
        return false;
    }
    SYNC_STATE.lock(|s| {
        s.borrow_mut().force_clear = true;
    });
    SYNC_SIGNAL.signal(());
    true
}

/// Notify message persistence that the BLE companion disconnected.
///
/// If persistence is enabled, wakes the storage task to pop any pre-existing
/// flash records consumed during the session and persist any unfetched RAM
/// messages.
pub fn on_ble_disconnected() {
    if !crate::state::persist_messages() {
        return;
    }
    if has_pending_sync(false) {
        SYNC_SIGNAL.signal(());
    }
}

/// Returns `true` if [`sync`] has work to perform under the given BLE policy.
pub fn has_pending_sync(ignore_ble_connected: bool) -> bool {
    let persist = crate::state::persist_messages();
    let ble_blocked = !ignore_ble_connected && BLE_CONNECTED.load(Ordering::Acquire);
    SYNC_STATE.lock(|s| {
        let st = s.borrow();
        if st.force_clear
            || (st.clear_if_nonempty
                && (st.flash_head < st.flash_tail || QUEUED.load(Ordering::Relaxed) > 0))
        {
            return true;
        }
        if !persist || ble_blocked {
            return false;
        }
        (st.flash_head < st.flash_tail && st.flash_head < st.ram_head)
            || st.flash_tail < st.ram_tail
    })
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
///
/// Called at boot only when offline message persistence is enabled.
pub async fn load<F>(flash: &mut F, region: &Range<u32>)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    MESSAGE_LOG.lock(|m| {
        m.borrow_mut().clear();
        SYNC_STATE.lock(|s| {
            let mut st = s.borrow_mut();
            st.ram_head = 0;
            st.ram_tail = 0;
            st.flash_head = 0;
            st.flash_tail = 0;
        });
    });

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
    // records than the companion window.
    let mut popped = 0usize;
    if count > MAX_MESSAGES {
        for _ in 0..(count - MAX_MESSAGES) {
            if queue.pop(&mut buf.0).await.is_err() {
                break;
            }
            popped += 1;
        }
    }

    let loaded = count.min(MAX_MESSAGES);
    let flash_remaining = count.saturating_sub(popped);
    MESSAGE_LOG.lock(|m| {
        let len = m.borrow().len() as u64;
        SYNC_STATE.lock(|s| {
            let mut st = s.borrow_mut();
            st.ram_head = (count - loaded) as u64;
            st.ram_tail = st.ram_head + len;
            st.flash_head = popped as u64;
            st.flash_tail = count as u64;
        });
    });
    QUEUED.store(flash_remaining as u32, Ordering::Relaxed);
    info!("Persistence: loaded {} messages from queue", loaded);
}

enum PopResult {
    Popped,
    Empty,
    Error,
}

async fn pop_one<F>(flash: &mut F, region: &Range<u32>) -> PopResult
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
            PopResult::Popped
        }
        Ok(None) => {
            QUEUED.store(0, Ordering::Relaxed);
            PopResult::Empty
        }
        Err(e) => {
            error!("Persistence: message queue pop failed: {:?}", e);
            PopResult::Error
        }
    }
}

async fn append_record<F>(flash: &mut F, region: &Range<u32>, msg: &ReceivedMessage) -> bool
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    #[repr(C, align(4))]
    struct Aligned([u8; MESSAGE_RECORD_MAX]);

    let encoded = message_to_record(msg);
    let mut aligned = Aligned([0u8; MESSAGE_RECORD_MAX]);
    let Some(len) = encoded.encode(&mut aligned.0) else {
        error!("Persistence: message record too large");
        return false;
    };
    let mut queue = open(flash, region);
    match queue.push(&aligned.0[..len], true).await {
        Ok(()) => {
            RECORD_SAVES.fetch_add(1, Ordering::Relaxed);
            QUEUED.fetch_add(1, Ordering::Relaxed);
            true
        }
        Err(e) => {
            error!("Persistence: message queue push failed: {:?}", e);
            false
        }
    }
}

async fn append_channel_data_record<F>(
    flash: &mut F,
    region: &Range<u32>,
    data: &ReceivedChannelData,
) -> bool
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
        return false;
    };
    let mut aligned = Aligned([0u8; MESSAGE_RECORD_MAX]);
    let Some(len) = rec.encode(&mut aligned.0) else {
        error!("Persistence: channel data record too large");
        return false;
    };
    let mut queue = open(flash, region);
    match queue.push(&aligned.0[..len], true).await {
        Ok(()) => {
            RECORD_SAVES.fetch_add(1, Ordering::Relaxed);
            QUEUED.fetch_add(1, Ordering::Relaxed);
            true
        }
        Err(e) => {
            error!("Persistence: channel data push failed: {:?}", e);
            false
        }
    }
}

/// Erase only the pages in `region` that contain non-`0xFF` data.
///
/// The external message region is 1 MB (256 × 4 KB pages). A monolithic
/// `erase_all()` takes ~15 s on MX25R1635F, stalls `STORAGE_HEARTBEAT` past
/// the watchdog threshold, blocks config saves, and wears clean pages.
async fn erase_dirty_pages<F>(flash: &mut F, region: &Range<u32>) -> bool
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    let page_size = F::ERASE_SIZE as u32;
    let mut chunk = super::util::AlignedBuf::<256>::new();
    let mut addr = region.start;

    while addr < region.end {
        crate::state::bump_storage_heartbeat();
        let page_end = (addr + page_size).min(region.end);

        let mut dirty = false;
        let mut off = addr;
        while off < page_end {
            let len = ((page_end - off) as usize).min(chunk.0.len());
            if let Err(e) = flash.read(off, &mut chunk.0[..len]).await {
                error!("Persistence: message page scan failed: {:?}", e);
                return false;
            }
            if chunk.0[..len].iter().any(|&b| b != 0xFF) {
                dirty = true;
                break;
            }
            off += len as u32;
        }

        if dirty {
            let mut counted = CountedFlash::new(flash, &PAGE_ERASES);
            if let Err(e) = counted.erase(addr, page_end).await {
                error!("Persistence: message page erase failed: {:?}", e);
                return false;
            }
        }

        addr = page_end;
    }

    true
}

/// Reconcile the flash FIFO with `MESSAGE_LOG`.
///
/// - Pending erase requests (toggle transitions or explicit `clear messages`)
///   always run first.
/// - When persistence is OFF, returns immediately after any pending erase.
/// - When `ignore_ble_connected` is `false` and `BLE_CONNECTED` is `true`,
///   skips all flash pops and appends until BLE disconnects.
pub async fn sync<F>(flash: &mut F, region: &Range<u32>, ignore_ble_connected: bool)
where
    F: NorFlash + MultiwriteNorFlash,
    F::Error: defmt::Format,
{
    loop {
        let do_erase = SYNC_STATE.lock(|s| {
            let mut st = s.borrow_mut();
            if st.force_clear {
                st.force_clear = false;
                st.clear_if_nonempty = false;
                true
            } else if st.clear_if_nonempty {
                st.clear_if_nonempty = false;
                st.flash_head < st.flash_tail || QUEUED.load(Ordering::Relaxed) > 0
            } else {
                false
            }
        });

        if do_erase {
            if !erase_dirty_pages(flash, region).await {
                return;
            }
            QUEUED.store(0, Ordering::Relaxed);
            SYNC_STATE.lock(|s| {
                let mut st = s.borrow_mut();
                st.flash_head = st.ram_head;
                st.flash_tail = st.ram_head;
            });
            continue;
        }

        if !crate::state::persist_messages() {
            return;
        }

        if !ignore_ble_connected && BLE_CONNECTED.load(Ordering::Acquire) {
            return;
        }

        enum Action {
            Done,
            PopOne,
            Push(InboxItem),
        }

        let action = MESSAGE_LOG.lock(|m| {
            let list = m.borrow();
            SYNC_STATE.lock(|s| {
                let mut st = s.borrow_mut();

                if st.flash_head < st.flash_tail && st.flash_head < st.ram_head {
                    return Action::PopOne;
                }

                if st.flash_head == st.flash_tail && st.flash_tail < st.ram_head {
                    st.flash_head = st.ram_head;
                    st.flash_tail = st.ram_head;
                }

                if st.flash_tail < st.ram_tail {
                    let idx = (st.flash_tail - st.ram_head) as usize;
                    if let Some(item) = list.get(idx) {
                        return Action::Push(item.clone());
                    }
                }

                Action::Done
            })
        });

        match action {
            Action::Done => return,
            Action::PopOne => match pop_one(flash, region).await {
                PopResult::Popped => {
                    SYNC_STATE.lock(|s| {
                        let mut st = s.borrow_mut();
                        if st.flash_head < st.flash_tail {
                            st.flash_head += 1;
                        }
                    });
                }
                PopResult::Empty => {
                    SYNC_STATE.lock(|s| {
                        let mut st = s.borrow_mut();
                        st.flash_head = st.flash_tail;
                    });
                }
                PopResult::Error => return,
            },
            Action::Push(item) => {
                let ok = match &item {
                    InboxItem::Text(msg) => append_record(flash, region, msg).await,
                    InboxItem::ChannelData(data) => {
                        append_channel_data_record(flash, region, data).await
                    }
                };
                if !ok {
                    return;
                }
                SYNC_STATE.lock(|s| {
                    s.borrow_mut().flash_tail += 1;
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::storage::util::block_on;
    use embedded_storage_async::nor_flash::{
        ErrorType, NorFlashError, NorFlashErrorKind, ReadNorFlash,
    };

    const PAGE_SIZE: usize = 4096;
    const NUM_PAGES: usize = 8;
    const FLASH_SIZE: usize = PAGE_SIZE * NUM_PAGES;
    const REGION: Range<u32> = 0..(FLASH_SIZE as u32);

    #[derive(Debug, defmt::Format)]
    struct MemErr;

    impl NorFlashError for MemErr {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    struct MemFlash {
        data: std::vec::Vec<u8>,
        writes: u32,
        erases: u32,
    }

    impl MemFlash {
        fn new() -> Self {
            Self {
                data: std::vec![0xFF; FLASH_SIZE],
                writes: 0,
                erases: 0,
            }
        }
    }

    impl ErrorType for MemFlash {
        type Error = MemErr;
    }

    impl ReadNorFlash for MemFlash {
        const READ_SIZE: usize = 4;

        async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            let end = start + bytes.len();
            if end > self.data.len() {
                return Err(MemErr);
            }
            bytes.copy_from_slice(&self.data[start..end]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MemFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = PAGE_SIZE;

        async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            let start = from as usize;
            let end = to as usize;
            if start % PAGE_SIZE != 0 || end % PAGE_SIZE != 0 || end > self.data.len() {
                return Err(MemErr);
            }
            for b in &mut self.data[start..end] {
                *b = 0xFF;
            }
            self.erases += ((end - start) / PAGE_SIZE) as u32;
            Ok(())
        }

        async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            let end = start + bytes.len();
            if start % 4 != 0 || bytes.len() % 4 != 0 || end > self.data.len() {
                return Err(MemErr);
            }
            // Enforce NOR flash 1->0 bit programming semantics.
            for (dst, &src) in self.data[start..end].iter_mut().zip(bytes.iter()) {
                if src & !*dst != 0 {
                    return Err(MemErr);
                }
                *dst &= src;
            }
            self.writes += 1;
            Ok(())
        }
    }

    impl MultiwriteNorFlash for MemFlash {}

    fn reset_globals() {
        crate::state::init_persist_messages(false);
        BLE_CONNECTED.store(false, Ordering::Release);
        MESSAGE_LOG.lock(|m| {
            m.borrow_mut().clear();
            SYNC_STATE.lock(|s| *s.borrow_mut() = SyncState::new());
        });
        SYNC_SIGNAL.reset();
        RECORD_SAVES.store(0, Ordering::Relaxed);
        PAGE_ERASES.store(0, Ordering::Relaxed);
        QUEUED.store(0, Ordering::Relaxed);
    }

    fn text_item(body: &str, ts: u32) -> InboxItem {
        let mut sender = String::<32>::new();
        let _ = sender.push_str("Alice");
        let mut text = String::<160>::new();
        let _ = text.push_str(body);
        InboxItem::Text(ReceivedMessage {
            channel_idx: 0,
            path_len: 1,
            sender,
            sender_pub: [0x11; 32],
            text,
            timestamp: ts,
            rssi: -80,
            snr: 5,
            is_direct: true,
            txt_type: 0,
            signed_prefix: [0; 4],
        })
    }

    fn item_text(item: &InboxItem) -> &str {
        match item {
            InboxItem::Text(m) => m.text.as_str(),
            InboxItem::ChannelData(_) => "<data>",
        }
    }

    #[test]
    fn default_off_never_touches_flash() {
        let _guard = crate::state::TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_globals();
        let mut flash = MemFlash::new();

        assert!(!crate::state::persist_messages());
        push_inbox(text_item("hello", 1));
        push_inbox(text_item("world", 2));
        assert!(!has_pending_sync(false));
        block_on(sync(&mut flash, &REGION, false));

        assert_eq!(flash.writes, 0);
        assert_eq!(flash.erases, 0);
        assert_eq!(QUEUED.load(Ordering::Relaxed), 0);
        assert_eq!(RECORD_SAVES.load(Ordering::Relaxed), 0);

        let first = pop_inbox().unwrap();
        assert_eq!(item_text(&first), "hello");
        queue_consume_now();
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(flash.writes, 0);
        assert_eq!(flash.erases, 0);

        clear_inbox();
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(flash.writes, 0);
        assert_eq!(flash.erases, 0);
        reset_globals();
    }

    #[test]
    fn persist_on_skips_writes_while_ble_connected_and_flushes_on_disconnect() {
        let _guard = crate::state::TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_globals();
        let mut flash = MemFlash::new();

        // Turn persistence ON on blank flash: skips erasing already-blank pages.
        assert!(set_persist_messages(true));
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(flash.erases, 0);
        let writes_after_enable = flash.writes;
        let erases_after_enable = flash.erases;

        // Receive one offline message -> written to flash.
        push_inbox(text_item("offline-1", 10));
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(QUEUED.load(Ordering::Relaxed), 1);
        assert_eq!(RECORD_SAVES.load(Ordering::Relaxed), 1);
        let writes_before_ble = flash.writes;
        let erases_before_ble = flash.erases;

        // BLE connects.
        BLE_CONNECTED.store(true, Ordering::Release);

        // App fetches offline-1, then 2 new messages arrive; app fetches first,
        // leaves second unread. Zero flash writes happen while BLE is connected.
        assert_eq!(item_text(&pop_inbox().unwrap()), "offline-1");
        queue_consume_now();
        push_inbox(text_item("ble-fetched", 20));
        assert_eq!(item_text(&pop_inbox().unwrap()), "ble-fetched");
        queue_consume_now();
        push_inbox(text_item("ble-unread", 30));

        assert!(!has_pending_sync(false));
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(flash.writes, writes_before_ble);
        assert_eq!(flash.erases, erases_before_ble);
        assert_eq!(RECORD_SAVES.load(Ordering::Relaxed), 1);

        // BLE disconnects -> reconciler pops offline-1, skips ble-fetched
        // completely (never written!), and writes only ble-unread.
        BLE_CONNECTED.store(false, Ordering::Release);
        on_ble_disconnected();
        assert!(has_pending_sync(false));
        block_on(sync(&mut flash, &REGION, false));

        assert_eq!(QUEUED.load(Ordering::Relaxed), 1);
        assert_eq!(RECORD_SAVES.load(Ordering::Relaxed), 2);
        assert!(flash.writes > writes_after_enable);
        assert_eq!(flash.erases, erases_after_enable);

        // Simulate reboot: reload from flash -> only ble-unread comes back.
        block_on(load(&mut flash, &REGION));
        assert_eq!(MESSAGE_LOG.lock(|m| m.borrow().len()), 1);
        assert_eq!(item_text(&pop_inbox().unwrap()), "ble-unread");
        reset_globals();
    }

    #[test]
    fn ble_connected_with_all_messages_fetched_causes_zero_writes_on_disconnect() {
        let _guard = crate::state::TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_globals();
        let mut flash = MemFlash::new();

        assert!(set_persist_messages(true));
        block_on(sync(&mut flash, &REGION, false));
        let writes0 = flash.writes;
        let erases0 = flash.erases;

        BLE_CONNECTED.store(true, Ordering::Release);
        push_inbox(text_item("live-1", 1));
        push_inbox(text_item("live-2", 2));
        assert_eq!(item_text(&pop_inbox().unwrap()), "live-1");
        assert_eq!(item_text(&pop_inbox().unwrap()), "live-2");
        queue_consume_now();
        block_on(sync(&mut flash, &REGION, false));

        BLE_CONNECTED.store(false, Ordering::Release);
        on_ble_disconnected();
        block_on(sync(&mut flash, &REGION, false));

        // Fast-forward happened in RAM; zero flash writes or erases.
        assert_eq!(flash.writes, writes0);
        assert_eq!(flash.erases, erases0);
        assert_eq!(QUEUED.load(Ordering::Relaxed), 0);
        assert_eq!(RECORD_SAVES.load(Ordering::Relaxed), 0);
        reset_globals();
    }

    #[test]
    fn toggle_on_erases_stale_flash_and_toggle_off_keeps_ram_and_erases_flash() {
        let _guard = crate::state::TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_globals();
        let mut flash = MemFlash::new();

        // Pretend an older firmware left a stale message on flash.
        assert!(set_persist_messages(true));
        block_on(sync(&mut flash, &REGION, false));
        push_inbox(text_item("stale", 1));
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(QUEUED.load(Ordering::Relaxed), 1);

        // Simulated reboot with persistence OFF (load is not called).
        reset_globals();
        push_inbox(text_item("fresh-in-ram", 2));

        // User turns persistence ON -> only the dirty page with stale data is
        // erased (not the whole region), then fresh-in-ram is saved.
        let erases_before = flash.erases;
        assert!(set_persist_messages(true));
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(flash.erases - erases_before, 1);
        assert_eq!(QUEUED.load(Ordering::Relaxed), 1);

        block_on(load(&mut flash, &REGION));
        assert_eq!(MESSAGE_LOG.lock(|m| m.borrow().len()), 1);
        let first = MESSAGE_LOG.lock(|m| m.borrow()[0].clone());
        assert_eq!(item_text(&first), "fresh-in-ram");

        // User turns persistence OFF -> flash erased, RAM preserved.
        assert!(set_persist_messages(false));
        assert!(!set_persist_messages(false)); // idempotent
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(QUEUED.load(Ordering::Relaxed), 0);
        assert_eq!(MESSAGE_LOG.lock(|m| m.borrow().len()), 1);

        // Further messages stay in RAM only.
        let writes_after_off = flash.writes;
        push_inbox(text_item("ram-only", 3));
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(flash.writes, writes_after_off);
        assert_eq!(MESSAGE_LOG.lock(|m| m.borrow().len()), 2);
        reset_globals();
    }

    #[test]
    fn flush_ignores_ble_connected_and_persists_channel_data() {
        let _guard = crate::state::TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        reset_globals();
        let mut flash = MemFlash::new();

        assert!(set_persist_messages(true));
        block_on(sync(&mut flash, &REGION, false));

        BLE_CONNECTED.store(true, Ordering::Release);
        let mut payload = heapless::Vec::new();
        let _ = payload.extend_from_slice(&[1, 2, 3, 4]);
        push_inbox(InboxItem::ChannelData(ReceivedChannelData {
            channel_idx: 2,
            path_len: 1,
            snr: 4,
            data_type: 7,
            data: payload,
        }));

        // Normal sync skips while BLE connected.
        block_on(sync(&mut flash, &REGION, false));
        assert_eq!(QUEUED.load(Ordering::Relaxed), 0);

        // Reboot flush persists it even while BLE is connected.
        assert!(has_pending_sync(true));
        block_on(sync(&mut flash, &REGION, true));
        assert_eq!(QUEUED.load(Ordering::Relaxed), 1);

        block_on(load(&mut flash, &REGION));
        match pop_inbox().unwrap() {
            InboxItem::ChannelData(d) => {
                assert_eq!(d.channel_idx, 2);
                assert_eq!(d.data_type, 7);
                assert_eq!(d.data.as_slice(), &[1, 2, 3, 4]);
            }
            _ => panic!("expected channel data"),
        }
        reset_globals();
    }
}
