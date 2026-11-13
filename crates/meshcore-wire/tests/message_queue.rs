//! Host tests for the message queue geometry and behaviour.
//!
//! These exercise the same `sequential-storage` queue and record format the
//! firmware uses on the nRF52840 (4-byte words, 4 KiB pages, 128 KiB region),
//! using the crate's in-memory flash. They catch layout, wrap-around and
//! capacity regressions without flashing a device.

use futures::executor::block_on;
use meshcore_wire::message_record::{MessageRecord, MESSAGE_RECORD_SIZE};
use sequential_storage::cache::Cache;
use sequential_storage::mock_flash::{MockFlashBase, WriteCountCheck};
use sequential_storage::queue::{QueueConfig, QueueStorage};

const PAGE_SIZE: usize = 4096;
const PAGES: usize = 32;
const REGION: core::ops::Range<u32> = 0..(PAGES * PAGE_SIZE) as u32;

type Flash = MockFlashBase<PAGES, 4, 1024>;

fn make_queue() -> QueueStorage<Flash, Cache<sequential_storage::cache::Uncached, sequential_storage::cache::Uncached, sequential_storage::cache::Uncached, ()>> {
    let flash = Flash::new(WriteCountCheck::Twice, None, false);
    QueueStorage::new(flash, QueueConfig::new(REGION), Cache::new_uncached())
}

fn record(n: u32) -> MessageRecord {
    MessageRecord::new(
        (n % 4) as u8,
        1,
        n % 2 == 0,
        "sender",
        [n as u8; 32],
        &format!("message {n}"),
        1_700_000_000 + n,
        -(n as i16 % 100),
        (n % 20) as i8,
    )
}

#[test]
fn fifo_roundtrip() {
    block_on(async {
        let mut queue = make_queue();
        for n in 0..20 {
            queue.push(&record(n).to_bytes(), false).await.unwrap();
        }
        let mut buf = [0u8; MESSAGE_RECORD_SIZE];
        for n in 0..20 {
            let data = queue.pop(&mut buf).await.unwrap().unwrap();
            let decoded = MessageRecord::from_bytes(data).unwrap();
            assert_eq!(decoded, record(n));
        }
        assert!(queue.pop(&mut buf).await.unwrap().is_none());
    });
}

#[test]
fn wraps_and_drops_oldest_when_full() {
    block_on(async {
        let mut queue = make_queue();
        // Far more than the region can hold, so pages are erased and reused.
        let total = 1200u32;
        for n in 0..total {
            queue.push(&record(n).to_bytes(), true).await.unwrap();
        }

        let mut buf = [0u8; MESSAGE_RECORD_SIZE];
        let mut values = Vec::new();
        while let Some(data) = queue.pop(&mut buf).await.unwrap() {
            values.push(MessageRecord::from_bytes(data).unwrap().timestamp);
        }

        // The newest record must still be the last one written.
        assert_eq!(*values.last().unwrap(), record(total - 1).timestamp);
        // Dropped records must be a contiguous oldest prefix.
        let first = record(total - values.len() as u32);
        assert_eq!(values[0], first.timestamp);
        // The queue must hold a useful number of records: roughly the region
        // size divided by the per-item footprint.
        assert!(
            values.len() > 400 && values.len() < 600,
            "unexpected capacity {}",
            values.len()
        );
    });
}

#[test]
fn iterates_without_consuming() {
    block_on(async {
        let mut queue = make_queue();
        for n in 0..10 {
            queue.push(&record(n).to_bytes(), false).await.unwrap();
        }
        let mut buf = [0u8; MESSAGE_RECORD_SIZE];
        {
            let mut it = queue.iter().await.unwrap();
            let mut seen = 0u32;
            while let Some(entry) = it.next(&mut buf).await.unwrap() {
                let rec = MessageRecord::from_bytes(entry.into_buf()).unwrap();
                assert_eq!(rec, record(seen));
                seen += 1;
            }
            assert_eq!(seen, 10);
        }
        // Iteration must not have removed anything.
        let mut it = queue.iter().await.unwrap();
        assert!(it.next(&mut buf).await.unwrap().is_some());
    });
}
