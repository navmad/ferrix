//! Host tests for the message queue geometry and behaviour.
//!
//! These exercise the same `sequential-storage` queue and record format the
//! firmware uses on the nRF52840 (4-byte words, 4 KiB pages, 128 KiB region),
//! using the crate's in-memory flash. They catch layout, wrap-around and
//! capacity regressions without flashing a device.

use futures::executor::block_on;
use meshcore_wire::message_record::{MessageRecord, MESSAGE_RECORD_MAX};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::mock_flash::{MockFlashBase, WriteCountCheck};
use sequential_storage::queue::{QueueConfig, QueueStorage};

const PAGE_SIZE: usize = 4096;
const PAGES: usize = 32;
const REGION: core::ops::Range<u32> = 0..(PAGES * PAGE_SIZE) as u32;

type Flash = MockFlashBase<PAGES, 4, 1024>;

fn make_queue() -> QueueStorage<Flash, Cache<Uncached, Uncached, Uncached>> {
    let flash = Flash::new(WriteCountCheck::Twice, None, false);
    QueueStorage::new(flash, QueueConfig::new(REGION), Cache::new_uncached())
}

fn record(n: u32) -> MessageRecord {
    let mut rec = MessageRecord::new(
        (n % 4) as u8,
        1,
        n % 2 == 0,
        "sender",
        if n % 2 == 0 { [n as u8; 32] } else { [0; 32] },
        &format!("message {n}"),
        1_700_000_000 + n,
        -(n as i16 % 100),
        (n % 20) as i8,
    );
    rec.txt_type = (n % 4) as u8;
    rec
}

#[test]
fn fifo_roundtrip() {
    block_on(async {
        let mut queue = make_queue();
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        for n in 0..20 {
            let rec = record(n);
            let len = rec.encode(&mut buf).unwrap();
            queue.push(&buf[..len], false).await.unwrap();
        }
        let mut out = [0u8; MESSAGE_RECORD_MAX];
        for n in 0..20 {
            let data = queue.pop(&mut out).await.unwrap().unwrap();
            let decoded = MessageRecord::decode(data).unwrap();
            assert_eq!(decoded, record(n));
        }
        assert!(queue.pop(&mut out).await.unwrap().is_none());
    });
}

#[test]
fn wraps_and_drops_oldest_when_full() {
    block_on(async {
        let mut queue = make_queue();
        // Compact records mean the region holds thousands of messages; push
        // well past one full ring so pages are erased and reused.
        let total = 5000u32;
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        for n in 0..total {
            let rec = record(n);
            let len = rec.encode(&mut buf).unwrap();
            queue.push(&buf[..len], true).await.unwrap();
        }

        let mut out = [0u8; MESSAGE_RECORD_MAX];
        let mut values = Vec::new();
        while let Some(data) = queue.pop(&mut out).await.unwrap() {
            values.push(MessageRecord::decode(data).unwrap().timestamp);
        }

        // The newest record must still be the last one written.
        assert_eq!(*values.last().unwrap(), record(total - 1).timestamp);
        // Dropped records must be a contiguous oldest prefix.
        let first = record(total - values.len() as u32);
        assert_eq!(values[0], first.timestamp);
        assert!(
            values.len() > 1800 && values.len() < 2800,
            "unexpected capacity {}",
            values.len()
        );
    });
}

#[test]
fn iterates_without_consuming() {
    block_on(async {
        let mut queue = make_queue();
        let mut buf = [0u8; MESSAGE_RECORD_MAX];
        for n in 0..10 {
            let rec = record(n);
            let len = rec.encode(&mut buf).unwrap();
            queue.push(&buf[..len], false).await.unwrap();
        }
        let mut out = [0u8; MESSAGE_RECORD_MAX];
        {
            let mut it = queue.iter().await.unwrap();
            let mut seen = 0u32;
            while let Some(entry) = it.next(&mut out).await.unwrap() {
                let rec = MessageRecord::decode(entry.into_buf()).unwrap();
                assert_eq!(rec, record(seen));
                seen += 1;
            }
            assert_eq!(seen, 10);
        }
        // Iteration must not have removed anything.
        let mut it = queue.iter().await.unwrap();
        assert!(it.next(&mut out).await.unwrap().is_some());
    });
}
