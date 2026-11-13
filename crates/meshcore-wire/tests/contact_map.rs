//! Host tests for the contact key-value map.
//!
//! The firmware stores each contact as a `[u8; 32]` public-key → 156-byte
//! record in `sequential-storage`'s map over the 112 KiB contact region.
//! These tests use the same geometry on the crate's mock flash and verify
//! capacity, update-in-place and removal semantics.

use futures::executor::block_on;
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage};
use sequential_storage::mock_flash::{MockFlashBase, WriteCountCheck};

const PAGE_SIZE: usize = 4096;
const PAGES: usize = 28; // 0xB0000..0xCC000
const REGION: core::ops::Range<u32> = 0..(PAGES * PAGE_SIZE) as u32;
const VALUE_SIZE: usize = 156;

type Flash = MockFlashBase<PAGES, 4, 1024>;
type ContactMap =
    MapStorage<[u8; 32], Flash, Cache<Uncached, Uncached, Uncached, [u8; 32]>>;

fn make_map() -> ContactMap {
    let flash = Flash::new(WriteCountCheck::Twice, None, false);
    MapStorage::new(flash, MapConfig::new(REGION), Cache::new_uncached())
}

fn key(n: u32) -> [u8; 32] {
    let mut k = [0u8; 32];
    k[..4].copy_from_slice(&n.to_le_bytes());
    k
}

fn value(n: u32) -> [u8; VALUE_SIZE] {
    let mut v = [0u8; VALUE_SIZE];
    v[..4].copy_from_slice(&n.to_le_bytes());
    for (i, b) in v[4..].iter_mut().enumerate() {
        *b = (n as usize + i) as u8;
    }
    v
}

#[repr(C, align(4))]
struct AlignedBuf([u8; 192]);

#[test]
fn store_fetch_update_remove() {
    block_on(async {
        let mut map = make_map();
        let mut buf = AlignedBuf([0u8; 192]);

        for n in 0..8 {
            let bytes: &[u8] = &value(n);
            map.store_item(&mut buf.0, &key(n), &bytes).await.unwrap();
        }

        let v: &[u8] = &value(3);
        let bytes: &[u8] = &v;
        map.store_item(&mut buf.0, &key(3), &bytes).await.unwrap();

        map.remove_item(&mut buf.0, &key(5)).await.unwrap();

        let mut found = std::collections::BTreeMap::<[u8; 32], Vec<u8>>::new();
        {
            let mut iter = map.fetch_all_items(&mut buf.0).await.unwrap();
            while let Some((k, v)) = iter.next::<&[u8]>(&mut buf.0).await.unwrap() {
                found.insert(k, v.to_vec());
            }
        }
        assert_eq!(found.len(), 7);
        assert_eq!(found.get(&key(3)).unwrap(), &value(3).to_vec());
        assert!(!found.contains_key(&key(5)));
    });
}

#[test]
fn holds_a_full_contact_table() {
    block_on(async {
        let mut map = make_map();
        let mut buf = AlignedBuf([0u8; 192]);

        for n in 0..350 {
            let bytes: &[u8] = &value(n);
            map.store_item(&mut buf.0, &key(n), &bytes).await.unwrap();
        }

        // Updating every contact once must still fit after compaction.
        for n in 0..350 {
            let bytes: &[u8] = &value(n + 1000);
            map.store_item(&mut buf.0, &key(n), &bytes).await.unwrap();
        }

        let mut count = 0;
        let mut newest = std::collections::BTreeMap::<[u8; 32], u32>::new();
        {
            let mut iter = map.fetch_all_items(&mut buf.0).await.unwrap();
            while let Some((k, v)) = iter.next::<&[u8]>(&mut buf.0).await.unwrap() {
                count += 1;
                newest.insert(k, u32::from_le_bytes(v[..4].try_into().unwrap()));
            }
        }
        assert_eq!(newest.len(), 350, "live contact count");
        assert!(count >= 350, "iterator must yield every stored version");
        assert_eq!(newest.get(&key(0)), Some(&1000));
        assert_eq!(newest.get(&key(349)), Some(&1349));
    });
}
