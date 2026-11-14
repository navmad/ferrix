//! Companion contact-list response encoding.
//!
//! These layouts are shared by the USB and BLE transports and depend only on
//! the contact record, so they live with the core protocol.

use heapless::Vec;

use crate::types::DiscoveredContact;

/// `RESP_CODE_CONTACTS_START` (2).
pub const RESP_CODE_CONTACTS_START: u8 = 2;
/// `RESP_CODE_CONTACT` (3).
pub const RESP_CODE_CONTACT: u8 = 3;
/// `RESP_CODE_END_OF_CONTACTS` (4).
pub const RESP_CODE_END_OF_CONTACTS: u8 = 4;

/// `RESP_CODE_CONTACTS_START` (2) `[code(2), count(u32_le)]`.
pub fn encode_contacts_start(count: u32) -> Vec<u8, 8> {
    let mut resp = Vec::<u8, 8>::new();
    let _ = resp.push(RESP_CODE_CONTACTS_START);
    let _ = resp.extend_from_slice(&count.to_le_bytes());
    resp
}

/// `RESP_CODE_END_OF_CONTACTS` (4) `[code(4), most_recent_lastmod(u32_le)]`.
pub fn encode_contacts_end(max_lastmod: u32) -> Vec<u8, 8> {
    let mut resp = Vec::<u8, 8>::new();
    let _ = resp.push(RESP_CODE_END_OF_CONTACTS);
    let _ = resp.extend_from_slice(&max_lastmod.to_le_bytes());
    resp
}

/// Serialize one contact in the companion `PACKET_CONTACT` layout.
pub fn serialize_contact(c: &DiscoveredContact) -> Vec<u8, 160> {
    let mut resp = Vec::<u8, 160>::new();
    let _ = resp.push(RESP_CODE_CONTACT);
    let _ = resp.extend_from_slice(&c.pub_key);
    let _ = resp.push(c.contact_type);
    let _ = resp.push(c.flags);
    let _ = resp.push(c.out_path_len);
    let _ = resp.extend_from_slice(&c.out_path);

    let mut adv_name = [0u8; 32];
    let cs = c.callsign.as_bytes();
    let n = cs.len().min(31);
    adv_name[..n].copy_from_slice(&cs[..n]);
    let _ = resp.extend_from_slice(&adv_name);

    let _ = resp.extend_from_slice(&c.last_advert_ts.to_le_bytes());
    let _ = resp.extend_from_slice(&c.latitude.to_le_bytes());
    let _ = resp.extend_from_slice(&c.longitude.to_le_bytes());
    let _ = resp.extend_from_slice(&c.last_seen_epoch.to_le_bytes());
    resp
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contact() -> DiscoveredContact {
        let mut callsign = heapless::String::new();
        callsign.push_str("alice").unwrap();
        DiscoveredContact {
            pub_key: [0xAB; 32],
            callsign,
            node_id: [1, 2, 3, 4],
            last_seen_epoch: 0x1122_3344,
            rssi: -70,
            snr: 7,
            latitude: 52_000_000,
            longitude: -1_000_000,
            flags: 0x01,
            contact_type: 1,
            out_path: [0xCD; 64],
            out_path_len: 0xFF,
            last_advert_ts: 0x5566_7788,
            saved_hash: 0,
            sync_since: 0,
        }
    }

    #[test]
    fn contact_record_layout() {
        let record = serialize_contact(&contact());
        assert_eq!(record[0], RESP_CODE_CONTACT);
        assert_eq!(&record[1..33], &[0xAB; 32]);
        assert_eq!(record[33], 1);
        assert_eq!(record[34], 0x01);
        assert_eq!(record[35], 0xFF);
        assert_eq!(&record[36..100], &[0xCD; 64]);
        assert_eq!(&record[100..105], b"alice");
        assert_eq!(record[105], 0);
        let name_end = 132;
        assert_eq!(&record[name_end..name_end + 4], &0x5566_7788u32.to_le_bytes());
        assert_eq!(record.len(), name_end + 16);
    }

    #[test]
    fn list_markers_carry_counts() {
        let start = encode_contacts_start(350);
        assert_eq!(start.as_slice(), &[2, 0x5E, 0x01, 0, 0]);
        let end = encode_contacts_end(0x0102_0304);
        assert_eq!(end.as_slice(), &[4, 0x04, 0x03, 0x02, 0x01]);
    }
}
