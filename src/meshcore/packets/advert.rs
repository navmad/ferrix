//! Node Advertisement Packet (`PAYLOAD_TYPE_ADVERT = 0x04`)
//!
//! Broadcasts node presence, public key identity, callsign, and GPS location.

use heapless::String;
use crate::meshcore::packet::{Packet, PAYLOAD_TYPE_ADVERT};
use crate::meshcore::identity::PUB_KEY_SIZE;

pub const ADV_TYPE_CHAT: u8 = 1;
pub const ADV_TYPE_REPEATER: u8 = 2;

#[derive(Clone, Debug, PartialEq)]
pub struct AdvertPacket {
    pub pub_key: [u8; PUB_KEY_SIZE],
    pub timestamp: u32,
    pub adv_type: u8,
    pub callsign: String<32>,
    pub latitude: f32,
    pub longitude: f32,
    pub battery_percentage: u8,
}

impl AdvertPacket {
    pub fn new(
        pub_key: [u8; PUB_KEY_SIZE],
        callsign: &str,
        adv_type: u8,
        timestamp: u32,
    ) -> Self {
        let mut cs = String::new();
        let _ = cs.push_str(callsign);
        Self {
            pub_key,
            timestamp,
            adv_type,
            callsign: cs,
            latitude: 0.0,
            longitude: 0.0,
            battery_percentage: 100,
        }
    }

    /// Encode advert payload into bytes:
    /// `[pub_key: 32] [timestamp: 4] [adv_type: 1] [battery_pct: 1] [lat: 4] [lon: 4] [cs_len: 1] [callsign...]`
    pub fn encode(&self, dest: &mut [u8]) -> Result<usize, ()> {
        let cs_bytes = self.callsign.as_bytes();
        let needed = 32 + 4 + 1 + 1 + 4 + 4 + 1 + cs_bytes.len();
        if dest.len() < needed {
            return Err(());
        }

        let mut offset = 0;
        // Public key
        dest[offset..offset + 32].copy_from_slice(&self.pub_key);
        offset += 32;

        // Timestamp (u32 LE)
        dest[offset..offset + 4].copy_from_slice(&self.timestamp.to_le_bytes());
        offset += 4;

        // Advert type
        dest[offset] = self.adv_type;
        offset += 1;

        // Battery percentage
        dest[offset] = self.battery_percentage;
        offset += 1;

        // GPS Coordinates
        dest[offset..offset + 4].copy_from_slice(&self.latitude.to_le_bytes());
        offset += 4;
        dest[offset..offset + 4].copy_from_slice(&self.longitude.to_le_bytes());
        offset += 4;

        // Callsign
        dest[offset] = cs_bytes.len() as u8;
        offset += 1;
        dest[offset..offset + cs_bytes.len()].copy_from_slice(cs_bytes);
        offset += cs_bytes.len();

        Ok(offset)
    }

    /// Decode advert payload from bytes
    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        if src.len() < 32 + 4 + 1 + 1 + 4 + 4 + 1 {
            return Err(());
        }

        let mut offset = 0;
        let mut pub_key = [0u8; 32];
        pub_key.copy_from_slice(&src[offset..offset + 32]);
        offset += 32;

        let timestamp = u32::from_le_bytes([
            src[offset],
            src[offset + 1],
            src[offset + 2],
            src[offset + 3],
        ]);
        offset += 4;

        let adv_type = src[offset];
        offset += 1;

        let battery_percentage = src[offset];
        offset += 1;

        let latitude = f32::from_le_bytes([
            src[offset],
            src[offset + 1],
            src[offset + 2],
            src[offset + 3],
        ]);
        offset += 4;

        let longitude = f32::from_le_bytes([
            src[offset],
            src[offset + 1],
            src[offset + 2],
            src[offset + 3],
        ]);
        offset += 4;

        let cs_len = src[offset] as usize;
        offset += 1;

        if src.len() < offset + cs_len {
            return Err(());
        }

        let cs_str = core::str::from_utf8(&src[offset..offset + cs_len]).map_err(|_| ())?;
        let mut callsign = String::new();
        let _ = callsign.push_str(cs_str);

        Ok(Self {
            pub_key,
            timestamp,
            adv_type,
            callsign,
            latitude,
            longitude,
            battery_percentage,
        })
    }

    /// Pack into a complete MeshCore Packet ready for LoRa transmission
    pub fn to_packet(&self, route_type: u8) -> Result<Packet, ()> {
        let mut pkt = Packet::new(route_type, PAYLOAD_TYPE_ADVERT);
        let mut buf = [0u8; 192];
        let len = self.encode(&mut buf)?;
        pkt.set_payload(&buf[..len])?;
        Ok(pkt)
    }
}
