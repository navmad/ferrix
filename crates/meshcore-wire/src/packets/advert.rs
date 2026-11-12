//! Node Advertisement Packet (`PAYLOAD_TYPE_ADVERT = 0x04`)
//!
//! Broadcasts node presence, public key identity, callsign, and GPS location.

use heapless::String;
use crate::packet::{Packet, PAYLOAD_TYPE_ADVERT};
use crate::identity::{PUB_KEY_SIZE, SIGNATURE_SIZE};

#[derive(Clone, Debug, PartialEq)]
pub struct AdvertPacket {
    pub pub_key: [u8; PUB_KEY_SIZE],
    pub timestamp: u32,
    pub signature: [u8; SIGNATURE_SIZE],  // 64-byte Ed25519 signature
    pub flags: u8,                         // bitmask of ADV_FLAG_*
    pub latitude: i32,                     // degrees * 1,000,000 (integer)
    pub longitude: i32,                    // degrees * 1,000,000 (integer)
    pub name: String<32>,                  // node name/callsign
}

impl AdvertPacket {

    // Appdata flags (bitmask)
    pub const ADV_FLAG_IS_CHAT:      u8 = 0x01;
    pub const ADV_FLAG_IS_REPEATER:  u8 = 0x02;
    pub const ADV_FLAG_IS_ROOM:      u8 = 0x03;
    pub const ADV_FLAG_IS_SENSOR:    u8 = 0x04;
    pub const ADV_FLAG_HAS_LOCATION: u8 = 0x10;
    pub const ADV_FLAG_HAS_FEATURE1: u8 = 0x20;
    pub const ADV_FLAG_HAS_FEATURE2: u8 = 0x40;
    pub const ADV_FLAG_HAS_NAME:     u8 = 0x80;

    /// Create a new advert packet (unsigned — call `sign()` before sending)
    pub fn new(
        pub_key: [u8; PUB_KEY_SIZE],
        timestamp: u32,
        flags: u8,
        latitude: i32,
        longitude: i32,
        name: &str,
    ) -> Self {
        let mut n = String::new();
        let _ = n.push_str(name);
        Self {
            pub_key,
            timestamp,
            signature: [0u8; SIGNATURE_SIZE],
            flags,
            latitude,
            longitude,
            name: n,
        }
    }

    /// Encode advert payload into bytes:
    /// `[pub_key: 32] [timestamp: 4] [signature: 64] [flags: 1] [lat: 4 if 0x10] [lon: 4 if 0x10] [name: variable if 0x80]`
    pub fn encode(&self, dest: &mut [u8]) -> Result<usize, ()> {
        // Calculate needed space:
        // pub_key(32) + timestamp(4) + signature(64) + flags(1) = 101 minimum
        let mut needed = 32 + 4 + 64 + 1;
        if self.flags & Self::ADV_FLAG_HAS_LOCATION != 0 {
            needed += 8; // lat(4) + lon(4)
        }
        if self.flags & Self::ADV_FLAG_HAS_NAME != 0 {
            needed += self.name.len();
        }

        if dest.len() < needed {
            return Err(());
        }

        let mut offset = 0;

        // Public key (32 bytes)
        dest[offset..offset + 32].copy_from_slice(&self.pub_key);
        offset += 32;

        // Timestamp (4 bytes, little-endian)
        dest[offset..offset + 4].copy_from_slice(&self.timestamp.to_le_bytes());
        offset += 4;

        // Signature (64 bytes)
        dest[offset..offset + 64].copy_from_slice(&self.signature);
        offset += 64;

        // Appdata starts here
        // Flags byte (1 byte)
        dest[offset] = self.flags;
        offset += 1;

        // Location (optional, 8 bytes if flag set)
        if self.flags & Self::ADV_FLAG_HAS_LOCATION != 0 {
            dest[offset..offset + 4].copy_from_slice(&self.latitude.to_le_bytes());
            offset += 4;
            dest[offset..offset + 4].copy_from_slice(&self.longitude.to_le_bytes());
            offset += 4;
        }

        // Name (optional, variable length if flag set)
        if self.flags & Self::ADV_FLAG_HAS_NAME != 0 {
            let name_bytes = self.name.as_bytes();
            dest[offset..offset + name_bytes.len()].copy_from_slice(name_bytes);
            offset += name_bytes.len();
        }

        Ok(offset)
    }

    /// Decode advert payload from bytes
    pub fn decode(src: &[u8]) -> Result<Self, ()> {
        // Minimum: pub_key(32) + timestamp(4) + signature(64) + flags(1) = 101
        if src.len() < 32 + 4 + 64 + 1 {
            return Err(());
        }

        let mut offset = 0;

        // Public key
        let mut pub_key = [0u8; PUB_KEY_SIZE];
        pub_key.copy_from_slice(&src[offset..offset + PUB_KEY_SIZE]);
        offset += PUB_KEY_SIZE;

        // Timestamp
        let timestamp = u32::from_le_bytes([
            src[offset], src[offset + 1], src[offset + 2], src[offset + 3],
        ]);
        offset += 4;

        // Signature
        let mut signature = [0u8; SIGNATURE_SIZE];
        signature.copy_from_slice(&src[offset..offset + SIGNATURE_SIZE]);
        offset += SIGNATURE_SIZE;

        // Flags byte (first byte of appdata)
        let flags = src[offset];
        offset += 1;

        // Location (optional)
        let mut latitude = 0i32;
        let mut longitude = 0i32;
        if flags & Self::ADV_FLAG_HAS_LOCATION != 0 {
            if src.len() < offset + 8 {
                return Err(());
            }
            latitude = i32::from_le_bytes([
                src[offset], src[offset + 1], src[offset + 2], src[offset + 3],
            ]);
            offset += 4;
            longitude = i32::from_le_bytes([
                src[offset], src[offset + 1], src[offset + 2], src[offset + 3],
            ]);
            offset += 4;
        }

        // Name (optional, rest of appdata)
        let mut name = String::new();
        if flags & Self::ADV_FLAG_HAS_NAME != 0 {
            let name_len = src.len() - offset;
            if name_len > 0 {
                let name_str = core::str::from_utf8(&src[offset..offset + name_len]).map_err(|_| ())?;
                let _ = name.push_str(name_str);
            }
        }

        Ok(Self {
            pub_key,
            timestamp,
            signature,
            flags,
            latitude,
            longitude,
            name,
        })
    }

    /// Build the unsigned message bytes for signing: [pub_key || timestamp || appdata]
    /// Appdata = [flags] [lat if 0x10] [lon if 0x10] [name if 0x80]
    pub fn build_message_for_signing(&self, buf: &mut [u8]) -> usize {
        let mut offset = 0;

        // pub_key + timestamp (same as wire format start)
        buf[offset..offset + 32].copy_from_slice(&self.pub_key);
        offset += 32;
        buf[offset..offset + 4].copy_from_slice(&self.timestamp.to_le_bytes());
        offset += 4;

        // Appdata
        buf[offset] = self.flags;
        offset += 1;

        if self.flags & Self::ADV_FLAG_HAS_LOCATION != 0 {
            buf[offset..offset + 4].copy_from_slice(&self.latitude.to_le_bytes());
            offset += 4;
            buf[offset..offset + 4].copy_from_slice(&self.longitude.to_le_bytes());
            offset += 4;
        }

        if self.flags & Self::ADV_FLAG_HAS_NAME != 0 {
            let name_bytes = self.name.as_bytes();
            buf[offset..offset + name_bytes.len()].copy_from_slice(name_bytes);
            offset += name_bytes.len();
        }

        offset
    }

    /// Sign this advert with the local identity's private key
    pub fn sign(&mut self, local_identity: &crate::identity::LocalIdentity) {
        let mut msg_buf = [0u8; 128];
        let msg_len = self.build_message_for_signing(&mut msg_buf);
        self.signature = local_identity.sign(&msg_buf[..msg_len]);
    }

    /// Verify the signature of this advert
    pub fn verify_signature(&self) -> bool {
        let mut msg_buf = [0u8; 128];
        let msg_len = self.build_message_for_signing(&mut msg_buf);
        let identity = crate::identity::Identity::new(self.pub_key, "");
        identity.verify(&msg_buf[..msg_len], &self.signature)
    }

    /// Convert integer GPS (degrees * 1,000,000) back to f32 for display
    pub fn latitude_f32(&self) -> f32 {
        self.latitude as f32 / 1_000_000.0
    }

    pub fn longitude_f32(&self) -> f32 {
        self.longitude as f32 / 1_000_000.0
    }

    /// Check if this advert marks the node as a chat node
    pub fn is_chat(&self) -> bool {
        self.flags & Self::ADV_FLAG_IS_CHAT != 0
    }

    /// Check if this advert marks the node as a repeater
    pub fn is_repeater(&self) -> bool {
        self.flags & Self::ADV_FLAG_IS_REPEATER != 0
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
