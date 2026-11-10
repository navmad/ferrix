//! MeshCore Packet Wire Protocol Definition & Serialization
//!
//! Header byte layout: `0bVVPPPPRR`
//! - Version (V): bits 7-6 (2 bits, default `0b00`)
//! - Payload Type (P): bits 5-2 (4 bits)
//! - Route Type (R): bits 1-0 (2 bits)

pub const MAX_PATH_SIZE: usize = 64;
pub const MAX_PACKET_PAYLOAD: usize = 192;
pub const MAX_PACKET_SIZE: usize = 256;

// Header Bit Masks & Shifts
pub const PH_ROUTE_MASK: u8 = 0x03;
pub const PH_TYPE_SHIFT: u8 = 2;
pub const PH_TYPE_MASK: u8 = 0x0F;
pub const PH_VER_SHIFT: u8 = 6;
pub const PH_VER_MASK: u8 = 0x03;

// Route Types (bits 1-0)
pub const ROUTE_TYPE_TRANSPORT_FLOOD: u8 = 0x00;
pub const ROUTE_TYPE_FLOOD: u8 = 0x01;
pub const ROUTE_TYPE_DIRECT: u8 = 0x02;
pub const ROUTE_TYPE_TRANSPORT_DIRECT: u8 = 0x03;

// Payload Types (bits 5-2)
pub const PAYLOAD_TYPE_REQ: u8 = 0x00;
pub const PAYLOAD_TYPE_RESPONSE: u8 = 0x01;
pub const PAYLOAD_TYPE_TXT_MSG: u8 = 0x02;
pub const PAYLOAD_TYPE_ACK: u8 = 0x03;
pub const PAYLOAD_TYPE_ADVERT: u8 = 0x04;
pub const PAYLOAD_TYPE_GRP_TXT: u8 = 0x05;
pub const PAYLOAD_TYPE_GRP_DATA: u8 = 0x06;
pub const PAYLOAD_TYPE_ANON_REQ: u8 = 0x07;
pub const PAYLOAD_TYPE_PATH: u8 = 0x08;
pub const PAYLOAD_TYPE_TRACE: u8 = 0x09;
pub const PAYLOAD_TYPE_MULTIPART: u8 = 0x0A;
pub const PAYLOAD_TYPE_CONTROL: u8 = 0x0B;
pub const PAYLOAD_TYPE_RAW_CUSTOM: u8 = 0x0F;

/// Fundamental Transmission Unit for MeshCore
#[derive(Clone, Debug, PartialEq)]
pub struct Packet {
    pub header: u8,
    pub transport_codes: [u16; 2],
    pub path_len: u8,
    pub path: [u8; MAX_PATH_SIZE],
    pub payload_len: usize,
    pub payload: [u8; MAX_PACKET_PAYLOAD],
    pub snr: i8,
}

impl Packet {
    /// Create a new packet with specified route type and payload type
    pub fn new(route_type: u8, payload_type: u8) -> Self {
        let header = ((payload_type & PH_TYPE_MASK) << PH_TYPE_SHIFT)
            | (route_type & PH_ROUTE_MASK);

        Self {
            header,
            transport_codes: [0, 0],
            path_len: 0,
            path: [0u8; MAX_PATH_SIZE],
            payload_len: 0,
            payload: [0u8; MAX_PACKET_PAYLOAD],
            snr: 0,
        }
    }

    /// Extract Route Type (bits 1-0)
    #[inline]
    pub fn route_type(&self) -> u8 {
        self.header & PH_ROUTE_MASK
    }

    /// Extract Payload Type (bits 5-2)
    #[inline]
    pub fn payload_type(&self) -> u8 {
        (self.header >> PH_TYPE_SHIFT) & PH_TYPE_MASK
    }

    /// Extract Protocol Version (bits 7-6)
    #[inline]
    pub fn version(&self) -> u8 {
        (self.header >> PH_VER_SHIFT) & PH_VER_MASK
    }

    /// Returns true if this packet is routed via flooding
    #[inline]
    pub fn is_flood(&self) -> bool {
        let rt = self.route_type();
        rt == ROUTE_TYPE_FLOOD || rt == ROUTE_TYPE_TRANSPORT_FLOOD
    }

    /// Returns true if transport codes are present
    #[inline]
    pub fn is_transport(&self) -> bool {
        let rt = self.route_type();
        rt == ROUTE_TYPE_TRANSPORT_FLOOD || rt == ROUTE_TYPE_TRANSPORT_DIRECT
    }

    /// Set payload bytes
    pub fn set_payload(&mut self, data: &[u8]) -> Result<(), ()> {
        if data.len() > MAX_PACKET_PAYLOAD {
            return Err(());
        }
        self.payload[..data.len()].copy_from_slice(data);
        self.payload_len = data.len();
        Ok(())
    }

    /// Appends a node hash to the path if within capacity
    pub fn append_hop(&mut self, node_hash: u8) -> bool {
        if (self.path_len as usize) < MAX_PATH_SIZE {
            self.path[self.path_len as usize] = node_hash;
            self.path_len += 1;
            true
        } else {
            false
        }
    }

    /// Serialize packet into raw wire format:
    /// `[header] [transport_codes (opt: 4 bytes)] [path_len] [path...] [payload...]`
    pub fn write_to(&self, dest: &mut [u8]) -> Result<usize, ()> {
        let mut offset = 0;

        // 1. Header byte
        if dest.len() <= offset {
            return Err(());
        }
        dest[offset] = self.header;
        offset += 1;

        // 2. Transport codes (if transport route type)
        if self.is_transport() {
            if dest.len() < offset + 4 {
                return Err(());
            }
            dest[offset..offset + 2].copy_from_slice(&self.transport_codes[0].to_le_bytes());
            dest[offset + 2..offset + 4].copy_from_slice(&self.transport_codes[1].to_le_bytes());
            offset += 4;
        }

        // 3. Path length
        if dest.len() <= offset {
            return Err(());
        }
        dest[offset] = self.path_len;
        offset += 1;

        // 4. Path bytes
        let path_bytes = self.path_len as usize;
        if dest.len() < offset + path_bytes {
            return Err(());
        }
        dest[offset..offset + path_bytes].copy_from_slice(&self.path[..path_bytes]);
        offset += path_bytes;

        // 5. Payload bytes
        if dest.len() < offset + self.payload_len {
            return Err(());
        }
        dest[offset..offset + self.payload_len].copy_from_slice(&self.payload[..self.payload_len]);
        offset += self.payload_len;

        Ok(offset)
    }

    /// Deserialize raw wire buffer into a Packet
    pub fn read_from(src: &[u8]) -> Result<Self, ()> {
        if src.is_empty() {
            return Err(());
        }

        let mut offset = 0;
        let header = src[offset];
        offset += 1;

        let is_transport = {
            let rt = header & PH_ROUTE_MASK;
            rt == ROUTE_TYPE_TRANSPORT_FLOOD || rt == ROUTE_TYPE_TRANSPORT_DIRECT
        };

        let mut transport_codes = [0u16; 2];
        if is_transport {
            if src.len() < offset + 4 {
                return Err(());
            }
            transport_codes[0] = u16::from_le_bytes([src[offset], src[offset + 1]]);
            transport_codes[1] = u16::from_le_bytes([src[offset + 2], src[offset + 3]]);
            offset += 4;
        }

        if src.len() <= offset {
            return Err(());
        }
        let path_len = src[offset];
        offset += 1;

        let path_bytes = path_len as usize;
        if path_bytes > MAX_PATH_SIZE || src.len() < offset + path_bytes {
            return Err(());
        }

        let mut path = [0u8; MAX_PATH_SIZE];
        path[..path_bytes].copy_from_slice(&src[offset..offset + path_bytes]);
        offset += path_bytes;

        let payload_len = src.len() - offset;
        if payload_len > MAX_PACKET_PAYLOAD {
            return Err(());
        }

        let mut payload = [0u8; MAX_PACKET_PAYLOAD];
        payload[..payload_len].copy_from_slice(&src[offset..]);

        Ok(Self {
            header,
            transport_codes,
            path_len,
            path,
            payload_len,
            payload,
            snr: 0,
        })
    }

    /// Calculate a fast 4-byte packet hash for deduplication and acknowledgment tracking
    pub fn calculate_packet_hash(&self) -> [u8; 4] {
        // FNV-1a 32-bit hash over header + payload
        let mut hash = 0x811c9dc5u32;
        hash ^= self.header as u32;
        hash = hash.wrapping_mul(0x01000193);

        for &b in &self.payload[..self.payload_len] {
            hash ^= b as u32;
            hash = hash.wrapping_mul(0x01000193);
        }

        hash.to_le_bytes()
    }
}
