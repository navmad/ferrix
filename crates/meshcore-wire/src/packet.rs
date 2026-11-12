//! MeshCore Packet Wire Protocol Definition & Serialization
//!
//! Header byte layout: `0bVVPPPPRR`
//! - Version (V): bits 7-6 (2 bits, default `0b00`)
//! - Payload Type (P): bits 5-2 (4 bits)
//! - Route Type (R): bits 1-0 (2 bits)

pub const MAX_PATH_SIZE: usize = 64;
pub const MAX_PACKET_PAYLOAD: usize = 184;
pub const MAX_PACKET_SIZE: usize = 255;

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

    /// Returns the hash size in bytes for each hop in the path (1 to 3 bytes).
    /// Encoded in bits 7-6: `(path_len >> 6) + 1`
    #[inline]
    pub fn path_hash_size(&self) -> usize {
        ((self.path_len >> 6) as usize) + 1
    }

    /// Returns the number of hops recorded in the path (0 to 63).
    /// Encoded in bits 5-0: `path_len & 63`
    #[inline]
    pub fn path_hash_count(&self) -> usize {
        (self.path_len & 63) as usize
    }

    /// Total byte length occupied by path hashes on the wire (`count * size`)
    #[inline]
    pub fn path_byte_len(&self) -> usize {
        self.path_hash_count() * self.path_hash_size()
    }

    /// Check if a wire `path_len` byte encodes a valid hop count and size
    #[inline]
    pub fn is_valid_path_len(path_len: u8) -> bool {
        let count = (path_len & 63) as usize;
        let size = ((path_len >> 6) as usize) + 1;
        size <= 3 && count * size <= MAX_PATH_SIZE
    }

    /// Set path hash size and hop count
    pub fn set_path_hash_size_and_count(&mut self, size: u8, count: u8) {
        let sz = if size == 0 { 0 } else if size > 3 { 3 } else { (size - 1) & 0x03 };
        self.path_len = (sz << 6) | (count & 63);
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
        self.append_hop_bytes(&[node_hash])
    }

    /// Appends multi-byte node hash bytes to the path matching the packet's path_hash_size
    pub fn append_hop_bytes(&mut self, hash_bytes: &[u8]) -> bool {
        let count = self.path_hash_count();
        let size = self.path_hash_size();
        let next_bytes = (count + 1) * size;
        if next_bytes <= MAX_PATH_SIZE {
            let offset = count * size;
            let copy_len = hash_bytes.len().min(size);
            self.path[offset..offset + copy_len].copy_from_slice(&hash_bytes[..copy_len]);
            for i in copy_len..size {
                self.path[offset + i] = 0;
            }
            self.set_path_hash_size_and_count(size as u8, (count + 1) as u8);
            true
        } else {
            false
        }
    }

    /// Checks if a node hash is already present in the path to detect loops
    pub fn contains_hop(&self, node_hash: &[u8]) -> bool {
        let count = self.path_hash_count();
        let size = self.path_hash_size();
        let check_len = node_hash.len().min(size);
        if check_len == 0 {
            return false;
        }
        for i in 0..count {
            let offset = i * size;
            if offset + check_len <= MAX_PATH_SIZE {
                if &self.path[offset..offset + check_len] == &node_hash[..check_len] {
                    return true;
                }
            }
        }
        false
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

        // 3. Path length byte (encodes both hash_size and hash_count)
        if dest.len() <= offset {
            return Err(());
        }
        dest[offset] = self.path_len;
        offset += 1;

        // 4. Path bytes (hash_count * hash_size)
        let path_bytes = self.path_byte_len();
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
        if header >> PH_VER_SHIFT != 0 {
            return Err(());
        }
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

        if !Packet::is_valid_path_len(path_len) {
            return Err(());
        }
        let hash_count = (path_len & 63) as usize;
        let hash_size = ((path_len >> 6) as usize) + 1;
        let path_bytes = hash_count * hash_size;

        if src.len() < offset + path_bytes {
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

    /// MeshCore duplicate-detection hash: `SHA256(type ‖ [path_len u16 if TRACE] ‖
    /// payload)[0..8]`.
    ///
    /// The header, path and transport codes are not part of the hash. TRACE is
    /// special-cased because a trace can revisit the same node on the return
    /// path; the C++ hashes `path_len` as a little-endian `uint16_t`.
    pub fn calculate_packet_hash(&self) -> [u8; 8] {
        use sha2::{Digest, Sha256};
        let t = self.payload_type();
        let mut hasher = Sha256::new();
        hasher.update([t]);
        if t == PAYLOAD_TYPE_TRACE {
            hasher.update((self.path_len as u16).to_le_bytes());
        }
        hasher.update(&self.payload[..self.payload_len]);
        let full: [u8; 32] = hasher.finalize().into();

        let mut hash = [0u8; 8];
        hash.copy_from_slice(&full[..8]);
        hash
    }
}
