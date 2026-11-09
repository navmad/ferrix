//! MeshCore Companion Framing Protocol
//!
//! Handles frame delimitation for USB-Serial and BLE transports:
//! - USB Serial:
//!   - App -> Node (Inbound):  '<' (0x3C) + 2-byte little-endian length + payload
//!   - Node -> App (Outbound): '>' (0x3E) + 2-byte little-endian length + payload
//! - BLE:
//!   - Direct payload matching MTU size over Nordic UART Service (NUS).

pub const USB_INBOUND_DELIMITER: u8 = 0x3C;  // '<'
pub const USB_OUTBOUND_DELIMITER: u8 = 0x3E; // '>'

pub const MAX_FRAME_PAYLOAD_SIZE: usize = 512;
pub const MAX_USB_FRAME_SIZE: usize = MAX_FRAME_PAYLOAD_SIZE + 3; // Delimiter + 2 bytes length

/// Reader for assembling streaming USB frames into complete packets.
pub struct UsbFrameReader {
    buffer: [u8; MAX_FRAME_PAYLOAD_SIZE],
    position: usize,
    expected_length: usize,
    state: ReaderState,
}

#[derive(Copy, Clone, Debug, PartialEq)]
enum ReaderState {
    WaitingForDelimiter,
    LengthLow,
    LengthHigh(u8),
    ReadingPayload,
}

impl UsbFrameReader {
    pub const fn new() -> Self {
        Self {
            buffer: [0u8; MAX_FRAME_PAYLOAD_SIZE],
            position: 0,
            expected_length: 0,
            state: ReaderState::WaitingForDelimiter,
        }
    }

    /// Reset internal parser state
    pub fn reset(&mut self) {
        self.position = 0;
        self.expected_length = 0;
        self.state = ReaderState::WaitingForDelimiter;
    }

    /// Feeds a single byte into the frame parser.
    /// Returns `Some(&[u8])` with the full payload slice when a complete frame is received.
    pub fn push_byte(&mut self, byte: u8) -> Option<&[u8]> {
        match self.state {
            ReaderState::WaitingForDelimiter => {
                if byte == USB_INBOUND_DELIMITER {
                    self.position = 0;
                    self.expected_length = 0;
                    self.state = ReaderState::LengthLow;
                }
                None
            }
            ReaderState::LengthLow => {
                self.state = ReaderState::LengthHigh(byte);
                None
            }
            ReaderState::LengthHigh(len_low) => {
                let len = (byte as usize) << 8 | (len_low as usize);
                if len == 0 || len > MAX_FRAME_PAYLOAD_SIZE {
                    // Invalid length, reset
                    self.reset();
                    None
                } else {
                    self.expected_length = len;
                    self.position = 0;
                    self.state = ReaderState::ReadingPayload;
                    None
                }
            }
            ReaderState::ReadingPayload => {
                if self.position < self.buffer.len() {
                    self.buffer[self.position] = byte;
                    self.position += 1;

                    if self.position == self.expected_length {
                        self.state = ReaderState::WaitingForDelimiter;
                        Some(&self.buffer[..self.position])
                    } else {
                        None
                    }
                } else {
                    // Overflow protection
                    self.reset();
                    None
                }
            }
        }
    }
}

/// Encloses a payload into an outbound USB frame: `> [len_lo] [len_hi] [payload]`.
/// Returns the total number of bytes written to `out`.
pub fn encode_usb_frame(payload: &[u8], out: &mut [u8]) -> Result<usize, ()> {
    let len = payload.len();
    if len > MAX_FRAME_PAYLOAD_SIZE || out.len() < len + 3 {
        return Err(());
    }

    out[0] = USB_OUTBOUND_DELIMITER;
    out[1] = (len & 0xFF) as u8;
    out[2] = ((len >> 8) & 0xFF) as u8;
    out[3..3 + len].copy_from_slice(payload);

    Ok(len + 3)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_usb_framing_roundtrip() {
        let mut reader = UsbFrameReader::new();
        let payload = b"HELLO MESHCORE";
        let mut framed = [0u8; 64];
        let frame_len = encode_usb_frame(payload, &mut framed).unwrap();

        // Feed delimiter '<' instead of '>' for inbound
        framed[0] = USB_INBOUND_DELIMITER;

        let mut received = None;
        for &b in &framed[..frame_len] {
            if let Some(p) = reader.push_byte(b) {
                received = Some(p);
            }
        }

        assert_eq!(received, Some(&payload[..]));
    }
}
