//! MeshCore Companion Protocol Command & Response Definitions
//!
//! Handles command parsing from companion apps (Mobile/Desktop/Web)
//! and generates protocol response frames.

use heapless::Vec;

pub const CMD_APP_START: u8 = 1;
pub const CMD_SEND_MSG: u8 = 2;
pub const CMD_SEND_CHANNEL_MSG: u8 = 3;
pub const CMD_GET_CONTACTS: u8 = 4;
pub const CMD_GET_DEVICE_TIME: u8 = 5;
pub const CMD_SET_DEVICE_TIME: u8 = 6;
pub const CMD_SEND_SELF_ADVERT: u8 = 7;
pub const CMD_GET_MSG: u8 = 11;
pub const CMD_GET_BAT: u8 = 19;
pub const CMD_DEVICE_QUERY: u8 = 22;

pub const RESP_CODE_OK: u8 = 0;
pub const RESP_CODE_ERR: u8 = 1;
pub const RESP_CODE_CONTACTS: u8 = 4;
pub const RESP_CODE_SELF_INFO: u8 = 5;
pub const RESP_CODE_DEVICE_INFO: u8 = 13;
pub const RESP_CODE_BAT: u8 = 19;
pub const RESP_CODE_TIME: u8 = 20;
pub const RESP_CODE_MSG: u8 = 21;

/// Device capabilities bitmask
pub const CAP_ROUTER: u32 = 1 << 0;
pub const CAP_CLIENT: u32 = 1 << 1;
pub const CAP_GPS: u32 = 1 << 2;
pub const CAP_EPD: u32 = 1 << 3;
pub const CAP_BLE: u32 = 1 << 4;

/// ThinkNode M1 Companion Node State
pub struct CompanionNode {
    pub callsign: &'static str,
    pub public_key: [u8; 32],
    pub battery_millivolts: u16,
    pub battery_percentage: u8,
    pub epoch_time: u64,
    pub unread_messages: u16,
    pub gps_fix: bool,
    pub latitude: f32,
    pub longitude: f32,
}

impl CompanionNode {
    pub fn new(callsign: &'static str) -> Self {
        let mut key = [0u8; 32];
        // Placeholder unique key derived from device identifier
        key[0..8].copy_from_slice(b"TN-M1-RS");

        Self {
            callsign,
            public_key: key,
            battery_millivolts: 4120, // 4.12V default nominal
            battery_percentage: 95,
            epoch_time: 0,
            unread_messages: 0,
            gps_fix: false,
            latitude: 0.0,
            longitude: 0.0,
        }
    }

    /// Process an inbound frame payload from the companion app
    /// and generate the corresponding response payload.
    pub fn handle_command(&mut self, payload: &[u8]) -> Option<Vec<u8, 256>> {
        if payload.is_empty() {
            return None;
        }

        let cmd = payload[0];
        let mut resp: Vec<u8, 256> = Vec::new();

        match cmd {
            CMD_DEVICE_QUERY => {
                // RESP_CODE_DEVICE_INFO (13)
                // Payload: [RESP_CODE, hardware_model, firmware_version, capabilities_u32_le]
                resp.push(RESP_CODE_DEVICE_INFO).ok()?;
                
                // Hardware identifier: "ThinkNode-M1-Rust"
                let hw = b"ThinkNode-M1-RS";
                resp.push(hw.len() as u8).ok()?;
                resp.extend_from_slice(hw).ok()?;

                // Firmware version: "0.1.0"
                let fw = b"0.1.0-embassy";
                resp.push(fw.len() as u8).ok()?;
                resp.extend_from_slice(fw).ok()?;

                // Device capabilities: CLIENT | GPS | EPD | BLE
                let caps = CAP_CLIENT | CAP_GPS | CAP_EPD | CAP_BLE;
                resp.extend_from_slice(&caps.to_le_bytes()).ok()?;
                Some(resp)
            }

            CMD_APP_START => {
                // RESP_CODE_SELF_INFO (5)
                // Payload: [RESP_CODE, pubkey(32), callsign_len, callsign]
                resp.push(RESP_CODE_SELF_INFO).ok()?;
                resp.extend_from_slice(&self.public_key).ok()?;
                
                let cs_bytes = self.callsign.as_bytes();
                resp.push(cs_bytes.len() as u8).ok()?;
                resp.extend_from_slice(cs_bytes).ok()?;
                Some(resp)
            }

            CMD_GET_BAT => {
                // RESP_CODE_BAT (19)
                // Payload: [RESP_CODE, mV_low, mV_high, percentage]
                resp.push(RESP_CODE_BAT).ok()?;
                resp.extend_from_slice(&self.battery_millivolts.to_le_bytes()).ok()?;
                resp.push(self.battery_percentage).ok()?;
                Some(resp)
            }

            CMD_GET_DEVICE_TIME => {
                // RESP_CODE_TIME (20)
                // Payload: [RESP_CODE, epoch_seconds_u64_le]
                resp.push(RESP_CODE_TIME).ok()?;
                resp.extend_from_slice(&self.epoch_time.to_le_bytes()).ok()?;
                Some(resp)
            }

            CMD_SET_DEVICE_TIME => {
                // Extract time from payload: [CMD, epoch_seconds_u64_le]
                if payload.len() >= 9 {
                    let mut time_bytes = [0u8; 8];
                    time_bytes.copy_from_slice(&payload[1..9]);
                    self.epoch_time = u64::from_le_bytes(time_bytes);
                }
                resp.push(RESP_CODE_OK).ok()?;
                Some(resp)
            }

            CMD_GET_CONTACTS => {
                // RESP_CODE_CONTACTS (4)
                // Empty contacts list for initial discovery
                resp.push(RESP_CODE_CONTACTS).ok()?;
                resp.push(0).ok()?; // 0 contacts
                Some(resp)
            }

            CMD_SEND_SELF_ADVERT => {
                // Acknowledge advertisement trigger
                resp.push(RESP_CODE_OK).ok()?;
                Some(resp)
            }

            _ => {
                // Generic OK for unsupported unhandled commands to keep companion session alive
                resp.push(RESP_CODE_OK).ok()?;
                Some(resp)
            }
        }
    }
}
