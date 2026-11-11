//! MeshCore Companion Protocol Command & Response Definitions
//!
//! Handles command parsing from companion apps (Mobile/Desktop/Web)
//! and generates protocol response frames.

use heapless::{String, Vec};

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

/// MeshCore Companion Node State
pub struct CompanionNode {
    pub callsign: String<32>,
    pub hw_model: &'static str,
    pub capabilities: u32,
    pub public_key: [u8; 32],
    pub private_key: [u8; 32],
    pub battery_millivolts: u16,
    pub battery_percentage: u8,
    pub epoch_time: u64,
    pub unread_messages: u16,
    pub gps_fix: bool,
    pub latitude: f32,
    pub longitude: f32,
    pub tx_beacons: u32,
    pub rx_packets: u32,
    pub last_rssi: i16,
    pub last_snr: i8,
    pub current_page: u8,
}

impl CompanionNode {
    pub fn new(
        callsign: &str,
        hw_model: &'static str,
        capabilities: u32,
        public_key: [u8; 32],
        private_key: [u8; 32],
    ) -> Self {
        let mut cs = String::new();
        let _ = cs.push_str(callsign);

        Self {
            callsign: cs,
            hw_model,
            capabilities,
            public_key,
            private_key,
            battery_millivolts: 4120, // 4.12V default nominal
            battery_percentage: 95,
            epoch_time: 0,
            unread_messages: 0,
            gps_fix: false,
            latitude: 0.0,
            longitude: 0.0,
            tx_beacons: 0,
            rx_packets: 0,
            last_rssi: 0,
            last_snr: 0,
            current_page: 0,
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
                
                // Hardware identifier
                let hw = self.hw_model.as_bytes();
                resp.push(hw.len() as u8).ok()?;
                resp.extend_from_slice(hw).ok()?;

                // Firmware version: "0.1.0-embassy"
                let fw = b"0.1.0-embassy";
                resp.push(fw.len() as u8).ok()?;
                resp.extend_from_slice(fw).ok()?;

                // Dynamic board capabilities
                resp.extend_from_slice(&self.capabilities.to_le_bytes()).ok()?;
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
                // Payload: [RESP_CODE, count, [pubkey(32), name_len, name, epoch_le(4), rssi_le(2), snr, bat]...]
                resp.push(RESP_CODE_CONTACTS).ok()?;
                crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                    let list = c.borrow();
                    let count = list.len().min(8) as u8; // limit chunk size to stay within 256B frame
                    resp.push(count).ok()?;
                    for contact in list.iter().take(count as usize) {
                        resp.extend_from_slice(&contact.pub_key).ok()?;
                        let cs = contact.callsign.as_bytes();
                        let cs_len = cs.len().min(16) as u8;
                        resp.push(cs_len).ok()?;
                        resp.extend_from_slice(&cs[..cs_len as usize]).ok()?;
                        resp.extend_from_slice(&contact.last_seen_epoch.to_le_bytes()).ok()?;
                        resp.extend_from_slice(&contact.rssi.to_le_bytes()).ok()?;
                        resp.push(contact.snr as u8).ok()?;
                        resp.push(contact.battery_percentage).ok()?;
                    }
                    Some(())
                });
                Some(resp)
            }

            CMD_SEND_CHANNEL_MSG => {
                // Inbound channel message: [CMD(3), channel_idx, timestamp(4 bytes le), text...]
                if payload.len() > 6 {
                    let text_bytes = &payload[6..];
                    if let Ok(msg_str) = core::str::from_utf8(text_bytes) {
                        let mut s = String::<128>::new();
                        let _ = s.push_str(msg_str);
                        let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(crate::companion::usb::UsbCommand::SendGroupText(s));
                    }
                }
                resp.push(RESP_CODE_OK).ok()?;
                Some(resp)
            }

            CMD_SEND_MSG => {
                // Inbound direct message: [CMD(2), dest_pubkey(32), timestamp(4 bytes le), text...]
                if payload.len() > 37 {
                    let dest_hash = [payload[1], payload[2], payload[3], payload[4]];
                    let text_bytes = &payload[37..];
                    if let Ok(msg_str) = core::str::from_utf8(text_bytes) {
                        let mut s = String::<128>::new();
                        let _ = s.push_str(msg_str);
                        let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(crate::companion::usb::UsbCommand::SendDirectText {
                            dest_hash,
                            text: s,
                        });
                    }
                }
                resp.push(RESP_CODE_OK).ok()?;
                Some(resp)
            }

            CMD_SEND_SELF_ADVERT => {
                let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(crate::companion::usb::UsbCommand::TriggerAdvert);
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

