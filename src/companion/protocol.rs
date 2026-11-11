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
pub const CMD_SYNC_NEXT_MESSAGE: u8 = 10;
pub const CMD_GET_MSG: u8 = 11;
pub const CMD_GET_BAT: u8 = 19;
pub const CMD_DEVICE_QUERY: u8 = 22;
pub const CMD_GET_CHANNEL: u8 = 31;
pub const CMD_SET_CHANNEL: u8 = 32;
pub const CMD_SEND_CHANNEL_DATA: u8 = 0x3E;

pub const RESP_CODE_OK: u8 = 0;
pub const RESP_CODE_ERR: u8 = 1;
pub const RESP_CODE_CONTACTS_START: u8 = 2;
pub const RESP_CODE_CONTACT: u8 = 3;
pub const RESP_CODE_END_OF_CONTACTS: u8 = 4;
pub const RESP_CODE_SELF_INFO: u8 = 5;
pub const RESP_CODE_CURR_TIME: u8 = 9;
pub const RESP_CODE_NO_MORE_MESSAGES: u8 = 10;
pub const RESP_CODE_DEVICE_INFO: u8 = 13;
pub const RESP_CODE_CONTACT_MSG_V3: u8 = 0x10;
pub const RESP_CODE_CHANNEL_MSG_V3: u8 = 0x11;
pub const RESP_CODE_CHANNEL_INFO: u8 = 18;
pub const RESP_CODE_BAT: u8 = 19;
pub const RESP_CODE_TIME: u8 = 20;
pub const RESP_CODE_MSG: u8 = 21;

pub const PUSH_CODE_SEND_CONFIRMED: u8 = 0x82;
pub const PUSH_CODE_MSG_WAITING: u8 = 0x83;
pub const PUSH_CODE_LOG_RX_DATA: u8 = 0x88;

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
    /// and generate corresponding response frames via the `emit` callback.
    pub fn handle_command<F: FnMut(&[u8])>(&mut self, payload: &[u8], mut emit: F) {
        if payload.is_empty() {
            return;
        }

        let cmd = payload[0];
        defmt::info!("MeshCore Companion CMD: {} (len {})", cmd, payload.len());

        match cmd {
            CMD_DEVICE_QUERY => {
                // RESP_CODE_DEVICE_INFO (13)
                // Protocol v3+ official layout:
                // [code(13), firmware_ver(3), max_contacts_div_2(50), max_channels(8), ble_pin(u32_le), build_date(12), model(40), semver(20)]
                let mut resp = Vec::<u8, 128>::new();
                let _ = resp.push(RESP_CODE_DEVICE_INFO);
                let _ = resp.push(3); // firmware_ver
                let _ = resp.push(50); // max_contacts_div_2 (100 contacts)
                let _ = resp.push(8); // max_channels
                let _ = resp.extend_from_slice(&123456u32.to_le_bytes()); // ble_pin: 123456

                let mut bdate = [0u8; 12];
                let d = b"Nov 10 2026";
                bdate[..d.len()].copy_from_slice(d);
                let _ = resp.extend_from_slice(&bdate);

                let mut mmodel = [0u8; 40];
                let m = b"Elecrow ThinkNode M1";
                mmodel[..m.len()].copy_from_slice(m);
                let _ = resp.extend_from_slice(&mmodel);

                let mut sver = [0u8; 20];
                let s = b"0.1.0-embassy";
                sver[..s.len()].copy_from_slice(s);
                let _ = resp.extend_from_slice(&sver);

                emit(&resp);
            }

            CMD_APP_START => {
                // RESP_CODE_SELF_INFO (5)
                // Layout:
                // [code(5), type, tx_power, max_tx_power, pubkey(32), adv_lat(i32_le), adv_lon(i32_le),
                //  reserved(0), advert_loc_policy(0), telemetry_modes(0), manual_add_contacts(0),
                //  radio_freq(u32_le_hz), radio_bw(u32_le_hz), radio_sf(u8), radio_cr(u8), name(str)]
                let mut resp = Vec::<u8, 128>::new();
                let _ = resp.push(RESP_CODE_SELF_INFO);
                let (is_rep, f, tx, sf, bw, cr) = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
                    let s = r.borrow();
                    (s.is_repeater, s.frequency_hz, s.tx_power_dbm, s.sf, s.bw_khz, s.cr)
                });
                let _ = resp.push(if is_rep { 2 } else { 1 }); // 1 = client, 2 = repeater
                let _ = resp.push(tx as u8); // current tx power
                let _ = resp.push(22); // max tx power
                let _ = resp.extend_from_slice(&self.public_key);
                let _ = resp.extend_from_slice(&0i32.to_le_bytes()); // adv_lat
                let _ = resp.extend_from_slice(&0i32.to_le_bytes()); // adv_lon
                let _ = resp.push(0); // reserved
                let _ = resp.push(0); // advert_loc_policy
                let _ = resp.push(0); // telemetry_modes
                let _ = resp.push(0); // manual_add_contacts
                let _ = resp.extend_from_slice(&f.to_le_bytes()); // radio_freq in Hz
                let bw_hz = bw * 1000;
                let _ = resp.extend_from_slice(&bw_hz.to_le_bytes()); // radio_bw in Hz
                let _ = resp.push(sf);
                let _ = resp.push(cr);
                let active_name = crate::storage::get_callsign();
                let _ = resp.extend_from_slice(active_name.as_bytes());

                emit(&resp);
            }

            CMD_GET_BAT => {
                // RESP_CODE_BAT (19)
                // Payload: [RESP_CODE, mV_low, mV_high, percentage]
                let mut resp = Vec::<u8, 8>::new();
                let _ = resp.push(RESP_CODE_BAT);
                let _ = resp.extend_from_slice(&self.battery_millivolts.to_le_bytes());
                let _ = resp.push(self.battery_percentage);
                emit(&resp);
            }

            CMD_GET_DEVICE_TIME => {
                // RESP_CODE_CURR_TIME (9)
                // Payload: [RESP_CODE(9), epoch_seconds_u32_le(4)]
                let mut resp = Vec::<u8, 8>::new();
                let _ = resp.push(RESP_CODE_CURR_TIME);
                let epoch = crate::companion::usb::get_device_time();
                let _ = resp.extend_from_slice(&epoch.to_le_bytes());
                emit(&resp);
            }

            CMD_SET_DEVICE_TIME => {
                // Payload from app: [CMD(6), epoch_seconds_u32_le(4)]
                if payload.len() >= 5 {
                    let mut time_bytes = [0u8; 4];
                    time_bytes.copy_from_slice(&payload[1..5]);
                    let epoch = u32::from_le_bytes(time_bytes);
                    self.epoch_time = epoch as u64;
                    crate::companion::usb::set_device_time(epoch);
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_GET_CHANNEL => {
                // Inbound: [CMD(31 / 0x1F), channel_idx]
                let channel_idx = if payload.len() > 1 { payload[1] } else { 0 };
                let mut resp = Vec::<u8, 64>::new();
                let _ = resp.push(RESP_CODE_CHANNEL_INFO); // 18 (0x12)
                let _ = resp.push(channel_idx);

                crate::companion::usb::CHANNELS.lock(|c| {
                    let chs = c.borrow();
                    if (channel_idx as usize) < chs.len() {
                        let ch = &chs[channel_idx as usize];
                        let _ = resp.extend_from_slice(&ch.name);
                        let _ = resp.extend_from_slice(&ch.secret);
                    } else {
                        let empty_name = [0u8; 32];
                        let empty_secret = [0u8; 16];
                        let _ = resp.extend_from_slice(&empty_name);
                        let _ = resp.extend_from_slice(&empty_secret);
                    }
                });

                emit(&resp);
            }

            CMD_SET_CHANNEL => {
                // Inbound: [CMD(32 / 0x20), channel_idx, name(32 bytes), secret(16 bytes)]
                if payload.len() >= 50 {
                    let channel_idx = payload[1] as usize;
                    if channel_idx < 8 {
                        let mut name = [0u8; 32];
                        name.copy_from_slice(&payload[2..34]);
                        let mut secret = [0u8; 16];
                        secret.copy_from_slice(&payload[34..50]);
                        crate::companion::usb::CHANNELS.lock(|c| {
                            c.borrow_mut()[channel_idx] = crate::companion::usb::ChannelData { name, secret };
                        });
                        crate::storage::request_save_config();
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_SYNC_NEXT_MESSAGE => {
                let next_msg = crate::companion::usb::MESSAGE_LOG.lock(|m| {
                    let mut list = m.borrow_mut();
                    if !list.is_empty() {
                        Some(list.remove(0))
                    } else {
                        None
                    }
                });

                if let Some(msg) = next_msg {
                    let mut resp = Vec::<u8, 256>::new();
                    if msg.is_direct {
                        // V3 DM: [0x10, (snr*4), 0, 0, 6B pubkey, path_len, txt_type, 4B ts, text]
                        let _ = resp.push(RESP_CODE_CONTACT_MSG_V3);
                        let _ = resp.push((msg.snr * 4) as u8);
                        let _ = resp.push(0);
                        let _ = resp.push(0);
                        let pk_prefix = [0u8; 6];
                        let _ = resp.extend_from_slice(&pk_prefix);
                        let _ = resp.push(msg.path_len);
                        let _ = resp.push(0); // TXT_TYPE_PLAIN
                        let _ = resp.extend_from_slice(&msg.timestamp.to_le_bytes());
                        let _ = resp.extend_from_slice(msg.text.as_bytes());
                    } else {
                        // V3 Channel: [0x11, (snr*4), 0, 0, channel_idx, path_len, txt_type, 4B ts, text]
                        let _ = resp.push(RESP_CODE_CHANNEL_MSG_V3);
                        let _ = resp.push((msg.snr * 4) as u8);
                        let _ = resp.push(0);
                        let _ = resp.push(0);
                        let _ = resp.push(msg.channel_idx);
                        let _ = resp.push(msg.path_len);
                        let _ = resp.push(0); // TXT_TYPE_PLAIN
                        let _ = resp.extend_from_slice(&msg.timestamp.to_le_bytes());
                        let _ = resp.extend_from_slice(msg.sender.as_bytes());
                        let _ = resp.extend_from_slice(b": ");
                        let _ = resp.extend_from_slice(msg.text.as_bytes());
                    }
                    emit(&resp);
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_NO_MORE_MESSAGES);
                    emit(&resp);
                }
            }

            CMD_GET_CONTACTS => {
                let contacts = crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| c.borrow().clone());
                let count = contacts.len() as u32;

                // 1. RESP_CODE_CONTACTS_START (2) [code(2), count(u32_le)]
                let mut start_resp = Vec::<u8, 8>::new();
                let _ = start_resp.push(RESP_CODE_CONTACTS_START);
                let _ = start_resp.extend_from_slice(&count.to_le_bytes());
                emit(&start_resp);

                // 2. Stream individual RESP_CODE_CONTACT (3)
                // Layout: [code(3), pubkey(32), type(1), flags(0), out_path_len(0), out_path(64), adv_name(32), last_advert(u32_le), adv_lat(i32_le), adv_lon(i32_le), lastmod(u32_le)]
                let mut max_lastmod = 0u32;
                for c in contacts.iter() {
                    let mut c_resp = Vec::<u8, 160>::new();
                    let _ = c_resp.push(RESP_CODE_CONTACT);
                    let _ = c_resp.extend_from_slice(&c.pub_key);
                    let _ = c_resp.push(1); // type = client
                    let _ = c_resp.push(0); // flags = 0
                    let _ = c_resp.push(0); // out_path_len = 0
                    let out_path = [0u8; 64];
                    let _ = c_resp.extend_from_slice(&out_path);

                    let mut adv_name = [0u8; 32];
                    let cs = c.callsign.as_bytes();
                    let cs_len = cs.len().min(31);
                    adv_name[..cs_len].copy_from_slice(&cs[..cs_len]);
                    let _ = c_resp.extend_from_slice(&adv_name);

                    let last_advert = c.last_seen_epoch as u32;
                    if last_advert > max_lastmod {
                        max_lastmod = last_advert;
                    }
                    let _ = c_resp.extend_from_slice(&last_advert.to_le_bytes());
                    let _ = c_resp.extend_from_slice(&0i32.to_le_bytes()); // adv_lat
                    let _ = c_resp.extend_from_slice(&0i32.to_le_bytes()); // adv_lon
                    let _ = c_resp.extend_from_slice(&last_advert.to_le_bytes()); // lastmod
                    emit(&c_resp);
                }

                // 3. RESP_CODE_END_OF_CONTACTS (4) [code(4), most_recent_lastmod(u32_le)]
                let mut end_resp = Vec::<u8, 8>::new();
                let _ = end_resp.push(RESP_CODE_END_OF_CONTACTS);
                let _ = end_resp.extend_from_slice(&max_lastmod.to_le_bytes());
                emit(&end_resp);
            }

            CMD_SEND_CHANNEL_MSG => {
                // Inbound channel message layout:
                // [CMD(3), txt_type(1), channel_idx(1), timestamp(4 bytes le), text...]
                if payload.len() > 7 {
                    let channel_idx = payload[2];
                    let timestamp = u32::from_le_bytes([payload[3], payload[4], payload[5], payload[6]]);
                    let text_bytes = &payload[7..];
                    if let Ok(msg_str) = core::str::from_utf8(text_bytes) {
                        let mut s = String::<128>::new();
                        let _ = s.push_str(msg_str);
                        let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(crate::companion::usb::UsbCommand::SendGroupText {
                            channel_idx,
                            timestamp,
                            text: s,
                        });
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_SEND_CHANNEL_DATA => {
                // Inbound channel datagram layout:
                // [CMD(0x3E), channel_idx(1), path_len(1), path..., data_type(2), payload...]
                if payload.len() >= 5 {
                    let channel_idx = payload[1];
                    let path_len = payload[2];
                    let hash_count = (path_len & 63) as usize;
                    let hash_size = ((path_len >> 6) as usize) + 1;
                    let path_bytes = if path_len == 0xFF { 0 } else { hash_count * hash_size };
                    let data_offset = 3 + path_bytes;
                    if payload.len() >= data_offset + 2 {
                        let _data_type = u16::from_le_bytes([payload[data_offset], payload[data_offset + 1]]);
                        let text_bytes = &payload[data_offset + 2..];
                        if let Ok(msg_str) = core::str::from_utf8(text_bytes) {
                            let mut s = String::<128>::new();
                            let _ = s.push_str(msg_str);
                            let now = crate::companion::usb::get_device_time();
                            let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(crate::companion::usb::UsbCommand::SendGroupText {
                                channel_idx,
                                timestamp: now,
                                text: s,
                            });
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
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
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_SEND_SELF_ADVERT => {
                let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(crate::companion::usb::UsbCommand::TriggerAdvert);
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            _ => {
                // Generic OK for unsupported unhandled commands to keep companion session alive
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }
        }
    }
}

