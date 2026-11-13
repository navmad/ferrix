//! MeshCore Companion Protocol Command & Response Definitions
//!
//! Handles command parsing from companion apps (Mobile/Desktop/Web)
//! and generates protocol response frames.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use heapless::{String, Vec};

use crate::meshcore::packet::{Packet, PAYLOAD_TYPE_ADVERT};
use crate::meshcore::packets::AdvertPacket;

/// Signing scratch buffer. Advertised to the app by `SIGN_START`.
pub const MAX_SIGN_DATA: usize = 4096;
static SIGN_ACTIVE: AtomicBool = AtomicBool::new(false);
static SIGN_BUFFER: Mutex<CriticalSectionRawMutex, RefCell<Vec<u8, MAX_SIGN_DATA>>> =
    Mutex::new(RefCell::new(Vec::new()));

/// Diagnostics for companion app capability negotiation. These are shared by
/// every transport so `get companion` can show what the connected app asked.
pub static DEVICE_QUERY_COUNT: AtomicU32 = AtomicU32::new(0);
pub static GET_AUTOADD_COUNT: AtomicU32 = AtomicU32::new(0);
pub static SET_AUTOADD_COUNT: AtomicU32 = AtomicU32::new(0);
pub static LAST_CMD: AtomicU8 = AtomicU8::new(0);

#[derive(Clone, Copy)]
struct ScopeState {
    send_key: Option<[u8; 16]>,
    force_unscoped: bool,
    default_name: [u8; 31],
    default_key: [u8; 16],
}
static SCOPE: Mutex<CriticalSectionRawMutex, RefCell<ScopeState>> =
    Mutex::new(RefCell::new(ScopeState {
        send_key: None,
        force_unscoped: false,
        default_name: [0; 31],
        default_key: [0; 16],
    }));

pub const CMD_APP_START: u8 = 1;
pub const CMD_SEND_MSG: u8 = 2;
pub const CMD_SEND_CHANNEL_MSG: u8 = 3;
pub const CMD_GET_CONTACTS: u8 = 4;
pub const CMD_GET_DEVICE_TIME: u8 = 5;
pub const CMD_SET_DEVICE_TIME: u8 = 6;
pub const CMD_SEND_SELF_ADVERT: u8 = 7;
pub const CMD_SET_ADVERT_NAME: u8 = 8;
pub const CMD_ADD_UPDATE_CONTACT: u8 = 9;
pub const CMD_SYNC_NEXT_MESSAGE: u8 = 10;
pub const CMD_SET_RADIO_PARAMS: u8 = 0x0B;
pub const CMD_SET_RADIO_TX_POWER: u8 = 0x0C;
pub const CMD_RESET_PATH: u8 = 0x0D;
pub const CMD_SET_ADVERT_LATLON: u8 = 0x0E;
pub const CMD_REMOVE_CONTACT: u8 = 15;
pub const CMD_EXPORT_CONTACT: u8 = 0x11;
pub const CMD_IMPORT_CONTACT: u8 = 0x12;
pub const CMD_REBOOT: u8 = 0x13;
pub const CMD_GET_BATT_AND_STORAGE: u8 = 0x14;
pub const CMD_SET_TUNING_PARAMS: u8 = 0x15;
pub const CMD_EXPORT_PRIVATE_KEY: u8 = 0x17;
pub const CMD_IMPORT_PRIVATE_KEY: u8 = 0x18;
pub const CMD_GET_CONTACT_BY_KEY: u8 = 0x1E;
pub const CMD_SIGN_START: u8 = 0x21;
pub const CMD_SIGN_DATA: u8 = 0x22;
pub const CMD_SIGN_FINISH: u8 = 0x23;
pub const CMD_SEND_TRACE_PATH: u8 = 0x24;
pub const CMD_SET_DEVICE_PIN: u8 = 0x25;
pub const CMD_SET_OTHER_PARAMS: u8 = 0x26;
pub const CMD_GET_CUSTOM_VARS: u8 = 0x28;
pub const CMD_SET_CUSTOM_VAR: u8 = 0x29;
pub const CMD_DEVICE_QUERY: u8 = 22;
pub const CMD_GET_ADVERT_PATH: u8 = 0x2A;
pub const CMD_GET_TUNING_PARAMS: u8 = 0x2B;
pub const CMD_GET_CHANNEL: u8 = 31;
pub const CMD_SET_CHANNEL: u8 = 32;
pub const CMD_SET_FLOOD_SCOPE_KEY: u8 = 0x36;
pub const CMD_SEND_CONTROL_DATA: u8 = 0x37;
pub const CMD_GET_STATS: u8 = 0x38;
pub const CMD_SET_AUTOADD_CONFIG: u8 = 0x3A;
pub const CMD_GET_AUTOADD_CONFIG: u8 = 0x3B;
pub const CMD_GET_ALLOWED_REPEAT_FREQ: u8 = 0x3C;
pub const CMD_SET_PATH_HASH_MODE: u8 = 0x3D;
pub const CMD_SEND_CHANNEL_DATA: u8 = 0x3E;
pub const CMD_SET_DEFAULT_FLOOD_SCOPE: u8 = 0x3F;
pub const CMD_GET_DEFAULT_FLOOD_SCOPE: u8 = 0x40;
pub const CMD_SEND_RAW_PACKET: u8 = 0x41;
pub const CMD_SEND_RAW_DATA: u8 = 0x19;

pub const RESP_CODE_OK: u8 = 0;
pub const RESP_CODE_ERR: u8 = 1;
pub const RESP_CODE_CONTACTS_START: u8 = 2;
pub const RESP_CODE_CONTACT: u8 = 3;
pub const RESP_CODE_END_OF_CONTACTS: u8 = 4;
pub const RESP_CODE_SELF_INFO: u8 = 5;
pub const RESP_CODE_CURR_TIME: u8 = 9;
pub const RESP_CODE_NO_MORE_MESSAGES: u8 = 10;
pub const RESP_CODE_DEVICE_INFO: u8 = 13;
pub const RESP_CODE_EXPORT_CONTACT: u8 = 0x0B;
pub const RESP_CODE_BATTERY: u8 = 0x0C;
pub const RESP_CODE_PRIVATE_KEY: u8 = 0x0E;
pub const RESP_CODE_CONTACT_MSG_V3: u8 = 0x10;
pub const RESP_CODE_CHANNEL_MSG_V3: u8 = 0x11;
pub const RESP_CODE_CHANNEL_INFO: u8 = 18;
pub const RESP_CODE_SIGN_START: u8 = 0x13;
pub const RESP_CODE_SIGNATURE: u8 = 0x14;
pub const RESP_CODE_CUSTOM_VARS: u8 = 0x15;
pub const RESP_CODE_ADVERT_PATH: u8 = 0x16;
pub const RESP_CODE_TUNING: u8 = 0x17;
pub const RESP_CODE_STATS: u8 = 0x18;
pub const RESP_CODE_AUTOADD: u8 = 0x19;
pub const RESP_CODE_ALLOWED_REPEAT_FREQ: u8 = 0x1A;
pub const RESP_CODE_DEFAULT_FLOOD_SCOPE: u8 = 0x1C;
pub const RESP_CODE_TIME: u8 = 20;
pub const RESP_CODE_MSG: u8 = 21;

pub const PUSH_CODE_SEND_CONFIRMED: u8 = 0x82;
pub const PUSH_CODE_MSG_WAITING: u8 = 0x83;
pub const PUSH_CODE_LOG_RX_DATA: u8 = 0x88;
pub const PUSH_CODE_CONTACTS_FULL: u8 = 0x89;
pub const PUSH_CODE_CONTACT_DELETED: u8 = 0x8A;

/// Auto-add configuration bitmask (matches MeshCore)
pub const AUTO_ADD_OVERWRITE_OLDEST: u8 = 1 << 0;
pub const AUTO_ADD_CHAT: u8 = 1 << 1;
pub const AUTO_ADD_REPEATER: u8 = 1 << 2;
pub const AUTO_ADD_ROOM_SERVER: u8 = 1 << 3;
pub const AUTO_ADD_SENSOR: u8 = 1 << 4;

/// Contact type constants (from advert flags)
pub const ADV_TYPE_CHAT: u8 = 1;
pub const ADV_TYPE_REPEATER: u8 = 2;
pub const ADV_TYPE_ROOM: u8 = 3;
pub const ADV_TYPE_SENSOR: u8 = 4;

/// Contact flags bitmask
pub const CONTACT_FLAG_FAVORITE: u8 = 1 << 0;

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
    /// MeshCore's 64-byte expanded private key (`scalar ‖ nonce_prefix`).
    pub private_key: [u8; 64],
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
    pub path_hash_mode: u8,
    /// Auto-add configuration bitmask (matches MeshCore)
    pub autoadd_config: u8,
    /// Max hops for auto-adding contacts (0 = no limit)
    pub autoadd_max_hops: u8,
    /// Receive delay base in milliseconds (tuning parameter).
    pub rx_delay_ms: u32,
    /// Airtime factor × 1000 (tuning parameter).
    pub airtime_factor_milli: u32,
    pub manual_add_contacts: u8,
    pub telemetry_mode_base: u8,
    pub telemetry_mode_loc: u8,
    pub telemetry_mode_env: u8,
    pub advert_loc_policy: u8,
    pub multi_acks: u8,
    /// Tracks how many channels have been synced since the last connection.
    /// Used to defer connection parameter updates until after the sync burst.
    pub channels_synced: u8,
}

impl CompanionNode {
    pub fn new(
        callsign: &str,
        hw_model: &'static str,
        capabilities: u32,
        public_key: [u8; 32],
        private_key: [u8; 64],
    ) -> Self {
        let mut cs = String::new();
        let _ = cs.push_str(callsign);
        let auto = crate::companion::usb::AUTOADD_SETTINGS.lock(|a| *a.borrow());
        let battery_mv = crate::telemetry::battery_millivolts();
        let (battery_millivolts, battery_percentage) =
            if battery_mv >= crate::telemetry::MIN_PLAUSIBLE_BATTERY_MV {
                (battery_mv, crate::telemetry::millivolts_to_percentage(battery_mv))
            } else {
                (4120, 95)
            };

        Self {
            callsign: cs,
            hw_model,
            capabilities,
            public_key,
            private_key,
            battery_millivolts,
            battery_percentage,
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
            path_hash_mode: crate::storage::get_path_hash_mode(),
            autoadd_config: auto.config,
            autoadd_max_hops: auto.max_hops,
            rx_delay_ms: 0,
            airtime_factor_milli: 1000,
            manual_add_contacts: auto.manual_add_contacts,
            telemetry_mode_base: 0,
            telemetry_mode_loc: 0,
            telemetry_mode_env: 0,
            advert_loc_policy: 0,
            multi_acks: 0,
            channels_synced: 0,
        }
    }

    /// Determine the contact type from advert flags.
    /// The low nibble is a single kind value: 1 chat, 2 repeater, 3 room, 4 sensor.
    pub fn get_contact_type_from_flags(flags: u8) -> u8 {
        match flags & 0x0F {
            2 => ADV_TYPE_REPEATER,
            3 => ADV_TYPE_ROOM,
            4 => ADV_TYPE_SENSOR,
            _ => ADV_TYPE_CHAT,
        }
    }

    /// Check if a contact type should be auto-added based on current config
    /// Returns true if the contact type is enabled for auto-add
    pub fn should_auto_add_contact_type(&self, contact_type: u8) -> bool {
        // The shared policy is authoritative: the app may have changed it on
        // the BLE transport while this instance serves USB (or vice versa).
        let auto = crate::companion::usb::AUTOADD_SETTINGS.lock(|a| *a.borrow());
        // Bit 0 of manual_add_contacts: 0 = add every advert, 1 = only the
        // contact types enabled in the autoadd_config bitmask.
        if (auto.manual_add_contacts & 1) == 0 {
            return true;
        }
        let type_bit = match contact_type {
            ADV_TYPE_CHAT => AUTO_ADD_CHAT,
            ADV_TYPE_REPEATER => AUTO_ADD_REPEATER,
            ADV_TYPE_ROOM => AUTO_ADD_ROOM_SERVER,
            ADV_TYPE_SENSOR => AUTO_ADD_SENSOR,
            _ => return false,
        };
        (auto.config & type_bit) != 0
    }

    /// Check whether an advert's hop count is within the auto-add limit.
    pub fn autoadd_allows_hops(&self, hops: u8) -> bool {
        let max_hops = crate::companion::usb::AUTOADD_SETTINGS.lock(|a| a.borrow().max_hops);
        max_hops == 0 || hops < max_hops
    }

    /// Check if oldest non-favorite contacts should be overwritten when the list is full
    pub fn should_overwrite_when_full(&self) -> bool {
        let config = crate::companion::usb::AUTOADD_SETTINGS.lock(|a| a.borrow().config);
        (config & AUTO_ADD_OVERWRITE_OLDEST) != 0
    }

    /// Process an inbound frame payload from the companion app
    /// and generate corresponding response frames via the `emit` callback.
    pub fn handle_command<F: FnMut(&[u8])>(&mut self, payload: &[u8], mut emit: F) {
        if payload.is_empty() {
            return;
        }

        // Refresh the battery snapshot from the shared telemetry state (the
        // ADC is sampled by the main loop). Keep the last good value while the
        // ADC has not produced a plausible sample yet; overwriting it with 0
        // makes the app briefly show 0%.
        let battery_mv = crate::telemetry::battery_millivolts();
        if battery_mv >= crate::telemetry::MIN_PLAUSIBLE_BATTERY_MV {
            self.battery_millivolts = battery_mv;
            self.battery_percentage = crate::telemetry::battery_percentage();
        }

        let cmd = payload[0];
        LAST_CMD.store(cmd, Ordering::Relaxed);
        match cmd {
            CMD_DEVICE_QUERY => {
                DEVICE_QUERY_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            CMD_GET_AUTOADD_CONFIG => {
                GET_AUTOADD_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            CMD_SET_AUTOADD_CONFIG => {
                SET_AUTOADD_COUNT.fetch_add(1, Ordering::Relaxed);
            }
            _ => {}
        }
        // NOTE: this runs in `command_fut` (normal task context), never inside
        // the SoftDevice GATT callback. Logging is therefore safe here.

        match cmd {
            CMD_DEVICE_QUERY => {
                // Reset channel sync counter on each new device query (app reconnection).
                self.channels_synced = 0;
                // RESP_CODE_DEVICE_INFO (13)
                // Protocol v3+ official layout:
                // [code(13), firmware_ver(14), max_contacts_div_2(50), max_channels(8), ble_pin(u32_le), build_date(12), model(40), semver(20), offgrid(0), path_hash_mode(u8)]
                let mut resp = Vec::<u8, 128>::new();
                let _ = resp.push(RESP_CODE_DEVICE_INFO);
                let _ = resp.push(14); // firmware_ver: 14 (v10+ required for path_hash_mode in apps)
                let _ = resp.push((crate::companion::usb::MAX_CONTACTS / 2).min(255) as u8);
                let _ = resp.push(crate::companion::usb::MAX_CHANNELS as u8);
                let _ = resp.extend_from_slice(
                    &crate::companion::ble::BLE_PIN
                        .load(Ordering::Relaxed)
                        .to_le_bytes(),
                );

                let mut bdate = [0u8; 12];
                let d = b"Nov 10 2026";
                bdate[..d.len()].copy_from_slice(d);
                let _ = resp.extend_from_slice(&bdate);

                let mut mmodel = [0u8; 40];
                let m = b"Elecrow ThinkNode M1";
                mmodel[..m.len()].copy_from_slice(m);
                let _ = resp.extend_from_slice(&mmodel);

                let mut sver = [0u8; 20];
                // The app gates several settings on the firmware version
                // string, so report the MeshCore-compatible one.
                let s = b"v1.18.0";
                sver[..s.len()].copy_from_slice(s);
                let _ = resp.extend_from_slice(&sver);

                let _ = resp.push(if crate::companion::usb::offgrid_active() { 1 } else { 0 }); // offgrid mode
                let _ = resp.push(self.path_hash_mode); // path_hash_mode (0=1B, 1=2B, 2=3B)

                emit(&resp);
            }

            CMD_APP_START => {
                // RESP_CODE_SELF_INFO (5)
                // Layout:
                // [code(5), type, tx_power, max_tx_power, pubkey(32), adv_lat(i32_le), adv_lon(i32_le),
                //  reserved(0), advert_loc_policy(0), telemetry_modes(0), manual_add_contacts(0),
                //  radio_freq(u32_le_khz), radio_bw(u32_le_hz), radio_sf(u8), radio_cr(u8), name(str)]
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
                let _ = resp.push(self.advert_loc_policy);
                let _ = resp.push(
                    (self.telemetry_mode_env << 4)
                        | (self.telemetry_mode_loc << 2)
                        | self.telemetry_mode_base,
                );
                let _ = resp.push(self.manual_add_contacts);
                // MeshCore SELF_INFO carries frequency in kHz and bandwidth in
                // Hz (the app divides freq by 1000 to show MHz).
                let _ = resp.extend_from_slice(&(f / 1000).to_le_bytes()); // radio_freq in kHz
                let bw_hz = crate::companion::usb::bw_khz_to_hz(bw);
                let _ = resp.extend_from_slice(&bw_hz.to_le_bytes()); // radio_bw in Hz
                let _ = resp.push(sf);
                let _ = resp.push(cr);
                let active_name = crate::storage::get_callsign();
                let _ = resp.extend_from_slice(active_name.as_bytes());

                emit(&resp);
            }

            CMD_GET_BATT_AND_STORAGE => {
                // [PACKET_BATTERY][mV u16][storage used KiB u32][storage total KiB u32]
                let mut resp = Vec::<u8, 16>::new();
                let _ = resp.push(RESP_CODE_BATTERY);
                let _ = resp.extend_from_slice(&self.battery_millivolts.to_le_bytes());
                let _ = resp.extend_from_slice(&0u32.to_le_bytes());
                let _ = resp.extend_from_slice(&0u32.to_le_bytes());
                emit(&resp);
            }

            CMD_SET_ADVERT_NAME => {
                let mut ok = false;
                if payload.len() >= 2 {
                    let raw = &payload[1..];
                    let end = raw.iter().position(|&b| b == 0).unwrap_or(raw.len());
                    if let Ok(text) = core::str::from_utf8(&raw[..end]) {
                        let mut name = String::<32>::new();
                        if !text.is_empty() && name.push_str(text).is_ok() {
                            self.callsign = name.clone();
                            crate::storage::set_callsign(text);
                            let _ = crate::companion::usb::USB_COMMAND_CHANNEL
                                .try_send(crate::companion::usb::UsbCommand::SetName(name));
                            ok = true;
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_RADIO_PARAMS => {
                let mut ok = false;
                if payload.len() >= 11 {
                    let freq_khz =
                        u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
                    let bw_hz =
                        u32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);
                    let sf = payload[9];
                    let cr = payload[10];
                    // v9+ appends the client-repeat (off-grid) flag. It is
                    // only valid on an allowed repeat band.
                    let repeat = payload.get(11).map_or(false, |&b| b != 0);
                    if (100_000..=1_000_000).contains(&freq_khz)
                        && (7_800..=500_000).contains(&bw_hz)
                        && (5..=12).contains(&sf)
                        && (5..=8).contains(&cr)
                        && (!repeat || crate::companion::usb::is_offgrid_freq_khz(freq_khz))
                    {
                        // The app re-sends its stored settings on every connect.
                        // Only touch flash and interrupt RX when something changed.
                        let (rf_changed, repeat_changed, forward_changed, forward) =
                            crate::companion::usb::RADIO_SETTINGS.lock(|r| {
                                let mut s = r.borrow_mut();
                                let old_forward = s.is_repeater
                                    || (s.client_repeat
                                        && crate::companion::usb::is_offgrid_freq_hz(s.frequency_hz));
                                let new_freq = freq_khz * 1000;
                                let new_bw = bw_hz / 1000;
                                let rf_changed = s.frequency_hz != new_freq
                                    || s.bw_khz != new_bw
                                    || s.sf != sf
                                    || s.cr != cr;
                                let repeat_changed = s.client_repeat != repeat;
                                s.frequency_hz = new_freq;
                                s.bw_khz = new_bw;
                                s.sf = sf;
                                s.cr = cr;
                                s.client_repeat = repeat;
                                // The app owns the frequency, so forget the
                                // saved home frequency.
                                s.offgrid_home_freq = 0;
                                let forward = s.is_repeater
                                    || (repeat
                                        && crate::companion::usb::is_offgrid_freq_khz(freq_khz));
                                (rf_changed, repeat_changed, forward != old_forward, forward)
                            });
                        if rf_changed || repeat_changed {
                            crate::storage::request_save_config();
                        }
                        if rf_changed {
                            crate::radio::lora_phy::request_radio_config();
                        }
                        if forward_changed {
                            let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(
                                crate::companion::usb::UsbCommand::SetRepeat(forward),
                            );
                        }
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_RADIO_TX_POWER => {
                let mut ok = false;
                if payload.len() >= 2 {
                    let power = payload[1] as i8;
                    if (-9..=22).contains(&power) {
                        let changed = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
                            let mut s = r.borrow_mut();
                            let changed = s.tx_power_dbm != power;
                            s.tx_power_dbm = power;
                            changed
                        });
                        if changed {
                            crate::storage::request_save_config();
                            crate::radio::lora_phy::request_radio_config();
                        }
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_ADVERT_LATLON => {
                let mut ok = false;
                if payload.len() >= 9 {
                    let lat =
                        i32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
                    let lon =
                        i32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);
                    if (-90_000_000..=90_000_000).contains(&lat)
                        && (-180_000_000..=180_000_000).contains(&lon)
                    {
                        self.latitude = lat as f32 / 1_000_000.0;
                        self.longitude = lon as f32 / 1_000_000.0;
                        let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(
                            crate::companion::usb::UsbCommand::SetAdvertLatLon { lat, lon },
                        );
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_RESET_PATH => {
                let mut ok = false;
                if payload.len() >= 33 {
                    let mut pub_key = [0u8; 32];
                    pub_key.copy_from_slice(&payload[1..33]);
                    ok = crate::companion::ble::CONTACT_OP_CHANNEL
                        .try_send(crate::companion::ble::ContactOperation::ResetPath { pub_key })
                        .is_ok();
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_EXPORT_PRIVATE_KEY => {
                // [PACKET_PRIVATE_KEY][64-byte expanded private key]
                let mut resp = Vec::<u8, 66>::new();
                let _ = resp.push(RESP_CODE_PRIVATE_KEY);
                let _ = resp.extend_from_slice(&self.private_key);
                emit(&resp);
            }

            CMD_IMPORT_PRIVATE_KEY => {
                let mut ok = false;
                if payload.len() >= 65 {
                    let mut key = [0u8; 64];
                    key.copy_from_slice(&payload[1..65]);
                    if let Some(id) = crate::meshcore::identity::LocalIdentity::from_private(
                        self.callsign.as_str(),
                        &key,
                    ) {
                        self.private_key = key;
                        self.public_key = *id.pub_key();
                        let _ = crate::companion::usb::USB_COMMAND_CHANNEL
                            .try_send(crate::companion::usb::UsbCommand::SetPrivateKey(key));
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_TUNING_PARAMS => {
                let mut ok = false;
                if payload.len() >= 9 {
                    self.rx_delay_ms =
                        u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]);
                    self.airtime_factor_milli =
                        u32::from_le_bytes([payload[5], payload[6], payload[7], payload[8]]);
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_TUNING_PARAMS => {
                let mut resp = Vec::<u8, 9>::new();
                let _ = resp.push(RESP_CODE_TUNING);
                let _ = resp.extend_from_slice(&self.rx_delay_ms.to_le_bytes());
                let _ = resp.extend_from_slice(&self.airtime_factor_milli.to_le_bytes());
                emit(&resp);
            }

            CMD_SET_AUTOADD_CONFIG => {
                let mut ok = false;
                if payload.len() >= 2 {
                    let config = payload[1];
                    let max_hops = payload.get(2).map_or(self.autoadd_max_hops, |&b| b.min(64));
                    let changed = config != self.autoadd_config || max_hops != self.autoadd_max_hops;
                    self.autoadd_config = config;
                    self.autoadd_max_hops = max_hops;
                    crate::companion::usb::AUTOADD_SETTINGS.lock(|a| {
                        let mut st = a.borrow_mut();
                        st.config = config;
                        st.max_hops = max_hops;
                    });
                    if changed {
                        crate::storage::request_save_config();
                    }
                    let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(
                        crate::companion::usb::UsbCommand::SetAutoAddConfig {
                            config: self.autoadd_config,
                            max_hops: self.autoadd_max_hops,
                        },
                    );
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_AUTOADD_CONFIG => {
                let mut resp = Vec::<u8, 3>::new();
                let _ = resp.push(RESP_CODE_AUTOADD);
                let _ = resp.push(self.autoadd_config);
                let _ = resp.push(self.autoadd_max_hops);
                emit(&resp);
            }

            CMD_GET_ADVERT_PATH => {
                let mut found = None;
                if payload.len() >= 9 {
                    let prefix = &payload[2..9];
                    found = crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow()
                            .iter()
                            .find(|ct| &ct.pub_key[..7] == prefix)
                            .map(|ct| (ct.out_path_len, ct.out_path, ct.last_advert_ts))
                    });
                }
                if let Some((path_len, path, ts)) = found {
                    if path_len != 0xFF
                        && crate::meshcore::packet::Packet::is_valid_path_len(path_len)
                    {
                        let n = (path_len & 63) as usize * (((path_len >> 6) as usize) + 1);
                        let mut resp = Vec::<u8, 72>::new();
                        let _ = resp.push(RESP_CODE_ADVERT_PATH);
                        let _ = resp.extend_from_slice(&ts.to_le_bytes());
                        let _ = resp.push(path_len);
                        let _ = resp.extend_from_slice(&path[..n]);
                        emit(&resp);
                    } else {
                        let mut resp = Vec::<u8, 4>::new();
                        let _ = resp.push(RESP_CODE_ERR);
                        emit(&resp);
                    }
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    emit(&resp);
                }
            }

            CMD_GET_STATS => {
                let mut handled = false;
                if payload.len() >= 2 {
                    let kind = payload[1];
                    if kind == 0 {
                        let uptime = embassy_time::Instant::now().as_secs() as u32;
                        let mut resp = Vec::<u8, 11>::new();
                        let _ = resp.push(RESP_CODE_STATS);
                        let _ = resp.push(0);
                        let _ = resp.extend_from_slice(&self.battery_millivolts.to_le_bytes());
                        let _ = resp.extend_from_slice(&uptime.to_le_bytes());
                        let _ = resp.extend_from_slice(&0u16.to_le_bytes());
                        let _ = resp.push(0);
                        emit(&resp);
                        handled = true;
                    } else if kind == 1 {
                        let (rssi, snr) = crate::companion::usb::RADIO_TELEMETRY
                            .lock(|t| (t.borrow().last_rssi, t.borrow().last_snr));
                        let mut resp = Vec::<u8, 14>::new();
                        let _ = resp.push(RESP_CODE_STATS);
                        let _ = resp.push(1);
                        let _ = resp.extend_from_slice(&0i16.to_le_bytes());
                        let _ = resp.push(rssi as i8 as u8);
                        let _ = resp.push((snr * 4) as u8);
                        let _ = resp.extend_from_slice(&0u32.to_le_bytes());
                        let _ = resp.extend_from_slice(&0u32.to_le_bytes());
                        emit(&resp);
                        handled = true;
                    }
                }
                if !handled {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    emit(&resp);
                }
            }

            CMD_REBOOT => {
                if payload.len() >= 7 && &payload[1..7] == b"reboot" {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_OK);
                    emit(&resp);
                    cortex_m::peripheral::SCB::sys_reset();
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    emit(&resp);
                }
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
                self.channels_synced = self.channels_synced.saturating_add(1);
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
                    if channel_idx < crate::companion::usb::MAX_CHANNELS {
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
                if next_msg.is_some() {
                    // The app consumed a message; persist the shorter queue.
                    crate::storage::request_messages();
                }

                if let Some(msg) = next_msg {
                    let mut resp = Vec::<u8, 256>::new();
                    if msg.is_direct {
                        // V3 DM: [0x10, (snr*4), 0, 0, 6B pubkey, path_len, txt_type, 4B ts, text]
                        let _ = resp.push(RESP_CODE_CONTACT_MSG_V3);
                        let _ = resp.push((msg.snr * 4) as u8);
                        let _ = resp.push(0);
                        let _ = resp.push(0);
                        let _ = resp.extend_from_slice(&msg.sender_pub[..6]);
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
                // The transports handle this with async streaming: with up to
                // 350 contacts, the start + N contacts + end burst does not fit
                // the bounded push channel. See `ble::stream_contacts` and
                // `usb::stream_contacts_usb`.
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_ERR);
                emit(&resp);
            }

            CMD_SEND_CHANNEL_MSG => {
                // Inbound channel message layout:
                // [CMD(3), txt_type(1), channel_idx(1), timestamp(4 bytes le), text...]
                if payload.len() > 7 {
                    let channel_idx = payload[2];
                    let timestamp = u32::from_le_bytes([payload[3], payload[4], payload[5], payload[6]]);
                    let text_bytes = &payload[7..];
                    if let Ok(msg_str) = core::str::from_utf8(text_bytes) {
                        let msg_str = msg_str.trim_end_matches('\0');
                        let mut s = String::<160>::new();
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
                            let msg_str = msg_str.trim_end_matches('\0');
                            let mut s = String::<160>::new();
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
                // [cmd][txt_type][attempt][timestamp u32 LE][pubkey_prefix 6][text...]
                // Minimum: 1 + 1 + 1 + 4 + 6 + 1 = 14 bytes.
                let mut resp = Vec::<u8, 4>::new();
                let mut queued = false;
                if payload.len() >= 14 {
                    let txt_type = payload[1];
                    let attempt = payload[2];
                    let timestamp = u32::from_le_bytes([
                        payload[3], payload[4], payload[5], payload[6],
                    ]);
                    let mut dest_prefix = [0u8; 6];
                    dest_prefix.copy_from_slice(&payload[7..13]);
                    if let Ok(text_str) = core::str::from_utf8(&payload[13..]) {
                        let text_str = text_str.trim_end_matches('\0');
                        let mut s = String::<160>::new();
                        if s.push_str(text_str).is_ok() {
                            queued = crate::companion::usb::USB_COMMAND_CHANNEL
                                .try_send(crate::companion::usb::UsbCommand::SendDirectText {
                                    dest_prefix,
                                    timestamp,
                                    txt_type,
                                    attempt,
                                    text: s,
                                })
                                .is_ok();
                        }
                    }
                }
                let _ = resp.push(if queued { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_PATH_HASH_MODE => {
                // Inbound: [0x3D, 0, mode] or [0x3D, mode]
                let mode_opt = if payload.len() >= 3 && payload[1] == 0 {
                    Some(payload[2])
                } else if payload.len() >= 2 {
                    Some(payload[1])
                } else {
                    None
                };

                if let Some(mode) = mode_opt {
                    if mode <= 2 {
                        self.path_hash_mode = mode;
                        crate::storage::set_path_hash_mode(mode);
                        let _ = crate::companion::usb::USB_COMMAND_CHANNEL.try_send(crate::companion::usb::UsbCommand::SetPathHashMode(mode));
                        let mut resp = Vec::<u8, 4>::new();
                        let _ = resp.push(RESP_CODE_OK);
                        emit(&resp);
                        return;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_ERR);
                emit(&resp);
            }

            CMD_SEND_SELF_ADVERT => {
                let flood = payload.len() >= 2 && payload[1] == 1;
                let _ = crate::companion::usb::USB_COMMAND_CHANNEL
                    .try_send(crate::companion::usb::UsbCommand::TriggerAdvert { flood });
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_ADD_UPDATE_CONTACT => {
                // Inbound: [CMD(9), public_key(32), type(1), flags(1), out_path_len(1), out_path(64), adv_name(32), last_advert(4)]
                // Optional: gps_lat(4), gps_lon(4), lastmod(4)
                // Minimum required: 1 + 32 + 1 + 1 + 1 + 64 + 32 + 4 = 136 bytes (matches MeshCore)
                //
                // This handler runs in `command_fut` (normal task context), but contact
                // storage is still deferred to `contact_task` via CONTACT_OP_CHANNEL so
                // a slow or full contact list cannot back up the BLE task.
                if payload.len() >= 1 + 32 + 1 + 1 + 1 + 64 + 32 + 4 {
                    let mut i = 1;
                    let pub_key: [u8; 32] = payload[i..i+32].try_into().unwrap(); i += 32;
                    let contact_type = payload[i]; i += 1;
                    let flags = payload[i]; i += 1;
                    let out_path_len = payload[i]; i += 1;
                    let mut out_path = [0u8; 64];
                    out_path.copy_from_slice(&payload[i..i+64]); i += 64;
                    let adv_name_bytes = &payload[i..i+32]; i += 32;
                    let adv_name = core::str::from_utf8(adv_name_bytes).unwrap_or("").trim_end_matches('\0');
                    let last_advert = u32::from_le_bytes([payload[i], payload[i+1], payload[i+2], payload[i+3]]); i += 4;

                    // Optional fields after the required 136 bytes
                    let (adv_lat, adv_lon, lastmod) = if payload.len() >= i + 8 {
                        let lat = i32::from_le_bytes([payload[i], payload[i+1], payload[i+2], payload[i+3]]); i += 4;
                        let lon = i32::from_le_bytes([payload[i], payload[i+1], payload[i+2], payload[i+3]]); i += 4;
                        if payload.len() >= i + 4 {
                            let lm = u32::from_le_bytes([payload[i], payload[i+1], payload[i+2], payload[i+3]]);
                            (lat, lon, lm)
                        } else {
                            let lm = last_advert;
                            (lat, lon, lm)
                        }
                    } else {
                        (0, 0, last_advert)
                    };

                    let node_id = [pub_key[0], pub_key[1], pub_key[2], pub_key[3]];

                    let mut callsign = heapless::String::new();
                    let _ = callsign.push_str(adv_name);

                    // Defer the actual contact storage to the background task to avoid
                    // locking DISCOVERED_CONTACTS inside the SoftDevice critical section.
                    // If the channel is full, we drop the operation — this is acceptable
                    // since the app can retry.
                    let _ = crate::companion::ble::CONTACT_OP_CHANNEL.try_send(
                        crate::companion::ble::ContactOperation::AddOrUpdate {
                            pub_key,
                            callsign,
                            node_id,
                            last_seen_epoch: lastmod,
                            latitude: adv_lat,
                            longitude: adv_lon,
                            flags,
                            contact_type,
                            out_path,
                            out_path_len,
                            last_advert_ts: last_advert,
                        }
                    );

                    // Return OK immediately — the actual storage happens asynchronously.
                    // The companion also returns OK before the contact is fully persisted.
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_OK);
                    emit(&resp);
                    return;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_ERR);
                emit(&resp);
            }

            CMD_REMOVE_CONTACT => {
                // Inbound: [CMD(15), public_key(32)]
                // Defer the actual removal to `contact_task`, like AddOrUpdate.
                if payload.len() >= 33 {
                    let pub_key: [u8; 32] = payload[1..33].try_into().unwrap();
                    let _ = crate::companion::ble::CONTACT_OP_CHANNEL.try_send(
                        crate::companion::ble::ContactOperation::Remove { pub_key }
                    );
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_OK);
                emit(&resp);
            }

            CMD_GET_CONTACT_BY_KEY => {
                let found = if payload.len() >= 33 {
                    let mut key = [0u8; 32];
                    key.copy_from_slice(&payload[1..33]);
                    crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| {
                        c.borrow().iter().find(|ct| ct.pub_key == key).cloned()
                    })
                } else {
                    None
                };
                match found {
                    Some(ct) => emit(&serialize_contact(&ct)),
                    None => {
                        let mut resp = Vec::<u8, 4>::new();
                        let _ = resp.push(RESP_CODE_ERR);
                        emit(&resp);
                    }
                }
            }

            CMD_SIGN_START => {
                SIGN_ACTIVE.store(true, Ordering::Relaxed);
                SIGN_BUFFER.lock(|b| b.borrow_mut().clear());
                let mut resp = Vec::<u8, 6>::new();
                let _ = resp.push(RESP_CODE_SIGN_START);
                let _ = resp.push(0);
                let _ = resp.extend_from_slice(&(MAX_SIGN_DATA as u32).to_le_bytes());
                emit(&resp);
            }

            CMD_SIGN_DATA => {
                let mut ok = false;
                if payload.len() > 1 && SIGN_ACTIVE.load(Ordering::Relaxed) {
                    ok = SIGN_BUFFER.lock(|b| {
                        b.borrow_mut().extend_from_slice(&payload[1..]).is_ok()
                    });
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SIGN_FINISH => {
                let mut signature = None;
                if SIGN_ACTIVE.swap(false, Ordering::Relaxed) {
                    SIGN_BUFFER.lock(|b| {
                        let data = b.borrow();
                        if !data.is_empty() {
                            if let Some(id) = crate::meshcore::identity::LocalIdentity::from_private(
                                self.callsign.as_str(),
                                &self.private_key,
                            ) {
                                signature = Some(id.sign(&data));
                            }
                        }
                    });
                    SIGN_BUFFER.lock(|b| b.borrow_mut().clear());
                }
                match signature {
                    Some(sig) => {
                        let mut resp = Vec::<u8, 65>::new();
                        let _ = resp.push(RESP_CODE_SIGNATURE);
                        let _ = resp.extend_from_slice(&sig);
                        emit(&resp);
                    }
                    None => {
                        let mut resp = Vec::<u8, 4>::new();
                        let _ = resp.push(RESP_CODE_ERR);
                        emit(&resp);
                    }
                }
            }

            CMD_EXPORT_CONTACT => {
                // Export self as a signed advert packet. Exporting a stored
                // contact would require retaining its original advert blob.
                if payload.len() < 33 {
                    let id = crate::meshcore::identity::LocalIdentity::from_private(
                        self.callsign.as_str(),
                        &self.private_key,
                    );
                    match id {
                        Some(id) => {
                            let ts = crate::companion::usb::get_device_time();
                            let lat = (self.latitude * 1_000_000.0) as i32;
                            let lon = (self.longitude * 1_000_000.0) as i32;
                            let mut adv = AdvertPacket::new(
                                *id.pub_key(),
                                ts,
                                crate::meshcore::packets::advert::ADV_TYPE_CHAT,
                                lat,
                                lon,
                                self.callsign.as_str(),
                            );
                            adv.sign(&id);
                            if let Ok(pkt) =
                                adv.to_packet(crate::meshcore::packet::ROUTE_TYPE_FLOOD)
                            {
                                let mut raw = [0u8; 184];
                                if let Ok(n) = pkt.write_to(&mut raw) {
                                    let mut resp = Vec::<u8, 185>::new();
                                    let _ = resp.push(RESP_CODE_EXPORT_CONTACT);
                                    let _ = resp.extend_from_slice(&raw[..n]);
                                    emit(&resp);
                                }
                            }
                        }
                        None => {
                            let mut resp = Vec::<u8, 4>::new();
                            let _ = resp.push(RESP_CODE_ERR);
                            emit(&resp);
                        }
                    }
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_ERR);
                    emit(&resp);
                }
            }

            CMD_IMPORT_CONTACT => {
                let mut ok = false;
                if payload.len() > 1 {
                    if let Ok(pkt) = Packet::read_from(&payload[1..]) {
                        if pkt.payload_type() == PAYLOAD_TYPE_ADVERT {
                            if let Ok(adv) = AdvertPacket::decode(&pkt.payload[..pkt.payload_len]) {
                                if adv.verify_signature() {
                                    let mut callsign = String::<32>::new();
                                    let _ = callsign.push_str(adv.name.as_str());
                                    let mut out_path = [0u8; 64];
                                    let plen = pkt.path_byte_len();
                                    out_path[..plen].copy_from_slice(&pkt.path[..plen]);
                                    let node_id = [
                                        adv.pub_key[0],
                                        adv.pub_key[1],
                                        adv.pub_key[2],
                                        adv.pub_key[3],
                                    ];
                                    ok = crate::companion::ble::CONTACT_OP_CHANNEL
                                        .try_send(crate::companion::ble::ContactOperation::AddOrUpdate {
                                            pub_key: adv.pub_key,
                                            callsign,
                                            node_id,
                                            last_seen_epoch: crate::companion::usb::get_device_time(),
                                            latitude: adv.latitude,
                                            longitude: adv.longitude,
                                            flags: adv.flags,
                                            contact_type: CompanionNode::get_contact_type_from_flags(adv.flags),
                                            out_path,
                                            out_path_len: pkt.path_len,
                                            last_advert_ts: adv.timestamp,
                                        })
                                        .is_ok();
                                }
                            }
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_OTHER_PARAMS => {
                let mut ok = false;
                if payload.len() >= 2 {
                    let manual = payload[1];
                    let auto_changed = manual != self.manual_add_contacts;
                    self.manual_add_contacts = manual;
                    crate::companion::usb::AUTOADD_SETTINGS.lock(|a| {
                        a.borrow_mut().manual_add_contacts = manual;
                    });
                    if auto_changed {
                        crate::storage::request_save_config();
                    }
                    if payload.len() >= 3 {
                        self.telemetry_mode_base = payload[2] & 0x03;
                        self.telemetry_mode_loc = (payload[2] >> 2) & 0x03;
                        self.telemetry_mode_env = (payload[2] >> 4) & 0x03;
                        if payload.len() >= 4 {
                            self.advert_loc_policy = payload[3];
                        }
                        if payload.len() >= 5 {
                            self.multi_acks = payload[4];
                        }
                    }
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_DEVICE_PIN => {
                let mut ok = false;
                if payload.len() >= 5 {
                    let pin = u32::from_le_bytes([
                        payload[1], payload[2], payload[3], payload[4],
                    ]);
                    if pin == 0 || (100_000..=999_999).contains(&pin) {
                        crate::companion::ble::BLE_PIN.store(pin, Ordering::Relaxed);
                        ok = true;
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_CUSTOM_VARS => {
                // No device-specific custom variables are exposed yet.
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_CUSTOM_VARS);
                emit(&resp);
            }

            CMD_SET_CUSTOM_VAR => {
                let ok = payload.len() >= 4;
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_ALLOWED_REPEAT_FREQ => {
                // Allowed client-repeat (off-grid) bands (kHz, lower/upper).
                let mut resp = Vec::<u8, 32>::new();
                let _ = resp.push(RESP_CODE_ALLOWED_REPEAT_FREQ);
                for (lo, hi) in crate::companion::usb::OFFGRID_FREQ_RANGES {
                    let _ = resp.extend_from_slice(&lo.to_le_bytes());
                    let _ = resp.extend_from_slice(&hi.to_le_bytes());
                }
                emit(&resp);
            }

            CMD_SET_FLOOD_SCOPE_KEY => {
                let mut ok = false;
                if payload.len() >= 2 && payload[1] == 0 {
                    SCOPE.lock(|s| {
                        let mut scope = s.borrow_mut();
                        if payload.len() >= 18 {
                            let mut key = [0u8; 16];
                            key.copy_from_slice(&payload[2..18]);
                            scope.send_key = Some(key);
                        } else {
                            scope.send_key = None;
                        }
                        scope.force_unscoped = false;
                    });
                    ok = true;
                } else if payload.len() == 2 && payload[1] == 1 {
                    SCOPE.lock(|s| {
                        let mut scope = s.borrow_mut();
                        scope.send_key = None;
                        scope.force_unscoped = true;
                    });
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SET_DEFAULT_FLOOD_SCOPE => {
                let mut ok = false;
                if payload.len() >= 48 {
                    let name = &payload[1..32];
                    let n = name.iter().position(|&b| b == 0).unwrap_or(31);
                    if n > 0 && n < 31 {
                        SCOPE.lock(|s| {
                            let mut scope = s.borrow_mut();
                            scope.default_name = [0u8; 31];
                            scope.default_name[..n].copy_from_slice(&name[..n]);
                            scope.default_key.copy_from_slice(&payload[32..48]);
                        });
                        ok = true;
                    }
                } else if payload.len() == 1 {
                    SCOPE.lock(|s| {
                        let mut scope = s.borrow_mut();
                        scope.default_name = [0u8; 31];
                        scope.default_key = [0u8; 16];
                    });
                    ok = true;
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_GET_DEFAULT_FLOOD_SCOPE => {
                let (name, key) = SCOPE.lock(|s| {
                    let scope = s.borrow();
                    (scope.default_name, scope.default_key)
                });
                if name[0] != 0 {
                    let mut resp = Vec::<u8, 48>::new();
                    let _ = resp.push(RESP_CODE_DEFAULT_FLOOD_SCOPE);
                    let _ = resp.extend_from_slice(&name);
                    let _ = resp.extend_from_slice(&key);
                    emit(&resp);
                } else {
                    let mut resp = Vec::<u8, 4>::new();
                    let _ = resp.push(RESP_CODE_DEFAULT_FLOOD_SCOPE);
                    emit(&resp);
                }
            }

            CMD_SEND_CONTROL_DATA => {
                let mut ok = false;
                if payload.len() >= 2 && (payload[1] & 0x80) != 0 {
                    let mut data = Vec::<u8, 183>::new();
                    if data.extend_from_slice(&payload[1..]).is_ok() {
                        ok = crate::companion::usb::USB_COMMAND_CHANNEL
                            .try_send(crate::companion::usb::UsbCommand::SendControlData { data })
                            .is_ok();
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SEND_RAW_DATA => {
                let mut ok = false;
                if payload.len() >= 6 {
                    let path_len = payload[1];
                    if Packet::is_valid_path_len(path_len) {
                        let n = (path_len & 63) as usize
                            * (((path_len >> 6) as usize) + 1);
                        if payload.len() >= 2 + n + 4 {
                            let mut path = [0u8; 64];
                            path[..n].copy_from_slice(&payload[2..2 + n]);
                            let mut data = Vec::<u8, 184>::new();
                            if data.extend_from_slice(&payload[2 + n..]).is_ok() {
                                ok = crate::companion::usb::USB_COMMAND_CHANNEL
                                    .try_send(
                                        crate::companion::usb::UsbCommand::SendRawData {
                                            path_len,
                                            path,
                                            payload: data,
                                        },
                                    )
                                    .is_ok();
                            }
                        }
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SEND_RAW_PACKET => {
                let mut ok = false;
                if payload.len() >= 4 && Packet::read_from(&payload[2..]).is_ok() {
                    let mut raw = Vec::<u8, 255>::new();
                    if raw.extend_from_slice(&payload[2..]).is_ok() {
                        ok = crate::companion::usb::USB_COMMAND_CHANNEL
                            .try_send(crate::companion::usb::UsbCommand::SendRawPacket { raw })
                            .is_ok();
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            CMD_SEND_TRACE_PATH => {
                let mut ok = false;
                if payload.len() > 10 {
                    let tag = u32::from_le_bytes([
                        payload[1], payload[2], payload[3], payload[4],
                    ]);
                    let auth = u32::from_le_bytes([
                        payload[5], payload[6], payload[7], payload[8],
                    ]);
                    let flags = payload[9];
                    let path_len = payload.len() - 10;
                    let hash_bytes = 1usize << (flags & 0x03);
                    // Our Packet path encoding supports 1- and 2-byte hashes.
                    if hash_bytes <= 2 && path_len <= 64 && path_len % hash_bytes == 0 {
                        let mut path = [0u8; 64];
                        path[..path_len].copy_from_slice(&payload[10..]);
                        ok = crate::companion::usb::USB_COMMAND_CHANNEL
                            .try_send(crate::companion::usb::UsbCommand::SendTracePath {
                                tag,
                                auth,
                                flags,
                                path_len: path_len as u8,
                                path,
                            })
                            .is_ok();
                    }
                }
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(if ok { RESP_CODE_OK } else { RESP_CODE_ERR });
                emit(&resp);
            }

            _ => {
                // Unsupported command: report an error rather than a misleading OK.
                let mut resp = Vec::<u8, 4>::new();
                let _ = resp.push(RESP_CODE_ERR);
                emit(&resp);
            }
        }
    }
}


/// Number of stored contacts.
pub fn contacts_count() -> u32 {
    crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| c.borrow().len()) as u32
}

/// Clone one contact out of the shared list (a small, fixed-size copy).
pub fn contact_at(idx: usize) -> Option<crate::meshcore::mesh::DiscoveredContact> {
    crate::companion::usb::DISCOVERED_CONTACTS.lock(|c| c.borrow().get(idx).cloned())
}

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
pub(crate) fn serialize_contact(c: &crate::meshcore::mesh::DiscoveredContact) -> Vec<u8, 160> {
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
