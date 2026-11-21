//! Application task wiring: inbound dispatch, companion commands, input
//! actions, display refresh and battery sampling.
//!
//! Board-specific hardware is reached through [`AppBoard`]; the mesh engine,
//! companion node and radio are supplied by the board's startup code.

use defmt::info;
use embassy_futures::select::{select4, Either4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use ferrix_core::airtime::estimate_airtime_ms;
use ferrix_core::command::{push_companion, UsbCommand, USB_COMMAND_CHANNEL};
use ferrix_core::companion::protocol::{self, CompanionNode};
use ferrix_core::engine::{InboundAction, MeshEngine};
use ferrix_core::state::*;
use ferrix_core::storage;
use ferrix_core::types::*;
use ferrix_hal::radio::LoraRadio;
use heapless::Vec;
use meshcore_wire::packet::Packet;

/// Semantic events from the input task.
pub static INPUT_EVENT_CHANNEL: Channel<CriticalSectionRawMutex, InputEvent, 4> = Channel::new();

/// Semantic user input actions across any hardware platform.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InputEvent {
    NextPage,
    PrevPage,
    ActionPrimary,
    ActionSecondary,
    Up,
    Down,
    PowerToggle,
}

/// Semantic sound requests for boards with a buzzer.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UiSound {
    /// Page navigation or light UI feedback.
    Click,
    /// A confirmed page action.
    Action,
    /// A new message arrived.
    Notify,
}

/// Sort metric for the radar page.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum RadarSort {
    #[default]
    Age,
    Signal,
}

/// Small rolling RSSI/SNR history for the signal page.
pub struct SignalHistory {
    rssi: [i8; 24],
    snr: [i8; 24],
    len: usize,
    next: usize,
}

impl Default for SignalHistory {
    fn default() -> Self {
        Self {
            rssi: [0; 24],
            snr: [0; 24],
            len: 0,
            next: 0,
        }
    }
}

impl SignalHistory {
    pub fn push(&mut self, rssi: i16, snr: i8) {
        let rssi = rssi.clamp(i8::MIN as i16, i8::MAX as i16) as i8;
        self.rssi[self.next] = rssi;
        self.snr[self.next] = snr;
        self.next = (self.next + 1) % 24;
        if self.len < 24 {
            self.len += 1;
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.next = 0;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn at(&self, index: usize) -> Option<(i8, i8)> {
        if index >= self.len {
            return None;
        }
        let start = (self.next + 24 - self.len) % 24;
        let i = (start + index) % 24;
        Some((self.rssi[i], self.snr[i]))
    }

    pub fn last(&self) -> Option<(i8, i8)> {
        if self.len == 0 {
            None
        } else {
            Some((self.rssi[(self.next + 23) % 24], self.snr[(self.next + 23) % 24]))
        }
    }
}

/// UI-only state shared by the application and the board renderer.
pub struct UiState {
    pub inbox_cursor: usize,
    pub radar_sort: RadarSort,
    /// Whether the radar page labels the nearest contacts.
    pub radar_labels: bool,
    pub signal: SignalHistory,
}

impl Default for UiState {
    fn default() -> Self {
        Self {
            inbox_cursor: 0,
            radar_sort: RadarSort::default(),
            radar_labels: true,
            signal: SignalHistory::default(),
        }
    }
}

/// Board services used by the application loop.
#[allow(async_fn_in_trait)]
pub trait AppBoard {
    fn page_count(&self) -> u8;
    fn has_display(&self) -> bool;
    fn has_battery_adc(&self) -> bool;
    fn gps_enabled(&self) -> bool;
    fn set_gps_enabled(&mut self, on: bool);
    fn set_led(&mut self, on: bool);
    fn advert_feedback(&mut self);
    /// Queue or perform a non-blocking display refresh. Returns `true` if the
    /// frame was accepted (or the board has no display), or `false` if the
    /// display task is still busy flushing the previous frame.
    fn render(&mut self, node: &CompanionNode<'_>, ui: &UiState, full: bool) -> bool;
    fn input_wake(&mut self);
    fn tick(&mut self, node: &CompanionNode<'_>, ui: &UiState, tick: u32);
    async fn sample_battery(&mut self) -> Option<u16>;
    fn set_leds_disabled(&mut self, disabled: bool);
    /// Play a UI sound without blocking. Boards without a buzzer keep the default no-op.
    fn beep(&mut self, _sound: UiSound) {}
    /// Prepare the board (e.g., show OTA notice and sleep display) before BLE OTA DFU reboot.
    async fn prepare_ota_dfu(&mut self, _node: &CompanionNode<'_>, _ui: &UiState) {}
    /// Reboot into BLE OTA DFU bootloader mode (`GPREGRET = 0xA8`).
    fn reboot_to_ota_dfu(&mut self) -> ! {
        loop {
            core::hint::spin_loop();
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct PendingMessageAck {
    pub crc: u32,
    pub dest_pub: [u8; 32],
    pub send_instant_ms: u64,
}

static PENDING_MESSAGE_ACKS: Mutex<
    CriticalSectionRawMutex,
    core::cell::RefCell<Vec<PendingMessageAck, 16>>,
> = Mutex::new(core::cell::RefCell::new(Vec::new()));

fn push_path_updated(pub_key: &[u8; 32]) {
    let mut frame = Vec::<u8, 256>::new();
    let _ = frame.push(protocol::PUSH_CODE_PATH_UPDATED);
    let _ = frame.extend_from_slice(pub_key);
    push_companion(&frame);
}

fn track_pending_ack(crc: u32, dest_pub: [u8; 32]) {
    let now = Instant::now().as_millis();
    PENDING_MESSAGE_ACKS.lock(|a| {
        let mut list = a.borrow_mut();
        if list.is_full() {
            list.remove(0);
        }
        let _ = list.push(PendingMessageAck {
            crc,
            dest_pub,
            send_instant_ms: now,
        });
    });
}

fn handle_direct_message_confirmed(crc: u32) -> Option<[u8; 32]> {
    let now = Instant::now().as_millis();
    let matched = PENDING_MESSAGE_ACKS.lock(|a| {
        let mut list = a.borrow_mut();
        list.iter().position(|p| p.crc == crc).map(|pos| {
            let item = list.remove(pos);
            (item.dest_pub, now.saturating_sub(item.send_instant_ms) as u32)
        })
    });
    let (dest_pub, rtt_ms) = matched?;
    info!(
        "MeshCore Direct Message Delivery ACK Confirmed! (crc {:08x}, RTT: {} ms)",
        crc, rtt_ms
    );
    let mut frame = Vec::<u8, 256>::new();
    let _ = frame.push(protocol::PUSH_CODE_SEND_CONFIRMED);
    let _ = frame.extend_from_slice(&crc.to_le_bytes());
    let _ = frame.extend_from_slice(&rtt_ms.to_le_bytes());
    push_companion(&frame);
    Some(dest_pub)
}

/// Hand queued packets to the radio task.
pub async fn flush_tx_queue<R: LoraRadio>(mesh: &mut MeshEngine<'_>, radio: &mut R) {
    while let Some(queued) = mesh.tx_queue.front() {
        let mut buf = [0u8; 256];
        let Ok(len) = queued.packet.write_to(&mut buf) else {
            let _ = mesh.tx_queue.pop();
            continue;
        };
        if radio
            .transmit_lane_at(
                &buf[..len],
                queued.lane,
                queued.queued_at_ms,
                queued.due_at_ms,
                queued.packet.calculate_packet_hash(),
            )
            .await
            .is_err()
        {
            break;
        }
        let _ = mesh.tx_queue.pop();
    }
}

fn send_advert<B: AppBoard>(
    board: &mut B,
    mesh: &mut MeshEngine<'_>,
    node: &mut CompanionNode<'_>,
    flood: bool,
) {
    node.tx_beacons = node.tx_beacons.saturating_add(1);
    let (lat, lon) = protocol::advert_location(node.latitude, node.longitude);
    let _ = mesh.queue_advert(node.epoch_time as u32, lat, lon, flood);
    board.advert_feedback();
}

/// Run the application loop until the executor stops.
fn sync_gps(node: &mut CompanionNode<'_>) {
    let gps = gps_state();
    node.gps_fix = gps.fix;
    if gps.fix && advert_lat_lon_e6() == (0, 0) {
        node.latitude = gps.latitude;
        node.longitude = gps.longitude;
    }
}

/// Minimum interval between automatic EPD refreshes for continuous telemetry
/// pages (`RADAR`, `SIGNAL`, and `POSITION` coordinate drift) when
/// `display_refresh_interval_secs() == 0`.
const LIVE_PAGE_REFRESH_COOLDOWN_SECS: u64 = 30;

/// Snapshot of displayed state used to decide whether an active page has a
/// meaningful update when `display_refresh_interval_secs() == 0` (excluding
/// 1-minute wall-clock and uptime ticks).
#[derive(Clone, Copy, PartialEq, Eq)]
struct UiSnapshot {
    page: u8,
    battery_pct: u8,
    time_synced: bool,
    msg_len: usize,
    msg_epoch: u32,
    ble_connected: bool,
    forwarding: bool,
    advert_epoch: u32,
    radio_freq_hz: u32,
    radio_bw_khz: u32,
    radio_sf: u8,
    radio_cr: u8,
    radio_tx_dbm: i8,
    radio_rx_duty: bool,
    radio_af_tenths: u8,
    client_repeat: bool,
    path_hash_mode: u8,
    rx_packets: u32,
    tx_beacons: u32,
    gps_enabled: bool,
    gps_fix: bool,
    gps_lat_e4: i32,
    gps_lon_e4: i32,
    advert_loc_policy: u8,
    leds_disabled: bool,
    buzzer_enabled: bool,
}

impl UiSnapshot {
    fn capture<B: AppBoard>(
        board: &B,
        node: &CompanionNode<'_>,
        msg_epoch: u32,
        advert_epoch: u32,
    ) -> Self {
        let msg_len = MESSAGE_LOG.lock(|m| m.borrow().len());
        let (
            radio_freq_hz,
            radio_bw_khz,
            radio_sf,
            radio_cr,
            radio_tx_dbm,
            radio_rx_duty,
            radio_af_tenths,
            client_repeat,
            path_hash_mode,
        ) = RADIO_SETTINGS.lock(|r| {
            let s = r.borrow();
            (
                s.frequency_hz,
                s.bw_khz,
                s.sf,
                s.cr,
                s.tx_power_dbm,
                s.rx_duty_cycle,
                s.airtime_factor_tenths,
                s.client_repeat,
                s.path_hash_mode,
            )
        });
        let (rx_packets, tx_beacons) = RADIO_TELEMETRY.lock(|t| {
            let tel = t.borrow();
            (tel.rx_packets, tel.tx_beacons.max(node.tx_beacons))
        });

        Self {
            page: node.current_page,
            battery_pct: node.battery_percentage,
            time_synced: node.epoch_time > 1_700_000_000,
            msg_len,
            msg_epoch,
            ble_connected: BLE_CONNECTED.load(core::sync::atomic::Ordering::Relaxed),
            forwarding: forwarding_enabled(),
            advert_epoch,
            radio_freq_hz,
            radio_bw_khz,
            radio_sf,
            radio_cr,
            radio_tx_dbm,
            radio_rx_duty,
            radio_af_tenths,
            client_repeat,
            path_hash_mode,
            rx_packets,
            tx_beacons,
            gps_enabled: board.gps_enabled(),
            gps_fix: node.gps_fix,
            gps_lat_e4: (node.latitude * 10_000.0) as i32,
            gps_lon_e4: (node.longitude * 10_000.0) as i32,
            advert_loc_policy: prefs().advert_loc_policy,
            leds_disabled: leds_disabled(),
            buzzer_enabled: buzzer_enabled(),
        }
    }

    fn message_changed(&self, prev: &Self) -> bool {
        self.msg_len != prev.msg_len || self.msg_epoch != prev.msg_epoch
    }

    fn immediate_change_on_visible_page(&self, prev: &Self) -> bool {
        if self.page != prev.page
            || self.battery_pct != prev.battery_pct
            || self.time_synced != prev.time_synced
        {
            return true;
        }
        match self.page {
            // Page 0: MESH
            0 => {
                self.forwarding != prev.forwarding
                    || self.message_changed(prev)
                    || self.ble_connected != prev.ble_connected
            }
            // Page 1: INBOX
            1 => self.message_changed(prev),
            // Page 3: RADIO
            3 => {
                self.radio_freq_hz != prev.radio_freq_hz
                    || self.radio_bw_khz != prev.radio_bw_khz
                    || self.radio_sf != prev.radio_sf
                    || self.radio_cr != prev.radio_cr
                    || self.radio_tx_dbm != prev.radio_tx_dbm
                    || self.radio_rx_duty != prev.radio_rx_duty
                    || self.radio_af_tenths != prev.radio_af_tenths
            }
            // Page 5: POSITION (power/fix/policy transitions stay immediate; lat/lon drift is rate-limited)
            5 => {
                self.gps_enabled != prev.gps_enabled
                    || self.gps_fix != prev.gps_fix
                    || self.advert_loc_policy != prev.advert_loc_policy
            }
            // Page 6: OFF-GRID
            6 => self.client_repeat != prev.client_repeat || self.radio_freq_hz != prev.radio_freq_hz,
            // Page 7: SETTINGS
            7 => self.leds_disabled != prev.leds_disabled || self.buzzer_enabled != prev.buzzer_enabled,
            // Page 8: SYSTEM
            8 => self.radio_rx_duty != prev.radio_rx_duty || self.path_hash_mode != prev.path_hash_mode,
            _ => false,
        }
    }

    fn throttled_change_on_visible_page(&self, prev: &Self) -> bool {
        match self.page {
            // Page 2: RADAR
            2 => self.advert_epoch != prev.advert_epoch,
            // Page 4: SIGNAL
            4 => self.rx_packets != prev.rx_packets || self.tx_beacons != prev.tx_beacons,
            // Page 5: POSITION coordinate drift
            5 => self.gps_lat_e4 != prev.gps_lat_e4 || self.gps_lon_e4 != prev.gps_lon_e4,
            _ => false,
        }
    }

    fn differs_on_visible_page(&self, prev: &Self, cooldown_elapsed: bool) -> bool {
        self.immediate_change_on_visible_page(prev)
            || (cooldown_elapsed && self.throttled_change_on_visible_page(prev))
    }
}

#[inline(never)]
fn push_inbound_text<B: AppBoard>(
    board: &mut B,
    message: ReceivedMessage,
    msg_epoch: &mut u32,
) {
    let dropped = MESSAGE_LOG.lock(|m| {
        let mut list = m.borrow_mut();
        let dropped = list.is_full();
        if dropped {
            list.remove(0);
        }
        let _ = list.push(InboxItem::Text(message.clone()));
        dropped
    });
    if dropped {
        storage::messages::queue_consume_now();
    }
    let _ = storage::messages::try_queue_append(message);
    *msg_epoch = msg_epoch.wrapping_add(1);
    board.beep(UiSound::Notify);

    // Prompt companion app with PUSH_CODE_MSG_WAITING (0x83)
    let mut msg_wait = heapless::Vec::<u8, 256>::new();
    let _ = msg_wait.push(protocol::PUSH_CODE_MSG_WAITING);
    push_companion(&msg_wait);
}

#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn handle_inbound_rx<B: AppBoard>(
    board: &mut B,
    mesh: &mut MeshEngine<'_>,
    node: &mut CompanionNode<'_>,
    ui: &mut UiState,
    msg_epoch: &mut u32,
    advert_epoch: &mut u32,
    raw: &[u8],
    rssi: i16,
    snr: i8,
) {
    let len = raw.len();
    node.rx_packets = node.rx_packets.saturating_add(1);
    node.last_rssi = rssi;
    node.last_snr = snr;
    ui.signal.push(rssi, snr);

    // Stream raw packet to companion app via PUSH_CODE_LOG_RX_DATA (0x88)
    // Layout: [0x88, (snr * 4) as i8, rssi as i8, raw_lora_packet_bytes...]
    let mut log_rx_push = heapless::Vec::<u8, 256>::new();
    let _ = log_rx_push.push(protocol::PUSH_CODE_LOG_RX_DATA);
    let _ = log_rx_push.push(snr.saturating_mul(4) as u8);
    let _ = log_rx_push.push(rssi as i8 as u8);
    let copy_lora_len = len.min(256 - 3);
    let _ = log_rx_push.extend_from_slice(&raw[..copy_lora_len]);
    push_companion(&log_rx_push);

    // Synchronize real radio telemetry to shared USB CLI status
    let rx_air = RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        estimate_airtime_ms(len, s.sf, s.bw_khz, s.cr, if s.sf <= 8 { 32 } else { 16 })
    });
    RADIO_TELEMETRY.lock(|t| {
        let mut tel = t.borrow_mut();
        tel.rx_packets = tel.rx_packets.saturating_add(1);
        tel.rx_air_ms = tel.rx_air_ms.saturating_add(rx_air);
        tel.last_rssi = rssi;
        tel.last_snr = snr;
        tel.has_rx = true;
    });

    // Deserialize and dispatch through MeshCore Network Engine
    if let Ok(pkt) = Packet::read_from(raw) {
        let advert_path_len = pkt.path_len;
        let advert_path = pkt.path;
        // Retain the signed payload so the contact can be exported.
        let advert_payload = if pkt.payload_type() == meshcore_wire::packet::PAYLOAD_TYPE_ADVERT {
            let mut v = heapless::Vec::<u8, 184>::new();
            let _ = v.extend_from_slice(&pkt.payload[..pkt.payload_len]);
            Some(v)
        } else {
            None
        };
        let action = mesh.handle_inbound_packet(pkt, snr);
        dispatch_inbound_action(
            board,
            mesh,
            node,
            msg_epoch,
            advert_epoch,
            action,
            advert_path_len,
            &advert_path,
            advert_payload.as_deref(),
            rssi,
            snr,
        );
    } else {
        info!("Received non-MeshCore or legacy beacon frame");
    }
}

#[inline(never)]
#[allow(clippy::too_many_arguments)]
fn dispatch_inbound_action<B: AppBoard>(
    board: &mut B,
    mesh: &mut MeshEngine<'_>,
    node: &mut CompanionNode<'_>,
    msg_epoch: &mut u32,
    advert_epoch: &mut u32,
    action: InboundAction,
    advert_path_len: u8,
    advert_path: &[u8; 64],
    advert_payload: Option<&[u8]>,
    rssi: i16,
    snr: i8,
) {
        match action {
            InboundAction::ReceivedAdvert(adv) => {
                if protocol::process_inbound_advert(
                    node,
                    adv,
                    advert_path_len,
                    advert_path,
                    advert_payload,
                    rssi,
                    snr,
                ) {
                    *advert_epoch = advert_epoch.wrapping_add(1);
                }
            }
            InboundAction::ReceivedGroupText {
                channel_idx,
                path_len,
                packet: grp,
            } => {
                info!(
                    "MeshCore Group Text [ch {}]: <{}> {}",
                    channel_idx,
                    grp.sender.as_str(),
                    grp.message.as_str()
                );
                let mut s_sender = heapless::String::new();
                let _ = s_sender.push_str(grp.sender.as_str());
                let mut s_text = heapless::String::new();
                let _ = s_text.push_str(grp.message.as_str());
                let message = ReceivedMessage {
                    channel_idx,
                    path_len,
                    sender: s_sender,
                    sender_pub: [0u8; 32],
                    text: s_text,
                    timestamp: grp.timestamp,
                    rssi,
                    snr,
                    is_direct: false,
                    txt_type: grp.txt_type,
                    signed_prefix: [0u8; 4],
                };
                push_inbound_text(board, message, msg_epoch);
            }
            InboundAction::ReceivedGroupData {
                channel_idx,
                path_len,
                packet,
            } => {
                let item = ReceivedChannelData {
                    channel_idx,
                    path_len,
                    snr,
                    data_type: packet.data_type,
                    data: packet.data.clone(),
                };
                let dropped = MESSAGE_LOG.lock(|m| {
                    let mut list = m.borrow_mut();
                    let dropped = list.is_full();
                    if dropped {
                        list.remove(0);
                    }
                    let _ = list.push(InboxItem::ChannelData(item.clone()));
                    dropped
                });
                if dropped {
                    storage::messages::queue_consume_now();
                }
                let _ = storage::messages::try_queue_append_channel_data(item);
                *msg_epoch = msg_epoch.wrapping_add(1);

                let mut msg_wait = heapless::Vec::<u8, 256>::new();
                let _ = msg_wait.push(protocol::PUSH_CODE_MSG_WAITING);
                push_companion(&msg_wait);
            }
            InboundAction::ReceivedDirectText {
                msg,
                sender_pub,
                path_len,
            } => {
                info!(
                    "MeshCore Direct Msg from {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}: {}",
                    sender_pub[0],
                    sender_pub[1],
                    sender_pub[2],
                    sender_pub[3],
                    sender_pub[4],
                    sender_pub[5],
                    msg.text.as_str()
                );
                // Signed room posts advance the room-sync watermark so a
                // later login does not replay the whole backlog.
                if msg.txt_type == meshcore_wire::packets::txtmsg::TXT_TYPE_SIGNED_PLAIN {
                    let changed = DISCOVERED_CONTACTS.lock(|c| {
                        if let Some(ct) = c
                            .borrow_mut()
                            .iter_mut()
                            .find(|ct| ct.pub_key == sender_pub)
                        {
                            if msg.timestamp > ct.sync_since {
                                ct.sync_since = msg.timestamp;
                                true
                            } else {
                                false
                            }
                        } else {
                            false
                        }
                    });
                    if changed {
                        storage::request_contacts();
                    }
                }
                let mut s_sender = heapless::String::new();
                use core::fmt::Write;
                let _ = write!(
                    s_sender,
                    "{:02x}{:02x}{:02x}{:02x}{:02x}{:02x}",
                    sender_pub[0],
                    sender_pub[1],
                    sender_pub[2],
                    sender_pub[3],
                    sender_pub[4],
                    sender_pub[5]
                );
                let mut s_text = heapless::String::new();
                let _ = s_text.push_str(msg.text.as_str());
                let message = ReceivedMessage {
                    channel_idx: 0,
                    path_len,
                    sender: s_sender,
                    sender_pub,
                    text: s_text,
                    timestamp: msg.timestamp,
                    rssi,
                    snr,
                    is_direct: true,
                    txt_type: msg.txt_type,
                    signed_prefix: msg.signed_prefix,
                };
                push_inbound_text(board, message, msg_epoch);
            }
            InboundAction::ReceivedRepeaterEcho { pkt_hash, hops } => {
                info!(
                    "MeshCore Message echoed by repeater! ({} hops, pkt_hash: {:?})",
                    hops, pkt_hash
                );
            }
            InboundAction::ReceivedAck {
                ack,
                is_flood,
                arrival_path_len,
                arrival_path,
            } => {
                info!("MeshCore ACK received (crc {:08x})", ack.crc);
                if !protocol::handle_keepalive_ack(ack.crc) {
                    if let Some(dest_pub) = handle_direct_message_confirmed(ack.crc) {
                        if is_flood {
                            let peer = DISCOVERED_CONTACTS.lock(|c| {
                                c.borrow()
                                    .iter()
                                    .find(|ct| ct.pub_key == dest_pub)
                                    .map(|ct| (ct.out_path_len, ct.out_path))
                            });
                            if let Some((out_path_len, out_path)) = peer {
                                if out_path_len != 0xFF && Packet::is_valid_path_len(out_path_len) {
                                    let _ = mesh.queue_path_return_only(
                                        &dest_pub,
                                        out_path_len,
                                        &out_path,
                                        arrival_path_len,
                                        &arrival_path,
                                        3000,
                                    );
                                }
                            }
                        }
                    }
                }
            }
            InboundAction::ReceivedResponse { plain, sender_pub } => {
                if !protocol::handle_response(&plain, &sender_pub) {
                    info!(
                        "MeshCore RESPONSE from {:02x}{:02x} (no pending request)",
                        sender_pub[0], sender_pub[1]
                    );
                }
            }
            InboundAction::ReceivedRequest {
                plain,
                sender_pub,
                is_flood,
                arrival_path_len,
                arrival_path,
            } => {
                if let Some(reply) = protocol::handle_contact_request(node, &plain, &sender_pub) {
                    info!(
                        "MeshCore REQ from {:02x}{:02x}: queuing {}B telemetry reply (flood={})",
                        sender_pub[0],
                        sender_pub[1],
                        reply.len(),
                        is_flood
                    );
                    let _ = mesh.queue_request_reply(
                        &sender_pub,
                        is_flood,
                        arrival_path_len,
                        &arrival_path,
                        &reply,
                    );
                } else {
                    info!(
                        "MeshCore REQ from {:02x}{:02x} ({} bytes, denied/ignored)",
                        sender_pub[0],
                        sender_pub[1],
                        plain.len()
                    );
                }
            }
            InboundAction::ReceivedControl { data, path_len } => {
                info!("MeshCore CONTROL data ({} bytes)", data.len());
                let mut push = heapless::Vec::<u8, 256>::new();
                let _ = push.push(protocol::PUSH_CODE_CONTROL_DATA);
                let _ = push.push((snr.saturating_mul(4)) as u8);
                let _ = push.push(rssi as i8 as u8);
                let _ = push.push(path_len);
                let _ = push.extend_from_slice(&data);
                if push.len() <= ferrix_core::companion::framing::MAX_FRAME_PAYLOAD_SIZE {
                    push_companion(&push);
                }
            }
            InboundAction::ReceivedTrace {
                tag,
                auth,
                flags,
                hashes,
                snrs,
                final_snr,
            } => {
                info!(
                    "MeshCore TRACE {:08x}: {} hash bytes, {} hops",
                    tag,
                    hashes.len(),
                    snrs.len()
                );
                protocol::note_trace_rx(tag, flags, hashes.len(), snrs.len());
                let mut push = heapless::Vec::<u8, 256>::new();
                let _ = push.push(protocol::PUSH_CODE_TRACE_DATA);
                let _ = push.push(0);
                let _ = push.push(hashes.len() as u8);
                let _ = push.push(flags);
                let _ = push.extend_from_slice(&tag.to_le_bytes());
                let _ = push.extend_from_slice(&auth.to_le_bytes());
                let _ = push.extend_from_slice(&hashes);
                let _ = push.extend_from_slice(&snrs);
                let _ = push.push((final_snr.saturating_mul(4)) as u8);
                if push.len() <= ferrix_core::companion::framing::MAX_FRAME_PAYLOAD_SIZE {
                    push_companion(&push);
                }
            }
            InboundAction::ReceivedPath {
                packet: path,
                sender_pub,
                is_flood,
                arrival_path_len,
                arrival_path,
            } => {
                info!(
                    "MeshCore PATH from {:02x}{:02x}: {} hops, extra {}",
                    sender_pub[0],
                    sender_pub[1],
                    path.hop_count(),
                    path.extra_type
                );
                let path_updated = DISCOVERED_CONTACTS.lock(|c| {
                    if let Some(ct) = c
                        .borrow_mut()
                        .iter_mut()
                        .find(|ct| ct.pub_key == sender_pub)
                    {
                        ct.out_path = path.path;
                        ct.out_path_len = path.path_len;
                        true
                    } else {
                        false
                    }
                });
                if path_updated {
                    push_path_updated(&sender_pub);
                }
                storage::request_contacts();
                if is_flood && path_updated {
                    let _ = mesh.queue_path_return_only(
                        &sender_pub,
                        path.path_len,
                        &path.path,
                        arrival_path_len,
                        &arrival_path,
                        500,
                    );
                }
                // A PATH return can carry an ACK or a RESPONSE as its
                // extra. Flooded logins/requests use a RESPONSE extra.
                if path.extra_type & 0x0F == 3 && path.extra.len() >= 4 {
                    if let Ok(ack) =
                        meshcore_wire::packets::AckPacket::decode(path.extra.as_slice())
                    {
                        if !protocol::handle_keepalive_ack(ack.crc) {
                            handle_direct_message_confirmed(ack.crc);
                        }
                    }
                } else if path.extra_type & 0x0F == 1 && !path.extra.is_empty() {
                    let tag = u32::from_le_bytes([
                        path.extra[0],
                        path.extra[1],
                        path.extra[2],
                        path.extra[3],
                    ]);
                    if !protocol::handle_path_discovery(
                        tag,
                        &sender_pub,
                        path.path_len,
                        &path.path,
                        arrival_path_len,
                        &arrival_path,
                    ) && !protocol::handle_response(&path.extra, &sender_pub)
                    {
                        info!(
                            "MeshCore PATH RESPONSE extra unhandled ({} bytes)",
                            path.extra.len()
                        );
                    }
                }
            }
            InboundAction::ReceivedRawCustom { data } => {
                let mut push = heapless::Vec::<u8, 256>::new();
                let _ = push.push(protocol::PUSH_CODE_RAW_DATA);
                let _ = push.push((snr.saturating_mul(4)) as u8);
                let _ = push.push(rssi as i8 as u8);
                let _ = push.push(0xFF);
                let _ = push.extend_from_slice(&data);
                if push.len() <= 247 {
                    push_companion(&push);
                }
            }
            InboundAction::DropDuplicate => {
                info!("MeshCore: dropped duplicate packet");
            }
            InboundAction::DropHopLimitExceeded => {
                info!("MeshCore: dropped packet exceeding hop limit");
            }
            _ => {}
        }
}

#[inline(never)]
fn handle_usb_command(
    mesh: &mut MeshEngine<'_>,
    node: &mut CompanionNode<'_>,
    advert_epoch: &mut u32,
    cmd: UsbCommand,
) -> bool {
    let mut needs_render = false;
    match cmd {
        UsbCommand::SetName(new_name) => {
            info!("Updating node callsign to: {}", new_name.as_str());
            mesh.set_callsign(new_name.as_str());
            node.callsign = new_name;
            needs_render = true;
        }
        UsbCommand::SetPrivateKey(priv_key) => {
            info!("Updating node private & public keys");
            if mesh.set_private_key(&priv_key) {
                node.private_key = priv_key;
                node.public_key = *mesh.identity.pub_key();
                set_identity(*mesh.identity.pub_key(), priv_key);
                // Idempotent for the companion-app path (which already
                // persisted it) and required for any other caller.
                storage::set_custom_priv_key(priv_key);
            } else {
                info!("Rejected reserved private key");
            }
        }
        UsbCommand::SetTime(epoch) => {
            info!("Node epoch time updated to: {}", epoch);
            node.epoch_time = epoch;
            set_device_time(epoch as u32);
        }
        UsbCommand::TriggerAdvert { flood } => {
            info!("Queuing MeshCore Advert (flood: {})", flood);
            node.tx_beacons = node.tx_beacons.saturating_add(1);
            let time = get_device_time();
            let (lat, lon) = protocol::advert_location(node.latitude, node.longitude);
            let _ = mesh.queue_advert(time, lat, lon, flood);
        }
        UsbCommand::SetAdvertLatLon { lat, lon } => {
            set_advert_lat_lon_e6(lat, lon);
            node.latitude = lat as f32 / 1_000_000.0;
            node.longitude = lon as f32 / 1_000_000.0;
        }
        UsbCommand::SetAutoAddConfig { config, max_hops } => {
            node.autoadd_config = config;
            node.autoadd_max_hops = max_hops;
        }
        UsbCommand::SendGroupText {
            channel_idx,
            timestamp,
            text,
        } => {
            info!(
                "Queuing MeshCore Group Text on ch {}: {} (ts: {})",
                channel_idx,
                text.as_str(),
                timestamp
            );
            // Copy the channel secret out of the lock before computing channel_hash.
            // Running SHA-256 inside the lock callback can block the BLE SoftDevice.
            let secret = CHANNELS.lock(|chs| {
                let list = chs.borrow();
                if (channel_idx as usize) < list.len() {
                    list[channel_idx as usize].secret
                } else {
                    [0u8; 16]
                }
            });
            let ch_hash = meshcore_wire::crypto::channel_hash(&secret);
            node.tx_beacons = node.tx_beacons.saturating_add(1);
            let _ = mesh.queue_group_text(ch_hash, &secret, text.as_str(), timestamp);
        }
        UsbCommand::SendChannelData {
            channel_idx,
            path_len,
            path,
            data_type,
            data,
        } => {
            info!(
                "Queuing channel datagram on ch {} type {} ({} bytes)",
                channel_idx,
                data_type,
                data.len()
            );
            let secret = CHANNELS.lock(|chs| {
                let list = chs.borrow();
                if (channel_idx as usize) < list.len() {
                    list[channel_idx as usize].secret
                } else {
                    [0u8; 16]
                }
            });
            let ch_hash = meshcore_wire::crypto::channel_hash(&secret);
            node.tx_beacons = node.tx_beacons.saturating_add(1);
            let _ = mesh.queue_group_data(
                ch_hash,
                &secret,
                data_type,
                data.as_slice(),
                path_len,
                &path,
            );
        }
        UsbCommand::SendDirectText {
            dest_prefix,
            timestamp,
            txt_type,
            attempt,
            text,
        } => {
            info!(
                "Queuing MeshCore Direct Text to {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}: {}",
                dest_prefix[0],
                dest_prefix[1],
                dest_prefix[2],
                dest_prefix[3],
                dest_prefix[4],
                dest_prefix[5],
                text.as_str()
            );
            node.tx_beacons = node.tx_beacons.saturating_add(1);
            let peer = DISCOVERED_CONTACTS.lock(|c| {
                c.borrow()
                    .iter()
                    .find(|ct| &ct.pub_key[..6] == &dest_prefix[..])
                    .map(|ct| (ct.pub_key, ct.out_path, ct.out_path_len))
            });
            match peer {
                Some((dest_pub, out_path, out_path_len)) => {
                    match mesh.queue_direct_text(
                        &dest_pub,
                        text.as_str(),
                        timestamp,
                        txt_type,
                        attempt,
                        out_path_len,
                        &out_path,
                    ) {
                        Ok(crc) => {
                            if txt_type == meshcore_wire::packets::txtmsg::TXT_TYPE_PLAIN {
                                track_pending_ack(crc, dest_pub);
                            }
                        }
                        Err(_) => info!("Direct message send failed (text or encryption)"),
                    }
                }
                None => info!("Direct message send failed: contact not found"),
            }
        }
        UsbCommand::SendControlData { data } => {
            if !data.is_empty() {
                if let Ok(cp) = meshcore_wire::packets::ControlPacket::new(data[0], &data[1..]) {
                    if let Ok(pkt) = cp.to_packet(meshcore_wire::packet::ROUTE_TYPE_DIRECT) {
                        mesh.seen_filter.mark_seen(pkt.calculate_packet_hash());
                        let _ = mesh
                            .tx_queue
                            .push_control(pkt, embassy_time::Instant::now().as_millis() as u64);
                    }
                }
            }
        }
        UsbCommand::SendRawData {
            path_len,
            path,
            payload,
        } => {
            if Packet::is_valid_path_len(path_len) {
                let mut pkt = Packet::new(
                    meshcore_wire::packet::ROUTE_TYPE_DIRECT,
                    meshcore_wire::packet::PAYLOAD_TYPE_RAW_CUSTOM,
                );
                if pkt.set_payload(payload.as_slice()).is_ok() {
                    pkt.path_len = path_len;
                    let n = pkt.path_byte_len();
                    pkt.path[..n].copy_from_slice(&path[..n]);
                    mesh.seen_filter.mark_seen(pkt.calculate_packet_hash());
                    let _ = mesh
                        .tx_queue
                        .push_control(pkt, embassy_time::Instant::now().as_millis() as u64);
                }
            }
        }
        UsbCommand::SendRawPacket { raw } => {
            if let Ok(pkt) = Packet::read_from(raw.as_slice()) {
                mesh.seen_filter.mark_seen(pkt.calculate_packet_hash());
                let _ = mesh
                    .tx_queue
                    .push_control(pkt, embassy_time::Instant::now().as_millis() as u64);
            }
        }
        UsbCommand::ImportContact { raw } => {
            if protocol::process_imported_contact(node, raw.as_slice()) {
                *advert_epoch = advert_epoch.wrapping_add(1);
                needs_render = true;
            }
        }
        UsbCommand::SendTracePath {
            tag,
            auth,
            flags,
            path_len,
            path,
        } => {
            // TRACE paths are a list of hashes carried in the payload,
            // not the packet's routing path. Send it zero-hop.
            if path_len as usize <= 64 {
                if let Ok(tp) = meshcore_wire::packets::TracePacket::new(
                    tag,
                    auth,
                    flags,
                    &path[..path_len as usize],
                ) {
                    if let Ok(pkt) = tp.to_packet(meshcore_wire::packet::ROUTE_TYPE_DIRECT) {
                        mesh.seen_filter.mark_seen(pkt.calculate_packet_hash());
                        let _ = mesh
                            .tx_queue
                            .push_control(pkt, embassy_time::Instant::now().as_millis() as u64);
                    }
                }
            }
        }
        UsbCommand::SetRepeat(rep) => {
            info!("Setting repeater mode: {}", rep);
            mesh.allow_forward = rep;
            if !rep {
                mesh.cancel_pending_forwards();
            }
        }
        UsbCommand::SetFrequency(f) => {
            info!("Frequency updated to: {} Hz", f);
            // Moving off an off-grid band can disable client repeat.
            let was_forwarding = mesh.allow_forward;
            mesh.allow_forward = forwarding_enabled();
            if was_forwarding && !mesh.allow_forward {
                mesh.cancel_pending_forwards();
            }
            let _ = crate::radio::request_radio_config();
        }
        UsbCommand::SetRadio {
            frequency_hz,
            bw_khz,
            sf,
            cr,
        } => {
            info!(
                "Radio updated: {} Hz, BW {} kHz, SF{}, CR 4/{}",
                frequency_hz, bw_khz, sf, cr
            );
            RADIO_SETTINGS.lock(|r| {
                let mut s = r.borrow_mut();
                s.frequency_hz = frequency_hz;
                s.bw_khz = bw_khz;
                s.sf = sf;
                s.cr = cr;
                s.offgrid_home_freq = 0;
                if !is_offgrid_freq_hz(frequency_hz) {
                    s.client_repeat = false;
                }
            });
            mesh.allow_forward = forwarding_enabled();
            let _ = crate::radio::request_radio_config();
        }
        UsbCommand::SetTxPower(tx) => {
            info!("TX power updated to: {} dBm", tx);
            let _ = crate::radio::request_radio_config();
        }
        UsbCommand::SetSpreadingFactor(sf) => {
            info!("SF updated to: {}", sf);
            let _ = crate::radio::request_radio_config();
        }
        UsbCommand::SetBandwidth(bw) => {
            info!("BW updated to: {} kHz", bw);
            let _ = crate::radio::request_radio_config();
        }
        UsbCommand::SetCodingRate(cr) => {
            info!("CR updated to: 4/{}", cr);
            let _ = crate::radio::request_radio_config();
        }
        UsbCommand::SetPathHashMode(mode) => {
            info!(
                "Setting path.hash.mode to: {} ({} bytes/hop)",
                mode,
                mode + 1
            );
            mesh.set_path_hash_mode(mode);
            node.path_hash_mode = mode;
        }
        UsbCommand::RebootToOtaDfu => {}
    }
    needs_render
}

#[inline(never)]
pub async fn run<B: AppBoard, R: LoraRadio>(
    board: &mut B,
    mesh: &mut MeshEngine<'_>,
    node: &mut CompanionNode<'_>,
    lora: &mut R,
) -> ! {
    let mut rx_buf = [0u8; 256];
    sync_gps(node);
    let mut next_tick = Instant::now() + Duration::from_secs(2);
    let mut led_off_at: Option<Instant> = None;
    let mut render_retry_at: Option<Instant> = None;
    let mut tick: u32 = 0;
    let mut ui = UiState::default();
    let mut msg_epoch: u32 = 0;
    let mut advert_epoch: u32 = 0;
    let mut last_snapshot = UiSnapshot::capture(board, node, msg_epoch, advert_epoch);
    let mut last_render_at = Instant::now();

    loop {
        // Keep the app node aligned with shared state changed by a transport
        // or CLI command (identity, callsign, clock, preferences).
        node.sync_from_shared();
        bump_app_heartbeat();
        // Move packets queued by the previous iteration into the radio
        // scheduler before sleeping.
        flush_tx_queue(mesh, lora).await;

        let mut next_deadline = next_tick;
        if let Some(t) = led_off_at {
            if t < next_deadline {
                next_deadline = t;
            }
        }
        if let Some(t) = render_retry_at {
            if t < next_deadline {
                next_deadline = t;
            }
        }

        let wake = select4(
            lora.receive(&mut rx_buf),
            USB_COMMAND_CHANNEL.receive(),
            INPUT_EVENT_CHANNEL.receive(),
            Timer::at(next_deadline),
        )
        .await;

        match wake {
        Either4::First((len, rssi, snr)) => {
            info!("Received LoRa packet: {} bytes, RSSI: {} dBm, SNR: {} dB", len, rssi, snr);
            if !leds_disabled() {
                board.set_led(true); // ON
                let off = Instant::now() + Duration::from_millis(50);
                if led_off_at.map_or(true, |prev| off > prev) {
                    led_off_at = Some(off);
                }
            }
            handle_inbound_rx(
                board,
                mesh,
                node,
                &mut ui,
                &mut msg_epoch,
                &mut advert_epoch,
                &rx_buf[..len],
                rssi,
                snr,
            );
        }
        Either4::Second(UsbCommand::RebootToOtaDfu) => {
            info!("Entering BLE OTA DFU bootloader...");
            // Let pending BLE ATT write responses, PACKET_CLI_REPLY, or USB text drain.
            Timer::after_millis(250).await;
            let _ = storage::flush_pending().await;
            board.prepare_ota_dfu(node, &ui).await;
            board.reboot_to_ota_dfu();
        }
        Either4::Second(cmd) => {
            if handle_usb_command(mesh, node, &mut advert_epoch, cmd) {
                if board.render(node, &ui, false) {
                    last_snapshot = UiSnapshot::capture(board, node, msg_epoch, advert_epoch);
                    last_render_at = Instant::now();
                    render_retry_at = None;
                } else {
                    render_retry_at = Some(Instant::now() + Duration::from_millis(100));
                }
            }
        }
        Either4::Third(event) => {
            // Any interaction wakes the frontlight and restarts its idle timer.
            board.input_wake();
            let sound = match event {
                InputEvent::ActionPrimary | InputEvent::ActionSecondary => UiSound::Action,
                _ => UiSound::Click,
            };
            board.beep(sound);

            match event {
                InputEvent::NextPage => {
                    node.current_page = (node.current_page + 1) % board.page_count();
                    if board.render(node, &ui, false) {
                        last_snapshot = UiSnapshot::capture(board, node, msg_epoch, advert_epoch);
                        last_render_at = Instant::now();
                        render_retry_at = None;
                    } else {
                        render_retry_at = Some(Instant::now() + Duration::from_millis(100));
                    }
                }
                InputEvent::PrevPage => {
                    let count = board.page_count().max(1);
                    node.current_page = if node.current_page == 0 {
                        count - 1
                    } else {
                        node.current_page - 1
                    };
                    if board.render(node, &ui, false) {
                        last_snapshot = UiSnapshot::capture(board, node, msg_epoch, advert_epoch);
                        last_render_at = Instant::now();
                        render_retry_at = None;
                    } else {
                        render_retry_at = Some(Instant::now() + Duration::from_millis(100));
                    }
                }
                InputEvent::ActionPrimary | InputEvent::ActionSecondary => {
                    let action = if matches!(event, InputEvent::ActionPrimary) {
                        1
                    } else {
                        2
                    };

                    match (node.current_page, action) {
                        (0, 1) => {
                            info!("Input: zero-hop advert");
                            send_advert(board, mesh, node, false);
                            if !leds_disabled() {
                                let off = Instant::now() + Duration::from_millis(120);
                                if led_off_at.map_or(true, |prev| off > prev) {
                                    led_off_at = Some(off);
                                }
                            }
                        }
                        (0, _) => {
                            info!("Input: flood advert");
                            send_advert(board, mesh, node, true);
                            if !leds_disabled() {
                                let off = Instant::now() + Duration::from_millis(120);
                                if led_off_at.map_or(true, |prev| off > prev) {
                                    led_off_at = Some(off);
                                }
                            }
                        }
                        (1, 1) => {
                            let count = MESSAGE_LOG.lock(|m| m.borrow().len());
                            if count > 0 {
                                ui.inbox_cursor = (ui.inbox_cursor + 1) % count;
                            } else {
                                ui.inbox_cursor = 0;
                            }
                            info!("Input: inbox next {}", ui.inbox_cursor);
                        }
                        (1, _) => {
                            let count = MESSAGE_LOG.lock(|m| m.borrow().len());
                            if count > 0 {
                                ui.inbox_cursor = (ui.inbox_cursor + count - 1) % count;
                            } else {
                                ui.inbox_cursor = 0;
                            }
                            info!("Input: inbox previous {}", ui.inbox_cursor);
                        }
                        (2, 1) => {
                            ui.radar_labels = !ui.radar_labels;
                            info!(
                                "Input: radar labels {}",
                                if ui.radar_labels { "on" } else { "off" }
                            );
                        }
                        (2, _) => {
                            ui.radar_sort = match ui.radar_sort {
                                RadarSort::Age => RadarSort::Signal,
                                RadarSort::Signal => RadarSort::Age,
                            };
                            info!(
                                "Input: radar sort {}",
                                match ui.radar_sort {
                                    RadarSort::Age => "age",
                                    RadarSort::Signal => "signal",
                                }
                            );
                        }
                        (3, 1) => {
                            let next = RADIO_SETTINGS.lock(|r| {
                                let mut s = r.borrow_mut();
                                s.tx_power_dbm = match s.tx_power_dbm {
                                    22 => 17,
                                    17 => 14,
                                    14 => 10,
                                    _ => 22,
                                };
                                s.tx_power_dbm
                            });
                            storage::request_save_config();
                            let _ = crate::radio::request_radio_config();
                            info!("Input: TX power -> {} dBm", next);
                        }
                        (3, _) => {
                            let on = RADIO_SETTINGS.lock(|r| {
                                let mut s = r.borrow_mut();
                                s.rx_duty_cycle = !s.rx_duty_cycle;
                                s.rx_duty_cycle
                            });
                            storage::request_save_config();
                            let _ = crate::radio::request_radio_config();
                            info!("Input: RX duty cycle {}", if on { "on" } else { "off" });
                        }
                        (4, 1) => {
                            RADIO_TELEMETRY.lock(|t| {
                                let mut t = t.borrow_mut();
                                t.tx_beacons = 0;
                                t.rx_packets = 0;
                            });
                            node.tx_beacons = 0;
                            node.rx_packets = 0;
                            info!("Input: counters reset");
                        }
                        (4, _) => {
                            ui.signal.reset();
                            info!("Input: signal history reset");
                        }
                        (5, 1) => {
                            let on = !board.gps_enabled();
                            board.set_gps_enabled(on);
                            info!("Input: GPS power {}", if on { "on" } else { "off" });
                        }
                        (5, _) => {
                            let mut on = 0u8;
                            update_prefs(|p| {
                                p.advert_loc_policy =
                                    if p.advert_loc_policy == 0 { 1 } else { 0 };
                                on = p.advert_loc_policy;
                            });
                            storage::request_save_config();
                            info!("Input: advert location {}", if on != 0 { "on" } else { "off" });
                        }
                        (6, 1) => {
                            let active = offgrid_active();
                            let change = offgrid_set(!active);
                            if change.no_band {
                                info!("Input: no off-grid frequency in this band");
                            } else {
                                if change.changed {
                                    storage::request_save_config();
                                }
                                if change.freq_changed {
                                    let _ = crate::radio::request_radio_config();
                                }
                                mesh.allow_forward = forwarding_enabled();
                                info!("Input: offgrid {}", if active { "off" } else { "on" });
                            }
                        }
                        (6, _) => {
                            // Off-grid has one action; the secondary is unused.
                        }
                        (7, 1) => {
                            let disabled = !leds_disabled();
                            board.set_leds_disabled(disabled);
                            board.set_led(!disabled);
                            if disabled {
                                led_off_at = None;
                            } else {
                                let off = Instant::now() + Duration::from_millis(120);
                                if led_off_at.map_or(true, |prev| off > prev) {
                                    led_off_at = Some(off);
                                }
                            }
                            info!(
                                "Input: blue LED {}",
                                if disabled { "disabled" } else { "enabled" }
                            );
                        }
                        (7, _) => {
                            let enabled = !ferrix_core::state::buzzer_enabled();
                            ferrix_core::state::set_buzzer_enabled(enabled);
                            storage::request_save_config();
                            info!(
                                "Input: sound {}",
                                if enabled { "on" } else { "off" }
                            );
                            if enabled {
                                board.beep(UiSound::Action);
                            }
                        }
                        (8, 1) => {
                            let next = (storage::get_path_hash_mode() + 1) % 3;
                            storage::set_path_hash_mode(next);
                            mesh.set_path_hash_mode(next);
                            node.path_hash_mode = next;
                            info!("Input: path hash {} byte(s)/hop", next + 1);
                        }
                        (8, 2) => {
                            info!("Input: entering BLE OTA DFU bootloader");
                            Timer::after_millis(100).await;
                            let _ = storage::flush_pending().await;
                            board.prepare_ota_dfu(node, &ui).await;
                            board.reboot_to_ota_dfu();
                        }
                        (_, _) => {}
                    }

                    if board.render(node, &ui, false) {
                        last_snapshot = UiSnapshot::capture(board, node, msg_epoch, advert_epoch);
                        last_render_at = Instant::now();
                        render_retry_at = None;
                    } else {
                        render_retry_at = Some(Instant::now() + Duration::from_millis(100));
                    }
                }
                _ => {}
            }
        }
        Either4::Fourth(_) => {
            let now = Instant::now();
            if led_off_at.is_some_and(|t| now >= t) {
                board.set_led(false); // OFF
                led_off_at = None;
            }
            if render_retry_at.is_some_and(|t| now >= t) {
                if board.render(node, &ui, false) {
                    last_snapshot = UiSnapshot::capture(board, node, msg_epoch, advert_epoch);
                    last_render_at = Instant::now();
                    render_retry_at = None;
                } else {
                    render_retry_at = Some(now + Duration::from_millis(100));
                }
            }
            if now >= next_tick {
                next_tick = now + Duration::from_secs(2);
                tick = tick.wrapping_add(1);
                protocol::keepalive_tick();
                sync_gps(node);

                // Sample the battery every 10 s (5 ticks) before checking for UI
                // updates so sustained changes are reflected when interval == 0.
                if tick % 5 == 0 {
                    if let Some(mv) = board.sample_battery().await {
                        node.battery_millivolts = mv;
                        node.battery_percentage = ferrix_core::battery::apply_percentage_hysteresis(
                            node.battery_percentage,
                            ferrix_core::battery::millivolts_to_percentage(mv),
                        );
                    }
                }

                let snapshot = UiSnapshot::capture(board, node, msg_epoch, advert_epoch);
                let needs_render = if display_refresh_interval_secs() == 0 {
                    let cooldown_elapsed =
                        last_render_at.elapsed() >= Duration::from_secs(LIVE_PAGE_REFRESH_COOLDOWN_SECS);
                    snapshot.differs_on_visible_page(&last_snapshot, cooldown_elapsed)
                } else {
                    // Keep unread/inbox responsive between longer periodic intervals.
                    snapshot.page <= 1 && snapshot.message_changed(&last_snapshot)
                };
                if needs_render {
                    if board.render(node, &ui, false) {
                        last_snapshot = snapshot;
                        last_render_at = Instant::now();
                        render_retry_at = None;
                    } else {
                        render_retry_at = Some(Instant::now() + Duration::from_millis(100));
                    }
                }

                board.tick(node, &ui, tick);

                // Heartbeat LED flash every 2 s (each tick), unless disabled.
                if !leds_disabled() {
                    board.set_led(true); // ON
                    let off = Instant::now() + Duration::from_millis(20);
                    if led_off_at.map_or(true, |prev| off > prev) {
                        led_off_at = Some(off);
                    }
                } else {
                    board.set_led(false); // OFF
                    led_off_at = None;
                }
            }
        }
        }

    }
}

#[cfg(test)]
mod tests {
    use super::UiSnapshot;

    fn base_snapshot() -> UiSnapshot {
        UiSnapshot {
            page: 0,
            battery_pct: 85,
            time_synced: true,
            msg_len: 0,
            msg_epoch: 0,
            ble_connected: false,
            forwarding: false,
            advert_epoch: 0,
            radio_freq_hz: 910_525_000,
            radio_bw_khz: 62_500,
            radio_sf: 7,
            radio_cr: 5,
            radio_tx_dbm: 22,
            radio_rx_duty: false,
            radio_af_tenths: 10,
            client_repeat: false,
            path_hash_mode: 0,
            rx_packets: 0,
            tx_beacons: 0,
            gps_enabled: false,
            gps_fix: false,
            gps_lat_e4: 0,
            gps_lon_e4: 0,
            advert_loc_policy: 0,
            leds_disabled: false,
            buzzer_enabled: true,
        }
    }

    #[test]
    fn snapshot_ignores_unrelated_changes_per_page() {
        let base = base_snapshot();

        // Battery change affects every page (top bar) immediately.
        let mut bat = base;
        bat.battery_pct = 84;
        assert!(bat.differs_on_visible_page(&base, false));

        // BLE connect triggers on page 0 (MESH), ignored on page 3 (RADIO).
        let mut ble = base;
        ble.ble_connected = true;
        assert!(ble.differs_on_visible_page(&base, false));
        let mut radio = base;
        radio.page = 3;
        let mut radio_ble = radio;
        radio_ble.ble_connected = true;
        assert!(!radio_ble.differs_on_visible_page(&radio, true));

        // New message triggers immediately on page 0 and page 1, ignored on page 3.
        let mut msg = base;
        msg.msg_len = 1;
        msg.msg_epoch = 1;
        assert!(msg.differs_on_visible_page(&base, false));
        let mut inbox = base;
        inbox.page = 1;
        let mut inbox_msg = inbox;
        inbox_msg.msg_epoch = 1;
        assert!(inbox_msg.differs_on_visible_page(&inbox, false));
        let mut radio_msg = radio;
        radio_msg.msg_len = 1;
        radio_msg.msg_epoch = 1;
        assert!(!radio_msg.differs_on_visible_page(&radio, true));

        // Continuous telemetry pages (RADAR, SIGNAL, POSITION drift) wait for cooldown.
        let mut radar = base;
        radar.page = 2;
        let mut radar_adv = radar;
        radar_adv.advert_epoch = 1;
        assert!(!radar_adv.differs_on_visible_page(&radar, false));
        assert!(radar_adv.differs_on_visible_page(&radar, true));

        let mut sig = base;
        sig.page = 4;
        let mut sig_rx = sig;
        sig_rx.rx_packets = 1;
        assert!(!sig_rx.differs_on_visible_page(&sig, false));
        assert!(sig_rx.differs_on_visible_page(&sig, true));

        let mut pos = base;
        pos.page = 5;
        let mut pos_drift = pos;
        pos_drift.gps_lat_e4 = 377_749;
        assert!(!pos_drift.differs_on_visible_page(&pos, false));
        assert!(pos_drift.differs_on_visible_page(&pos, true));

        // GPS fix transition stays immediate even before cooldown elapses.
        let mut pos_fix = pos_drift;
        pos_fix.gps_fix = true;
        assert!(pos_fix.differs_on_visible_page(&pos, false));
    }
}
