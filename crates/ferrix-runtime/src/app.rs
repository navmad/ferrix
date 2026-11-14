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
use embassy_time::{Duration, Instant, Ticker, Timer};
use ferrix_core::airtime::estimate_airtime_ms;
use ferrix_core::command::{UsbCommand, COMPANION_PUSH_CHANNEL, USB_COMMAND_CHANNEL};
use ferrix_core::companion::protocol::{self, CompanionNode};
use ferrix_core::engine::{InboundAction, MeshEngine};
use ferrix_core::state::*;
use ferrix_core::storage;
use ferrix_core::types::*;
use ferrix_hal::radio::LoraRadio;
use heapless::Vec;
use meshcore_wire::packet::Packet;

/// E-ink pages between full refreshes.
const CLICKS_PER_FULL_REFRESH: u32 = 10;

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
#[derive(Default)]
pub struct UiState {
    pub inbox_cursor: usize,
    pub radar_sort: RadarSort,
    pub signal: SignalHistory,
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
    async fn advert_feedback(&mut self);
    async fn render(&mut self, node: &CompanionNode<'_>, ui: &UiState, full: bool);
    async fn input_wake(&mut self);
    async fn tick(&mut self, node: &CompanionNode<'_>, ui: &UiState, tick: u32);
    async fn sample_battery(&mut self) -> Option<u16>;
    fn set_leds_disabled(&mut self, disabled: bool);
}

#[derive(Clone, Copy, Debug)]
pub struct PendingMessageAck {
    pub crc: u32,
    pub send_instant_ms: u64,
}

static PENDING_MESSAGE_ACKS: Mutex<
    CriticalSectionRawMutex,
    core::cell::RefCell<Vec<PendingMessageAck, 8>>,
> = Mutex::new(core::cell::RefCell::new(Vec::new()));

/// Tell the app that a contact was discovered (full record) or re-advertised.
fn push_new_advert(ct: &DiscoveredContact) {
    let record = ferrix_core::companion::contacts::serialize_contact(ct);
    let mut frame = Vec::<u8, 256>::new();
    let _ = frame.push(protocol::PUSH_CODE_NEW_ADVERT);
    let _ = frame.extend_from_slice(&record[1..]);
    let _ = COMPANION_PUSH_CHANNEL.try_send(frame);
}

fn push_advert(pub_key: &[u8; 32]) {
    let mut frame = Vec::<u8, 256>::new();
    let _ = frame.push(protocol::PUSH_CODE_ADVERT);
    let _ = frame.extend_from_slice(pub_key);
    let _ = COMPANION_PUSH_CHANNEL.try_send(frame);
}

fn push_path_updated(pub_key: &[u8; 32]) {
    let mut frame = Vec::<u8, 256>::new();
    let _ = frame.push(protocol::PUSH_CODE_PATH_UPDATED);
    let _ = frame.extend_from_slice(pub_key);
    let _ = COMPANION_PUSH_CHANNEL.try_send(frame);
}

fn track_pending_ack(crc: u32) {
    let now = Instant::now().as_millis();
    PENDING_MESSAGE_ACKS.lock(|a| {
        let mut list = a.borrow_mut();
        if list.is_full() {
            list.remove(0);
        }
        let _ = list.push(PendingMessageAck {
            crc,
            send_instant_ms: now,
        });
    });
}

fn handle_direct_message_confirmed(crc: u32) {
    let now = Instant::now().as_millis();
    let rtt_ms = PENDING_MESSAGE_ACKS.lock(|a| {
        let mut list = a.borrow_mut();
        list.iter().position(|p| p.crc == crc).map(|pos| {
            let item = list.remove(pos);
            now.saturating_sub(item.send_instant_ms) as u32
        })
    });
    let Some(rtt_ms) = rtt_ms else {
        return;
    };
    info!(
        "MeshCore Direct Message Delivery ACK Confirmed! (crc {:08x}, RTT: {} ms)",
        crc, rtt_ms
    );
    let mut frame = Vec::<u8, 256>::new();
    let _ = frame.push(protocol::PUSH_CODE_SEND_CONFIRMED);
    let _ = frame.extend_from_slice(&crc.to_le_bytes());
    let _ = frame.extend_from_slice(&rtt_ms.to_le_bytes());
    let _ = COMPANION_PUSH_CHANNEL.try_send(frame);
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
            .transmit_lane(
                &buf[..len],
                queued.lane,
                queued.queued_at_ms,
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

async fn send_advert<B: AppBoard>(
    board: &mut B,
    mesh: &mut MeshEngine<'_>,
    node: &mut CompanionNode<'_>,
    flood: bool,
) {
    node.tx_beacons = node.tx_beacons.saturating_add(1);
    board.advert_feedback().await;
    let (lat, lon) = protocol::advert_location(node.latitude, node.longitude);
    let _ = mesh.queue_advert(node.epoch_time as u32, lat, lon, flood);
}

/// Run the application loop until the executor stops.
fn sync_gps(node: &mut CompanionNode<'_>) {
    let gps = gps_state();
    node.gps_fix = gps.fix;
    node.latitude = gps.latitude;
    node.longitude = gps.longitude;
}

pub async fn run<B: AppBoard, R: LoraRadio>(
    board: &mut B,
    mesh: &mut MeshEngine<'_>,
    node: &mut CompanionNode<'_>,
    lora: &mut R,
) -> ! {
    let mut rx_buf = [0u8; 256];
    sync_gps(node);
    let mut ticker = Ticker::every(Duration::from_secs(2));
    let mut tick: u32 = 0;
    let mut ui = UiState::default();
    let mut clicks_since_full: u32 = 0;

    loop {
        // Move packets queued by the previous iteration into the radio
        // scheduler before sleeping.
        flush_tx_queue(mesh, lora).await;

        let wake = select4(
            lora.receive(&mut rx_buf),
            USB_COMMAND_CHANNEL.receive(),
            INPUT_EVENT_CHANNEL.receive(),
            ticker.next(),
        )
        .await;

        match wake {
        Either4::First((len, rssi, snr)) => {
            info!("Received LoRa packet: {} bytes, RSSI: {} dBm, SNR: {} dB", len, rssi, snr);
            board.set_led(true); // ON
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
            let _ = log_rx_push.extend_from_slice(&rx_buf[..copy_lora_len]);
            let _ = COMPANION_PUSH_CHANNEL.try_send(log_rx_push);

            // Synchronize real radio telemetry to shared USB CLI status
            let rx_air = RADIO_SETTINGS.lock(|r| {
                let s = r.borrow();
                estimate_airtime_ms(
                    len,
                    s.sf,
                    s.bw_khz,
                    s.cr,
                    if s.sf <= 8 { 32 } else { 16 },
                )
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
            if let Ok(pkt) = Packet::read_from(&rx_buf[..len]) {
                let advert_hops = pkt.path_hash_count() as u8;
                // Retain the signed payload so the contact can be exported.
                let advert_payload =
                    if pkt.payload_type() == meshcore_wire::packet::PAYLOAD_TYPE_ADVERT {
                        let mut v = heapless::Vec::<u8, 184>::new();
                        let _ = v.extend_from_slice(&pkt.payload[..pkt.payload_len]);
                        Some(v)
                    } else {
                        None
                    };
                match mesh.handle_inbound_packet(pkt, snr) {
                    InboundAction::ReceivedAdvert(adv) => {
                        info!("MeshCore Advert from: {}", adv.name.as_str());
                        let had_blob =
                            storage::adverts::has_blob(&adv.pub_key);
                        protocol::note_advert(
                            &adv.pub_key,
                            adv.timestamp,
                            had_blob,
                        );
                        let node_id = [adv.pub_key[0], adv.pub_key[1], adv.pub_key[2], adv.pub_key[3]];

                        // Check if this contact already exists in our list
                        let contact_type = CompanionNode::get_contact_type_from_flags(adv.flags);
                        let should_add = node.should_auto_add_contact_type(contact_type)
                            && node.autoadd_allows_hops(advert_hops);

                        let mut store_blob = false;
                        let mut overwritten_key: Option<[u8; 32]> = None;
                        DISCOVERED_CONTACTS.lock(|c| {
                            let mut list = c.borrow_mut();
                            if let Some(existing) = list.iter_mut().find(|ct| ct.pub_key == adv.pub_key) {
                                // Anti-replay: ignore an advert no newer than the last accepted one.
                                if existing.last_advert_ts != 0 && adv.timestamp <= existing.last_advert_ts {
                                    return;
                                }
                                // Only rewrite the shared advert blob when the
                                // contact's advertised identity actually changed.
                                // A missing blob is not backfilled on every
                                // advert: that would append a record per advert
                                // and wear the log. The next real change (or a
                                // re-add) stores it.
                                if existing.callsign != adv.name
                                    || existing.contact_type != contact_type
                                    || existing.latitude != adv.latitude
                                    || existing.longitude != adv.longitude
                                {
                                    store_blob = true;
                                }
                                existing.callsign = adv.name.clone();
                                existing.last_seen_epoch = node.epoch_time as u32;
                                existing.last_advert_ts = adv.timestamp;
                                existing.rssi = rssi;
                                existing.snr = snr;
                                existing.latitude = adv.latitude;
                                existing.longitude = adv.longitude;
                                existing.contact_type = contact_type;
                                // Contact flags (favorite/telemetry permissions) are
                                // independent of the advert flags; keep the stored ones.
                                push_advert(&adv.pub_key);
                                return;
                            }

                            // New contact. The app is told even when auto-add
                            // declines it, matching the reference discovery flow.
                            let mut cs = heapless::String::new();
                            let _ = cs.push_str(adv.name.as_str());
                            let discovered = DiscoveredContact {
                                pub_key: adv.pub_key,
                                callsign: cs,
                                node_id,
                                last_seen_epoch: node.epoch_time as u32,
                                rssi,
                                snr,
                                latitude: adv.latitude,
                                longitude: adv.longitude,
                                // Contact flags start cleared; the advert flags
                                // only classified `contact_type` above.
                                flags: 0,
                                contact_type,
                                out_path: [0u8; 64],
                                out_path_len: 0xFF,
                                last_advert_ts: adv.timestamp,
                                saved_hash: 0,
                            sync_since: 0,
                            };

                            let mut added = false;
                            if should_add {
                                if list.len() < list.capacity() {
                                    let _ = list.push(discovered.clone());
                                    added = true;
                                } else if node.should_overwrite_when_full() {
                                    // Find the oldest non-favorite contact to overwrite.
                                    let mut oldest_idx = None;
                                    let mut oldest_lastmod = u32::MAX;
                                    for (idx, ct) in list.iter().enumerate() {
                                        let is_favorite = (ct.flags
                                            & protocol::CONTACT_FLAG_FAVORITE)
                                            != 0;
                                        if !is_favorite && ct.last_seen_epoch < oldest_lastmod {
                                            oldest_lastmod = ct.last_seen_epoch;
                                            oldest_idx = Some(idx);
                                        }
                                    }
                                    if let Some(idx) = oldest_idx {
                                        let overwritten = list[idx].pub_key;
                                        let mut push = heapless::Vec::<u8, 256>::new();
                                        let _ = push.push(
                                            protocol::PUSH_CODE_CONTACT_DELETED,
                                        );
                                        let _ = push.extend_from_slice(&overwritten);
                                        let _ =
                                            COMPANION_PUSH_CHANNEL.try_send(push);
                                        storage::adverts::forget(&overwritten);
                                        overwritten_key = Some(overwritten);
                                        list[idx] = discovered.clone();
                                        added = true;
                                    }
                                }
                            }

                            if added {
                                store_blob = true;
                            }
                            push_new_advert(&discovered);

                            if !added && list.len() >= list.capacity() {
                                let mut push = heapless::Vec::<u8, 256>::new();
                                let _ = push.push(
                                    protocol::PUSH_CODE_CONTACTS_FULL,
                                );
                                let _ =
                                    COMPANION_PUSH_CHANNEL.try_send(push);
                            }
                        });
                        if store_blob {
                            if let Some(payload) = advert_payload.as_ref() {
                                storage::adverts::remember(&adv.pub_key, payload);
                            }
                        }
                        if let Some(key) = overwritten_key {
                            storage::contacts::queue_remove(key).await;
                        }
                        storage::request_contacts();
                    }
                    InboundAction::ReceivedGroupText { channel_idx, path_len, packet: grp } => {
                        info!("MeshCore Group Text [ch {}]: <{}> {}", channel_idx, grp.sender.as_str(), grp.message.as_str());
                        node.unread_messages = node.unread_messages.saturating_add(1);
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
                        let dropped = MESSAGE_LOG.lock(|m| {
                            let mut list = m.borrow_mut();
                            let dropped = list.is_full();
                            if dropped {
                                list.remove(0);
                            }
                            let _ = list.push(InboxItem::Text(
                                message.clone(),
                            ));
                            dropped
                        });
                        if dropped {
                            storage::messages::queue_consume().await;
                        }
                        storage::messages::queue_append(message).await;

                        // Prompt companion app with PUSH_CODE_MSG_WAITING (0x83)
                        let mut msg_wait = heapless::Vec::<u8, 256>::new();
                        let _ = msg_wait.push(protocol::PUSH_CODE_MSG_WAITING);
                        let _ = COMPANION_PUSH_CHANNEL.try_send(msg_wait);
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
                            let _ = list.push(InboxItem::ChannelData(
                                item.clone(),
                            ));
                            dropped
                        });
                        if dropped {
                            storage::messages::queue_consume().await;
                        }
                        storage::messages::queue_append_channel_data(item).await;

                        let mut msg_wait = heapless::Vec::<u8, 256>::new();
                        let _ = msg_wait.push(protocol::PUSH_CODE_MSG_WAITING);
                        let _ = COMPANION_PUSH_CHANNEL.try_send(msg_wait);
                    }
                    InboundAction::ReceivedDirectText { msg, sender_pub } => {
                        info!(
                            "MeshCore Direct Msg from {:02x}{:02x}{:02x}{:02x}{:02x}{:02x}: {}",
                            sender_pub[0], sender_pub[1], sender_pub[2],
                            sender_pub[3], sender_pub[4], sender_pub[5],
                            msg.text.as_str()
                        );
                        node.unread_messages = node.unread_messages.saturating_add(1);
                        // Signed room posts advance the room-sync watermark so a
                        // later login does not replay the whole backlog.
                        if msg.txt_type
                            == meshcore_wire::packets::txtmsg::TXT_TYPE_SIGNED_PLAIN
                        {
                            let changed =
                                DISCOVERED_CONTACTS.lock(|c| {
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
                            path_len: 0,
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
                        let dropped = MESSAGE_LOG.lock(|m| {
                            let mut list = m.borrow_mut();
                            let dropped = list.is_full();
                            if dropped {
                                list.remove(0);
                            }
                            let _ = list.push(InboxItem::Text(
                                message.clone(),
                            ));
                            dropped
                        });
                        if dropped {
                            storage::messages::queue_consume().await;
                        }
                        storage::messages::queue_append(message).await;

                        // Prompt companion app with PUSH_CODE_MSG_WAITING (0x83)
                        let mut msg_wait = heapless::Vec::<u8, 256>::new();
                        let _ = msg_wait.push(protocol::PUSH_CODE_MSG_WAITING);
                        let _ = COMPANION_PUSH_CHANNEL.try_send(msg_wait);
                    }
                    InboundAction::ReceivedRepeaterEcho { pkt_hash, hops } => {
                        info!("MeshCore Message echoed by repeater! ({} hops, pkt_hash: {:?})", hops, pkt_hash);
                    }
                    InboundAction::ReceivedAck(ack) => {
                        info!("MeshCore ACK received (crc {:08x})", ack.crc);
                        if !protocol::handle_keepalive_ack(ack.crc) {
                            handle_direct_message_confirmed(ack.crc);
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
                    InboundAction::ReceivedRequest { plain, sender_pub } => {
                        info!(
                            "MeshCore REQ from {:02x}{:02x} ({} bytes, ignored)",
                            sender_pub[0],
                            sender_pub[1],
                            plain.len()
                        );
                    }
                    InboundAction::ReceivedControl { data, path_len } => {
                        info!("MeshCore CONTROL data ({} bytes)", data.len());
                        let mut push = heapless::Vec::<u8, 256>::new();
                        let _ = push.push(
                            protocol::PUSH_CODE_CONTROL_DATA,
                        );
                        let _ = push.push((snr.saturating_mul(4)) as u8);
                        let _ = push.push(rssi as i8 as u8);
                        let _ = push.push(path_len);
                        let _ = push.extend_from_slice(&data);
                        if push.len() <= 176 {
                            let _ = COMPANION_PUSH_CHANNEL
                                .try_send(push);
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
                        protocol::note_trace_rx(
                            tag,
                            flags,
                            hashes.len(),
                            snrs.len(),
                        );
                        let mut push = heapless::Vec::<u8, 256>::new();
                        let _ = push.push(
                            protocol::PUSH_CODE_TRACE_DATA,
                        );
                        let _ = push.push(0);
                        let _ = push.push(hashes.len() as u8);
                        let _ = push.push(flags);
                        let _ = push.extend_from_slice(&tag.to_le_bytes());
                        let _ = push.extend_from_slice(&auth.to_le_bytes());
                        let _ = push.extend_from_slice(&hashes);
                        let _ = push.extend_from_slice(&snrs);
                        let _ = push.push((final_snr.saturating_mul(4)) as u8);
                        if push.len() <= 176 {
                            let _ = COMPANION_PUSH_CHANNEL
                                .try_send(push);
                        }
                    }
                    InboundAction::ReceivedPath {
                        packet: path,
                        sender_pub,
                        arrival_path_len,
                        arrival_path,
                    } => {
                        info!(
                            "MeshCore PATH from {:02x}{:02x}: {} hops, extra {}",
                            sender_pub[0], sender_pub[1], path.hop_count(), path.extra_type
                        );
                        let path_updated = DISCOVERED_CONTACTS
                            .lock(|c| {
                                if let Some(ct) =
                                    c.borrow_mut().iter_mut().find(|ct| ct.pub_key == sender_pub)
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
                            ) && !protocol::handle_response(
                                &path.extra,
                                &sender_pub,
                            ) {
                                info!(
                                    "MeshCore PATH RESPONSE extra unhandled ({} bytes)",
                                    path.extra.len()
                                );
                            }
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
            } else {
                info!("Received non-MeshCore or legacy beacon frame");
            }

            Timer::after_millis(50).await;
            board.set_led(false); // OFF
        }
        Either4::Second(cmd) => {
            match cmd {
                UsbCommand::SetName(new_name) => {
                    info!("Updating node callsign to: {}", new_name.as_str());
                    mesh.set_callsign(new_name.as_str());
                    node.callsign = new_name;
                    board.render(node, &ui, false).await;
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
                    let (lat, lon) = protocol::advert_location(
                        node.latitude,
                        node.longitude,
                    );
                    let _ = mesh.queue_advert(time, lat, lon, flood);
                }
                UsbCommand::SetAdvertLatLon { lat, lon } => {
                    node.latitude = lat as f32 / 1_000_000.0;
                    node.longitude = lon as f32 / 1_000_000.0;
                }
                UsbCommand::SetAutoAddConfig { config, max_hops } => {
                    node.autoadd_config = config;
                    node.autoadd_max_hops = max_hops;
                }
                UsbCommand::SendGroupText { channel_idx, timestamp, text } => {
                    info!("Queuing MeshCore Group Text on ch {}: {} (ts: {})", channel_idx, text.as_str(), timestamp);
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
                        dest_prefix[0], dest_prefix[1], dest_prefix[2],
                        dest_prefix[3], dest_prefix[4], dest_prefix[5],
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
                                Ok(crc) => track_pending_ack(crc),
                                Err(_) => info!("Direct message send failed (text or encryption)"),
                            }
                        }
                        None => info!("Direct message send failed: contact not found"),
                    }
                }
                UsbCommand::SendControlData { data } => {
                    if !data.is_empty() {
                        if let Ok(cp) = meshcore_wire::packets::ControlPacket::new(
                            data[0],
                            &data[1..],
                        ) {
                            if let Ok(pkt) = cp.to_packet(
                                meshcore_wire::packet::ROUTE_TYPE_DIRECT,
                            ) {
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
                            if let Ok(pkt) = tp.to_packet(
                                meshcore_wire::packet::ROUTE_TYPE_DIRECT,
                            ) {
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
                    mesh.is_repeater = rep;
                    if !rep {
                        mesh.cancel_pending_forwards();
                    }
                }
                UsbCommand::SetFrequency(f) => {
                    info!("Frequency updated to: {} Hz", f);
                    // Moving off an off-grid band can disable client repeat.
                    let was_forwarding = mesh.is_repeater;
                    mesh.is_repeater = forwarding_enabled();
                    if was_forwarding && !mesh.is_repeater {
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
                    mesh.is_repeater = forwarding_enabled();
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
                    info!("Setting path.hash.mode to: {} ({} bytes/hop)", mode, mode + 1);
                    mesh.set_path_hash_mode(mode);
                    node.path_hash_mode = mode;
                }
            }
        }
        Either4::Third(event) => {
            // Any interaction wakes the frontlight and restarts its idle timer.
            board.input_wake().await;

            let clicks = match event {
                InputEvent::NextPage | InputEvent::ActionPrimary => 1,
                InputEvent::PrevPage | InputEvent::ActionSecondary => 2,
                _ => 0,
            };
            clicks_since_full += clicks;
            let full = if clicks_since_full >= CLICKS_PER_FULL_REFRESH {
                clicks_since_full -= CLICKS_PER_FULL_REFRESH;
                true
            } else {
                false
            };

            match event {
                InputEvent::NextPage => {
                    node.current_page = (node.current_page + 1) % board.page_count();
                    board.render(node, &ui, full).await;
                }
                InputEvent::PrevPage => {
                    let count = board.page_count().max(1);
                    node.current_page = if node.current_page == 0 {
                        count - 1
                    } else {
                        node.current_page - 1
                    };
                    board.render(node, &ui, full).await;
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
                            send_advert(board, mesh, node, false).await;
                        }
                        (0, _) => {
                            info!("Input: flood advert");
                            send_advert(board, mesh, node, true).await;
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
                            info!("Input: radar ping (zero-hop advert)");
                            send_advert(board, mesh, node, false).await;
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
                                mesh.is_repeater = forwarding_enabled();
                                info!("Input: offgrid {}", if active { "off" } else { "on" });
                            }
                        }
                        (6, _) => {
                            let on = RADIO_SETTINGS.lock(|r| {
                                let mut s = r.borrow_mut();
                                s.is_repeater = !s.is_repeater;
                                s.is_repeater
                            });
                            mesh.is_repeater = forwarding_enabled();
                            if !mesh.is_repeater {
                                mesh.cancel_pending_forwards();
                            }
                            storage::request_save_config();
                            info!("Input: repeater role {}", if on { "on" } else { "off" });
                        }
                        (7, 1) => {
                            let disabled = !leds_disabled();
                            board.set_leds_disabled(disabled);
                            board.set_led(!disabled);
                            info!(
                                "Input: blue LED {}",
                                if disabled { "disabled" } else { "enabled" }
                            );
                        }
                        (_, _) => {
                            let next = (storage::get_path_hash_mode() + 1) % 3;
                            storage::set_path_hash_mode(next);
                            mesh.set_path_hash_mode(next);
                            node.path_hash_mode = next;
                            info!(
                                "Input: path hash {} byte(s)/hop",
                                next + 1
                            );
                        }
                    }

                    board.render(node, &ui, full).await;
                }
                _ => {}
            }
        }
        Either4::Fourth(_) => {
            tick = tick.wrapping_add(1);
            protocol::keepalive_tick();
            sync_gps(node);

            board.tick(node, &ui, tick).await;

            // Sample the battery every 10 s (5 ticks).
            if tick % 5 == 0 {
                if let Some(mv) = board.sample_battery().await {
                    node.battery_millivolts = mv;
                    node.battery_percentage = ferrix_core::battery::millivolts_to_percentage(mv);
                }
            }

            // Heartbeat LED flash every 2 s (each tick), unless disabled.
            if !leds_disabled() {
                board.set_led(true); // ON
                Timer::after_millis(20).await;
            }
            board.set_led(false); // OFF
        }
        }

    }
}
