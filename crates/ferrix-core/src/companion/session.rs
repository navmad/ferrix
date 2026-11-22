//! Transport-neutral companion session.
//!
//! A link moves bytes or frames and owns connection lifecycle. The session owns
//! the companion node and the USB/binary/text protocol state machine, so USB
//! and BLE share command dispatch, contact sync and CLI handling.

use heapless::{String, Vec};

use crate::companion::cli::{handle_text_cli, CliHost, TextSink};
use crate::companion::contacts;
use crate::companion::framing::{
    UsbFrameReader, MAX_FRAME_PAYLOAD_SIZE, USB_INBOUND_DELIMITER,
};
use crate::companion::protocol::{self, CompanionNode};

/// Output side of a companion transport.
#[allow(async_fn_in_trait)]
pub trait CompanionLink {
    /// Write one companion frame (without transport framing).
    async fn write_frame(&mut self, frame: &[u8]);

    /// Write raw CLI text to the host.
    async fn write_text(&mut self, text: &[u8]);
}

// `protocol::handle_command` emits at most 1 response frame per command
// (`CMD_GET_CONTACTS` streams directly via `stream_contacts`). Keeping this
// small lets each frame hold full `MAX_FRAME_PAYLOAD_SIZE` bytes without
// inflating the Embassy task arena.
const MAX_RESPONSES: usize = 2;
const TEXT_LINE: usize = 256;

/// One companion connection's protocol state.
pub struct CompanionSession<'a> {
    node: CompanionNode<'a>,
    host: &'a dyn CliHost,
    frame_reader: UsbFrameReader,
    text_line: [u8; TEXT_LINE],
    text_len: usize,
    binary_mode: bool,
    binary_session: bool,
}

impl<'a> CompanionSession<'a> {
    pub fn new(
        callsign: &str,
        hw_model: &'static str,
        capabilities: u32,
        public_key: [u8; 32],
        private_key: [u8; 64],
        host: &'a dyn CliHost,
    ) -> Self {
        Self {
            node: CompanionNode::new(
                callsign,
                hw_model,
                capabilities,
                public_key,
                private_key,
                host,
            ),
            host,
            frame_reader: UsbFrameReader::new(),
            text_line: [0u8; TEXT_LINE],
            text_len: 0,
            binary_mode: false,
            binary_session: false,
        }
    }

    /// Reset per-connection framing and text state.
    pub fn reset(&mut self) {
        self.frame_reader.reset();
        self.node.sync_from_shared();
        self.text_len = 0;
        self.binary_mode = false;
        self.binary_session = false;
    }

    /// True once this connection has used the binary companion protocol.
    pub fn binary_session(&self) -> bool {
        self.binary_session
    }

    /// Feed one byte from a stream-oriented link (USB CDC).
    pub async fn feed_byte<L: CompanionLink>(&mut self, byte: u8, link: &mut L) {
        if self.binary_mode || byte == USB_INBOUND_DELIMITER {
            self.binary_mode = true;
            self.binary_session = true;

            let mut payload = Vec::<u8, MAX_FRAME_PAYLOAD_SIZE>::new();
            let complete = match self.frame_reader.push_byte(byte) {
                Some(p) => {
                    let _ = payload.extend_from_slice(p);
                    true
                }
                None => false,
            };
            if complete {
                self.binary_mode = false;
                self.handle_frame(&payload, link).await;
            }
            return;
        }

        // Text sideband: any text byte leaves companion mode so pushes stop.
        self.binary_session = false;
        if byte == b'\r' || byte == b'\n' {
            if self.text_len > 0 {
                let mut line = String::<TEXT_LINE>::new();
                if let Ok(s) = core::str::from_utf8(&self.text_line[..self.text_len]) {
                    let _ = line.push_str(s);
                }
                self.text_len = 0;
                if !line.is_empty() {
                    self.run_cli(line.as_str(), link, true).await;
                }
            }
        } else if byte == 0x08 || byte == 0x7F {
            if self.text_len > 0 {
                self.text_len -= 1;
            }
        } else if self.text_len < self.text_line.len() && (0x20..=0x7E).contains(&byte) {
            self.text_line[self.text_len] = byte;
            self.text_len += 1;
        }
    }

    /// Handle one complete companion frame.
    pub async fn handle_frame<L: CompanionLink>(&mut self, payload: &[u8], link: &mut L) {
        self.node.sync_from_shared();
        if payload.first() == Some(&protocol::CMD_GET_CONTACTS) {
            let since = if payload.len() >= 5 {
                u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]])
            } else {
                0
            };
            self.stream_contacts(link, since).await;
            return;
        }

        let mut responses: Vec<Vec<u8, MAX_FRAME_PAYLOAD_SIZE>, MAX_RESPONSES> = Vec::new();
        protocol::handle_command(&mut self.node, payload, self.host, |resp| {
            let mut out = Vec::new();
            let _ = out.extend_from_slice(&resp[..resp.len().min(MAX_FRAME_PAYLOAD_SIZE)]);
            let _ = responses.push(out);
        })
        .await;

        for response in responses.iter() {
            link.write_frame(response).await;
        }

        if protocol::take_pending_reboot() {
            embassy_time::Timer::after_millis(150).await;
            self.host.reboot();
        }
    }

    /// Stream the contact list through the link, optionally filtering to contacts
    /// modified strictly after `since` (when `since > 0`).
    pub async fn stream_contacts<L: CompanionLink>(&mut self, link: &mut L, since: u32) {
        let count = protocol::contacts_count();
        link.write_frame(&protocol::encode_contacts_start(count)).await;

        let mut max_lastmod = 0u32;
        for idx in 0..count as usize {
            let Some(c) = protocol::contact_at(idx) else {
                continue;
            };
            if since > 0 && c.last_seen_epoch <= since {
                continue;
            }
            if c.last_seen_epoch > max_lastmod {
                max_lastmod = c.last_seen_epoch;
            }
            let record = contacts::serialize_contact(&c);
            link.write_frame(&record).await;
        }

        link.write_frame(&protocol::encode_contacts_end(max_lastmod))
            .await;
    }

    /// Run one text CLI line.
    pub async fn run_cli<L: CompanionLink>(&mut self, line: &str, link: &mut L, echo: bool) {
        self.node.sync_from_shared();
        let mut sink = LinkText(link);
        handle_text_cli(&mut sink, line, &mut self.node, self.host, echo).await;
    }
}

/// Adapt a companion link to the CLI's raw text sink.
struct LinkText<'a, L>(&'a mut L);

impl<L: CompanionLink> TextSink for LinkText<'_, L> {
    async fn send(&mut self, data: &[u8]) -> Result<(), ()> {
        self.0.write_text(data).await;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::companion::cli::{PowerClockStats, RadioStats, StorageStats};
    use crate::companion::protocol::CompanionHost;
    use crate::types::DiscoveredContact;

    struct DummyHost;

    impl CompanionHost for DummyHost {
        fn battery_millivolts(&self) -> u16 {
            4000
        }
        fn set_callsign(&self, _name: &str) {}
        fn set_custom_priv_key(&self, _key: [u8; 64]) {}
        fn set_path_hash_mode(&self, _mode: u8) {}
        fn request_save_config(&self) {}
        fn request_radio_config(&self) {}
        fn request_factory_reset(&self) {}
        fn queue_consume_now(&self) {}
        fn advert_remember(&self, _pub_key: &[u8; 32], _payload: &[u8]) {}
        fn reboot(&self) -> ! {
            panic!("reboot in test")
        }
    }

    impl CliHost for DummyHost {
        fn leds_disabled(&self) -> bool { false }
        fn set_leds_disabled(&self, _disabled: bool) {}
        fn storage_stats(&self) -> StorageStats {
            StorageStats {
                contact_saves: 0,
                contact_erases: 0,
                message_records: 0,
                message_page_erases: 0,
                message_queued: 0,
                advert_records: 0,
                advert_erases: 0,
                config_saves: 0,
                config_erases: 0,
                external_profile: false,
                external_unavailable: false,
                external_drops: 0,
            }
        }
        fn radio_stats(&self) -> RadioStats {
            RadioStats {
                override_ms: 0,
                slot_ms: 0,
                used: 0,
                forward_scheduled: 0,
                forward_sent: 0,
                queue_full: [0; 3],
                ack_stats: [0; 3],
                duplicate_deltas: [0; 6],
            }
        }
        fn power_clock_stats(&self) -> Option<PowerClockStats> { None }
        fn set_tx_slot_override(&self, _ms: u32) {}
        fn clear_radio_stats(&self) {}
        fn advert_stats(&self) -> (usize, usize, bool) { (0, 0, false) }
        fn advert_slots(&self) -> usize { 0 }
        fn gps_enabled(&self) -> bool { false }
        fn set_gps_enabled(&self, _on: bool) {}
        fn reboot_to_bootloader(&self) -> ! { panic!("bootloader") }
        fn reboot_to_ota_dfu(&self) -> ! { panic!("ota") }
        fn board_line(&self) -> &'static str { "test" }
    }

    struct CaptureLink {
        frames: std::vec::Vec<std::vec::Vec<u8>>,
    }

    impl CompanionLink for CaptureLink {
        async fn write_frame(&mut self, frame: &[u8]) {
            self.frames.push(frame.to_vec());
        }
        async fn write_text(&mut self, _text: &[u8]) {}
    }

    fn poll_ready<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, Waker};
        let mut cx = Context::from_waker(Waker::noop());
        let mut fut = core::pin::pin!(fut);
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(out) => out,
            Poll::Pending => panic!("unexpected Pending"),
        }
    }

    fn make_contact(byte: u8, lastmod: u32) -> DiscoveredContact {
        DiscoveredContact {
            pub_key: [byte; 32],
            callsign: String::new(),
            node_id: [byte; 4],
            last_seen_epoch: lastmod,
            rssi: 0,
            snr: 0,
            latitude: 0,
            longitude: 0,
            flags: 0,
            contact_type: 1,
            out_path: [0u8; 64],
            out_path_len: 0xFF,
            last_advert_ts: 0,
            saved_hash: 0,
            sync_since: 0,
        }
    }

    #[test]
    fn get_contacts_since_filters_older_records_but_reports_total_count() {
        let _guard = crate::state::TEST_LOCK
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        crate::state::DISCOVERED_CONTACTS.lock(|c| {
            let mut list = c.borrow_mut();
            list.clear();
            let _ = list.push(make_contact(1, 0));
            let _ = list.push(make_contact(2, 100));
            let _ = list.push(make_contact(3, 250));
        });

        let mut session = CompanionSession::new("n", "b", 0, [1; 32], [2; 64], &DummyHost);
        let mut link = CaptureLink { frames: std::vec::Vec::new() };

        // since == 0 returns all 3 contacts.
        poll_ready(session.handle_frame(&[protocol::CMD_GET_CONTACTS], &mut link));
        assert_eq!(link.frames.len(), 5);
        assert_eq!(link.frames[0][0], contacts::RESP_CODE_CONTACTS_START);
        assert_eq!(u32::from_le_bytes(link.frames[0][1..5].try_into().unwrap()), 3);
        assert_eq!(link.frames[4][0], contacts::RESP_CODE_END_OF_CONTACTS);
        assert_eq!(u32::from_le_bytes(link.frames[4][1..5].try_into().unwrap()), 250);

        // since == 100 returns only contact 3 (lastmod 250 > 100), while start carries total count 3.
        link.frames.clear();
        let mut req = [protocol::CMD_GET_CONTACTS, 0, 0, 0, 0];
        req[1..5].copy_from_slice(&100u32.to_le_bytes());
        poll_ready(session.handle_frame(&req, &mut link));
        assert_eq!(link.frames.len(), 3);
        assert_eq!(u32::from_le_bytes(link.frames[0][1..5].try_into().unwrap()), 3);
        assert_eq!(link.frames[1][1], 3);
        assert_eq!(u32::from_le_bytes(link.frames[2][1..5].try_into().unwrap()), 250);
    }

    #[test]
    fn sync_next_message_preserves_frames_above_200_bytes() {
        let mut sender = String::<32>::new();
        for _ in 0..32 {
            let _ = sender.push('S');
        }
        let mut text = String::<160>::new();
        for _ in 0..160 {
            let _ = text.push('M');
        }
        let msg = crate::types::ReceivedMessage {
            channel_idx: 1,
            path_len: 0xFF,
            sender,
            sender_pub: [0u8; 32],
            text,
            timestamp: 1_700_000_000,
            rssi: -80,
            snr: 7,
            is_direct: false,
            txt_type: 0,
            signed_prefix: [0u8; 4],
        };

        crate::state::MESSAGE_LOG.lock(|m| {
            let mut list = m.borrow_mut();
            list.clear();
            let _ = list.push(crate::types::InboxItem::Text(msg));
        });

        let mut session = CompanionSession::new("n", "b", 0, [1; 32], [2; 64], &DummyHost);
        let mut link = CaptureLink {
            frames: std::vec::Vec::new(),
        };

        poll_ready(session.handle_frame(&[protocol::CMD_SYNC_NEXT_MESSAGE], &mut link));
        assert_eq!(link.frames.len(), 1);
        let frame = &link.frames[0];
        // 11-byte V3 header + 32-byte sender + ": " + 160-byte text = 205 bytes.
        assert_eq!(frame.len(), 205);
        assert_eq!(frame[0], protocol::RESP_CODE_CHANNEL_MSG_V3);
        assert!(frame.ends_with(b": ".as_slice()) == false);
        assert_eq!(&frame[11..43], &[b'S'; 32]);
        assert_eq!(&frame[43..45], b": ");
        assert_eq!(&frame[45..205], &[b'M'; 160]);

        let mut usb_out = [0u8; crate::companion::framing::MAX_USB_FRAME_SIZE];
        let usb_len = crate::companion::framing::encode_usb_frame(frame, &mut usb_out).unwrap();
        assert_eq!(usb_len, 208);
    }

    #[test]
    fn feed_byte_accepts_usb_frame_above_176_bytes() {
        let mut session = CompanionSession::new("n", "b", 0, [1; 32], [2; 64], &DummyHost);
        let mut link = CaptureLink {
            frames: std::vec::Vec::new(),
        };

        // Start signing session, then feed a 200-byte CMD_SIGN_DATA USB frame (>176 B).
        poll_ready(session.handle_frame(&[protocol::CMD_SIGN_START], &mut link));
        assert_eq!(link.frames.len(), 1);
        assert_eq!(link.frames[0][0], protocol::RESP_CODE_SIGN_START);
        link.frames.clear();

        let mut payload = [0x5Au8; 200];
        payload[0] = protocol::CMD_SIGN_DATA;
        let mut framed = [0u8; crate::companion::framing::MAX_USB_FRAME_SIZE];
        let frame_len = crate::companion::framing::encode_usb_frame(&payload, &mut framed).unwrap();
        framed[0] = USB_INBOUND_DELIMITER;

        for &b in &framed[..frame_len] {
            poll_ready(session.feed_byte(b, &mut link));
        }

        assert_eq!(link.frames.len(), 1);
        assert_eq!(link.frames[0], &[protocol::RESP_CODE_OK]);
    }
}

