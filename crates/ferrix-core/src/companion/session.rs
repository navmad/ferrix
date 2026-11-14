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

const MAX_RESPONSES: usize = 10;
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
        if payload.first() == Some(&protocol::CMD_GET_CONTACTS) {
            self.stream_contacts(link).await;
            return;
        }

        let mut responses: Vec<Vec<u8, 200>, MAX_RESPONSES> = Vec::new();
        protocol::handle_command(&mut self.node, payload, self.host, |resp| {
            let mut out = Vec::new();
            let _ = out.extend_from_slice(&resp[..resp.len().min(200)]);
            let _ = responses.push(out);
        })
        .await;

        for response in responses.iter() {
            link.write_frame(response).await;
        }
    }

    /// Stream the contact list through the link.
    pub async fn stream_contacts<L: CompanionLink>(&mut self, link: &mut L) {
        let count = protocol::contacts_count();
        link.write_frame(&protocol::encode_contacts_start(count)).await;

        let mut max_lastmod = 0u32;
        for idx in 0..count as usize {
            let Some(c) = protocol::contact_at(idx) else {
                continue;
            };
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

