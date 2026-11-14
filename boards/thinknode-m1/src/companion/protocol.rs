//! Board binding for the companion protocol now owned by `ferrix-core`.
//!
//! The command parser and response encoders live in
//! [`ferrix_core::companion::protocol`]. This module supplies the board
//! services it needs and keeps the local CLI dispatched by the USB task.

pub use ferrix_core::companion::protocol::{CompanionHost, CompanionNode};
use heapless::Vec;

pub use ferrix_core::companion::protocol::*;
pub(crate) use ferrix_core::companion::contacts::serialize_contact;

/// Board implementation of the protocol's persistence/telemetry interface.
pub struct BoardCompanionHost;

pub static BOARD_HOST: BoardCompanionHost = BoardCompanionHost;

impl CompanionHost for BoardCompanionHost {
    fn battery_millivolts(&self) -> u16 {
        crate::telemetry::battery_millivolts()
    }

    fn set_callsign(&self, name: &str) {
        crate::storage::set_callsign(name);
    }

    fn set_custom_priv_key(&self, key: [u8; 64]) {
        crate::storage::set_custom_priv_key(key);
    }

    fn set_path_hash_mode(&self, mode: u8) {
        crate::storage::set_path_hash_mode(mode);
    }

    fn request_save_config(&self) {
        crate::storage::request_save_config();
    }

    fn request_radio_config(&self) {
        crate::radio::lora_phy::request_radio_config();
    }

    fn request_factory_reset(&self) {
        crate::storage::request_factory_reset();
    }

    fn queue_consume_now(&self) {
        crate::storage::messages::queue_consume_now();
    }

    fn advert_remember(&self, pub_key: &[u8; 32], payload: &[u8]) {
        crate::storage::adverts::remember(pub_key, payload);
    }

    fn advert_payload(&self, pub_key: &[u8; 32]) -> Option<Vec<u8, 132>> {
        crate::storage::adverts::payload(pub_key)
    }

    fn advert_export(&self, pub_key: &[u8; 32]) -> Option<Vec<u8, 134>> {
        crate::storage::adverts::export(pub_key)
    }

    fn reboot(&self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

/// Forward deferred protocol pushes to the connected companion transport.
#[embassy_executor::task]
pub async fn delayed_push_task() -> ! {
    loop {
        let frame = ferrix_core::companion::protocol::next_delayed_push().await;
        let _ = ferrix_core::command::COMPANION_PUSH_CHANNEL.try_send(frame);
    }
}

/// Dispatch a companion frame, keeping the local CLI on the board side.
///
/// `CMD_RUN_CLI_COMMAND` needs the full text CLI (display, GPS, DFU and radio
/// diagnostics), so the transport intercepts it here before the core parser.
pub async fn handle_command<F: FnMut(&[u8])>(
    node: &mut CompanionNode<'_>,
    payload: &[u8],
    mut emit: F,
) {
    if payload.first() == Some(&CMD_RUN_CLI_COMMAND) {
        let line = core::str::from_utf8(&payload[1..])
            .unwrap_or("")
            .trim_end_matches('\0');
        let mut out = heapless::String::<512>::new();
        crate::companion::usb::cli_reply(line, node, &mut out).await;
        let mut resp =
            Vec::<u8, { ferrix_core::companion::framing::MAX_FRAME_PAYLOAD_SIZE }>::new();
        let _ = resp.push(PACKET_CLI_REPLY);
        let bytes = out.as_bytes();
        let max = ferrix_core::companion::framing::MAX_FRAME_PAYLOAD_SIZE - 1;
        let _ = resp.extend_from_slice(&bytes[..bytes.len().min(max)]);
        emit(&resp);
    } else {
        node.handle_command(payload, emit).await;
    }
}
