//! Shared application data types.
//!
//! These are transport- and hardware-independent: contacts, inbox entries,
//! radio settings and telemetry counters.

use heapless::String;

use meshcore_wire::packets::grpdata::MAX_GROUP_DATA;

pub const MAX_CONTACTS: usize = 350;
pub const MAX_CHANNELS: usize = 40;
pub const MAX_MESSAGES: usize = 256;

/// Discovered MeshCore contact.
#[derive(Clone, Debug)]
pub struct DiscoveredContact {
    pub pub_key: [u8; 32],
    pub callsign: String<32>,
    pub node_id: [u8; 4],
    pub last_seen_epoch: u32,
    pub rssi: i16,
    pub snr: i8,
    pub latitude: i32,
    pub longitude: i32,
    pub flags: u8,
    pub contact_type: u8,
    pub out_path: [u8; 64],
    /// `0xFF` means unknown; otherwise a validated encoded path byte.
    pub out_path_len: u8,
    pub last_advert_ts: u32,
    /// Hash of the fields last written to flash (0 = never saved).
    pub saved_hash: u32,
    /// Newest signed-message timestamp accepted from this contact.
    pub sync_since: u32,
}

/// Inbox text message.
#[derive(Clone, Debug)]
pub struct ReceivedMessage {
    pub channel_idx: u8,
    pub path_len: u8,
    pub sender: String<32>,
    /// Full sender public key for direct messages; zero for channel messages.
    pub sender_pub: [u8; 32],
    pub text: String<160>,
    pub timestamp: u32,
    pub rssi: i16,
    pub snr: i8,
    pub is_direct: bool,
    /// MeshCore text type (`0` plain, `1` CLI data, `2` signed, `3` CLI command).
    pub txt_type: u8,
    /// Signing-key prefix for signed messages; zero otherwise.
    pub signed_prefix: [u8; 4],
}

/// Received channel datagram (`GRP_DATA`).
#[derive(Clone, Debug)]
pub struct ReceivedChannelData {
    pub channel_idx: u8,
    /// `0xFF` when the packet did not arrive over a flood route.
    pub path_len: u8,
    pub snr: i8,
    pub data_type: u16,
    pub data: heapless::Vec<u8, MAX_GROUP_DATA>,
}

/// One entry in the companion inbox. Text messages and channel datagrams are
/// kept in a single FIFO so `CMD_SYNC_NEXT_MESSAGE` preserves receive order.
#[derive(Clone, Debug)]
pub enum InboxItem {
    Text(ReceivedMessage),
    ChannelData(ReceivedChannelData),
}

#[derive(Clone, Copy, Debug)]
pub struct RadioSettings {
    pub frequency_hz: u32,
    pub tx_power_dbm: i8,
    pub sf: u8,
    pub bw_khz: u32,
    pub cr: u8,
    /// Reserved for the dedicated repeater role. The companion role ignores it
    /// and forwards only through client repeat on off-grid frequencies.
    pub is_repeater: bool,
    /// Off-grid "client repeat" mode (`client_repeat`).
    pub client_repeat: bool,
    /// Frequency to return to when off-grid mode is turned off.
    pub offgrid_home_freq: u32,
    pub path_hash_mode: u8,
    /// Listen in duty-cycle/sniff mode instead of continuous RX.
    pub rx_duty_cycle: bool,
    /// Transmit airtime factor in tenths (0..90; af=90 -> 10% duty).
    pub airtime_factor_tenths: u8,
}

/// Contact auto-add policy.
#[derive(Clone, Copy, Debug)]
pub struct AutoAddSettings {
    pub manual_add_contacts: u8,
    pub config: u8,
    pub max_hops: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct ChannelData {
    pub name: [u8; 32],
    pub secret: [u8; 16],
}

#[derive(Clone, Copy, Debug)]
pub struct RadioTelemetry {
    pub tx_beacons: u32,
    pub rx_packets: u32,
    pub last_rssi: i16,
    pub last_snr: i8,
    pub has_rx: bool,
    pub tx_air_ms: u32,
    pub rx_air_ms: u32,
    pub tx_wait_max_ms: u32,
    pub tx_control: u32,
    pub tx_flood: u32,
    pub tx_background: u32,
    pub tx_flood_sent: u32,
    pub tx_direct: u32,
    pub rx_flood: u32,
    pub rx_direct: u32,
    pub rx_errors: u32,
    pub tx_stale: u32,
    pub tx_cancelled: u32,
    pub tx_retries: u32,
    pub tx_dropped: u32,
    pub rx_dropped: u32,
    /// Live estimated idle noise floor in dBm (`0` = unseeded sentinel; use
    /// [`Self::effective_noise_floor_dbm`] for reporting).
    pub noise_floor_dbm: i16,
    /// Consecutive identical idle RSSI samples seen by the AGC watchdog.
    pub agc_frozen_streak: u8,
    /// Total SX1262 stuck-AGC recoveries completed since boot or `clear stats`.
    pub agc_resets: u32,
}

impl RadioTelemetry {
    /// Return the live estimated noise floor, falling back to `-118 dBm` before
    /// the first valid idle RSSI sample is captured.
    pub fn effective_noise_floor_dbm(&self) -> i16 {
        if self.noise_floor_dbm == 0 {
            crate::noise_floor::DEFAULT_NOISE_FLOOR_DBM
        } else {
            self.noise_floor_dbm
        }
    }
}
