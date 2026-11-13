//! Generic LoRa Radio Subsystem & Transceiver Drivers

pub mod commands;
pub mod lora_phy;
pub mod sx1262;

pub use lora_phy::{lora_phy_task, LoraPhyChannelRadio};
pub use sx1262::Sx1262;

/// Standard LoRa Modulation & RF Configuration
#[derive(Clone, Copy, Debug)]
pub struct LoraConfig {
    pub frequency_hz: u32,
    pub tx_power_dbm: i8,
    pub spreading_factor: u8,
    pub bandwidth_khz: u32,
    pub coding_rate: u8,
    pub preamble_length: u16,
    /// Listen in duty-cycle/sniff mode instead of continuous RX.
    pub rx_duty_cycle: bool,
}

impl Default for LoraConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 910_525_000, // US PNW (Puget / Portland / Cascadia) MeshCore default
            tx_power_dbm: 22,
            spreading_factor: 7,
            bandwidth_khz: 62,
            coding_rate: 5, // CR 4/5 (denominator, matching RADIO_SETTINGS)
            preamble_length: 16,
            rx_duty_cycle: false,
        }
    }
}

/// Generic Asynchronous LoRa Radio Trait
///
/// Implemented by Semtech transceiver drivers (SX1262, LR1110, LR1121, SX1276, etc.)
#[allow(async_fn_in_trait)]
pub trait LoraRadio {
    type Error: core::fmt::Debug;

    /// Initialize the radio with given parameters
    async fn init(&mut self, config: &LoraConfig) -> Result<(), Self::Error>;

    /// Transmit a packet asynchronously
    async fn transmit(&mut self, payload: &[u8]) -> Result<(), Self::Error>;

    /// Enter continuous reception mode
    async fn start_receive(&mut self);

    /// Poll for an incoming packet.
    /// Returns `Some((payload_len, rssi_dbm, snr_db))` if a valid packet arrived.
    async fn poll_receive(&mut self, buf: &mut [u8]) -> Option<(usize, i16, i8)>;

    /// Wait for the next packet without polling.
    /// Returns `(payload_len, rssi_dbm, snr_db)`.
    async fn receive(&mut self, buf: &mut [u8]) -> (usize, i16, i8);
}
