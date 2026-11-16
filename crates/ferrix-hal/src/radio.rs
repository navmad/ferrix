//! Radio hardware abstractions shared by the runtime and board drivers.

use meshcore_wire::tx::TxLane;

/// Convert the compact kHz setting to the exact bandwidth in Hz.
///
/// The narrow bandwidths are not exact kilohertz, so this table must be used
/// for airtime and duty-cycle math instead of multiplying by 1000.
pub fn bw_khz_to_hz(bw_khz: u32) -> u32 {
    match bw_khz {
        7 => 7_800,
        10 => 10_400,
        15 => 15_600,
        20 => 20_800,
        31 => 31_250,
        41 => 41_670,
        62 => 62_500,
        other => other * 1000,
    }
}

/// Standard LoRa modulation and RF configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
            frequency_hz: 910_525_000,
            tx_power_dbm: 22,
            spreading_factor: 7,
            bandwidth_khz: 62,
            coding_rate: 5,
            preamble_length: 32,
            rx_duty_cycle: false,
        }
    }
}

/// Result of one receive attempt.
#[derive(Clone, Copy, Debug)]
pub enum RxOutcome {
    Packet { len: usize, rssi: i16, snr: i8 },
    /// Duty-cycle receive window ended without a preamble; already re-armed.
    DutyCycleEnd,
    /// Radio error; the caller should back off and recover receive mode.
    Error,
}

/// Result of one transmit attempt.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TxOutcome {
    Sent,
    Failed,
}

/// A physical LoRa radio driven by the runtime scheduler.
#[allow(async_fn_in_trait)]
pub trait RadioDevice {
    /// Apply new modulation settings and prepare reception.
    async fn configure(&mut self, config: &LoraConfig) -> bool;

    /// Transmit one packet, including timeout and RX recovery.
    async fn transmit(&mut self, payload: &[u8]) -> TxOutcome;

    /// Enter receive mode if not already armed. No-op when already listening
    /// so duty-cycle sleep/wake phase is preserved across loop wakeups.
    async fn arm_receive(&mut self);

    /// Cancel-safe wait for a radio RX interrupt (must not perform SPI I/O).
    async fn wait_rx_irq(&mut self) -> bool;

    /// Process the pending RX interrupt to completion (never cancel mid-flight).
    /// Returns `None` for intermediate IRQs (preamble/header) or ignored errors.
    async fn handle_rx_irq(&mut self, buf: &mut [u8]) -> Option<RxOutcome>;

    /// True while a preamble/header was detected and the frame has not finished yet.
    fn is_receiving(&self) -> bool {
        false
    }

    /// Re-arm reception after a watchdog timeout or error.
    async fn recover_receive(&mut self);
}

/// Generic channel-driven LoRa radio interface used by the mesh engine.
#[allow(async_fn_in_trait)]
pub trait LoraRadio {
    type Error: core::fmt::Debug;

    async fn init(&mut self, config: &LoraConfig) -> Result<(), Self::Error>;

    async fn transmit(&mut self, payload: &[u8]) -> Result<(), Self::Error>;

    async fn transmit_lane(
        &mut self,
        payload: &[u8],
        lane: TxLane,
        queued_at_ms: u64,
        hash: [u8; 8],
    ) -> Result<(), Self::Error>;

    async fn start_receive(&mut self);

    async fn poll_receive(&mut self, buf: &mut [u8]) -> Option<(usize, i16, i8)>;

    async fn receive(&mut self, buf: &mut [u8]) -> (usize, i16, i8);
}
