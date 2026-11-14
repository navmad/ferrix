//! Generic LoRa Radio Subsystem & Transceiver Drivers

use core::sync::atomic::{AtomicU32, Ordering};

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

pub use ferrix_core::airtime::estimate_airtime_ms;

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

/// Flood calendar slot width override. Zero means derive it from the
/// modem's airtime at runtime.
pub static TX_SLOT_MS_OVERRIDE: AtomicU32 = AtomicU32::new(0);
/// Effective slot width and occupancy, for telemetry.
pub static TX_SLOT_MS_LAST: AtomicU32 = AtomicU32::new(0);
pub static TX_SLOT_USED: AtomicU32 = AtomicU32::new(0);

/// Forwarding counters used to derive the spreading window on a live mesh.
pub static FORWARD_SCHEDULED: AtomicU32 = AtomicU32::new(0);
pub static FORWARD_SENT: AtomicU32 = AtomicU32::new(0);

const DUP_BUCKETS: [AtomicU32; 6] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// Record how long after enqueueing a forward another copy was heard.
pub fn record_duplicate_delta(ms: u64) {
    let bucket = match ms {
        0..=19 => 0,
        20..=39 => 1,
        40..=79 => 2,
        80..=159 => 3,
        160..=319 => 4,
        _ => 5,
    };
    DUP_BUCKETS[bucket].fetch_add(1, Ordering::Relaxed);
}

pub fn duplicate_deltas() -> [u32; 6] {
    [
        DUP_BUCKETS[0].load(Ordering::Relaxed),
        DUP_BUCKETS[1].load(Ordering::Relaxed),
        DUP_BUCKETS[2].load(Ordering::Relaxed),
        DUP_BUCKETS[3].load(Ordering::Relaxed),
        DUP_BUCKETS[4].load(Ordering::Relaxed),
        DUP_BUCKETS[5].load(Ordering::Relaxed),
    ]
}

pub fn clear_duplicate_deltas() {
    for bucket in &DUP_BUCKETS {
        bucket.store(0, Ordering::Relaxed);
    }
}

pub use ferrix_core::scheduler::{tx_budget_max, tx_duty_pct};

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

    /// Queue a packet in a transmit lane. Control traffic is sent at the next
    /// opportunity; floods are scheduled into the slot calendar.
    async fn transmit_lane(
        &mut self,
        payload: &[u8],
        lane: meshcore_wire::tx::TxLane,
        queued_at_ms: u64,
        hash: [u8; 8],
    ) -> Result<(), Self::Error>;

    /// Enter continuous reception mode
    async fn start_receive(&mut self);

    /// Poll for an incoming packet.
    /// Returns `Some((payload_len, rssi_dbm, snr_db))` if a valid packet arrived.
    async fn poll_receive(&mut self, buf: &mut [u8]) -> Option<(usize, i16, i8)>;

    /// Wait for the next packet without polling.
    /// Returns `(payload_len, rssi_dbm, snr_db)`.
    async fn receive(&mut self, buf: &mut [u8]) -> (usize, i16, i8);
}
