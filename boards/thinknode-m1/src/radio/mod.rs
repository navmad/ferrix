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

/// Estimate LoRa airtime in milliseconds (preamble + header + payload + CRC).
pub fn estimate_airtime_ms(
    len: usize,
    sf: u8,
    bw_khz: u32,
    cr: u8,
    preamble: u16,
) -> u32 {
    let bw_hz = crate::companion::usb::bw_khz_to_hz(bw_khz) as u64;
    if bw_hz == 0 {
        return 0;
    }
    let sym_ns = ((1u64 << sf) * 1_000_000_000) / bw_hz;
    let de = if sym_ns > 16_380_000 { 1u64 } else { 0 };
    let num = (8u64 * len as u64).saturating_sub(4 * sf as u64) + 44;
    let den = (4 * (sf as u64 - 2 * de)).max(4);
    let nibbles = (num + den - 1) / den;
    let payload_symbols = 8 + nibbles * cr as u64;
    let preamble_ns = (preamble as u64 * 4 + 17) * sym_ns / 4;
    let total_ns = preamble_ns + payload_symbols * sym_ns;
    ((total_ns + 999_999) / 1_000_000) as u32
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

/// Duty-cycle percentage implied by the airtime factor setting.
pub fn tx_duty_pct(af_tenths: u8) -> i64 {
    1000 / (af_tenths as i64 + 10)
}

/// Airtime budget for a one-hour duty-cycle window.
pub fn tx_budget_max(af_tenths: u8) -> i64 {
    3_600_000 * tx_duty_pct(af_tenths) / 100
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
