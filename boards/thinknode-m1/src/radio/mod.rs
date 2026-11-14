//! Generic LoRa Radio Subsystem & Transceiver Drivers

pub mod commands;
pub mod lora_phy;
pub mod sx1262;

pub use lora_phy::{lora_phy_task, LoraPhyChannelRadio};
pub use sx1262::Sx1262;

pub use ferrix_core::airtime::estimate_airtime_ms;
pub use ferrix_core::scheduler::{tx_budget_max, tx_duty_pct};
pub use ferrix_core::state::{
    clear_duplicate_deltas, duplicate_deltas, record_duplicate_delta, FORWARD_SCHEDULED,
    FORWARD_SENT, TX_SLOT_MS_LAST, TX_SLOT_MS_OVERRIDE, TX_SLOT_USED,
};
pub use ferrix_hal::radio::{LoraConfig, LoraRadio};
