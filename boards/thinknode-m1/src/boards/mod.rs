//! Generic Board Support Layer for ferrix
//!
//! Provides a unified interface for nRF52840 mesh boards, mapping
//! their hardware pinouts, radio transceivers, displays, sensors, and telemetry.

#[cfg(feature = "thinknode-m1")]
pub mod thinknode_m1;

// Compile-time check: Ensure at least one board is selected
#[cfg(not(feature = "thinknode-m1"))]
compile_error!("No target board selected! Please build with a board feature flag, e.g. --features thinknode-m1");

// Active Board Selection
#[cfg(feature = "thinknode-m1")]
pub use thinknode_m1::{
    hardware_seed, BoardPins as CurrentBoardPins, ThinkNodeM1 as CurrentBoard,
};

/// Common Board Trait defining board metadata, capabilities, and peripheral initialization
pub trait Board {
    /// Human-readable board name for MeshCore advertisements & logging
    const NAME: &'static str;

    /// Compile-time subsystem capability flags
    const HAS_DISPLAY: bool;
    const HAS_GPS: bool;
    const HAS_BATTERY_ADC: bool;

    /// MeshCore capability bitmask (e.g. CAP_CLIENT | CAP_EPD | CAP_BLE)
    const CAPABILITIES: u32;

    /// Specific board pinout & peripheral structure
    type Pins;

    /// Initialize the board pins and peripherals from Embassy nRF peripherals
    fn init(p: embassy_nrf::Peripherals) -> Self::Pins;
}

