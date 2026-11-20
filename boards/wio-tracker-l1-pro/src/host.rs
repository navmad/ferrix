//! Board bindings for the companion protocol, text CLI and routing engine.
//!
//! The implementations are shared by every nRF52 board in `ferrix-nrf52`; this
//! module only supplies the Wio Tracker L1 Pro's identity line and storage profile.

use ferrix_nrf52::host::{NrfCompanionHost, NrfEngineContext};

pub use ferrix_nrf52::host::set_leds_disabled;

/// Board implementation of the protocol's persistence/telemetry interface.
pub static BOARD_HOST: NrfCompanionHost = NrfCompanionHost::new(
    "Wio Tracker L1 Pro (nRF52840, SX1262 LoRa)",
    cfg!(not(feature = "internal-storage")),
);

/// Routing engine host backed by the shared state.
pub static BOARD_ENGINE_CONTEXT: NrfEngineContext = NrfEngineContext;
