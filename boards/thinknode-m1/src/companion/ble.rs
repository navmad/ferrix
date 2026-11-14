//! Board-facing re-exports for the platform BLE transport.
//!
//! The SoftDevice, GATT services and advertising task live in
//! `platforms/nrf52`; the board supplies its identity, CLI host and fault
//! indicator when starting it.

pub use ferrix_core::command::{
    ContactOperation, COMPANION_PUSH_CHANNEL, CONTACT_OP_CHANNEL,
};
pub use ferrix_core::state::{BLE_CONNECTED, BLE_PIN, BLE_STATUS};
pub use ferrix_nrf52::ble::{contact_task, init_ble_companion};
