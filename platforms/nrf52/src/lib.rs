//! Shared nRF52 platform support: SoftDevice flash, reset and hardware identity.
//!
//! Board crates keep their pins, devices and renderers; this crate holds the
//! parts that every nRF52 board would otherwise duplicate.

#![cfg_attr(not(test), no_std)]

pub mod ble;
pub mod flash;
pub mod qspi;
pub mod reset;
pub mod seed;
pub mod startup;
pub mod usb;
pub mod watchdog;
