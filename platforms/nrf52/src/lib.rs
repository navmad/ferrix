//! Shared nRF52 platform support: SoftDevice flash, reset, hardware identity,
//! storage/GNSS/battery task bodies and the companion/CLI/engine host impls.
//!
//! Board crates keep their pins, devices and renderers; this crate holds the
//! parts that every nRF52 board would otherwise duplicate.

#![cfg_attr(not(test), no_std)]

pub mod battery;
pub mod ble;
pub mod buzzer;
pub mod fault;
pub mod flash;
pub mod gnss;
pub mod host;
pub mod qspi;
pub mod reset;
mod sd_ffi;
pub mod seed;
pub mod startup;
pub mod storage;
pub mod usb;
pub mod watchdog;
