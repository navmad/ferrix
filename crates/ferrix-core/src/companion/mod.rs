//! Companion protocol: transport framing and command handling.
//!
//! Framing is platform-independent. Command handling migrates here next; it
//! currently lives in the board crate while the storage and radio interfaces
//! are still being inverted.

pub mod contacts;
pub mod framing;
