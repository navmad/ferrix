//! Platform-independent Ferrix application: MeshCore engine, companion
//! command handling, transmit scheduling, storage formats and dashboard state.
//!
//! This crate must not depend on a platform, board or executor.
#![cfg_attr(not(test), no_std)]
