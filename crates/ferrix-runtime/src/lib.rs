//! Generic task wiring for Ferrix: radio scheduler, storage, companion
//! transports, input and display. Boards call [`storage::storage_loop`] with
//! their flash device.
#![cfg_attr(not(test), no_std)]

pub mod storage;
