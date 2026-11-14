//! Generic task wiring for Ferrix: radio scheduler, storage, companion
//! transports, input and display. Boards call [`run`] with their devices.
#![cfg_attr(not(test), no_std)]
