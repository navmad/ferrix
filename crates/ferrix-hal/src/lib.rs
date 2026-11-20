//! Hardware abstraction traits shared by the Ferrix runtime, platforms and
//! boards. No platform or application code belongs here.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

pub mod radio;

pub use radio::{LoraConfig, LoraRadio, RadioDevice, RxOutcome, RxState, TxOutcome};
