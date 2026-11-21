//! Host-testable logic of the Wio Tracker L1 Pro board.
//!
//! Everything here is free of embassy-nrf/SoftDevice types so it builds and
//! runs on the host. The board crate (`ferrix-wio-tracker-l1-pro`) owns the
//! hardware glue and re-exports these modules.
#![cfg_attr(not(test), no_std)]

#[cfg(test)]
extern crate std;

pub mod config;
pub mod input;
pub mod layout;
pub mod power;
pub mod radio;
pub mod sound;
pub mod ui;

#[cfg(test)]
mod defmt_test_stubs {
    #[no_mangle]
    extern "C" fn _defmt_acquire() {}
    #[no_mangle]
    extern "C" fn _defmt_release() {}
    #[no_mangle]
    extern "C" fn _defmt_write(_bytes: *const u8, _len: usize) {}
    #[no_mangle]
    extern "C" fn _defmt_timestamp() {}
    #[no_mangle]
    extern "C" fn _defmt_panic() -> ! {
        panic!("defmt panic in host test")
    }
    #[no_mangle]
    extern "C" fn __pender(_context: *mut ()) {}
}
