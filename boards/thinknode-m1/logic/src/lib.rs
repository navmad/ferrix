//! Host-testable logic of the ThinkNode M1 board.
//!
//! Everything here is free of embassy-nrf/SoftDevice types so it builds and
//! runs on the host. The board crate (`ferrix-thinknode-m1`) owns the hardware
//! glue and re-exports these modules.
#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), forbid(unsafe_code))]
#![cfg_attr(test, deny(unsafe_code))]

#[cfg(test)]
extern crate std;

pub mod config;
pub mod framebuffer;
pub mod input;
pub mod layout;
pub mod power;
pub mod radio;
pub mod sound;
pub mod ui;

#[cfg(test)]
#[allow(unsafe_code)]
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
