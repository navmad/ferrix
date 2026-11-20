//! Platform-independent Ferrix application: MeshCore engine, companion
//! command handling, transmit scheduling, storage formats and dashboard state.
//!
//! This crate must not depend on a platform, board or executor.
#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), forbid(unsafe_code))]
#![cfg_attr(test, deny(unsafe_code))]

pub mod airtime;
pub mod battery;
pub mod command;
pub mod companion;
pub mod engine;
pub mod gps;
pub mod scheduler;
pub mod state;
pub mod storage;
pub mod types;
pub mod version;

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

