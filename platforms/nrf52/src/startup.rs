//! Common nRF52 boot sequence for SoftDevice-based boards.
//!
//! Every board using the S140 SoftDevice has to reserve interrupt priorities
//! and capture the reset reason before anything else runs. Keeping that here
//! means a new board only supplies pins and device configuration.

use embassy_nrf::interrupt::{InterruptExt, Priority};

/// Initialise the nRF peripherals with SoftDevice-safe interrupt priorities.
///
/// The S140 stack reserves priorities 0, 1 and 4, so Embassy system and
/// peripheral interrupts are placed on 2 and 3.
pub fn init() -> embassy_nrf::Peripherals {
    let mut config = embassy_nrf::config::Config::default();
    config.gpiote_interrupt_priority = Priority::P2;
    config.time_interrupt_priority = Priority::P2;
    let peripherals = embassy_nrf::init(config);

    embassy_nrf::interrupt::USBD.set_priority(Priority::P3);
    embassy_nrf::interrupt::TWISPI0.set_priority(Priority::P3);
    embassy_nrf::interrupt::TWISPI1.set_priority(Priority::P3);
    embassy_nrf::interrupt::SAADC.set_priority(Priority::P3);
    embassy_nrf::interrupt::QSPI.set_priority(Priority::P3);
    embassy_nrf::interrupt::UARTE0.set_priority(Priority::P3);

    // POWER.RESETREAS is sticky; capture it before a later reset can change it.
    ferrix_core::state::RESET_REASON.store(
        reset_reason(),
        core::sync::atomic::Ordering::Relaxed,
    );

    peripherals
}

/// Read `POWER.RESETREAS` (0x4000_0400).
pub fn reset_reason() -> u32 {
    unsafe { core::ptr::read_volatile(0x4000_0400 as *const u32) }
}
