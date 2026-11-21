#![no_std]
#![no_main]
#![forbid(unsafe_code)]

//! Ferrix companion firmware for the Seeed Wio Tracker L1 Pro.
//!
//! The board crate owns the hardware; the role crate owns the companion
//! startup and application loop. This image only selects the pair.

use defmt_rtt as _;
use embassy_executor::Spawner;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let peripherals = ferrix_nrf52::startup::init_with(ferrix_wio_tracker_l1_pro::STARTUP_CONFIG);
    let board = ferrix_wio_tracker_l1_pro::WioTrackerBoard::new(peripherals);
    ferrix_role_companion::run(board, spawner).await;
}
