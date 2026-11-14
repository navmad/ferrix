//! MeshCore radio task for ThinkNode M1.
//!
//! The chip mechanics live in `ferrix-sx1262` and the channel bridge lives in
//! `ferrix-runtime`. This module only owns the board wiring: SPI device, power
//! rail and control pins.

use defmt::{error, info};
use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::spim::Spim;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;

use ferrix_core::scheduler::preamble_for;
use ferrix_hal::radio::LoraConfig;
use ferrix_sx1262::{Sx1262BoardConfig, Sx1262Radio, TcxoCtrlVoltage};

pub use ferrix_runtime::radio::{cancel_flood, request_radio_config};

/// Standalone Embassy task managing the SX1262 radio.
#[embassy_executor::task]
pub async fn sx1262_task(
    spim: Spim<'static, TWISPI0>,
    cs: Output<'static>,
    reset: Output<'static>,
    busy: Input<'static>,
    dio1: Input<'static>,
    mut power_en: Output<'static>,
) {
    info!("Starting SX1262 radio task on ThinkNode M1...");

    // Ensure SX126X_POWER_EN (P0.21) is continuously HIGH.
    power_en.set_high();

    // Wrap Spim & CS into ExclusiveDevice and hand the bus to the driver.
    let spi_device = ExclusiveDevice::new(spim, cs, Delay).unwrap();

    let cfg = match lora_config_from_state() {
        Some(cfg) => cfg,
        None => return,
    };

    // ThinkNode M1 uses internal DIO2 RF switch control, so rf_switch is None.
    let board = Sx1262BoardConfig {
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V8),
        use_dcdc: true,
        rx_boost: true,
    };

    let device = match Sx1262Radio::new(spi_device, reset, dio1, busy, Delay, board, cfg).await {
        Ok(device) => device,
        Err(e) => {
            error!("sx1262: failed to initialise radio: {:?}", e);
            return;
        }
    };

    ferrix_runtime::radio::radio_task(device, cfg).await;
}

fn lora_config_from_state() -> Option<LoraConfig> {
    Some(ferrix_core::state::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        LoraConfig {
            frequency_hz: s.frequency_hz,
            tx_power_dbm: s.tx_power_dbm,
            spreading_factor: s.sf,
            bandwidth_khz: s.bw_khz,
            coding_rate: s.cr,
            preamble_length: preamble_for(s.sf),
            rx_duty_cycle: s.rx_duty_cycle,
        }
    }))
}
