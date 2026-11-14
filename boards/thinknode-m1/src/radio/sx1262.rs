//! MeshCore radio adapter for ThinkNode M1.
//!
//! The chip mechanics live in `ferrix-sx1262`. This module only owns the board
//! wiring (SPI device, power rail, control pins) and the channel bridge used by
//! the runtime scheduler.

use defmt::{error, info};
use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::spim::Spim;

use embassy_time::{Delay, Instant};
use embedded_hal_bus::spi::ExclusiveDevice;

use ferrix_core::scheduler::{preamble_for, TxPacket};
use ferrix_hal::radio::{LoraConfig, LoraRadio};
use ferrix_sx1262::{Sx1262BoardConfig, Sx1262Radio, TcxoCtrlVoltage};

/// ThinkNode M1 LoRa Radio Adapter implementing `LoraRadio` via channels.
pub struct ChannelRadio;

impl ChannelRadio {
    pub const fn new() -> Self {
        Self
    }
}

impl LoraRadio for ChannelRadio {
    type Error = ();

    async fn init(&mut self, _config: &LoraConfig) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn transmit(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        let now = Instant::now().as_millis() as u64;
        self.transmit_lane(payload, meshcore_wire::tx::TxLane::Control, now, [0; 8])
            .await
    }

    async fn transmit_lane(
        &mut self,
        payload: &[u8],
        lane: meshcore_wire::tx::TxLane,
        queued_at_ms: u64,
        hash: [u8; 8],
    ) -> Result<(), Self::Error> {
        if payload.is_empty() || payload.len() > 256 {
            return Ok(());
        }
        let mut pkt = TxPacket {
            len: payload.len(),
            data: [0u8; 256],
            lane,
            queued_at_ms,
            hash,
            attempts: 0,
        };
        pkt.data[..payload.len()].copy_from_slice(payload);
        ferrix_runtime::radio::LORA_TX_CHANNEL
            .try_send(pkt)
            .map_err(|_| ())
    }

    async fn start_receive(&mut self) {}

    async fn poll_receive(&mut self, buf: &mut [u8]) -> Option<(usize, i16, i8)> {
        if let Ok((len, rssi, snr, data)) = ferrix_runtime::radio::LORA_RX_CHANNEL.try_receive() {
            let copy_len = len.min(buf.len());
            buf[..copy_len].copy_from_slice(&data[..copy_len]);
            Some((copy_len, rssi, snr))
        } else {
            None
        }
    }

    async fn receive(&mut self, buf: &mut [u8]) -> (usize, i16, i8) {
        let (len, rssi, snr, data) = ferrix_runtime::radio::LORA_RX_CHANNEL.receive().await;
        let copy_len = len.min(buf.len());
        buf[..copy_len].copy_from_slice(&data[..copy_len]);
        (copy_len, rssi, snr)
    }
}

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
