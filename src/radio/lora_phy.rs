//! lora-phy Subsystem Adapter for ThinkNode M1 (SX1262)
//!
//! Bridges the community-standard `lora-phy` crate into Ferrix
//! using an asynchronous Embassy radio task and bounded IPC channels.

use defmt::{debug, error, info};
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::spim::Spim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::{Bandwidth, CodingRate, RxMode, SpreadingFactor};
use lora_phy::sx126x::{Config as Sx126xConfig, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;

use crate::radio::{LoraConfig, LoraRadio};

#[derive(Clone, Copy)]
pub struct TxPacket {
    pub len: usize,
    pub data: [u8; 256],
}

pub static LORA_RX_CHANNEL: Channel<CriticalSectionRawMutex, (usize, i16, i8, [u8; 256]), 4> = Channel::new();
pub static LORA_TX_CHANNEL: Channel<CriticalSectionRawMutex, TxPacket, 4> = Channel::new();

/// ThinkNode M1 LoRa Radio Adapter implementing `LoraRadio` via channels
pub struct LoraPhyChannelRadio;

impl LoraPhyChannelRadio {
    pub const fn new() -> Self {
        Self
    }
}

impl LoraRadio for LoraPhyChannelRadio {
    type Error = core::convert::Infallible;

    async fn init(&mut self, _config: &LoraConfig) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn transmit(&mut self, payload: &[u8]) -> Result<(), Self::Error> {
        if payload.is_empty() || payload.len() > 256 {
            return Ok(());
        }
        let mut pkt = TxPacket {
            len: payload.len(),
            data: [0u8; 256],
        };
        pkt.data[..payload.len()].copy_from_slice(payload);
        LORA_TX_CHANNEL.send(pkt).await;
        Ok(())
    }

    async fn start_receive(&mut self) {
        // Managed automatically in background by lora_phy_task
    }

    async fn poll_receive(&mut self, buf: &mut [u8]) -> Option<(usize, i16, i8)> {
        if let Ok((len, rssi, snr, data)) = LORA_RX_CHANNEL.try_receive() {
            let copy_len = len.min(buf.len());
            buf[..copy_len].copy_from_slice(&data[..copy_len]);
            Some((copy_len, rssi, snr))
        } else {
            None
        }
    }
}

/// Standalone Embassy task managing the SX1262 radio via `lora-phy`
#[embassy_executor::task]
pub async fn lora_phy_task(
    spim: Spim<'static, TWISPI0>,
    cs: Output<'static>,
    reset: Output<'static>,
    busy: Input<'static>,
    dio1: Input<'static>,
    mut power_en: Output<'static>,
) {
    info!("Starting lora-phy radio task on ThinkNode M1...");

    // 1. Ensure SX126X_POWER_EN (P0.21) is continuously HIGH
    power_en.set_high();

    // 2. Wrap Spim & CS into ExclusiveDevice
    let spi_device = ExclusiveDevice::new(spim, cs, Delay).unwrap();

    // 3. Build InterfaceVariant for SX1262
    // Note: ThinkNode M1 SX1262 uses internal DIO2 RF switch control, so rf_switch pins are None
    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

    // 4. Configure SX1262: 3.3V TCXO, DCDC enabled, high-power PA
    let config = Sx126xConfig {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl3V3),
        use_dcdc: true,
        rx_boost: false,
    };

    let radio_kind = Sx126x::new(spi_device, iv, config);

    // 5. Initialize LoRa instance with private network sync word (0x12 / 0x1424 for MeshCore)
    let mut lora = match LoRa::new(radio_kind, false, Delay).await {
        Ok(l) => {
            info!("lora-phy: LoRa instance initialized successfully!");
            l
        }
        Err(e) => {
            error!("lora-phy: Failed to initialize LoRa radio: {:?}", e);
            return;
        }
    };

    // 6. Modulation parameters: 910.525 MHz, SF7, BW 62.5 kHz, CR 4/5
    let mdltn_params = match lora.create_modulation_params(
        SpreadingFactor::_7,
        Bandwidth::_62KHz,
        CodingRate::_4_5,
        910_525_000,
    ) {
        Ok(p) => p,
        Err(e) => {
            error!("lora-phy: Failed to create modulation params: {:?}", e);
            return;
        }
    };

    // 7. Packet parameters for RX: 16 preamble symbols, explicit header, 255 max payload, CRC on, standard IQ
    let rx_pkt_params = match lora.create_rx_packet_params(16, false, 255, true, false, &mdltn_params) {
        Ok(p) => p,
        Err(e) => {
            error!("lora-phy: Failed to create RX packet params: {:?}", e);
            return;
        }
    };

    // 8. Prepare for continuous reception
    if let Err(e) = lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params).await {
        error!("lora-phy: Failed to prepare for RX: {:?}", e);
        return;
    }

    info!("lora-phy: Radio listening on Cascadia 910.525 MHz (SF7/BW62.5k)!");

    let mut rx_buf = [0u8; 256];

    loop {
        match select(lora.rx(&rx_pkt_params, &mut rx_buf), LORA_TX_CHANNEL.receive()).await {
            Either::First(rx_result) => {
                match rx_result {
                    Ok((len, status)) => {
                        info!("lora-phy: RX {} bytes, RSSI: {} dBm, SNR: {} dB", len, status.rssi, status.snr);
                        let mut packet_data = [0u8; 256];
                        let copy_len = (len as usize).min(256);
                        packet_data[..copy_len].copy_from_slice(&rx_buf[..copy_len]);
                        let _ = LORA_RX_CHANNEL.try_send((copy_len, status.rssi, status.snr as i8, packet_data));
                    }
                    Err(e) => {
                        debug!("lora-phy: RX error or timeout: {:?}", e);
                        // Re-enter RX continuous if an error occurred
                        let _ = lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params).await;
                    }
                }
            }
            Either::Second(tx_packet) => {
                info!("lora-phy: Transmitting {} bytes...", tx_packet.len);
                // Create TX packet params: 16 preamble symbols, explicit header, CRC on, standard IQ
                if let Ok(mut tx_pkt_params) = lora.create_tx_packet_params(16, false, true, false, &mdltn_params) {
                    let power_dbm = 22; // High-power PA max power
                    if let Ok(()) = lora.prepare_for_tx(&mdltn_params, &mut tx_pkt_params, power_dbm, &tx_packet.data[..tx_packet.len]).await {
                        if let Ok(()) = lora.tx().await {
                            info!("lora-phy: Transmission complete!");
                        } else {
                            error!("lora-phy: Transmission failed!");
                        }
                    }
                }
                // Return to continuous RX mode after transmission
                let _ = lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params).await;
            }
        }
    }
}
