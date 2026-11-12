//! lora-phy Subsystem Adapter for ThinkNode M1 (SX1262)
//!
//! Bridges the community-standard `lora-phy` crate into Ferrix
//! using an asynchronous Embassy radio task and bounded IPC channels.

use defmt::{debug, error, info};
use embassy_futures::select::{select3, Either3};
use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::spim::Spim;
use embassy_sync::channel::Channel;

/// Shared raw mutex type — see usb.rs for documentation.
type SharedRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
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

pub static LORA_RX_CHANNEL: Channel<SharedRawMutex, (usize, i16, i8, [u8; 256]), 4> = Channel::new();
pub static LORA_TX_CHANNEL: Channel<SharedRawMutex, TxPacket, 4> = Channel::new();
pub static LORA_CFG_CHANNEL: Channel<SharedRawMutex, LoraConfig, 4> = Channel::new();

/// Ask the radio task to apply the current `RADIO_SETTINGS`.
pub fn request_radio_config() -> bool {
    let cfg = crate::companion::usb::RADIO_SETTINGS.lock(|r| {
        let s = r.borrow();
        LoraConfig {
            frequency_hz: s.frequency_hz,
            tx_power_dbm: s.tx_power_dbm,
            spreading_factor: s.sf,
            bandwidth_khz: s.bw_khz,
            coding_rate: s.cr,
            preamble_length: 0,
        }
    });
    LORA_CFG_CHANNEL.try_send(cfg).is_ok()
}

fn sf_from_u8(sf: u8) -> SpreadingFactor {
    match sf {
        5 => SpreadingFactor::_5,
        6 => SpreadingFactor::_6,
        7 => SpreadingFactor::_7,
        8 => SpreadingFactor::_8,
        9 => SpreadingFactor::_9,
        10 => SpreadingFactor::_10,
        11 => SpreadingFactor::_11,
        _ => SpreadingFactor::_12,
    }
}

fn bw_from_khz(bw: u32) -> Bandwidth {
    match bw {
        7 => Bandwidth::_7KHz,
        10 => Bandwidth::_10KHz,
        15 => Bandwidth::_15KHz,
        20 => Bandwidth::_20KHz,
        31 => Bandwidth::_31KHz,
        41 => Bandwidth::_41KHz,
        62 => Bandwidth::_62KHz,
        125 => Bandwidth::_125KHz,
        250 => Bandwidth::_250KHz,
        _ => Bandwidth::_500KHz,
    }
}

fn cr_from_u8(cr: u8) -> CodingRate {
    match cr {
        5 => CodingRate::_4_5,
        6 => CodingRate::_4_6,
        7 => CodingRate::_4_7,
        _ => CodingRate::_4_8,
    }
}

/// MeshCore uses 32 preamble symbols for SF <= 8, otherwise 16.
fn preamble_for(sf: u8) -> u16 {
    if sf <= 8 {
        32
    } else {
        16
    }
}

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
        // Never block the main loop if the radio task is stalled or
        // reconfiguring; dropping is preferable to a frozen UI.
        let _ = LORA_TX_CHANNEL.try_send(pkt);
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

    // 6. Read the persisted radio settings and build the modulation params.
    let (freq_hz, tx_power, sf, bw_khz, cr) =
        crate::companion::usb::RADIO_SETTINGS.lock(|r| {
            let s = r.borrow();
            (s.frequency_hz, s.tx_power_dbm, s.sf, s.bw_khz, s.cr)
        });
    let mut tx_power_dbm = tx_power;
    let mut preamble = preamble_for(sf);
    let mut current_cfg = (freq_hz, tx_power, sf, bw_khz, cr);
    let mut mdltn_params = match lora.create_modulation_params(
        sf_from_u8(sf),
        bw_from_khz(bw_khz),
        cr_from_u8(cr),
        freq_hz,
    ) {
        Ok(p) => p,
        Err(e) => {
            error!("lora-phy: Failed to create modulation params: {:?}", e);
            return;
        }
    };

    // 7. Packet parameters for RX: MeshCore preamble, explicit header, 255 max payload, CRC on, standard IQ.
    let mut rx_pkt_params = match lora.create_rx_packet_params(preamble, false, 255, true, false, &mdltn_params) {
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

    info!(
        "lora-phy: Radio listening on {} Hz (SF{}/BW{}k, CR 4/{})",
        freq_hz, sf, bw_khz, cr
    );

    let mut rx_buf = [0u8; 256];

    loop {
        match select3(
            lora.rx(&rx_pkt_params, &mut rx_buf),
            LORA_TX_CHANNEL.receive(),
            LORA_CFG_CHANNEL.receive(),
        )
        .await
        {
            Either3::First(rx_result) => match rx_result {
                Ok((len, status)) => {
                    info!("lora-phy: RX {} bytes, RSSI: {} dBm, SNR: {} dB", len, status.rssi, status.snr);
                    let mut packet_data = [0u8; 256];
                    let copy_len = (len as usize).min(256);
                    packet_data[..copy_len].copy_from_slice(&rx_buf[..copy_len]);
                    let _ = LORA_RX_CHANNEL.try_send((copy_len, status.rssi, status.snr as i8, packet_data));
                }
                Err(e) => {
                    debug!("lora-phy: RX error or timeout: {:?}", e);
                    // Re-enter RX continuous if an error occurred, but never
                    // let a stuck BUSY line wedge the whole task forever.
                    match embassy_time::with_timeout(
                        embassy_time::Duration::from_secs(1),
                        lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params),
                    )
                    .await
                    {
                        Ok(Ok(())) => {}
                        Ok(Err(e)) => error!("lora-phy: re-enter RX failed: {:?}", e),
                        Err(_) => error!("lora-phy: re-enter RX timed out"),
                    }
                }
            },
            Either3::Second(tx_packet) => {
                info!("lora-phy: Transmitting {} bytes...", tx_packet.len);
                if let Ok(mut tx_pkt_params) =
                    lora.create_tx_packet_params(preamble, false, true, false, &mdltn_params)
                {
                    if let Ok(()) = lora
                        .prepare_for_tx(
                            &mdltn_params,
                            &mut tx_pkt_params,
                            tx_power_dbm as i32,
                            &tx_packet.data[..tx_packet.len],
                        )
                        .await
                    {
                        if let Ok(()) = lora.tx().await {
                            info!("lora-phy: Transmission complete!");
                        } else {
                            error!("lora-phy: Transmission failed!");
                        }
                    }
                }
                // Return to continuous RX mode after transmission
                let _ = embassy_time::with_timeout(
                    embassy_time::Duration::from_secs(1),
                    lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &rx_pkt_params),
                )
                .await;
            }
            Either3::Third(cfg) => {
                let requested = (
                    cfg.frequency_hz,
                    cfg.tx_power_dbm,
                    cfg.spreading_factor,
                    cfg.bandwidth_khz,
                    cfg.coding_rate,
                );
                if requested == current_cfg {
                    // The app re-sends its stored settings on every connect.
                    // Do not stop and restart RX for a no-op.
                    continue;
                }
                match lora.create_modulation_params(
                    sf_from_u8(cfg.spreading_factor),
                    bw_from_khz(cfg.bandwidth_khz),
                    cr_from_u8(cfg.coding_rate),
                    cfg.frequency_hz,
                ) {
                    Ok(new_mdltn) => {
                        let new_preamble = preamble_for(cfg.spreading_factor);
                        match lora.create_rx_packet_params(
                            new_preamble,
                            false,
                            255,
                            true,
                            false,
                            &new_mdltn,
                        ) {
                            Ok(new_rx) => {
                                mdltn_params = new_mdltn;
                                rx_pkt_params = new_rx;
                                preamble = new_preamble;
                                tx_power_dbm = cfg.tx_power_dbm;
                                current_cfg = requested;
                                match embassy_time::with_timeout(
                                    embassy_time::Duration::from_secs(1),
                                    lora.prepare_for_rx(
                                        RxMode::Continuous,
                                        &mdltn_params,
                                        &rx_pkt_params,
                                    ),
                                )
                                .await
                                {
                                    Ok(Ok(())) => {}
                                    Ok(Err(e)) => {
                                        error!("lora-phy: re-enter RX after cfg failed: {:?}", e)
                                    }
                                    Err(_) => error!("lora-phy: re-enter RX after cfg timed out"),
                                }
                                info!(
                                    "lora-phy: reconfigured to {} Hz (SF{}, BW{}k, CR 4/{}), {} dBm",
                                    cfg.frequency_hz,
                                    cfg.spreading_factor,
                                    cfg.bandwidth_khz,
                                    cfg.coding_rate,
                                    cfg.tx_power_dbm
                                );
                            }
                            Err(e) => error!("lora-phy: RX params on reconfigure failed: {:?}", e),
                        }
                    }
                    Err(e) => error!("lora-phy: reconfigure failed: {:?}", e),
                }
            }
        }
    }
}
