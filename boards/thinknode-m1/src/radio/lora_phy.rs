//! lora-phy Subsystem Adapter for ThinkNode M1 (SX1262)
//!
//! Bridges the community-standard `lora-phy` crate into Ferrix
//! using an asynchronous Embassy radio task and bounded IPC channels.

use defmt::{debug, error, info};
use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::spim::Spim;

use embassy_time::{Delay, Duration, Instant};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::{Bandwidth, CodingRate, DutyCycleParams, RadioError, RxMode, SpreadingFactor};
use lora_phy::sx126x::{Config as Sx126xConfig, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;

use ferrix_hal::radio::{LoraConfig, LoraRadio, RadioDevice, RxOutcome, TxOutcome};
use ferrix_core::scheduler::{preamble_for, TxPacket};

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

/// Estimate SX126x RX duty-cycle timings. The RX window only needs to catch
/// a preamble (`SetStopTimerOnPreamble` keeps the radio listening once it
/// does), but it must also leave room to finish a short packet.
fn duty_cycle_params(sf: u8, bw_khz: u32, preamble: u16) -> Option<DutyCycleParams> {
    let bw_hz = ferrix_core::airtime::bw_khz_to_hz(bw_khz) as u64;
    let sym_us = ((1u64 << sf) * 1_000_000) / bw_hz;
    let detect = if sf >= 7 { 8u64 } else { 12u64 };
    let p = preamble as u64;

    if p <= 2 * detect + 1 {
        return None;
    }

    let deaf_budget = (p - 2 * detect - 1) * sym_us;
    let deaf_us = deaf_budget * 90 / 100; // 10% margin for RC timer drift
    let wake_us = 3_000u64;
    if deaf_us <= wake_us + 2_000 {
        return None;
    }

    let sleep_us = deaf_us - wake_us;
    let complete_us = (p + 14) * sym_us;
    let mut rx_us = (detect + 1) * sym_us;
    if complete_us > sleep_us && rx_us < (complete_us - sleep_us + 1) / 2 {
        rx_us = (complete_us - sleep_us + 1) / 2;
    }

    // SetRxDutyCycle units are 15.625 us.
    let units = |us: u64| ((us * 64 + 500) / 1000).min(0xFF_FFFF) as u32;
    Some(DutyCycleParams {
        rx_time: units(rx_us),
        sleep_time: units(sleep_us),
    })
}

fn make_rx_mode(sf: u8, bw_khz: u32, duty_cycle: bool) -> RxMode {
    if duty_cycle {
        if let Some(params) = duty_cycle_params(sf, bw_khz, preamble_for(sf)) {
            return RxMode::DutyCycle(params);
        }
    }
    RxMode::Continuous
}


/// ThinkNode M1 LoRa Radio Adapter implementing `LoraRadio` via channels.
pub struct LoraPhyChannelRadio;

impl LoraPhyChannelRadio {
    pub const fn new() -> Self {
        Self
    }
}

impl LoraRadio for LoraPhyChannelRadio {
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

type SpiDevice = ExclusiveDevice<Spim<'static, TWISPI0>, Output<'static>, Delay>;
type InterfaceVariant = GenericSx126xInterfaceVariant<Output<'static>, Input<'static>>;
type RadioKind = Sx126x<SpiDevice, InterfaceVariant, Sx1262>;
type Phy = LoRa<RadioKind, Delay>;

/// SX1262 driver used by the runtime radio scheduler.
struct Sx1262Radio {
    lora: Phy,
    mdltn: lora_phy::mod_params::ModulationParams,
    rx_pkt: lora_phy::mod_params::PacketParams,
    rx_mode: RxMode,
    preamble: u16,
    tx_power_dbm: i8,
    cfg: LoraConfig,
}

impl Sx1262Radio {
    async fn reenter_rx(&mut self) {
        match embassy_time::with_timeout(
            Duration::from_secs(1),
            self.lora
                .prepare_for_rx(self.rx_mode, &self.mdltn, &self.rx_pkt),
        )
        .await
        {
            Ok(Ok(())) => {}
            Ok(Err(e)) => error!("lora-phy: re-enter RX failed: {:?}", e),
            Err(_) => error!("lora-phy: re-enter RX timed out"),
        }
    }
}

impl RadioDevice for Sx1262Radio {
    async fn configure(&mut self, cfg: &LoraConfig) -> bool {
        match self.lora.create_modulation_params(
            sf_from_u8(cfg.spreading_factor),
            bw_from_khz(cfg.bandwidth_khz),
            cr_from_u8(cfg.coding_rate),
            cfg.frequency_hz,
        ) {
            Ok(new_mdltn) => {
                let new_preamble = preamble_for(cfg.spreading_factor);
                match self.lora.create_rx_packet_params(
                    new_preamble,
                    false,
                    255,
                    true,
                    false,
                    &new_mdltn,
                ) {
                    Ok(new_rx) => {
                        self.mdltn = new_mdltn;
                        self.rx_pkt = new_rx;
                        self.preamble = new_preamble;
                        self.tx_power_dbm = cfg.tx_power_dbm;
                        self.rx_mode = make_rx_mode(
                            cfg.spreading_factor,
                            cfg.bandwidth_khz,
                            cfg.rx_duty_cycle,
                        );
                        self.cfg = *cfg;
                        self.reenter_rx().await;
                        true
                    }
                    Err(e) => {
                        error!("lora-phy: RX params on reconfigure failed: {:?}", e);
                        self.reenter_rx().await;
                        false
                    }
                }
            }
            Err(e) => {
                error!("lora-phy: reconfigure failed: {:?}", e);
                self.reenter_rx().await;
                false
            }
        }
    }

    async fn transmit(&mut self, payload: &[u8]) -> TxOutcome {
        let tx_result = embassy_time::with_timeout(Duration::from_secs(20), async {
            let mut tx_pkt = self.lora.create_tx_packet_params(
                self.preamble,
                false,
                true,
                false,
                &self.mdltn,
            )?;
            self.lora
                .prepare_for_tx(
                    &self.mdltn,
                    &mut tx_pkt,
                    self.tx_power_dbm as i32,
                    payload,
                )
                .await?;
            self.lora.tx().await
        })
        .await;

        let success = matches!(tx_result, Ok(Ok(())));
        match tx_result {
            Ok(Ok(())) => info!("lora-phy: Transmission complete!"),
            Ok(Err(e)) => error!("lora-phy: Transmission failed: {:?}", e),
            Err(_) => error!("lora-phy: Transmission timed out; forcing RX recovery"),
        }
        self.reenter_rx().await;
        if success {
            TxOutcome::Sent
        } else {
            TxOutcome::Failed
        }
    }

    async fn receive(&mut self, buf: &mut [u8]) -> RxOutcome {
        match self.lora.rx(&self.rx_pkt, buf).await {
            Ok((len, status)) => RxOutcome::Packet {
                len: len as usize,
                rssi: status.rssi,
                snr: status.snr as i8,
            },
            Err(RadioError::ReceiveTimeout)
                if matches!(self.rx_mode, RxMode::DutyCycle(_)) =>
            {
                RxOutcome::DutyCycleEnd
            }
            Err(e) => {
                debug!("lora-phy: RX error or timeout: {:?}", e);
                RxOutcome::Error
            }
        }
    }

    async fn recover_receive(&mut self) {
        self.reenter_rx().await;
    }
}

/// Standalone Embassy task managing the SX1262 radio via `lora-phy`.
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

    // 1. Ensure SX126X_POWER_EN (P0.21) is continuously HIGH.
    power_en.set_high();

    // 2. Wrap Spim & CS into ExclusiveDevice.
    let spi_device = ExclusiveDevice::new(spim, cs, Delay).unwrap();

    // 3. Build InterfaceVariant for SX1262.
    // ThinkNode M1 uses internal DIO2 RF switch control, so rf_switch is None.
    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();

    // 4. Configure SX1262: 1.8 V TCXO, DCDC, RX boosted gain.
    let config = Sx126xConfig {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V8),
        use_dcdc: true,
        rx_boost: true,
    };

    let radio_kind = Sx126x::new(spi_device, iv, config);

    // 5. Initialize LoRa with the private network sync word.
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

    // 6. Read persisted radio settings and build the modulation params.
    let cfg = match lora_config_from_state() {
        Some(cfg) => cfg,
        None => return,
    };
    let mdltn = match lora.create_modulation_params(
        sf_from_u8(cfg.spreading_factor),
        bw_from_khz(cfg.bandwidth_khz),
        cr_from_u8(cfg.coding_rate),
        cfg.frequency_hz,
    ) {
        Ok(p) => p,
        Err(e) => {
            error!("lora-phy: Failed to create modulation params: {:?}", e);
            return;
        }
    };
    let preamble = preamble_for(cfg.spreading_factor);
    let rx_pkt = match lora.create_rx_packet_params(preamble, false, 255, true, false, &mdltn) {
        Ok(p) => p,
        Err(e) => {
            error!("lora-phy: Failed to create RX packet params: {:?}", e);
            return;
        }
    };
    let rx_mode = make_rx_mode(cfg.spreading_factor, cfg.bandwidth_khz, cfg.rx_duty_cycle);
    if let Err(e) = lora.prepare_for_rx(rx_mode, &mdltn, &rx_pkt).await {
        error!("lora-phy: Failed to prepare for RX: {:?}", e);
        return;
    }
    info!(
        "lora-phy: Radio listening on {} Hz (SF{}/BW{}k, CR 4/{}, RX {})",
        cfg.frequency_hz, cfg.spreading_factor, cfg.bandwidth_khz, cfg.coding_rate, rx_mode
    );

    let device = Sx1262Radio {
        lora,
        mdltn,
        rx_pkt,
        rx_mode,
        preamble,
        tx_power_dbm: cfg.tx_power_dbm,
        cfg,
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
            preamble_length: 0,
            rx_duty_cycle: s.rx_duty_cycle,
        }
    }))
}
