//! SX1262 LoRa driver built on `lora-phy`.
//!
//! This crate owns only chip and modulation mechanics. MeshCore policy (which
//! spreading factor, preamble length, duty-cycle mode and so on) travels in
//! [`LoraConfig`]; the driver never reads protocol state or schedules tasks.
//!
//! The board supplies the SPI device and pins, so no MCU HAL types leak in.

#![no_std]
#![allow(async_fn_in_trait)]

use defmt::{debug, error, info};
use embassy_time::Duration;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::{
    Bandwidth, CodingRate, DutyCycleParams, ModulationParams, PacketParams, RxMode, SpreadingFactor,
};
use lora_phy::sx126x::{Config as Sx126xConfig, Sx1262, Sx126x};
use lora_phy::{LoRa, RxIrqEvent};

use ferrix_hal::radio::{bw_khz_to_hz, LoraConfig, RadioDevice, RxOutcome, RxState, TxOutcome};

pub use lora_phy::mod_params::RadioError;
pub use lora_phy::sx126x::TcxoCtrlVoltage;

/// Board-specific SX1262 wiring options that are not part of the radio config.
#[derive(Clone, Copy)]
pub struct Sx1262BoardConfig {
    /// TCXO supply voltage, or `None` when the board has a crystal.
    pub tcxo_ctrl: Option<TcxoCtrlVoltage>,
    /// Use the internal DC/DC converter instead of the LDO.
    pub use_dcdc: bool,
    /// Enable the boosted RX gain mode.
    pub rx_boost: bool,
}

impl Default for Sx1262BoardConfig {
    fn default() -> Self {
        Self {
            tcxo_ctrl: None,
            use_dcdc: true,
            rx_boost: false,
        }
    }
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

/// Estimate SX126x RX duty-cycle timings. The RX window only needs to catch
/// a preamble (`SetStopRxTimerOnPreamble` keeps the radio listening once it
/// does), while leaving margin for low-SNR correlation and TCXO wakeup.
fn duty_cycle_params(sf: u8, bw_khz: u32, preamble: u16) -> Option<DutyCycleParams> {
    let bw_hz = bw_khz_to_hz(bw_khz) as u64;
    if bw_hz == 0 {
        return None;
    }
    let sym_us = ((1u64 << sf) * 1_000_000) / bw_hz;
    let detect = if sf >= 7 { 6u64 } else { 10u64 };
    let p = preamble as u64;

    if p <= 2 * detect + 1 {
        return None;
    }

    let deaf_budget = (p - 2 * detect - 1) * sym_us;
    let deaf_us = deaf_budget * 90 / 100; // 10% margin for RC timer drift
    // Match lora-phy's 10 ms TCXO wakeup delay (`BRD_TCXO_WAKEUP_TIME`) plus
    // ~1 ms RC/PLL/RX settling so the real deaf window never exceeds `deaf_us`.
    let wake_us = 11_000u64;
    if deaf_us <= wake_us + 2_000 {
        return None;
    }

    let sleep_us = deaf_us - wake_us;
    let rx_us = (detect + 2) * sym_us;

    // SetRxDutyCycle units are 15.625 us.
    let units = |us: u64| ((us * 64 + 500) / 1000).min(0xFF_FFFF) as u32;
    Some(DutyCycleParams {
        rx_time: units(rx_us),
        sleep_time: units(sleep_us),
    })
}

fn make_rx_mode(sf: u8, bw_khz: u32, preamble: u16, duty_cycle: bool) -> RxMode {
    if duty_cycle {
        if let Some(params) = duty_cycle_params(sf, bw_khz, preamble) {
            return RxMode::DutyCycle(params);
        }
    }
    RxMode::Continuous
}

type InterfaceVariant<Reset, Pin> = GenericSx126xInterfaceVariant<Reset, Pin>;
type RadioKind<Spi, Reset, Pin> = Sx126x<Spi, InterfaceVariant<Reset, Pin>, Sx1262>;
type Phy<Spi, Reset, Pin, Dly> = LoRa<RadioKind<Spi, Reset, Pin>, Dly>;

/// An SX1262 radio driven by the Ferrix runtime scheduler.
pub struct Sx1262Radio<Spi, Reset, Pin, Dly>
where
    Spi: SpiDevice<u8>,
    Reset: OutputPin,
    Pin: Wait,
    Dly: DelayNs,
{
    lora: Phy<Spi, Reset, Pin, Dly>,
    mdltn: ModulationParams,
    rx_pkt: PacketParams,
    rx_mode: RxMode,
    preamble: u16,
    tx_power_dbm: i8,
    rx_armed: bool,
    rx_state: RxState,
}

impl<Spi, Reset, Pin, Dly> Sx1262Radio<Spi, Reset, Pin, Dly>
where
    Spi: SpiDevice<u8>,
    Reset: OutputPin,
    Pin: Wait,
    Dly: DelayNs,
{
    /// Initialise the chip and arm reception using `initial`.
    ///
    /// `reset`, `dio1` and `busy` must be the board's SX1262 control pins.
    /// `rf_switch_rx` is an optional RF-switch RX-enable GPIO, driven high
    /// while receiving and low otherwise. Boards whose switch is controlled
    /// only through DIO2 pass `None`.
    pub async fn new(
        spi: Spi,
        reset: Reset,
        dio1: Pin,
        busy: Pin,
        rf_switch_rx: Option<Reset>,
        delay: Dly,
        board: Sx1262BoardConfig,
        initial: LoraConfig,
    ) -> Result<Self, RadioError> {
        let iv = InterfaceVariant::new(reset, dio1, busy, rf_switch_rx, None)?;
        let config = Sx126xConfig {
            chip: Sx1262,
            tcxo_ctrl: board.tcxo_ctrl,
            use_dcdc: board.use_dcdc,
            rx_boost: board.rx_boost,
        };
        let radio_kind = Sx126x::new(spi, iv, config);

        // `false` selects the private network sync word used by MeshCore.
        let mut lora = LoRa::new(radio_kind, false, delay).await?;
        info!("sx1262: chip initialised");

        let mdltn = lora.create_modulation_params(
            sf_from_u8(initial.spreading_factor),
            bw_from_khz(initial.bandwidth_khz),
            cr_from_u8(initial.coding_rate),
            initial.frequency_hz,
        )?;
        let preamble = initial.preamble_length;
        let rx_pkt =
            lora.create_rx_packet_params(preamble, false, 255, true, false, &mdltn)?;
        let rx_mode = make_rx_mode(
            initial.spreading_factor,
            initial.bandwidth_khz,
            preamble,
            initial.rx_duty_cycle,
        );
        lora.prepare_for_rx(rx_mode, &mdltn, &rx_pkt).await?;

        info!(
            "sx1262: listening on {} Hz (SF{}/BW{}k, CR 4/{}, RX {})",
            initial.frequency_hz,
            initial.spreading_factor,
            initial.bandwidth_khz,
            initial.coding_rate,
            rx_mode
        );

        Ok(Self {
            lora,
            mdltn,
            rx_pkt,
            rx_mode,
            preamble,
            tx_power_dbm: initial.tx_power_dbm,
            rx_armed: false,
            rx_state: RxState::Idle,
        })
    }

    async fn reenter_rx(&mut self) {
        self.rx_armed = false;
        self.rx_state = RxState::Idle;
        match embassy_time::with_timeout(
            Duration::from_secs(1),
            self.lora
                .prepare_for_rx(self.rx_mode, &self.mdltn, &self.rx_pkt),
        )
        .await
        {
            Ok(Ok(())) => {}
            Ok(Err(e)) => error!("sx1262: re-enter RX failed: {:?}", e),
            Err(_) => error!("sx1262: re-enter RX timed out"),
        }
    }
}

impl<Spi, Reset, Pin, Dly> RadioDevice for Sx1262Radio<Spi, Reset, Pin, Dly>
where
    Spi: SpiDevice<u8>,
    Reset: OutputPin,
    Pin: Wait,
    Dly: DelayNs,
{
    async fn configure(&mut self, cfg: &LoraConfig) -> bool {
        match self.lora.create_modulation_params(
            sf_from_u8(cfg.spreading_factor),
            bw_from_khz(cfg.bandwidth_khz),
            cr_from_u8(cfg.coding_rate),
            cfg.frequency_hz,
        ) {
            Ok(new_mdltn) => {
                let new_preamble = cfg.preamble_length;
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
                            new_preamble,
                            cfg.rx_duty_cycle,
                        );
                        self.reenter_rx().await;
                        true
                    }
                    Err(e) => {
                        error!("sx1262: RX params on reconfigure failed: {:?}", e);
                        self.reenter_rx().await;
                        false
                    }
                }
            }
            Err(e) => {
                error!("sx1262: reconfigure failed: {:?}", e);
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
            Ok(Ok(())) => info!("sx1262: transmission complete"),
            Ok(Err(e)) => error!("sx1262: transmission failed: {:?}", e),
            Err(_) => error!("sx1262: transmission timed out; recovering RX"),
        }
        self.reenter_rx().await;
        if success {
            TxOutcome::Sent
        } else {
            TxOutcome::Failed
        }
    }

    async fn arm_receive(&mut self) {
        if self.rx_armed {
            return;
        }
        match self.lora.start_rx().await {
            Ok(()) => self.rx_armed = true,
            Err(e) => {
                error!("sx1262: start_rx failed: {:?}", e);
                self.reenter_rx().await;
                if let Err(e) = self.lora.start_rx().await {
                    error!("sx1262: retry start_rx failed: {:?}", e);
                } else {
                    self.rx_armed = true;
                }
            }
        }
    }

    async fn wait_rx_irq(&mut self) -> bool {
        self.lora.wait_for_irq().await.is_ok()
    }

    async fn handle_rx_irq(&mut self, buf: &mut [u8]) -> Option<RxOutcome> {
        match self.lora.handle_rx_irq(&self.rx_pkt, buf).await {
            Ok(RxIrqEvent::PreambleReceived) => {
                if self.rx_state != RxState::Header {
                    self.rx_state = RxState::Preamble;
                }
                None
            }
            Ok(RxIrqEvent::HeaderValid) => {
                self.rx_state = RxState::Header;
                None
            }
            Ok(RxIrqEvent::Packet(len, status)) => {
                self.rx_state = RxState::Idle;
                if matches!(self.rx_mode, RxMode::DutyCycle(_)) {
                    self.rx_armed = false;
                }
                Some(RxOutcome::Packet {
                    len: len as usize,
                    rssi: status.rssi,
                    snr: status.snr as i8,
                })
            }
            Ok(RxIrqEvent::None) => {
                self.rx_state = RxState::Idle;
                if matches!(self.rx_mode, RxMode::DutyCycle(_)) {
                    self.rx_armed = false;
                }
                None
            }
            Err(RadioError::ReceiveTimeout) if matches!(self.rx_mode, RxMode::DutyCycle(_)) => {
                self.rx_state = RxState::Idle;
                self.reenter_rx().await;
                Some(RxOutcome::DutyCycleEnd)
            }
            Err(e) => {
                self.rx_state = RxState::Idle;
                self.rx_armed = false;
                debug!("sx1262: RX error or timeout: {:?}", e);
                Some(RxOutcome::Error)
            }
        }
    }

    fn is_receiving(&self) -> bool {
        self.rx_state != RxState::Idle
    }

    fn rx_state(&self) -> RxState {
        self.rx_state
    }

    async fn recover_receive(&mut self) {
        self.reenter_rx().await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spreading_factor_mapping() {
        assert_eq!(sf_from_u8(5), SpreadingFactor::_5);
        assert_eq!(sf_from_u8(8), SpreadingFactor::_8);
        assert_eq!(sf_from_u8(12), SpreadingFactor::_12);
        assert_eq!(sf_from_u8(0), SpreadingFactor::_12);
        assert_eq!(sf_from_u8(255), SpreadingFactor::_12);
    }

    #[test]
    fn bandwidth_mapping() {
        assert_eq!(bw_from_khz(7), Bandwidth::_7KHz);
        assert_eq!(bw_from_khz(62), Bandwidth::_62KHz);
        assert_eq!(bw_from_khz(125), Bandwidth::_125KHz);
        assert_eq!(bw_from_khz(500), Bandwidth::_500KHz);
        assert_eq!(bw_from_khz(999), Bandwidth::_500KHz);
    }

    #[test]
    fn coding_rate_mapping() {
        assert_eq!(cr_from_u8(5), CodingRate::_4_5);
        assert_eq!(cr_from_u8(7), CodingRate::_4_7);
        assert_eq!(cr_from_u8(4), CodingRate::_4_8);
        assert_eq!(cr_from_u8(9), CodingRate::_4_8);
    }

    #[test]
    fn duty_cycle_rejects_short_preambles() {
        // 2 * detect + 1 symbols (13 for SF >= 7) is the minimum detectable preamble.
        assert!(duty_cycle_params(7, 62, 13).is_none());
        assert!(duty_cycle_params(8, 62, 13).is_none());
        // Just above it, or at fast bandwidths, the deaf window is <= wake cost (11 ms + 2 ms).
        assert!(duty_cycle_params(8, 62, 16).is_none());
        assert!(duty_cycle_params(7, 500, 32).is_none());
        assert!(duty_cycle_params(7, 0, 32).is_none());
    }

    #[test]
    fn duty_cycle_accepts_meshcore_preamble() {
        let sf7 = duty_cycle_params(7, 62, 32).expect("SF7 duty cycle params");
        assert!(sf7.rx_time > 0);
        assert!(sf7.sleep_time > 0);
        // SetStopRxTimerOnPreamble extends RX on preamble detection, so rx_time < sleep_time.
        assert!(sf7.rx_time < sf7.sleep_time);
        // 8 symbols (~16.4 ms) RX and ~24.0 ms sleep keeps ~30 ms margin on 32-symbol preambles.
        assert!(sf7.rx_time >= 1_000);
        assert!(sf7.sleep_time <= 1_600);

        // SF9-12 MeshCore preambles (16 symbols) also fit duty cycle.
        let sf9 = duty_cycle_params(9, 62, 16).expect("SF9 duty cycle params");
        assert!(sf9.rx_time > 0);
        assert!(sf9.sleep_time > 0);
        let sf10 = duty_cycle_params(10, 62, 16).expect("SF10 duty cycle params");
        assert!(sf10.rx_time > 0);
        assert!(sf10.sleep_time > 0);
    }

    #[test]
    fn rx_mode_follows_config() {
        assert!(matches!(
            make_rx_mode(7, 62, 32, false),
            RxMode::Continuous
        ));
        assert!(matches!(
            make_rx_mode(7, 62, 32, true),
            RxMode::DutyCycle(_)
        ));
        // Too short a preamble falls back to continuous RX.
        assert!(matches!(
            make_rx_mode(7, 62, 8, true),
            RxMode::Continuous
        ));
    }
}
