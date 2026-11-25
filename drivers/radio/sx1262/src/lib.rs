//! SX1262 LoRa driver built on `lora-phy`.
//!
//! This crate owns only chip and modulation mechanics. MeshCore policy (which
//! spreading factor, preamble length, duty-cycle mode and so on) travels in
//! [`LoraConfig`]; the driver never reads protocol state or schedules tasks.
//!
//! The board supplies the SPI device and pins, so no MCU HAL types leak in.

#![no_std]
#![forbid(unsafe_code)]
#![allow(async_fn_in_trait)]

use defmt::{debug, error, info, warn};
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

/// Compute SX126x RX duty-cycle timings per SX1261-2 Datasheet §13.1.7 and AN1200.36:
///
/// 1. Only the **deaf** phase (`sleep_time + wake_time`) must fit within the
///    detectable preamble margin `(P - 2*D - 1) * T_sym` (derated by 10% for
///    RC64k drift), where `D = 8` for `SF >= 7` and `12` for `SF < 7`.
/// 2. `rx_time >= (D + 1) * T_sym` so at least one full detection window runs
///    each wake cycle.
/// 3. `2 * rx_time + sleep_time >= (P + 14) * T_sym` because on preamble
///    detection the SX1262 restarts its internal timer with
///    `2 * rxPeriod + sleepPeriod` and aborts reception (`RxTxTimeout`) if
///    SyncWord + Header do not complete before that timer expires.
///
/// Returns `None` (falling back to continuous RX) whenever no valid `(rx, sleep)`
/// pair satisfies all three constraints — for example at `SF9/SF10` with a
/// 16-symbol preamble or at wide bandwidths where the deaf budget is smaller
/// than the TCXO wakeup delay.
fn duty_cycle_params(sf: u8, bw_khz: u32, preamble: u16) -> Option<DutyCycleParams> {
    let bw_hz = bw_khz_to_hz(bw_khz) as u64;
    if bw_hz == 0 {
        return None;
    }
    let sym_us = ((1u64 << sf) * 1_000_000) / bw_hz;
    if sym_us == 0 {
        return None;
    }
    let d = if sf >= 7 { 8u64 } else { 12u64 };
    let p = preamble as u64;

    if p <= 2 * d + 1 {
        return None;
    }

    // Match lora-phy's 10 ms TCXO wakeup delay (`BRD_TCXO_WAKEUP_TIME`) plus
    // ~1 ms RC/PLL/RX settling.
    let wake_us = 11_000u64;
    let deaf_budget_us = (p - 2 * d - 1) * sym_us;
    // Derate by 10% for RC64k oscillator drift.
    let deaf_us = deaf_budget_us * 90 / 100;
    if deaf_us <= wake_us + 2_000 {
        return None;
    }
    let sleep_us = deaf_us - wake_us;

    // SX1261-2 Datasheet §13.1.7:
    //   2 * rx_us + sleep_us >= (preamble + 4.25 + 8 + margin) * T_sym
    let complete_us = (p + 14) * sym_us;
    let min_rx_us = (d + 1) * sym_us;
    let rx_us = min_rx_us.max(complete_us.saturating_sub(sleep_us).div_ceil(2));

    // SetRxDutyCycle units are 15.625 us.
    let units = |us: u64| ((us * 64 + 500) / 1000).min(0xFF_FFFF) as u32;
    Some(DutyCycleParams {
        rx_time: units(rx_us),
        sleep_time: units(sleep_us),
    })
}

fn preamble_grace_ms(sf: u8, bw_khz: u32, preamble: u16) -> u64 {
    let bw_hz = bw_khz_to_hz(bw_khz) as u64;
    if bw_hz == 0 {
        return 200;
    }
    let sym_us = ((1u64 << sf.min(12)) * 1_000_000) / bw_hz;
    // Preamble + syncword + SFD + header duration.
    let coeff_x4 = if sf <= 6 { 25u64 } else { 17u64 };
    let preamble_us = (((preamble as u64 + 8) * 4 + coeff_x4) * sym_us) / 4;
    preamble_us.div_ceil(1_000).clamp(40, 2_500)
}

fn make_rx_mode(sf: u8, bw_khz: u32, preamble: u16, duty_cycle: bool) -> RxMode {
    if duty_cycle {
        if let Some(params) = duty_cycle_params(sf, bw_khz, preamble) {
            return RxMode::DutyCycle(params);
        }
    }
    RxMode::Continuous
}

/// Continuous CAD-busy duration before relaxing LBT `detPeak` by one step.
///
/// Normal back-to-back LoRa packets last a few hundred milliseconds to a few
/// seconds and must not desensitize LBT. Only prolonged starvation (elevated
/// RF noise floor blocking every TX attempt for a full minute) relaxes `detPeak`.
const LBT_STARVATION_MS: u64 = 60_000;
const LBT_MAX_PEAK_OFFSET: i8 = 12;
const LBT_RECOVER_CLEAN_COUNT: u8 = 16;

#[derive(Clone, Copy, Debug)]
struct LbtState {
    peak_offset: i8,
    clean_count: u8,
    busy_since_ms: Option<u64>,
}

impl LbtState {
    const fn new() -> Self {
        Self {
            peak_offset: 0,
            clean_count: 0,
            busy_since_ms: None,
        }
    }

    fn reset(&mut self) {
        self.peak_offset = 0;
        self.clean_count = 0;
        self.busy_since_ms = None;
    }

    fn clear_starvation(&mut self) {
        self.busy_since_ms = None;
    }

    fn offset(&self) -> i8 {
        self.peak_offset
    }

    /// Record a CAD-busy deferral at `now_ms`. Returns `true` when sustained
    /// starvation caused `peak_offset` to relax by +1.
    fn on_busy(&mut self, now_ms: u64) -> bool {
        self.clean_count = 0;
        let since = *self.busy_since_ms.get_or_insert(now_ms);
        if now_ms.saturating_sub(since) >= LBT_STARVATION_MS {
            self.busy_since_ms = Some(now_ms);
            if self.peak_offset < LBT_MAX_PEAK_OFFSET {
                self.peak_offset += 1;
                return true;
            }
        }
        false
    }

    /// Record a clear CAD pass. Ends any starvation streak and gradually
    /// restores sensitivity if `peak_offset` was previously relaxed.
    fn on_clear(&mut self) {
        self.busy_since_ms = None;
        if self.peak_offset > 0 {
            self.clean_count = self.clean_count.saturating_add(1);
            if self.clean_count >= LBT_RECOVER_CLEAN_COUNT {
                self.peak_offset -= 1;
                self.clean_count = 0;
            }
        }
    }
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
    preamble_grace_ms: u64,
    preamble_seen_at: Option<embassy_time::Instant>,
    lbt: LbtState,
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
            preamble_grace_ms: preamble_grace_ms(
                initial.spreading_factor,
                initial.bandwidth_khz,
                preamble,
            ),
            preamble_seen_at: None,
            lbt: LbtState::new(),
            tx_power_dbm: initial.tx_power_dbm,
            rx_armed: false,
            rx_state: RxState::Idle,
        })
    }

    async fn reenter_rx(&mut self) {
        self.rx_armed = false;
        self.rx_state = RxState::Idle;
        self.preamble_seen_at = None;
        let res = embassy_time::with_timeout(Duration::from_secs(1), async {
            self.lora
                .prepare_for_rx(self.rx_mode, &self.mdltn, &self.rx_pkt)
                .await?;
            self.lora.start_rx().await
        })
        .await;
        match res {
            Ok(Ok(())) => self.rx_armed = true,
            Ok(Err(e)) => error!("sx1262: re-enter RX failed: {:?}", e),
            Err(_) => error!("sx1262: re-enter RX timed out"),
        }
    }

    async fn rearm_rx_fast_or_recover(&mut self) {
        self.rx_armed = false;
        self.rx_state = RxState::Idle;
        self.preamble_seen_at = None;
        match embassy_time::with_timeout(
            Duration::from_millis(500),
            self.lora.rearm_rx_fast(self.rx_mode, &self.rx_pkt),
        )
        .await
        {
            Ok(Ok(())) => self.rx_armed = true,
            _ => self.reenter_rx().await,
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
                        self.preamble_grace_ms = preamble_grace_ms(
                            cfg.spreading_factor,
                            cfg.bandwidth_khz,
                            new_preamble,
                        );
                        self.lbt.reset();
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
        // 1. Never interrupt a confirmed incoming LoRa header.
        if self.rx_state == RxState::Header {
            return TxOutcome::ChannelBusy;
        }

        // 2. Non-destructive live GetIrqStatus check while in continuous RX:
        // clear HeaderError immediately, defer on a live HeaderValid without
        // disturbing RX, and give a fresh PreambleDetected one preamble window
        // before clearing it as noise.
        if self.rx_armed && matches!(self.rx_mode, RxMode::Continuous) {
            if let Ok((header_valid, preamble_detected, header_err)) =
                self.lora.poll_rx_activity().await
            {
                if header_err {
                    let _ = self.lora.clear_rx_activity_irqs().await;
                    self.rx_state = RxState::Idle;
                    self.preamble_seen_at = None;
                } else if header_valid {
                    self.rx_state = RxState::Header;
                    self.preamble_seen_at = None;
                    return TxOutcome::ChannelBusy;
                } else if preamble_detected {
                    let now = embassy_time::Instant::now();
                    match self.preamble_seen_at {
                        None => {
                            self.preamble_seen_at = Some(now);
                            return TxOutcome::ChannelBusy;
                        }
                        Some(since)
                            if now.saturating_duration_since(since).as_millis()
                                < self.preamble_grace_ms =>
                        {
                            return TxOutcome::ChannelBusy;
                        }
                        Some(_) => {
                            let _ = self.lora.clear_preamble_irq().await;
                            self.preamble_seen_at = None;
                        }
                    }
                } else {
                    self.preamble_seen_at = None;
                }
            }
        }

        // 3. Fast 4-symbol hardware LBT CAD check from warm standby with
        // starvation-gated `detPeak` relaxation: detects mid-payload LoRa chirp
        // energy while holding full sensitivity during normal traffic bursts,
        // relaxing only after 60 s of continuous LBT starvation.
        self.rx_armed = false;
        match embassy_time::with_timeout(
            Duration::from_millis(500),
            self.lora
                .cad_fast_with_offset(&self.mdltn, self.lbt.offset()),
        )
        .await
        {
            Ok(Ok(true)) => {
                let now_ms = embassy_time::Instant::now().as_millis();
                if self.lbt.on_busy(now_ms) {
                    warn!(
                        "sx1262: LBT starved 60s; relaxing detPeak offset to {}",
                        self.lbt.offset()
                    );
                } else {
                    debug!("sx1262: CAD busy; deferring TX");
                }
                self.rearm_rx_fast_or_recover().await;
                return TxOutcome::ChannelBusy;
            }
            Ok(Ok(false)) => {
                self.lbt.on_clear();
            }
            Ok(Err(e)) => {
                debug!("sx1262: CAD error ({:?}); proceeding with TX", e);
            }
            Err(_) => {
                error!("sx1262: CAD timed out; recovering RX");
                self.reenter_rx().await;
                return TxOutcome::Failed;
            }
        }

        // 4. Fast TX + sub-ms FS/STBY_XOSC->RX re-arm immediately after TxDone.
        let tx_result = embassy_time::with_timeout(Duration::from_secs(20), async {
            let mut tx_pkt = self.lora.create_tx_packet_params(
                self.preamble,
                false,
                true,
                false,
                &self.mdltn,
            )?;
            self.lora
                .prepare_for_tx_fast(
                    &self.mdltn,
                    &mut tx_pkt,
                    self.tx_power_dbm as i32,
                    payload,
                )
                .await?;
            self.lora.tx().await
        })
        .await;

        match tx_result {
            Ok(Ok(())) => {
                info!("sx1262: transmission complete");
                self.rearm_rx_fast_or_recover().await;
                TxOutcome::Sent
            }
            Ok(Err(e)) => {
                error!("sx1262: transmission failed: {:?}", e);
                self.reenter_rx().await;
                TxOutcome::Failed
            }
            Err(_) => {
                error!("sx1262: transmission timed out; recovering RX");
                self.reenter_rx().await;
                TxOutcome::Failed
            }
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
                self.preamble_seen_at = None;
                None
            }
            Ok(RxIrqEvent::Packet(len, status)) => {
                self.rx_state = RxState::Idle;
                self.preamble_seen_at = None;
                if matches!(self.rx_mode, RxMode::DutyCycle(_)) {
                    self.rx_armed = false;
                }
                Some(RxOutcome::Packet {
                    len: len as usize,
                    rssi: status.rssi,
                    snr: status.snr as i8,
                })
            }
            Ok(RxIrqEvent::RxError) => {
                self.rx_state = RxState::Idle;
                self.preamble_seen_at = None;
                if matches!(self.rx_mode, RxMode::DutyCycle(_)) {
                    self.rx_armed = false;
                }
                Some(RxOutcome::CrcError)
            }
            Ok(RxIrqEvent::None) => {
                self.rx_state = RxState::Idle;
                self.preamble_seen_at = None;
                if matches!(self.rx_mode, RxMode::DutyCycle(_)) {
                    self.rx_armed = false;
                }
                None
            }
            Err(RadioError::ReceiveTimeout) if matches!(self.rx_mode, RxMode::DutyCycle(_)) => {
                self.rearm_rx_fast_or_recover().await;
                Some(RxOutcome::DutyCycleEnd)
            }
            Err(e) => {
                self.rx_state = RxState::Idle;
                self.preamble_seen_at = None;
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
        self.lbt.clear_starvation();
        self.reenter_rx().await;
    }

    async fn sample_rssi_half_db(&mut self) -> Option<i16> {
        // Never touch SPI during duty-cycle RX (NSS would abort sleep/sniff
        // phase) or while a frame is in flight.
        if !self.rx_armed
            || self.rx_state != RxState::Idle
            || !matches!(self.rx_mode, RxMode::Continuous)
        {
            return None;
        }

        if let Ok((header_valid, preamble_detected, _)) = self.lora.poll_rx_activity().await {
            if header_valid {
                self.rx_state = RxState::Header;
                return None;
            }
            if preamble_detected {
                return None;
            }
        }

        // Burst of 8 instantaneous RSSI reads spaced 200 us apart; taking the
        // minimum rejects transient interference spikes without leaving RX.
        const BURST_SAMPLES: usize = 8;
        let mut min_half_db: Option<i16> = None;
        for i in 0..BURST_SAMPLES {
            match self.lora.get_rssi_inst_half_db().await {
                Ok(sample) => {
                    min_half_db = Some(match min_half_db {
                        Some(cur) => cur.min(sample),
                        None => sample,
                    });
                }
                Err(_) => return None,
            }
            if i + 1 < BURST_SAMPLES {
                embassy_time::Timer::after_micros(200).await;
            }
        }

        // Discard the burst if a LoRa preamble or header arrived mid-burst.
        if let Ok((header_valid, preamble_detected, _)) = self.lora.poll_rx_activity().await {
            if header_valid {
                self.rx_state = RxState::Header;
                return None;
            }
            if preamble_detected {
                return None;
            }
        }

        min_half_db
    }

    async fn reset_agc(&mut self) -> bool {
        self.rx_armed = false;
        self.rx_state = RxState::Idle;
        let reset_ok = match embassy_time::with_timeout(
            Duration::from_secs(1),
            self.lora.reset_agc(&self.mdltn),
        )
        .await
        {
            Ok(Ok(())) => true,
            Ok(Err(e)) => {
                error!("sx1262: AGC reset failed: {:?}", e);
                false
            }
            Err(_) => {
                error!("sx1262: AGC reset timed out");
                false
            }
        };
        self.reenter_rx().await;
        reset_ok && self.rx_armed
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
        // 2 * D + 1 symbols (17 for SF >= 7) is the minimum preamble with positive deaf window.
        assert!(duty_cycle_params(7, 62, 16).is_none());
        assert!(duty_cycle_params(7, 62, 17).is_none());
        assert!(duty_cycle_params(9, 62, 16).is_none());
        assert!(duty_cycle_params(10, 62, 16).is_none());
        // Wide bandwidths where deaf budget <= TCXO wake cost (11 ms + 2 ms).
        assert!(duty_cycle_params(7, 500, 32).is_none());
        assert!(duty_cycle_params(7, 0, 32).is_none());
    }

    #[test]
    fn duty_cycle_accepts_32_symbol_preamble() {
        // SF7 / BW 62.5 kHz / 32-symbol preamble fits cleanly with deaf-window math:
        // sleep ~16.6 ms, rx ~38.8 ms, and 2*rx + sleep >= 46 * 2.048 ms = 94.2 ms (~6029 units).
        let sf7 = duty_cycle_params(7, 62, 32).expect("SF7 32-sym duty cycle params");
        assert!(sf7.rx_time > 0);
        assert!(sf7.sleep_time > 0);
        assert!(2 * sf7.rx_time + sf7.sleep_time >= 6_025);

        let sf8 = duty_cycle_params(8, 62, 32).expect("SF8 32-sym duty cycle params");
        assert!(sf8.rx_time > 0);
        assert!(sf8.sleep_time > 0);
        assert!(2 * sf8.rx_time + sf8.sleep_time >= 12_050);

        // Longer preambles at SF9/10 also satisfy SX1261-2 §13.1.7.
        assert!(duty_cycle_params(9, 62, 32).is_some());
        assert!(duty_cycle_params(10, 62, 32).is_some());
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
            make_rx_mode(9, 62, 16, true),
            RxMode::Continuous
        ));
    }

    #[test]
    fn preamble_grace_scales_with_sf_and_preamble() {
        // SF7 / BW 62.5 kHz / 32-symbol preamble: ((32+8)*4 + 17) * 2048us / 4 = 90.624 ms -> 91 ms.
        assert_eq!(preamble_grace_ms(7, 62, 32), 91);
        // SF10 / BW 62.5 kHz / 16-symbol preamble: ((16+8)*4 + 17) * 16384us / 4 = 462.848 ms -> 463 ms.
        assert_eq!(preamble_grace_ms(10, 62, 16), 463);
    }

    #[test]
    fn lbt_holds_sensitivity_through_normal_traffic_bursts() {
        let mut lbt = LbtState::new();
        // Repeated busy hits over 59.9 seconds (normal multi-packet bursts) must not relax detPeak.
        for step in 0..400u64 {
            assert!(!lbt.on_busy(1_000 + step * 149));
            assert_eq!(lbt.offset(), 0);
        }
        // Any clear CAD resets the starvation timer.
        lbt.on_clear();
        assert!(!lbt.on_busy(1_000 + LBT_STARVATION_MS + 10_000));
        assert_eq!(lbt.offset(), 0);
    }

    #[test]
    fn lbt_relaxes_only_after_sustained_starvation_and_recovers_cleanly() {
        let mut lbt = LbtState::new();
        assert!(!lbt.on_busy(10_000));
        assert!(!lbt.on_busy(10_000 + LBT_STARVATION_MS - 1));
        assert_eq!(lbt.offset(), 0);

        // Full 60 s starvation relaxes by +1 and restarts the 60 s window.
        assert!(lbt.on_busy(10_000 + LBT_STARVATION_MS));
        assert_eq!(lbt.offset(), 1);
        assert!(!lbt.on_busy(10_000 + LBT_STARVATION_MS + 30_000));
        assert_eq!(lbt.offset(), 1);
        assert!(lbt.on_busy(10_000 + 2 * LBT_STARVATION_MS));
        assert_eq!(lbt.offset(), 2);

        // Gradual recovery after 16 consecutive clear passes; a busy hit resets the clean streak.
        for _ in 0..(LBT_RECOVER_CLEAN_COUNT - 1) {
            lbt.on_clear();
        }
        assert_eq!(lbt.offset(), 2);
        assert!(!lbt.on_busy(500_000));
        for _ in 0..(LBT_RECOVER_CLEAN_COUNT - 1) {
            lbt.on_clear();
        }
        assert_eq!(lbt.offset(), 2);
        lbt.on_clear();
        assert_eq!(lbt.offset(), 1);

        // Clamp at LBT_MAX_PEAK_OFFSET.
        let mut t = 1_000_000u64;
        for _ in 0..30 {
            let _ = lbt.on_busy(t);
            t += LBT_STARVATION_MS;
        }
        assert_eq!(lbt.offset(), LBT_MAX_PEAK_OFFSET);
        assert!(!lbt.on_busy(t + LBT_STARVATION_MS));
        assert_eq!(lbt.offset(), LBT_MAX_PEAK_OFFSET);
    }
}
