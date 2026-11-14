//! lora-phy Subsystem Adapter for ThinkNode M1 (SX1262)
//!
//! Bridges the community-standard `lora-phy` crate into Ferrix
//! using an asynchronous Embassy radio task and bounded IPC channels.

use core::sync::atomic::Ordering;

use defmt::{debug, error, info};
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::peripherals::TWISPI0;
use embassy_nrf::spim::Spim;
use embassy_sync::channel::Channel;

/// Shared raw mutex type — see usb.rs for documentation.
type SharedRawMutex = embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::mod_params::{Bandwidth, CodingRate, DutyCycleParams, RadioError, RxMode, SpreadingFactor};
use lora_phy::sx126x::{Config as Sx126xConfig, Sx1262, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;
use meshcore_wire::tx::TxLane;

use crate::radio::{LoraConfig, LoraRadio};
use ferrix_core::scheduler::{
    assign_flood, cancel_slot, due_flood, next_flood_due, preamble_for, remember_cancel,
    slot_width_ms, take_cancelled, FloodSlot, TxPacket, CONTROL_QUEUE, FLOOD_MAX_AGE_SLOTS,
    FLOOD_SLOTS, SLOT_MIN_MS, TX_GAP_MS, TX_MAX_ATTEMPTS,
};

pub static LORA_RX_CHANNEL: Channel<SharedRawMutex, (usize, i16, i8, [u8; 256]), 4> = Channel::new();
pub static LORA_TX_CHANNEL: Channel<SharedRawMutex, TxPacket, 8> = Channel::new();
pub static LORA_CFG_CHANNEL: Channel<SharedRawMutex, LoraConfig, 4> = Channel::new();
/// Hashes of scheduled floods that another repeater was heard for.
pub static LORA_CANCEL_CHANNEL: Channel<SharedRawMutex, [u8; 8], 8> = Channel::new();

/// Ask the radio task to drop a scheduled flood with this hash.
pub fn cancel_flood(hash: [u8; 8]) {
    let _ = LORA_CANCEL_CHANNEL.try_send(hash);
}

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
            rx_duty_cycle: s.rx_duty_cycle,
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

fn airtime_factor() -> u8 {
    crate::companion::usb::RADIO_SETTINGS
        .lock(|r| r.borrow().airtime_factor_tenths)
}

/// ThinkNode M1 LoRa Radio Adapter implementing `LoraRadio` via channels
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
        self.transmit_lane(payload, TxLane::Control, now, [0; 8]).await
    }

    async fn transmit_lane(
        &mut self,
        payload: &[u8],
        lane: TxLane,
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
        // The caller keeps the packet queued and retries when this fails.
        LORA_TX_CHANNEL.try_send(pkt).map_err(|_| ())
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

    async fn receive(&mut self, buf: &mut [u8]) -> (usize, i16, i8) {
        let (len, rssi, snr, data) = LORA_RX_CHANNEL.receive().await;
        let copy_len = len.min(buf.len());
        buf[..copy_len].copy_from_slice(&data[..copy_len]);
        (copy_len, rssi, snr)
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

    // 4. Configure SX1262: 1.8V TCXO (matches MeshCore ThinkNode M1),
    //    DCDC enabled, and RX boosted gain for ~3 dB better sensitivity.
    let config = Sx126xConfig {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V8),
        use_dcdc: true,
        rx_boost: true,
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
    let (freq_hz, tx_power, sf, bw_khz, cr, duty_cycle) =
        crate::companion::usb::RADIO_SETTINGS.lock(|r| {
            let s = r.borrow();
            (s.frequency_hz, s.tx_power_dbm, s.sf, s.bw_khz, s.cr, s.rx_duty_cycle)
        });
    let mut tx_power_dbm = tx_power;
    let mut preamble = preamble_for(sf);
    let mut rx_mode = make_rx_mode(sf, bw_khz, duty_cycle);
    let mut current_cfg = (freq_hz, tx_power, sf, bw_khz, cr, duty_cycle);
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

    // 8. Prepare for reception (continuous or duty cycle).
    if let Err(e) = lora.prepare_for_rx(rx_mode, &mdltn_params, &rx_pkt_params).await {
        error!("lora-phy: Failed to prepare for RX: {:?}", e);
        return;
    }

    info!(
        "lora-phy: Radio listening on {} Hz (SF{}/BW{}k, CR 4/{}, RX {})",
        freq_hz, sf, bw_khz, cr, rx_mode
    );

    let mut rx_buf = [0u8; 256];
    // If the radio starts returning immediate RX errors, never spin without
    // yielding: the executor is cooperative, and a tight loop here starves the
    // main loop, USB and BLE tasks.
    let mut rx_backoff_ms = 50u64;
    let mut control: heapless::Deque<TxPacket, CONTROL_QUEUE> = heapless::Deque::new();
    let mut background: heapless::Deque<TxPacket, 2> = heapless::Deque::new();
    let mut floods: [Option<FloodSlot>; FLOOD_SLOTS] = [None, None, None, None];
    let mut cancelled: heapless::Vec<([u8; 8], u64), 16> = heapless::Vec::new();
    let af0 = airtime_factor();
    let total0 = crate::radio::tx_budget_max(af0);
    let mut control_budget_ms = total0 * 25 / 100;
    let mut flood_budget_ms = total0 - control_budget_ms;
    let mut tx_budget_last = Instant::now();
    let mut next_tx_allowed_ms: u64 = 0;
    let start_ms = Instant::now().as_millis() as u64;
    let mut base_ms = start_ms;
    let mut auto_slot_ms = slot_width_ms(current_cfg.2, current_cfg.3, current_cfg.4, preamble);
    let mut slot_ms = auto_slot_ms;
    let mut slot_override_active = false;
    let mut last_adapt_ms = start_ms;
    let mut last_adapt_events = 0u32;
    let node_key = crate::companion::usb::current_public_key();

    loop {
        let now = Instant::now().as_millis() as u64;

        // Apply cancellations before choosing a slot to transmit. If the
        // packet is still in the channel, remember its hash and drop it on
        // arrival.
        while let Ok(hash) = LORA_CANCEL_CHANNEL.try_receive() {
            if cancel_slot(&mut floods, hash) {
                crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                    let mut tel = t.borrow_mut();
                    tel.tx_cancelled = tel.tx_cancelled.saturating_add(1);
                });
            } else {
                remember_cancel(&mut cancelled, hash, now);
            }
        }

        // Runtime slot-width override.
        let override_ms = crate::radio::TX_SLOT_MS_OVERRIDE.load(Ordering::Relaxed) as u64;
        if override_ms != 0 {
            if override_ms != slot_ms || !slot_override_active {
                slot_ms = override_ms.clamp(SLOT_MIN_MS, 5000);
                base_ms = now;
                slot_override_active = true;
            }
        } else if slot_override_active {
            slot_ms = auto_slot_ms;
            base_ms = now;
            slot_override_active = false;
        }
        if override_ms == 0
            && !slot_override_active
            && now.saturating_sub(last_adapt_ms) >= 30_000
        {
            if floods.iter().all(|slot| slot.is_none()) {
                let events = crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                    let tel = t.borrow();
                    tel.tx_stale.saturating_add(tel.tx_cancelled)
                });
                if events > last_adapt_events {
                    slot_ms = (slot_ms * 110 / 100).min(auto_slot_ms * 2);
                } else {
                    slot_ms = (slot_ms * 95 / 100)
                        .max(auto_slot_ms / 2)
                        .max(SLOT_MIN_MS);
                }
                base_ms = now;
                last_adapt_ms = now;
                last_adapt_events = events;
            } else {
                // Retry after the calendar drains.
                last_adapt_ms = now;
            }
        }
        crate::radio::TX_SLOT_MS_LAST.store(slot_ms as u32, Ordering::Relaxed);
        crate::radio::TX_SLOT_USED.store(
            floods.iter().filter(|slot| slot.is_some()).count() as u32,
            Ordering::Relaxed,
        );

        // Refill the duty-cycle budget.
        let af = airtime_factor();
        let duty_pct = crate::radio::tx_duty_pct(af);
        let total_max = crate::radio::tx_budget_max(af);
        let control_max = total_max * 25 / 100;
        let flood_max = total_max - control_max;
        let elapsed_ms = now.saturating_sub(tx_budget_last.as_millis() as u64) as i64;
        if elapsed_ms > 0 {
            let total_refill = elapsed_ms * duty_pct / 100;
            let control_refill = total_refill * 25 / 100;
            control_budget_ms = (control_budget_ms + control_refill).min(control_max);
            flood_budget_ms =
                (flood_budget_ms + total_refill - control_refill).min(flood_max);
            tx_budget_last = Instant::now();
        }
        control_budget_ms = control_budget_ms.min(control_max);
        flood_budget_ms = flood_budget_ms.min(flood_max);

        let mut wake_at = u64::MAX;
        let mut budget_wait_ms = 0u64;
        let mut pick: Option<(TxPacket, TxLane, u32)> = None;

        if now >= next_tx_allowed_ms {
            // Control lane first.
            if let Some(req) = control.front() {
                let airtime = crate::radio::estimate_airtime_ms(
                    req.len,
                    current_cfg.2,
                    current_cfg.3,
                    current_cfg.4,
                    preamble,
                );
                if duty_pct >= 100 || control_budget_ms >= airtime as i64 {
                    let req = control.pop_front().unwrap();
                    pick = Some((req, TxLane::Control, airtime));
                } else if duty_pct > 0 {
                    let deficit = airtime as i64 - control_budget_ms;
                    budget_wait_ms = (deficit * 10_000 / (duty_pct * 25)).max(1) as u64;
                }
            }

            // Then the next due flood slot.
            if pick.is_none() {
                if let Some((index, due)) = due_flood(&floods, now, base_ms, slot_ms) {
                    let req = floods[index].as_ref().unwrap().req;
                    let airtime = crate::radio::estimate_airtime_ms(
                        req.len,
                        current_cfg.2,
                        current_cfg.3,
                        current_cfg.4,
                        preamble,
                    );
                    if duty_pct >= 100 || flood_budget_ms >= airtime as i64 {
                        let slot = floods[index].take().unwrap();
                        if now
                            > due.saturating_add(
                                FLOOD_MAX_AGE_SLOTS.saturating_mul(slot_ms),
                            )
                        {
                            crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                                let mut tel = t.borrow_mut();
                                tel.tx_stale = tel.tx_stale.saturating_add(1);
                            });
                            continue;
                        }
                        pick = Some((slot.req, TxLane::Flood, airtime));
                    } else if duty_pct > 0 {
                        let deficit = airtime as i64 - flood_budget_ms;
                        let wait = (deficit * 10_000 / (duty_pct * 75)).max(1) as u64;
                        budget_wait_ms = budget_wait_ms.max(wait);
                    }
                }
            }

            // Background adverts only when neither lane has work ready.
            if pick.is_none() {
                if let Some(req) = background.front() {
                    let airtime = crate::radio::estimate_airtime_ms(
                        req.len,
                        current_cfg.2,
                        current_cfg.3,
                        current_cfg.4,
                        preamble,
                    );
                    if duty_pct >= 100 || flood_budget_ms >= airtime as i64 {
                        let req = background.pop_front().unwrap();
                        pick = Some((req, TxLane::Background, airtime));
                    } else if duty_pct > 0 {
                        let deficit = airtime as i64 - flood_budget_ms;
                        let wait = (deficit * 10_000 / (duty_pct * 75)).max(1) as u64;
                        budget_wait_ms = budget_wait_ms.max(wait);
                    }
                }
            }
        }

        if let Some((req, lane, airtime)) = pick {
            info!(
                "lora-phy: TX {} bytes ({}, queued {} ms ago)",
                req.len,
                match lane {
                    TxLane::Flood => "flood",
                    TxLane::Background => "bg",
                    TxLane::Control => "control",
                },
                now.saturating_sub(req.queued_at_ms)
            );
            let tx_result = embassy_time::with_timeout(
                Duration::from_secs(20),
                async {
                    let mut tx_pkt_params = lora.create_tx_packet_params(
                        preamble,
                        false,
                        true,
                        false,
                        &mdltn_params,
                    )?;
                    lora.prepare_for_tx(
                        &mdltn_params,
                        &mut tx_pkt_params,
                        tx_power_dbm as i32,
                        &req.data[..req.len],
                    )
                    .await?;
                    lora.tx().await
                },
            )
            .await;
            let success = matches!(tx_result, Ok(Ok(())));
            match tx_result {
                Ok(Ok(())) => info!("lora-phy: Transmission complete!"),
                Ok(Err(e)) => error!("lora-phy: Transmission failed: {:?}", e),
                Err(_) => error!("lora-phy: Transmission timed out; forcing RX recovery"),
            }
            let _ = embassy_time::with_timeout(
                Duration::from_secs(1),
                lora.prepare_for_rx(rx_mode, &mdltn_params, &rx_pkt_params),
            )
            .await;

            if success {
                next_tx_allowed_ms = Instant::now().as_millis() as u64 + TX_GAP_MS;
                if duty_pct < 100 {
                    if lane == TxLane::Control {
                        control_budget_ms = (control_budget_ms - airtime as i64).max(0);
                    } else {
                        flood_budget_ms = (flood_budget_ms - airtime as i64).max(0);
                    }
                }
                if lane == TxLane::Flood {
                    crate::radio::FORWARD_SENT.fetch_add(1, Ordering::Relaxed);
                }
                crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                    let mut tel = t.borrow_mut();
                    tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                    tel.tx_air_ms = tel.tx_air_ms.saturating_add(airtime);
                    let wait = now.saturating_sub(req.queued_at_ms) as u32;
                    tel.tx_wait_max_ms = tel.tx_wait_max_ms.max(wait);
                    match lane {
                        TxLane::Flood => {
                            tel.tx_flood = tel.tx_flood.saturating_add(1);
                        }
                        TxLane::Background => {
                            tel.tx_background = tel.tx_background.saturating_add(1);
                        }
                        TxLane::Control => {
                            tel.tx_control = tel.tx_control.saturating_add(1);
                        }
                    }
                });
            } else {
                crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                    let mut tel = t.borrow_mut();
                    tel.tx_retries = tel.tx_retries.saturating_add(1);
                    if req.attempts + 1 >= TX_MAX_ATTEMPTS {
                        tel.tx_dropped = tel.tx_dropped.saturating_add(1);
                    }
                });
                if req.attempts + 1 < TX_MAX_ATTEMPTS {
                    let mut retry = req;
                    retry.attempts += 1;
                    let now_retry = Instant::now().as_millis() as u64;
                    let requeued = match lane {
                        TxLane::Flood => assign_flood(
                            &mut floods,
                            retry,
                            now_retry,
                            base_ms,
                            slot_ms,
                            &node_key,
                        )
                        .is_ok(),
                        TxLane::Background => background.push_back(retry).is_ok(),
                        TxLane::Control => control.push_back(retry).is_ok(),
                    };
                    if !requeued {
                        crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                            let mut tel = t.borrow_mut();
                            tel.tx_dropped = tel.tx_dropped.saturating_add(1);
                        });
                    }
                }
            }
            continue;
        }

        if now < next_tx_allowed_ms {
            wake_at = next_tx_allowed_ms;
        }
        if let Some(due) = next_flood_due(&floods, base_ms, slot_ms) {
            if due > now {
                wake_at = wake_at.min(due);
            }
        }
        if budget_wait_ms > 0 {
            wake_at = wake_at.min(now.saturating_add(budget_wait_ms));
        }
        if (!control.is_empty() || !background.is_empty()) && budget_wait_ms == 0 {
            wake_at = now;
        }
        let wait_ms = if wake_at == u64::MAX {
            60_000
        } else {
            wake_at.saturating_sub(now).clamp(1, 60_000)
        };

    // Re-arm RX if no packet arrives for a minute. This is a safety net
        // for the case where the SX1262 stops raising DIO1 (for example after
        // a cancelled TX/RX operation) and `lora.rx` would otherwise wait
        // forever: the task would still be alive but permanently deaf.
        let rx_fut = embassy_time::with_timeout(
            Duration::from_secs(60),
            lora.rx(&rx_pkt_params, &mut rx_buf),
        );

        match select(
            select4(
                rx_fut,
                LORA_TX_CHANNEL.receive(),
                LORA_CFG_CHANNEL.receive(),
                Timer::after(Duration::from_millis(wait_ms as u64)),
            ),
            LORA_CANCEL_CHANNEL.receive(),
        )
        .await
        {
            Either::Second(hash) => {
                if cancel_slot(&mut floods, hash) {
                    crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                        let mut tel = t.borrow_mut();
                        tel.tx_cancelled = tel.tx_cancelled.saturating_add(1);
                    });
                } else {
                    remember_cancel(&mut cancelled, hash, Instant::now().as_millis() as u64);
                }
            }
            Either::First(Either4::First(rx_result)) => match rx_result {
                Ok(Ok((len, status))) => {
                    rx_backoff_ms = 50;
                    info!("lora-phy: RX {} bytes, RSSI: {} dBm, SNR: {} dB", len, status.rssi, status.snr);
                    let mut packet_data = [0u8; 256];
                    let copy_len = (len as usize).min(256);
                    packet_data[..copy_len].copy_from_slice(&rx_buf[..copy_len]);
                    let _ = LORA_RX_CHANNEL.try_send((copy_len, status.rssi, status.snr as i8, packet_data));
                }
                Ok(Err(e)) => {
                    // A duty-cycle window ending without a preamble is normal.
                    if matches!(rx_mode, RxMode::DutyCycle(_)) && e == RadioError::ReceiveTimeout {
                        match embassy_time::with_timeout(
                            Duration::from_secs(1),
                            lora.prepare_for_rx(rx_mode, &mdltn_params, &rx_pkt_params),
                        )
                        .await
                        {
                            Ok(Ok(())) => continue,
                            Ok(Err(e)) => error!("lora-phy: duty-cycle re-arm failed: {:?}", e),
                            Err(_) => error!("lora-phy: duty-cycle re-arm timed out"),
                        }
                    }

                    if rx_backoff_ms <= 50 {
                        debug!("lora-phy: RX error or timeout: {:?}", e);
                    }
                    // Yield first. Without this, an immediate RX error after a
                    // failed reconfiguration turns this task into a busy loop
                    // and the whole executor stops running.
                    Timer::after_millis(rx_backoff_ms).await;
                    rx_backoff_ms = (rx_backoff_ms * 2).min(1000);

                    // Re-enter RX if an error occurred, but never let a stuck
                    // BUSY line wedge the whole task forever.
                    match embassy_time::with_timeout(
                        Duration::from_secs(1),
                        lora.prepare_for_rx(rx_mode, &mdltn_params, &rx_pkt_params),
                    )
                    .await
                    {
                        Ok(Ok(())) => {}
                        Ok(Err(e)) => error!("lora-phy: re-enter RX failed: {:?}", e),
                        Err(_) => error!("lora-phy: re-enter RX timed out"),
                    }
                }
                Err(_) => {
                    // RX watchdog fired: no DIO1 for 60 s. Re-arm the receiver.
                    debug!("lora-phy: RX watchdog re-arm");
                    match embassy_time::with_timeout(
                        Duration::from_secs(1),
                        lora.prepare_for_rx(rx_mode, &mdltn_params, &rx_pkt_params),
                    )
                    .await
                    {
                        Ok(Ok(())) => {}
                        Ok(Err(e)) => error!("lora-phy: watchdog re-arm failed: {:?}", e),
                        Err(_) => error!("lora-phy: watchdog re-arm timed out"),
                    }
                }
            },
            Either::First(Either4::Second(req)) => {
                let received_at = Instant::now().as_millis() as u64;
                if take_cancelled(&mut cancelled, req.hash) {
                    crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                        let mut tel = t.borrow_mut();
                        tel.tx_cancelled = tel.tx_cancelled.saturating_add(1);
                    });
                } else {
                    let queued = match req.lane {
                        TxLane::Flood => assign_flood(
                            &mut floods,
                            req,
                            received_at,
                            base_ms,
                            slot_ms,
                            &node_key,
                        )
                        .is_ok(),
                        TxLane::Background => background.push_back(req).is_ok(),
                        TxLane::Control => control.push_back(req).is_ok(),
                    };
                    if !queued {
                        crate::companion::usb::RADIO_TELEMETRY.lock(|t| {
                            let mut tel = t.borrow_mut();
                            tel.tx_dropped = tel.tx_dropped.saturating_add(1);
                        });
                    }
                }
            }
            Either::First(Either4::Third(cfg)) => {
                let requested = (
                    cfg.frequency_hz,
                    cfg.tx_power_dbm,
                    cfg.spreading_factor,
                    cfg.bandwidth_khz,
                    cfg.coding_rate,
                    cfg.rx_duty_cycle,
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
                                rx_mode = make_rx_mode(
                                    cfg.spreading_factor,
                                    cfg.bandwidth_khz,
                                    cfg.rx_duty_cycle,
                                );
                                current_cfg = requested;
                                auto_slot_ms = slot_width_ms(
                                    current_cfg.2,
                                    current_cfg.3,
                                    current_cfg.4,
                                    preamble,
                                );
                                slot_ms = auto_slot_ms;
                                base_ms = Instant::now().as_millis() as u64;
                                last_adapt_ms = base_ms;
                                rx_backoff_ms = 50;
                                match embassy_time::with_timeout(
                                    Duration::from_secs(1),
                                    lora.prepare_for_rx(
                                        rx_mode,
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
                                    "lora-phy: reconfigured to {} Hz (SF{}, BW{}k, CR 4/{}), {} dBm, RX {}",
                                    cfg.frequency_hz,
                                    cfg.spreading_factor,
                                    cfg.bandwidth_khz,
                                    cfg.coding_rate,
                                    cfg.tx_power_dbm,
                                    rx_mode
                                );
                            }
                            Err(e) => {
                                error!("lora-phy: RX params on reconfigure failed: {:?}", e);
                                // The previous RX future was cancelled; restore it
                                // rather than leaving the radio idle and erroring.
                                let _ = embassy_time::with_timeout(
                                    Duration::from_secs(1),
                                    lora.prepare_for_rx(
                                        rx_mode,
                                        &mdltn_params,
                                        &rx_pkt_params,
                                    ),
                                )
                                .await;
                            }
                        }
                    }
                    Err(e) => {
                        error!("lora-phy: reconfigure failed: {:?}", e);
                        let _ = embassy_time::with_timeout(
                            Duration::from_secs(1),
                            lora.prepare_for_rx(
                                rx_mode,
                                &mdltn_params,
                                &rx_pkt_params,
                            ),
                        )
                        .await;
                    }
                }
            }
            Either::First(Either4::Fourth(_)) => {
                // Timer fired: loop around and re-evaluate eligibility and the
                // duty-cycle budget. RX remains armed.
            }
        }
    }
}
