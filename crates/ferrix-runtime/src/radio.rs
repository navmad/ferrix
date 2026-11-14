//! Radio task: lane scheduling, flood calendar, duty-cycle budget and RX/TX
//! multiplexing. Hardware access goes through [`RadioDevice`].

use core::sync::atomic::Ordering;

use defmt::{debug, error, info};
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use ferrix_core::scheduler::{
    assign_flood, cancel_slot, due_flood, next_flood_due, preamble_for, remember_cancel,
    slot_width_ms, take_cancelled, FloodSlot, TxPacket, CONTROL_QUEUE, FLOOD_MAX_AGE_SLOTS,
    FLOOD_SLOTS, SLOT_MIN_MS, TX_GAP_MS, TX_MAX_ATTEMPTS,
};
use ferrix_core::state::{
    current_public_key, RADIO_SETTINGS, RADIO_TELEMETRY, TX_SLOT_MS_LAST, TX_SLOT_MS_OVERRIDE,
    TX_SLOT_USED, FORWARD_SENT,
};
use ferrix_hal::radio::{LoraConfig, RadioDevice, RxOutcome, TxOutcome};
use heapless::{Deque, Vec};
use meshcore_wire::tx::TxLane;

pub static LORA_RX_CHANNEL: Channel<CriticalSectionRawMutex, (usize, i16, i8, [u8; 256]), 4> =
    Channel::new();
pub static LORA_TX_CHANNEL: Channel<CriticalSectionRawMutex, TxPacket, 8> = Channel::new();
pub static LORA_CFG_CHANNEL: Channel<CriticalSectionRawMutex, LoraConfig, 4> = Channel::new();
/// Hashes of scheduled floods that another repeater was heard for.
pub static LORA_CANCEL_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 8], 8> = Channel::new();

/// Ask the radio task to drop a scheduled flood with this hash.
pub fn cancel_flood(hash: [u8; 8]) {
    let _ = LORA_CANCEL_CHANNEL.try_send(hash);
}

/// Ask the radio task to apply the current `RADIO_SETTINGS`.
pub fn request_radio_config() -> bool {
    let cfg = RADIO_SETTINGS.lock(|r| {
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
    });
    LORA_CFG_CHANNEL.try_send(cfg).is_ok()
}

fn airtime_factor() -> u8 {
    RADIO_SETTINGS.lock(|r| r.borrow().airtime_factor_tenths)
}

fn add_telemetry(update: impl FnOnce(&mut ferrix_core::types::RadioTelemetry)) {
    RADIO_TELEMETRY.lock(|t| update(&mut t.borrow_mut()));
}

/// Run the radio scheduler until the executor stops.
pub async fn radio_task<D: RadioDevice>(mut device: D, initial: LoraConfig) -> ! {
    info!("radio: scheduler started");

    let mut rx_buf = [0u8; 256];
    // If the radio starts returning immediate RX errors, never spin without
    // yielding: the executor is cooperative, and a tight loop here starves the
    // main loop, USB and BLE tasks.
    let mut rx_backoff_ms = 50u64;
    let mut control: Deque<TxPacket, CONTROL_QUEUE> = Deque::new();
    let mut background: Deque<TxPacket, 2> = Deque::new();
    let mut floods: [Option<FloodSlot>; FLOOD_SLOTS] = [None; FLOOD_SLOTS];
    let mut cancelled: Vec<([u8; 8], u64), 16> = Vec::new();
    let af0 = airtime_factor();
    let total0 = ferrix_core::scheduler::tx_budget_max(af0);
    let mut control_budget_ms = total0 * 25 / 100;
    let mut flood_budget_ms = total0 - control_budget_ms;
    let mut tx_budget_last = Instant::now();
    let mut next_tx_allowed_ms: u64 = 0;
    let start_ms = Instant::now().as_millis() as u64;
    let mut base_ms = start_ms;

    let mut current_cfg = (
        initial.frequency_hz,
        initial.tx_power_dbm,
        initial.spreading_factor,
        initial.bandwidth_khz,
        initial.coding_rate,
        initial.rx_duty_cycle,
    );
    let preamble = preamble_for(initial.spreading_factor);
    let mut auto_slot_ms = slot_width_ms(
        initial.spreading_factor,
        initial.bandwidth_khz,
        initial.coding_rate,
        preamble,
    );
    let mut slot_ms = auto_slot_ms;
    let mut slot_override_active = false;
    let mut last_adapt_ms = start_ms;
    let mut last_adapt_events = 0u32;
    let node_key = current_public_key();

    loop {
        let now = Instant::now().as_millis() as u64;

        // Apply cancellations before choosing a slot to transmit. If the
        // packet is still in the channel, remember its hash and drop it on
        // arrival.
        while let Ok(hash) = LORA_CANCEL_CHANNEL.try_receive() {
            if cancel_slot(&mut floods, hash) {
                add_telemetry(|tel| {
                    tel.tx_cancelled = tel.tx_cancelled.saturating_add(1);
                });
            } else {
                remember_cancel(&mut cancelled, hash, now);
            }
        }

        // Runtime slot-width override.
        let override_ms = TX_SLOT_MS_OVERRIDE.load(Ordering::Relaxed) as u64;
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
                let events = RADIO_TELEMETRY.lock(|t| {
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
        TX_SLOT_MS_LAST.store(slot_ms as u32, Ordering::Relaxed);
        TX_SLOT_USED.store(
            floods.iter().filter(|slot| slot.is_some()).count() as u32,
            Ordering::Relaxed,
        );

        // Refill the duty-cycle budget.
        let af = airtime_factor();
        let duty_pct = ferrix_core::scheduler::tx_duty_pct(af);
        let total_max = ferrix_core::scheduler::tx_budget_max(af);
        let control_max = total_max * 25 / 100;
        let flood_max = total_max - control_max;
        let elapsed_ms = now.saturating_sub(tx_budget_last.as_millis() as u64) as i64;
        if elapsed_ms > 0 {
            let total_refill = elapsed_ms * duty_pct / 100;
            let control_refill = total_refill * 25 / 100;
            control_budget_ms = (control_budget_ms + control_refill).min(control_max);
            flood_budget_ms = (flood_budget_ms + total_refill - control_refill).min(flood_max);
            tx_budget_last = Instant::now();
        }
        control_budget_ms = control_budget_ms.min(control_max);
        flood_budget_ms = flood_budget_ms.min(flood_max);

        let preamble = preamble_for(current_cfg.2);
        let mut wake_at = u64::MAX;
        let mut budget_wait_ms = 0u64;
        let mut pick: Option<(TxPacket, TxLane, u32)> = None;

        if now >= next_tx_allowed_ms {
            // Control lane first.
            if let Some(req) = control.front() {
                let airtime = ferrix_core::airtime::estimate_airtime_ms(
                    req.len, current_cfg.2, current_cfg.3, current_cfg.4, preamble,
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
                    let airtime = ferrix_core::airtime::estimate_airtime_ms(
                        req.len, current_cfg.2, current_cfg.3, current_cfg.4, preamble,
                    );
                    if duty_pct >= 100 || flood_budget_ms >= airtime as i64 {
                        let slot = floods[index].take().unwrap();
                        if now > due.saturating_add(FLOOD_MAX_AGE_SLOTS.saturating_mul(slot_ms)) {
                            add_telemetry(|tel| {
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
                    let airtime = ferrix_core::airtime::estimate_airtime_ms(
                        req.len, current_cfg.2, current_cfg.3, current_cfg.4, preamble,
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
                "radio: TX {} bytes ({}, queued {} ms ago)",
                req.len,
                match lane {
                    TxLane::Flood => "flood",
                    TxLane::Background => "bg",
                    TxLane::Control => "control",
                },
                now.saturating_sub(req.queued_at_ms)
            );
            let success = device.transmit(&req.data[..req.len]).await == TxOutcome::Sent;
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
                    FORWARD_SENT.fetch_add(1, Ordering::Relaxed);
                }
                add_telemetry(|tel| {
                    tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                    tel.tx_air_ms = tel.tx_air_ms.saturating_add(airtime);
                    let wait = now.saturating_sub(req.queued_at_ms) as u32;
                    tel.tx_wait_max_ms = tel.tx_wait_max_ms.max(wait);
                    match lane {
                        TxLane::Flood => tel.tx_flood = tel.tx_flood.saturating_add(1),
                        TxLane::Background => {
                            tel.tx_background = tel.tx_background.saturating_add(1)
                        }
                        TxLane::Control => tel.tx_control = tel.tx_control.saturating_add(1),
                    }
                });
            } else {
                add_telemetry(|tel| {
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
                        add_telemetry(|tel| {
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

        // Re-arm RX if no packet arrives for a minute. Safety net for a stuck
        // DIO1 line.
        let rx_fut = embassy_time::with_timeout(
            Duration::from_secs(60),
            device.receive(&mut rx_buf),
        );

        match select(
            select4(
                rx_fut,
                LORA_TX_CHANNEL.receive(),
                LORA_CFG_CHANNEL.receive(),
                Timer::after(Duration::from_millis(wait_ms)),
            ),
            LORA_CANCEL_CHANNEL.receive(),
        )
        .await
        {
            Either::Second(hash) => {
                if cancel_slot(&mut floods, hash) {
                    add_telemetry(|tel| {
                        tel.tx_cancelled = tel.tx_cancelled.saturating_add(1);
                    });
                } else {
                    remember_cancel(&mut cancelled, hash, Instant::now().as_millis() as u64);
                }
            }
            Either::First(Either4::First(rx_result)) => match rx_result {
                Ok(RxOutcome::Packet { len, rssi, snr }) => {
                    rx_backoff_ms = 50;
                    info!("radio: RX {} bytes, RSSI: {} dBm, SNR: {} dB", len, rssi, snr);
                    let copy_len = len.min(256);
                    let mut packet_data = [0u8; 256];
                    packet_data[..copy_len].copy_from_slice(&rx_buf[..copy_len]);
                    let _ = LORA_RX_CHANNEL.try_send((copy_len, rssi, snr, packet_data));
                }
                Ok(RxOutcome::DutyCycleEnd) => {}
                Ok(RxOutcome::Error) => {
                    if rx_backoff_ms <= 50 {
                        debug!("radio: RX error");
                    }
                    // Yield first. Without this, an immediate RX error after a
                    // failed reconfiguration turns this task into a busy loop.
                    Timer::after_millis(rx_backoff_ms).await;
                    rx_backoff_ms = (rx_backoff_ms * 2).min(1000);
                    device.recover_receive().await;
                }
                Err(_) => {
                    debug!("radio: RX watchdog re-arm");
                    device.recover_receive().await;
                }
            },
            Either::First(Either4::Second(req)) => {
                let received_at = Instant::now().as_millis() as u64;
                if take_cancelled(&mut cancelled, req.hash) {
                    add_telemetry(|tel| {
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
                        add_telemetry(|tel| {
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
                    continue;
                }
                if device.configure(&cfg).await {
                    current_cfg = requested;
                    auto_slot_ms =
                        slot_width_ms(current_cfg.2, current_cfg.3, current_cfg.4, preamble_for(current_cfg.2));
                    slot_ms = auto_slot_ms;
                    base_ms = Instant::now().as_millis() as u64;
                    last_adapt_ms = base_ms;
                    rx_backoff_ms = 50;
                    info!(
                        "radio: reconfigured to {} Hz (SF{}, BW{}k, CR 4/{}), {} dBm",
                        cfg.frequency_hz,
                        cfg.spreading_factor,
                        cfg.bandwidth_khz,
                        cfg.coding_rate,
                        cfg.tx_power_dbm
                    );
                } else {
                    error!("radio: reconfigure failed; keeping previous settings");
                }
            }
            Either::First(Either4::Fourth(_)) => {}
        }
    }
}
