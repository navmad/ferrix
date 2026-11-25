//! Radio task: lane scheduling, flood calendar, duty-cycle budget and RX/TX
//! multiplexing. Hardware access goes through [`RadioDevice`].

use core::sync::atomic::Ordering;

use defmt::{debug, error, info};
use embassy_futures::select::{select, select4, Either, Either4};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use ferrix_core::noise_floor::{AgcWatchdog, NoiseFloorEstimator, NOISE_SAMPLE_INTERVAL_MS};
use ferrix_core::scheduler::{
    assign_flood, cancel_slot, charge_control, control_can_send, due_control, due_flood,
    next_flood_due, packet_wait_ms, preamble_for, preamble_wait_ms, rebase_floods, refill_budgets,
    remember_cancel, slot_width_ms, take_cancelled, FloodSlot, TxPacket, CONTROL_QUEUE,
    FLOOD_MAX_AGE_SLOTS, FLOOD_SLOTS, SLOT_MIN_MS, TX_GAP_MS, TX_MAX_ATTEMPTS,
};
use ferrix_core::state::{
    current_public_key, note_core_err, set_tx_queue_len, take_agc_reset_request,
    CORE_ERR_AGC_RECOVERY, CORE_ERR_RADIO_RX, CORE_ERR_TX_FAILED, FORWARD_SENT, RADIO_SETTINGS,
    RADIO_TELEMETRY, TX_SLOT_MS_LAST, TX_SLOT_MS_OVERRIDE, TX_SLOT_USED,
};
use ferrix_hal::radio::{LoraConfig, LoraRadio, RadioDevice, RxOutcome, RxState, TxOutcome};
use heapless::{Deque, Vec};
use meshcore_wire::tx::TxLane;

pub static LORA_RX_CHANNEL: Channel<CriticalSectionRawMutex, (usize, i16, i8, [u8; 256]), 8> =
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

/// LoRa radio adapter implementing `LoraRadio` through the runtime channels.
///
/// This is transport plumbing, not board code: it only turns a synchronous
/// radio handle into the channel-driven interface used by the mesh engine.
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
        self.transmit_lane(payload, TxLane::Control, now, [0; 8])
            .await
    }

    async fn transmit_lane(
        &mut self,
        payload: &[u8],
        lane: TxLane,
        queued_at_ms: u64,
        hash: [u8; 8],
    ) -> Result<(), Self::Error> {
        self.transmit_lane_at(payload, lane, queued_at_ms, queued_at_ms, hash)
            .await
    }

    async fn transmit_lane_at(
        &mut self,
        payload: &[u8],
        lane: TxLane,
        queued_at_ms: u64,
        due_at_ms: u64,
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
            due_at_ms,
            hash,
            attempts: 0,
        };
        pkt.data[..payload.len()].copy_from_slice(payload);
        LORA_TX_CHANNEL.try_send(pkt).map_err(|_| ())
    }

    async fn start_receive(&mut self) {}

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

fn airtime_factor() -> u8 {
    RADIO_SETTINGS.lock(|r| r.borrow().airtime_factor_tenths)
}

fn add_telemetry(update: impl FnOnce(&mut ferrix_core::types::RadioTelemetry)) {
    RADIO_TELEMETRY.lock(|t| update(&mut t.borrow_mut()));
}

#[inline(never)]
fn poll_noinline<F: core::future::Future>(
    fut: core::pin::Pin<&mut F>,
    cx: &mut core::task::Context<'_>,
) -> core::task::Poll<F::Output> {
    fut.poll(cx)
}

macro_rules! await_noinline {
    ($fut:expr) => {{
        let mut fut = core::pin::pin!($fut);
        core::future::poll_fn(|cx| poll_noinline(fut.as_mut(), cx)).await
    }};
}

/// Run the radio scheduler until the executor stops.
pub async fn radio_task<D: RadioDevice>(mut device: D, initial: LoraConfig) -> ! {
    info!("radio: scheduler started");

    let mut rx_buf = [0u8; 256];
    // If the radio starts returning consecutive immediate RX errors, back off
    // before recovering so the cooperative executor is never starved, while
    // recovering immediately (0 ms delay) on the first transient error.
    let mut rx_backoff_ms = 0u64;
    let mut control: Vec<TxPacket, CONTROL_QUEUE> = Vec::new();
    let mut background: Deque<TxPacket, 2> = Deque::new();
    let mut floods: [Option<FloodSlot>; FLOOD_SLOTS] = [None; FLOOD_SLOTS];
    let mut cancelled: Vec<([u8; 8], u64), 16> = Vec::new();
    let af0 = airtime_factor();
    let total0 = ferrix_core::scheduler::tx_budget_max(af0);
    let mut control_budget_ms = total0 * 25 / 100;
    let mut flood_budget_ms = total0 - control_budget_ms;
    let mut control_rem = 0i64;
    let mut flood_rem = 0i64;
    let start_ms = Instant::now().as_millis() as u64;
    let mut tx_budget_last_ms = start_ms;
    let mut next_tx_allowed_ms: u64 = 0;
    let mut base_ms = start_ms;
    let mut last_rx_activity_ms = start_ms;
    let mut rx_active_since: Option<(RxState, u64)> = None;
    let mut channel_busy_since: Option<u64> = None;

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
    let mut noise_floor = NoiseFloorEstimator::new();
    let mut agc_watchdog = AgcWatchdog::new(start_ms);
    let mut next_noise_sample_ms = start_ms.saturating_add(1_000);

    loop {
        let mut now = Instant::now().as_millis() as u64;
        // Re-arm RX if no IRQ or watchdog recovery occurred for a minute.
        if now.saturating_sub(last_rx_activity_ms) >= 60_000 {
            debug!("radio: RX watchdog re-arm");
            device.recover_receive().await;
            rx_active_since = None;
            channel_busy_since = None;
            now = Instant::now().as_millis() as u64;
            last_rx_activity_ms = now;
        }

        // Re-read every iteration: a private-key import changes the node hash
        // used to place floods, and must take effect without a reboot.
        let node_key = current_public_key();

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
            let clamped = override_ms.clamp(SLOT_MIN_MS, 5000);
            if clamped != slot_ms || !slot_override_active {
                let stale = rebase_floods(&mut floods, now, base_ms, slot_ms, clamped);
                if stale > 0 {
                    add_telemetry(|tel| {
                        tel.tx_stale = tel.tx_stale.saturating_add(stale);
                    });
                }
                slot_ms = clamped;
                base_ms = now;
                slot_override_active = true;
            }
        } else if slot_override_active {
            let stale = rebase_floods(&mut floods, now, base_ms, slot_ms, auto_slot_ms);
            if stale > 0 {
                add_telemetry(|tel| {
                    tel.tx_stale = tel.tx_stale.saturating_add(stale);
                });
            }
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
        let flood_used = floods.iter().filter(|slot| slot.is_some()).count();
        TX_SLOT_MS_LAST.store(slot_ms as u32, Ordering::Relaxed);
        TX_SLOT_USED.store(flood_used as u32, Ordering::Relaxed);
        let pending_tx = control.len() + flood_used + background.len() + LORA_TX_CHANNEL.len();
        set_tx_queue_len(pending_tx.min(u8::MAX as usize) as u8);

        // Refill the duty-cycle budget.
        let af = airtime_factor();
        let duty_pct = ferrix_core::scheduler::tx_duty_pct(af);
        let total_max = ferrix_core::scheduler::tx_budget_max(af);
        let control_max = total_max * 25 / 100;
        let flood_max = total_max - control_max;
        let elapsed_ms = now.saturating_sub(tx_budget_last_ms) as i64;
        refill_budgets(
            elapsed_ms,
            duty_pct,
            control_max,
            flood_max,
            &mut control_budget_ms,
            &mut flood_budget_ms,
            &mut control_rem,
            &mut flood_rem,
        );
        tx_budget_last_ms = now;

        let preamble = preamble_for(current_cfg.2);
        let mut wake_at = last_rx_activity_ms.saturating_add(60_000);
        let mut budget_wait_ms = 0u64;
        let mut pick: Option<(TxPacket, TxLane, u32)> = None;

        // Avoid transmitting on top of an inbound frame with a valid header.
        // Only recover RX when a confirmed Header stalls past packet_wait_ms;
        // never tear down continuous RX on a transient Preamble latch.
        let current_rx_state = device.rx_state();
        let rx_busy = match current_rx_state {
            RxState::Idle => {
                rx_active_since = None;
                false
            }
            RxState::Preamble | RxState::Header => {
                let (tracked_state, since) = match rx_active_since {
                    Some((st, ts)) if st == current_rx_state => (st, ts),
                    _ => {
                        rx_active_since = Some((current_rx_state, now));
                        (current_rx_state, now)
                    }
                };
                let limit_ms = match tracked_state {
                    RxState::Preamble => preamble_wait_ms(current_cfg.2, current_cfg.3, preamble),
                    RxState::Header => {
                        packet_wait_ms(current_cfg.2, current_cfg.3, current_cfg.4, preamble)
                    }
                    RxState::Idle => 0,
                };
                let until = since.saturating_add(limit_ms);
                if now < until {
                    wake_at = wake_at.min(until);
                    true
                } else {
                    if tracked_state == RxState::Header {
                        note_core_err(CORE_ERR_RADIO_RX);
                        device.recover_receive().await;
                        now = Instant::now().as_millis() as u64;
                        last_rx_activity_ms = now;
                    }
                    rx_active_since = None;
                    false
                }
            }
        };

        let manual_agc_reset = !rx_busy && take_agc_reset_request();
        let auto_agc_reset = !current_cfg.5 && !rx_busy && agc_watchdog.should_reset(now);
        if manual_agc_reset || auto_agc_reset {
            if manual_agc_reset {
                info!("radio: manual SX1262 AGC reset");
            } else {
                info!("radio: deaf-receiver watchdog resetting SX1262 AGC");
                note_core_err(CORE_ERR_AGC_RECOVERY);
            }
            let ok = await_noinline!(device.reset_agc());
            now = Instant::now().as_millis() as u64;
            rx_active_since = None;
            last_rx_activity_ms = now;
            agc_watchdog.on_agc_reset(now);
            next_noise_sample_ms = now.saturating_add(1_000);
            add_telemetry(|tel| {
                tel.agc_frozen_streak = 0;
                if ok {
                    tel.agc_resets = tel.agc_resets.saturating_add(1);
                }
            });
            continue;
        }

        if now >= next_tx_allowed_ms && !rx_busy {
            // Control lane first (highest-priority packet whose due_at_ms has arrived,
            // with FIFO tie-breaking within each priority class).
            if let Some(idx) = due_control(&control, now) {
                let airtime = ferrix_core::airtime::estimate_airtime_ms(
                    control[idx].len,
                    current_cfg.2,
                    current_cfg.3,
                    current_cfg.4,
                    preamble,
                );
                if duty_pct >= 100
                    || control_can_send(control_budget_ms, flood_budget_ms, airtime as i64)
                {
                    let req = control.remove(idx);
                    pick = Some((req, TxLane::Control, airtime));
                } else if duty_pct > 0 {
                    // Both shares refill, so the deficit closes at the full rate.
                    let deficit =
                        airtime as i64 - control_budget_ms.max(0) - flood_budget_ms.max(0);
                    budget_wait_ms = (deficit * 10_000 / (duty_pct * 100)).max(1) as u64;
                }
            }

            // Then the next due flood slot (evict stale slots regardless of budget).
            if pick.is_none() {
                if let Some((index, due)) = due_flood(&floods, now, base_ms, slot_ms) {
                    if now > due.saturating_add(FLOOD_MAX_AGE_SLOTS.saturating_mul(slot_ms)) {
                        floods[index] = None;
                        add_telemetry(|tel| {
                            tel.tx_stale = tel.tx_stale.saturating_add(1);
                        });
                        continue;
                    }
                    let req = floods[index].as_ref().unwrap().req;
                    let airtime = ferrix_core::airtime::estimate_airtime_ms(
                        req.len, current_cfg.2, current_cfg.3, current_cfg.4, preamble,
                    );
                    if duty_pct >= 100 || flood_budget_ms >= airtime as i64 {
                        let slot = floods[index].take().unwrap();
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
            let outcome = device.transmit(&req.data[..req.len]).await;
            let after_tx = Instant::now().as_millis() as u64;
            match outcome {
                TxOutcome::Sent => {
                    rx_active_since = None;
                    channel_busy_since = None;
                    last_rx_activity_ms = after_tx;
                    next_tx_allowed_ms = after_tx + TX_GAP_MS;
                    next_noise_sample_ms =
                        next_noise_sample_ms.max(after_tx.saturating_add(1_000));
                    if duty_pct < 100 {
                        if lane == TxLane::Control {
                            charge_control(
                                &mut control_budget_ms,
                                &mut flood_budget_ms,
                                airtime as i64,
                            );
                        } else {
                            flood_budget_ms = (flood_budget_ms - airtime as i64).max(0);
                        }
                    }
                    if lane == TxLane::Flood {
                        FORWARD_SENT.fetch_add(1, Ordering::Relaxed);
                    }
                    let route = req.data[0] & 0x03;
                    let is_flood_route = route == meshcore_wire::packet::ROUTE_TYPE_FLOOD
                        || route == meshcore_wire::packet::ROUTE_TYPE_TRANSPORT_FLOOD;
                    add_telemetry(|tel| {
                        tel.tx_beacons = tel.tx_beacons.saturating_add(1);
                        tel.tx_air_ms = tel.tx_air_ms.saturating_add(airtime);
                        let wait = now.saturating_sub(req.queued_at_ms) as u32;
                        tel.tx_wait_max_ms = tel.tx_wait_max_ms.max(wait);
                        if is_flood_route {
                            tel.tx_flood_sent = tel.tx_flood_sent.saturating_add(1);
                        } else {
                            tel.tx_direct = tel.tx_direct.saturating_add(1);
                        }
                        match lane {
                            TxLane::Flood => tel.tx_flood = tel.tx_flood.saturating_add(1),
                            TxLane::Background => {
                                tel.tx_background = tel.tx_background.saturating_add(1)
                            }
                            TxLane::Control => tel.tx_control = tel.tx_control.saturating_add(1),
                        }
                    });
                }
                TxOutcome::ChannelBusy => {
                    let busy_since = *channel_busy_since.get_or_insert(after_tx);
                    if after_tx.saturating_sub(busy_since) >= 4_000 {
                        // After 4 s of consecutive busy checks, recover RX to clear any stuck
                        // modem latch before retrying transmit.
                        device.recover_receive().await;
                        let recovered_at = Instant::now().as_millis() as u64;
                        rx_active_since = None;
                        channel_busy_since = None;
                        last_rx_activity_ms = recovered_at;
                        next_tx_allowed_ms = recovered_at;
                    } else {
                        // Preserve active Header tracking if live IRQ poll detected one.
                        match device.rx_state() {
                            RxState::Idle => rx_active_since = None,
                            st => {
                                if !matches!(rx_active_since, Some((prev, _)) if prev == st) {
                                    rx_active_since = Some((st, after_tx));
                                }
                            }
                        }
                        let mix = after_tx
                            .wrapping_mul(6364136223846793005)
                            .wrapping_add(
                                (req.hash[0] as u64)
                                    ^ ((node_key[0] as u64) << 8)
                                    ^ 0x9E37_79B9,
                            );
                        let backoff_ms = 100 + ((mix >> 16) % 101);
                        next_tx_allowed_ms = after_tx.saturating_add(backoff_ms);
                    }

                    let mut retry = req;
                    let age_ms = after_tx.saturating_sub(retry.queued_at_ms);
                    let keep = match lane {
                        TxLane::Control => {
                            retry.due_at_ms = next_tx_allowed_ms;
                            if age_ms >= 8_000 {
                                retry.attempts = retry.attempts.saturating_add(1);
                            }
                            retry.attempts < TX_MAX_ATTEMPTS
                        }
                        TxLane::Flood => true,
                        TxLane::Background => age_ms < 10_000,
                    };
                    let requeued = if keep {
                        match lane {
                            TxLane::Flood => assign_flood(
                                &mut floods,
                                retry,
                                after_tx,
                                base_ms,
                                slot_ms,
                                &node_key,
                            )
                            .is_ok(),
                            TxLane::Background => background.push_back(retry).is_ok(),
                            TxLane::Control => control.push(retry).is_ok(),
                        }
                    } else {
                        false
                    };
                    if !requeued {
                        add_telemetry(|tel| {
                            tel.tx_dropped = tel.tx_dropped.saturating_add(1);
                        });
                    }
                }
                TxOutcome::Failed => {
                    rx_active_since = None;
                    channel_busy_since = None;
                    last_rx_activity_ms = after_tx;
                    note_core_err(CORE_ERR_TX_FAILED);
                    add_telemetry(|tel| {
                        tel.tx_retries = tel.tx_retries.saturating_add(1);
                        if req.attempts + 1 >= TX_MAX_ATTEMPTS {
                            tel.tx_dropped = tel.tx_dropped.saturating_add(1);
                        }
                    });
                    if req.attempts + 1 < TX_MAX_ATTEMPTS {
                        let mut retry = req;
                        retry.attempts += 1;
                        let requeued = match lane {
                            TxLane::Flood => assign_flood(
                                &mut floods,
                                retry,
                                after_tx,
                                base_ms,
                                slot_ms,
                                &node_key,
                            )
                            .is_ok(),
                            TxLane::Background => background.push_back(retry).is_ok(),
                            TxLane::Control => control.push(retry).is_ok(),
                        };
                        if !requeued {
                            add_telemetry(|tel| {
                                tel.tx_dropped = tel.tx_dropped.saturating_add(1);
                            });
                        }
                    }
                }
            }
            continue;
        }

        if !current_cfg.5 && !rx_busy && now >= next_noise_sample_ms {
            device.arm_receive().await;
            if let Some(burst_min_half_db) = await_noinline!(device.sample_rssi_half_db()) {
                if let Some(floor_dbm) = noise_floor.update(burst_min_half_db) {
                    agc_watchdog.on_rssi_sample(burst_min_half_db);
                    let frozen = agc_watchdog.frozen_streak();
                    add_telemetry(|tel| {
                        tel.noise_floor_dbm = floor_dbm;
                        tel.agc_frozen_streak = frozen;
                    });
                }
            }
            now = Instant::now().as_millis() as u64;
            next_noise_sample_ms = now.saturating_add(NOISE_SAMPLE_INTERVAL_MS);
        }

        if !current_cfg.5 {
            wake_at = wake_at.min(next_noise_sample_ms);
        }
        if now < next_tx_allowed_ms {
            wake_at = wake_at.min(next_tx_allowed_ms);
        }
        if let Some(due) = control
            .iter()
            .filter(|req| req.due_at_ms > now)
            .map(|req| req.due_at_ms)
            .min()
        {
            wake_at = wake_at.min(due);
        }
        if let Some(due) = next_flood_due(&floods, base_ms, slot_ms) {
            if due > now {
                wake_at = wake_at.min(due);
            }
        }
        if budget_wait_ms > 0 {
            wake_at = wake_at.min(now.saturating_add(budget_wait_ms));
        }
        let control_ready = control.iter().any(|req| req.due_at_ms <= now);
        if now >= next_tx_allowed_ms
            && !rx_busy
            && (control_ready || !background.is_empty())
            && budget_wait_ms == 0
        {
            wake_at = now;
        }
        let wait_ms = wake_at.saturating_sub(now).clamp(1, 60_000);

        // Arm RX only if not already listening so wakeups preserve duty-cycle
        // sleep/wake phase, and await only the cancel-safe IRQ pin in `select`.
        device.arm_receive().await;

        match select(
            select4(
                device.wait_rx_irq(),
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
            Either::First(Either4::First(irq_ok)) => {
                if !irq_ok {
                    note_core_err(CORE_ERR_RADIO_RX);
                    if rx_backoff_ms > 0 {
                        Timer::after_millis(rx_backoff_ms).await;
                    }
                    rx_backoff_ms = if rx_backoff_ms == 0 {
                        50
                    } else {
                        (rx_backoff_ms * 2).min(1000)
                    };
                    device.recover_receive().await;
                    rx_active_since = None;
                    last_rx_activity_ms = Instant::now().as_millis() as u64;
                } else {
                    match device.handle_rx_irq(&mut rx_buf).await {
                        Some(RxOutcome::Packet { len, rssi, snr }) => {
                            let now_rx = Instant::now().as_millis() as u64;
                            rx_backoff_ms = 0;
                            rx_active_since = None;
                            last_rx_activity_ms = now_rx;
                            agc_watchdog.on_rx_proof(now_rx);
                            add_telemetry(|tel| {
                                tel.agc_frozen_streak = 0;
                            });
                            info!("radio: RX {} bytes, RSSI: {} dBm, SNR: {} dB", len, rssi, snr);
                            let copy_len = len.min(256);
                            let mut packet_data = [0u8; 256];
                            packet_data[..copy_len].copy_from_slice(&rx_buf[..copy_len]);
                            if LORA_RX_CHANNEL
                                .try_send((copy_len, rssi, snr, packet_data))
                                .is_err()
                            {
                                add_telemetry(|tel| {
                                    tel.rx_dropped = tel.rx_dropped.saturating_add(1);
                                });
                            }
                        }
                        Some(RxOutcome::DutyCycleEnd) => {
                            rx_backoff_ms = 0;
                            rx_active_since = None;
                            last_rx_activity_ms = Instant::now().as_millis() as u64;
                        }
                        Some(RxOutcome::CrcError) => {
                            rx_backoff_ms = 0;
                            rx_active_since = None;
                            last_rx_activity_ms = Instant::now().as_millis() as u64;
                            add_telemetry(|tel| {
                                tel.rx_errors = tel.rx_errors.saturating_add(1);
                            });
                        }
                        Some(RxOutcome::Error) => {
                            note_core_err(CORE_ERR_RADIO_RX);
                            if rx_backoff_ms <= 50 {
                                debug!("radio: RX error");
                            }
                            if rx_backoff_ms > 0 {
                                Timer::after_millis(rx_backoff_ms).await;
                            }
                            rx_backoff_ms = if rx_backoff_ms == 0 {
                                50
                            } else {
                                (rx_backoff_ms * 2).min(1000)
                            };
                            device.recover_receive().await;
                            rx_active_since = None;
                            last_rx_activity_ms = Instant::now().as_millis() as u64;
                        }
                        None => {
                            let now_irq = Instant::now().as_millis() as u64;
                            match device.rx_state() {
                                RxState::Idle => rx_active_since = None,
                                st => {
                                    if st == RxState::Header {
                                        agc_watchdog.on_rx_proof(now_irq);
                                        add_telemetry(|tel| {
                                            tel.agc_frozen_streak = 0;
                                        });
                                    }
                                    if !matches!(rx_active_since, Some((prev, _)) if prev == st) {
                                        rx_active_since = Some((st, now_irq));
                                    }
                                }
                            }
                            last_rx_activity_ms = now_irq;
                        }
                    }
                }
            }
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
                        TxLane::Control => control.push(req).is_ok(),
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
                    let now_cfg = Instant::now().as_millis() as u64;
                    let new_auto = slot_width_ms(
                        requested.2,
                        requested.3,
                        requested.4,
                        preamble_for(requested.2),
                    );
                    let stale = rebase_floods(&mut floods, now_cfg, base_ms, slot_ms, new_auto);
                    if stale > 0 {
                        add_telemetry(|tel| {
                            tel.tx_stale = tel.tx_stale.saturating_add(stale);
                        });
                    }
                    current_cfg = requested;
                    auto_slot_ms = new_auto;
                    slot_ms = auto_slot_ms;
                    // Let the top-of-loop override check re-apply a manual
                    // slot width against the new modulation parameters.
                    slot_override_active = false;
                    base_ms = now_cfg;
                    last_adapt_ms = now_cfg;
                    rx_backoff_ms = 0;
                    rx_active_since = None;
                    channel_busy_since = None;
                    last_rx_activity_ms = now_cfg;
                    noise_floor.reset();
                    agc_watchdog.on_reconfigure(now_cfg);
                    next_noise_sample_ms = now_cfg.saturating_add(1_000);
                    add_telemetry(|tel| {
                        tel.noise_floor_dbm = 0;
                        tel.agc_frozen_streak = 0;
                    });
                    info!(
                        "radio: reconfigured to {} Hz (SF{}, BW{}k, CR 4/{}), {} dBm",
                        cfg.frequency_hz,
                        cfg.spreading_factor,
                        cfg.bandwidth_khz,
                        cfg.coding_rate,
                        cfg.tx_power_dbm
                    );
                } else {
                    rx_active_since = None;
                    channel_busy_since = None;
                    last_rx_activity_ms = Instant::now().as_millis() as u64;
                    error!("radio: reconfigure failed; keeping previous settings");
                }
            }
            Either::First(Either4::Fourth(_)) => {}
        }
    }
}
