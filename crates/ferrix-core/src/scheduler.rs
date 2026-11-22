//! Transmit scheduling: lanes, the flood slot calendar and duty-cycle budgets.
//!
//! The policy is platform independent. The radio task owns the hardware and
//! calls these helpers with a monotonic millisecond clock.

use heapless::Vec;
use meshcore_wire::tx::{flood_slot, TxLane};

use crate::airtime::estimate_airtime_ms;

/// Flood calendar slots.
pub const FLOOD_SLOTS: usize = 4;
/// Control-lane queue depth.
pub const CONTROL_QUEUE: usize = 8;
/// Guard added to the reference-frame airtime when deriving the slot width.
pub const SLOT_GUARD_MS: u64 = 50;
pub const SLOT_MIN_MS: u64 = 100;
pub const SLOT_MAX_MS: u64 = 1500;
/// Quiet gap after a successful transmit.
pub const TX_GAP_MS: u64 = 30;
/// Delay between consecutive direct multi-ACK copies and the trailing ACK.
pub const MULTI_ACK_SPACING_MS: u64 = 300;
pub const TX_MAX_ATTEMPTS: u8 = 4;
/// A flood older than this many slot widths is dropped.
pub const FLOOD_MAX_AGE_SLOTS: u64 = 8;

/// One packet waiting in a transmit lane.
#[derive(Clone, Copy, Debug)]
pub struct TxPacket {
    pub len: usize,
    pub data: [u8; 256],
    pub lane: TxLane,
    pub queued_at_ms: u64,
    pub due_at_ms: u64,
    pub hash: [u8; 8],
    pub attempts: u8,
}

/// One occupied flood-calendar slot.
#[derive(Clone, Copy)]
pub struct FloodSlot {
    pub req: TxPacket,
    pub slot_id: u64,
}

/// MeshCore parity: longer preamble for SF <= 8 (32 symbols), 16 otherwise.
pub fn preamble_for(sf: u8) -> u16 {
    if sf <= 8 {
        32
    } else {
        16
    }
}

/// Duty-cycle percentage implied by the airtime factor setting.
pub fn tx_duty_pct(af_tenths: u8) -> i64 {
    1000 / (af_tenths as i64 + 10)
}

/// Airtime budget for a one-hour duty-cycle window.
pub fn tx_budget_max(af_tenths: u8) -> i64 {
    3_600_000 * tx_duty_pct(af_tenths) / 100
}

/// Slot width for the flood calendar, derived from the modem. A reference
/// frame of 128 bytes plus a guard keeps consecutive flood transmissions from
/// overlapping without quantizing short packets too coarsely.
pub fn slot_width_ms(sf: u8, bw_khz: u32, cr: u8, preamble: u16) -> u64 {
    let airtime = estimate_airtime_ms(128, sf, bw_khz, cr, preamble) as u64;
    (airtime + SLOT_GUARD_MS).clamp(SLOT_MIN_MS, SLOT_MAX_MS)
}

/// Maximum time to wait for `HeaderValid` after a `PreambleDetected` IRQ before
/// treating it as a false noise preamble and re-arming RX.
pub fn preamble_wait_ms(sf: u8, bw_khz: u32, preamble: u16) -> u64 {
    let bw_hz = ferrix_hal::radio::bw_khz_to_hz(bw_khz) as u64;
    if bw_hz == 0 {
        return 200;
    }
    let sym_us = ((1u64 << sf.min(12)) * 1_000_000) / bw_hz;
    let wait_ms = (preamble as u64 + 24) * sym_us / 1_000;
    wait_ms.clamp(80, 2_500)
}

/// Maximum time to wait for `RxDone` after `HeaderValid` before recovering RX.
pub fn packet_wait_ms(sf: u8, bw_khz: u32, cr: u8, preamble: u16) -> u64 {
    (estimate_airtime_ms(255, sf, bw_khz, cr, preamble) as u64 + 150).clamp(250, 4_000)
}

/// Place a flood in the first free slot at or after its hash-derived slot.
pub fn assign_flood(
    floods: &mut [Option<FloodSlot>; FLOOD_SLOTS],
    req: TxPacket,
    now: u64,
    base: u64,
    slot_ms: u64,
    node_key: &[u8; 32],
) -> Result<(), TxPacket> {
    let current = now.saturating_sub(base) / slot_ms.max(1);
    let first = current + flood_slot(&req.hash, &node_key[..4], FLOOD_SLOTS) as u64;
    for slot_id in first..=current + FLOOD_SLOTS as u64 {
        let index = (slot_id % FLOOD_SLOTS as u64) as usize;
        if floods[index].is_none() {
            floods[index] = Some(FloodSlot { req, slot_id });
            return Ok(());
        }
    }
    Err(req)
}

/// Remember a cancellation that arrived before the packet was scheduled.
pub fn remember_cancel(
    cancelled: &mut Vec<([u8; 8], u64), 16>,
    hash: [u8; 8],
    now: u64,
) {
    cancelled.retain(|(_, at)| now.saturating_sub(*at) < 8000);
    if cancelled.len() >= 16 {
        let _ = cancelled.remove(0);
    }
    let _ = cancelled.push((hash, now));
}

/// Consume a remembered cancellation for this hash, if any.
pub fn take_cancelled(cancelled: &mut Vec<([u8; 8], u64), 16>, hash: [u8; 8]) -> bool {
    if let Some(index) = cancelled.iter().position(|(h, _)| *h == hash) {
        cancelled.remove(index);
        true
    } else {
        false
    }
}

/// Drop a scheduled flood by hash.
pub fn cancel_slot(floods: &mut [Option<FloodSlot>; FLOOD_SLOTS], hash: [u8; 8]) -> bool {
    for slot in floods.iter_mut() {
        if slot.as_ref().is_some_and(|s| s.req.hash == hash) {
            *slot = None;
            return true;
        }
    }
    false
}

/// Earliest due flood at or before `now`: `(index, due_ms)`.
pub fn due_flood(
    floods: &[Option<FloodSlot>; FLOOD_SLOTS],
    now: u64,
    base: u64,
    slot_ms: u64,
) -> Option<(usize, u64)> {
    floods
        .iter()
        .enumerate()
        .filter_map(|(index, slot)| {
            slot.as_ref().map(|s| {
                (
                    index,
                    base.saturating_add(s.slot_id.saturating_mul(slot_ms)),
                )
            })
        })
        .filter(|(_, due)| *due <= now)
        .min_by_key(|(_, due)| *due)
}

/// Earliest due flood regardless of `now`.
pub fn next_flood_due(
    floods: &[Option<FloodSlot>; FLOOD_SLOTS],
    base: u64,
    slot_ms: u64,
) -> Option<u64> {
    floods
        .iter()
        .filter_map(|slot| {
            slot.as_ref()
                .map(|s| base.saturating_add(s.slot_id.saturating_mul(slot_ms)))
        })
        .min()
}

/// Recompute pending flood `slot_id`s relative to a new `(now, new_slot_ms)`
/// clock anchor so changing `slot_ms` or resetting `base = now` preserves
/// scheduled due times. Evicts slots that are already older than
/// `FLOOD_MAX_AGE_SLOTS * old_slot_ms` and returns how many were evicted.
pub fn rebase_floods(
    floods: &mut [Option<FloodSlot>; FLOOD_SLOTS],
    now: u64,
    old_base: u64,
    old_slot_ms: u64,
    new_slot_ms: u64,
) -> u32 {
    let denom = new_slot_ms.max(1);
    let max_age = FLOOD_MAX_AGE_SLOTS.saturating_mul(old_slot_ms);
    let mut stale = 0u32;
    for slot in floods.iter_mut() {
        if let Some(s) = slot {
            let old_due = old_base.saturating_add(s.slot_id.saturating_mul(old_slot_ms));
            if now > old_due.saturating_add(max_age) {
                *slot = None;
                stale = stale.saturating_add(1);
            } else {
                let rem_ms = old_due.saturating_sub(now);
                s.slot_id = rem_ms.div_ceil(denom);
            }
        }
    }
    stale
}

/// Refill control (25%) and flood (75%) duty-cycle airtime budgets (control may borrow unused flood budget, see `control_can_send`) using
/// sub-millisecond remainders (`1 / 10_000` ms units) so frequent wakeups never
/// truncate accrued airtime to zero.
pub fn refill_budgets(
    elapsed_ms: i64,
    duty_pct: i64,
    control_max: i64,
    flood_max: i64,
    control_budget_ms: &mut i64,
    flood_budget_ms: &mut i64,
    control_rem: &mut i64,
    flood_rem: &mut i64,
) {
    if elapsed_ms > 0 && duty_pct > 0 {
        let c_num = *control_rem + elapsed_ms * duty_pct * 25;
        let c_add = c_num / 10_000;
        *control_rem = c_num % 10_000;
        *control_budget_ms = (*control_budget_ms + c_add).min(control_max);
        if *control_budget_ms >= control_max {
            *control_rem = 0;
        }

        let f_num = *flood_rem + elapsed_ms * duty_pct * 75;
        let f_add = f_num / 10_000;
        *flood_rem = f_num % 10_000;
        *flood_budget_ms = (*flood_budget_ms + f_add).min(flood_max);
        if *flood_budget_ms >= flood_max {
            *flood_rem = 0;
        }
    }
    *control_budget_ms = (*control_budget_ms).min(control_max);
    *flood_budget_ms = (*flood_budget_ms).min(flood_max);
}

/// Whether a control-lane packet needing `airtime_ms` fits the duty-cycle
/// budget. The control lane spends its own 25% share first and borrows the
/// unused remainder from the flood share, so an ACK is not held back while
/// flood budget sits idle. Floods never borrow from the control share, which
/// keeps ACK headroom reserved.
pub fn control_can_send(control_budget_ms: i64, flood_budget_ms: i64, airtime_ms: i64) -> bool {
    control_budget_ms.max(0) + flood_budget_ms.max(0) >= airtime_ms
}

/// Charge a sent control packet: own budget first, then the flood budget.
pub fn charge_control(control_budget_ms: &mut i64, flood_budget_ms: &mut i64, airtime_ms: i64) {
    let from_control = airtime_ms.min((*control_budget_ms).max(0));
    *control_budget_ms -= from_control;
    *flood_budget_ms = (*flood_budget_ms - (airtime_ms - from_control)).max(0);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_borrows_idle_flood_budget() {
        // 25% share is exhausted but the flood share is full: the ACK may go.
        assert!(control_can_send(0, 300, 120));
        assert!(!control_can_send(0, 100, 120));
        assert!(control_can_send(40, 80, 120));
    }

    #[test]
    fn control_spends_its_own_budget_before_borrowing() {
        let (mut c, mut f) = (50, 300);
        charge_control(&mut c, &mut f, 120);
        assert_eq!((c, f), (0, 230));
        let (mut c, mut f) = (200, 300);
        charge_control(&mut c, &mut f, 120);
        assert_eq!((c, f), (80, 300));
    }

    #[test]
    fn charge_never_drives_budgets_negative() {
        let (mut c, mut f) = (0, 10);
        charge_control(&mut c, &mut f, 120);
        assert_eq!((c, f), (0, 0));
    }

    fn packet(hash: u8) -> TxPacket {
        TxPacket {
            len: 16,
            data: [0u8; 256],
            lane: TxLane::Flood,
            queued_at_ms: 0,
            due_at_ms: 0,
            hash: [hash; 8],
            attempts: 0,
        }
    }

    #[test]
    fn budget_matches_the_airtime_factor() {
        assert_eq!(tx_duty_pct(90), 10);
        assert_eq!(tx_budget_max(90), 360_000);
        assert_eq!(tx_duty_pct(0), 100);
        assert_eq!(tx_budget_max(0), 3_600_000);
    }

    #[test]
    fn budget_refill_does_not_lose_sub_ms_ticks() {
        let control_max = 90_000;
        let flood_max = 270_000;
        let mut c = 0;
        let mut f = 0;
        let mut c_rem = 0;
        let mut f_rem = 0;
        // 1,000 ticks of 1 ms at 10% duty cycle must accrue 25 ms control and 75 ms flood.
        for _ in 0..1_000 {
            refill_budgets(1, 10, control_max, flood_max, &mut c, &mut f, &mut c_rem, &mut f_rem);
        }
        assert_eq!(c, 25);
        assert_eq!(f, 75);
        assert_eq!(c_rem, 0);
        assert_eq!(f_rem, 0);
    }

    #[test]
    fn slot_width_is_clamped() {
        assert!(slot_width_ms(7, 125, 5, 16) >= SLOT_MIN_MS);
        assert_eq!(slot_width_ms(12, 7, 5, 32), SLOT_MAX_MS);
    }

    #[test]
    fn flood_assignment_and_cancellation() {
        let mut floods = [None; FLOOD_SLOTS];
        let req = packet(1);
        assign_flood(&mut floods, req, 0, 0, 500, &[9u8; 32]).unwrap();
        let scheduled = floods.iter().flatten().next().copied().unwrap();
        assert_eq!(scheduled.req.hash, [1u8; 8]);
        assert!(cancel_slot(&mut floods, [1u8; 8]));
        assert!(floods.iter().all(Option::is_none));
        assert!(!cancel_slot(&mut floods, [1u8; 8]));
    }

    #[test]
    fn rebase_preserves_due_time_and_evicts_stale() {
        let mut floods = [None; FLOOD_SLOTS];
        floods[0] = Some(FloodSlot {
            req: packet(1),
            slot_id: 5, // due at 0 + 5*500 = 2500
        });
        floods[1] = Some(FloodSlot {
            req: packet(2),
            slot_id: 1, // due at 500 -> stale if now > 500 + 8*500 = 4500
        });
        let stale = rebase_floods(&mut floods, 1_500, 0, 500, 250);
        assert_eq!(stale, 0);
        // slot 0: rem = 2500 - 1500 = 1000 -> 1000 / 250 = 4 -> new due = 1500 + 4*250 = 2500
        assert_eq!(floods[0].unwrap().slot_id, 4);
        // slot 1: already past due (500 <= 1500), not yet stale -> clamped to slot 0 (immediately due)
        assert_eq!(floods[1].unwrap().slot_id, 0);

        // Now advance past stale threshold for slot 1 (due 1500, max_age = 8*250 = 2000),
        // while slot 0 (due 2500, 2500 + 2000 = 4500 >= 4000) remains valid.
        let stale2 = rebase_floods(&mut floods, 4_000, 1_500, 250, 250);
        assert_eq!(stale2, 1);
        assert!(floods[1].is_none());
        assert!(floods[0].is_some());
    }

    #[test]
    fn calendar_is_full_after_four_floods() {
        let mut floods = [None; FLOOD_SLOTS];
        for hash in 0..FLOOD_SLOTS as u8 {
            assign_flood(&mut floods, packet(hash), 0, 0, 500, &[9u8; 32]).unwrap();
        }
        assert!(assign_flood(&mut floods, packet(99), 0, 0, 500, &[9u8; 32]).is_err());
    }

    #[test]
    fn due_and_next_use_the_slot_clock() {
        let mut floods = [None; FLOOD_SLOTS];
        assign_flood(&mut floods, packet(3), 0, 0, 500, &[9u8; 32]).unwrap();
        let due = next_flood_due(&floods, 0, 500).unwrap();
        assert!(due > 0);
        assert!(due_flood(&floods, due - 1, 0, 500).is_none());
        let (index, at) = due_flood(&floods, due, 0, 500).unwrap();
        assert_eq!(at, due);
        assert!(floods[index].is_some());
    }

    #[test]
    fn remembered_cancellations_expire() {
        let mut cancelled = Vec::new();
        remember_cancel(&mut cancelled, [4u8; 8], 1_000);
        assert!(take_cancelled(&mut cancelled, [4u8; 8]));
        assert!(!take_cancelled(&mut cancelled, [4u8; 8]));
        remember_cancel(&mut cancelled, [5u8; 8], 1_000);
        remember_cancel(&mut cancelled, [6u8; 8], 10_000);
        assert_eq!(cancelled.len(), 1);
        assert_eq!(cancelled[0].0, [6u8; 8]);
    }

    #[test]
    fn rx_wait_bounds_scale_with_modem_and_stay_bounded() {
        let sf7_preamble = preamble_wait_ms(7, 62, 32);
        let sf7_pkt = packet_wait_ms(7, 62, 5, 32);
        assert!(sf7_preamble >= 80 && sf7_preamble <= 150);
        assert!(sf7_pkt > sf7_preamble && sf7_pkt < 1_500);

        let sf12_preamble = preamble_wait_ms(12, 62, 16);
        let sf12_pkt = packet_wait_ms(12, 62, 5, 16);
        assert!(sf12_preamble <= 2_500);
        assert!(sf12_pkt <= 4_000);
    }
}
