# Local lora-phy patches

Vendored from crates.io `lora-phy` 3.0.1 (latest published version).

## Why

`create_modulation_params()` chose low-data-rate optimization (LDRO) with a
hard-coded SF/bandwidth table:

```rust
(SF11 or SF12) && BW125, or SF12 && BW250
```

The SX126x requirement is that LDRO is on whenever the LoRa symbol time exceeds
16.38 ms. The hard-coded table is only correct at 125 kHz: at the 62.5 kHz
default it left LDRO off for SF10, SF11 and SF12, so packets at those settings
did not interoperate with MeshCore nodes.

## Changes

- `src/sx126x/mod.rs`, `create_modulation_params()`:
  - Compute `symbol_time = 2^SF / bandwidth` and enable LDRO when it is greater
    than 16.38 ms, matching the datasheet's `should_enable_ldro()` rule.
- `src/sx126x/mod.rs`, `process_irq_event()`:
  - Return `Ok(Some(IrqState::RxError))` early on `HeaderError` and `CRCError`
    instead of falling through to `RxDone` and returning corrupt payloads.
- `src/lib.rs`, `LoRa::start_rx()` / `LoRa::handle_rx_irq()`:
  - Split `rx()` into arming (`start_rx`), cancel-safe IRQ wait
    (`wait_for_irq`), and uninterruptible IRQ handling (`handle_rx_irq`).
- `src/mod_traits.rs`, `src/lib.rs`, `src/sx126x/mod.rs`:
  - Distinguish `HeaderValid` (`IrqState::HeaderValid` / `RxIrqEvent::HeaderValid`)
    from `PreambleDetected` so false noise preambles do not look like
    in-flight packets, and surface `IrqState::RxError` / `RxIrqEvent::RxError`
    on `CRCError` or `HeaderError` without tearing down continuous RX.
  - Keep `PreambleDetected` and `HeaderError` enabled in the SX126x RX `irq_mask`
    for live non-destructive `GetIrqStatus` polling (`poll_rx_activity` /
    `clear_preamble_irq` / `clear_rx_activity_irqs`) while masking both off
    `DIO1` so noise preambles or corrupt headers do not wake the MCU on their own.
  - Clear latched IRQ flags (`ClrIrqStatus`) in `do_tx()`, `do_rx()`, and
    `do_cad_with_offset()` before starting TX, RX, or CAD.
  - Explicitly program `Register::OCP` (`0x08E7`) to `0x38` (140 mA) for
    `HighPowerPA` and `0x18` (60 mA) for `LowPowerPA` after `SetPaConfig`, and
    add `Register::OCP` to the retention list (SX1261-2 Datasheet §13.1.14 /
    Table 13-21).
  - Configure `SetRxTxFallbackMode(FS)` (`0x40`, SX1261-2 Datasheet §13.1.11)
    in `init_lora()`, add `set_standby_warm()` (`SetStandby(XOSC)`), and add
    `prepare_for_tx_fast()`, `rearm_rx_fast()`, `cad_fast()`, and
    `cad_fast_with_offset()` so RX→CAD→TX→RX transitions keep the TCXO and PLL
    locked without forcing `STBY_RC`.
  - Apply `SetStopRxTimerOnPreamble(1)` only for `RxMode::DutyCycle(_)` (0
    otherwise) and write `Register::RxGain` (`0x08AC`) after `SetRx` /
    `SetRxDutyCycle`.
  - Use 4-symbol CAD (`CADSymbols::_4`) with Semtech AN1200.48 bandwidth- and
    SF-dependent `cadDetPeak` (`detMin = 10`) and an optional adaptive peak offset.
  - Add `get_rssi_inst_half_db()` (`GetRSSIInst` `0x15` with RX chip-mode
    check) for idle continuous-RX noise-floor sampling.
  - Add `reset_agc()` (Warm Sleep `0x04` -> NSS wake to `STBY_RC` ->
    `Calibrate(0x7F)` -> `CalibrateImage(freq)` -> retention list, DC-DC, DIO2
    RF switch, fallback mode, and `Register::RxGain` restore) for SX126x
    stuck-AGC recovery.

## Upstreaming

These fixes should be sent upstream. Until a release contains them, keep this
vendored copy.
