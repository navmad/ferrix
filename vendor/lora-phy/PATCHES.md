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
  - Return `Ok(None)` early on `HeaderError` and `CRCError` instead of falling
    through to `RxDone` and returning corrupt payloads.
- `src/lib.rs`, `LoRa::start_rx()` / `LoRa::handle_rx_irq()`:
  - Split `rx()` into arming (`start_rx`), cancel-safe IRQ wait
    (`wait_for_irq`), and uninterruptible IRQ handling (`handle_rx_irq`).

## Upstreaming

This is a small, self-contained fix that should be sent upstream. Until a
release contains it, keep this vendored copy.
