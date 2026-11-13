# Local lora-phy patches

Vendored from crates.io `lora-phy` 3.0.1 (latest published version).

## Why

`create_modulation_params()` chose low-data-rate optimization (LDRO) with a
hard-coded SF/bandwidth table:

```rust
(SF11 or SF12) && BW125, or SF12 && BW250
```

The SX126x requirement is that LDRO is on whenever the LoRa symbol time exceeds
16.38 ms. the reference firmware computes it that way. The hard-coded table is only correct at
125 kHz: at the MeshCore default 62.5 kHz it left LDRO off for SF10, SF11 and
SF12, so packets at those settings did not interoperate with the reference firmware nodes.

## Changes

- `src/sx126x/mod.rs`, `create_modulation_params()`:
  - Compute `symbol_time = 2^SF / bandwidth` and enable LDRO when it is greater
    than 16.38 ms, matching the datasheet and the reference firmware's `should_enable_ldro()`.

## Upstreaming

This is a small, self-contained fix that should be sent upstream. Until a
release contains it, keep this vendored copy.
