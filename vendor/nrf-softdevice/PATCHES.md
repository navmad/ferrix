# Local nrf-softdevice patches

Vendored from `https://github.com/embassy-rs/nrf-softdevice.git` at commit
`5949a5b` (2025-06-26). Upstream HEAD (`b0ac850`, 2026-08-01) still has the
same bug.

## Why

Upstream panics when a GAP or GATTS event arrives after the peer disconnected:

```
bug: with_state_by_conn_handle on conn_handle that has no state
```

A late `BLE_GAP_EVT_CONN_PARAM_UPDATE` (or `DATA_LENGTH_UPDATE`, `RSSI_CHANGED`,
`PASSKEY_DISPLAY`, `AUTH_KEY_REQUEST`, `ATT_MTU` update) races with
`BLE_GAP_EVT_DISCONNECTED`. `on_disconnected` clears the handle->state mapping,
then the queued event calls `with_state_by_conn_handle`, which `unwrap!`s the
missing state and panics. The panic kills every task (USB, BLE, radio, main),
which looked like a full-device freeze when adding a companion contact.

## Changes

- `nrf-softdevice/src/ble/connection.rs`
  - Added `try_with_state_by_conn_handle()`, returning `Option<T>`.
  - Removed the panicking `with_state_by_conn_handle()`.
  - `ConnectionState::on_disconnected()` is now idempotent: a duplicate/stale
    disconnect event (state already disconnected, or handle mapping gone)
    returns instead of panicking with `bug: on_disconnected when already
    disconnected` / `bug: conn_handle has no index`.
- `nrf-softdevice/src/ble/gap.rs`
  - All event handlers use `try_with_state_by_conn_handle()` and ignore the
    event when the connection is already gone.
- `nrf-softdevice/src/ble/gatt_server.rs`
  - `ATT_MTU` update uses the non-panicking variant.
  - A GATTS event for a connection without state is ignored instead of
    `unwrap!`-panicking.

## Upstreaming

The correct upstream change is the same: make `with_state_by_conn_handle`
return `Option` (or skip events for unknown handles) and update the call sites.
