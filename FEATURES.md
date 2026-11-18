# Ferrix feature guide

Ferrix is a MeshCore-compatible companion firmware for the
Elecrow **ThinkNode M1**: nRF52840, SX1262 LoRa, 1.54" e-paper, two buttons,
battery monitoring, USB CDC and BLE. It targets **MeshCore v1.18** companion
apps and the v1.18 over-the-air protocol.

- Firmware version: `1.18.2-ferrix` (from root `Cargo.toml` `[workspace.package]`; overridable at build time via `FERRIX_VERSION` and `FERRIX_BUILD_DATE` / `SOURCE_DATE_EPOCH`)
- USB product/vendor: `ThinkNode M1 Companion` / `MeshCore`
- Board identity: `ThinkNode M1`
- Configuration layout: version 1

---

## 1. Hardware support

| Feature | Status |
|---|---|
| nRF52840 + SX1262 (SPI, DIO1, BUSY, TCXO 1.8 V, REG1 DC-DC, 32.768 kHz LFXO, RX boost) | Yes |
| 1.54" 200x200 e-paper (SSD1681, idle sleep, configurable refresh interval) | Yes |
| Two buttons + frontlight idle timeout | Yes |
| Passive buzzer on P0.06 (GPIO square wave, UI/CLI toggle) | Yes |
| MX25R1635F 2 MB external QSPI flash (or `--features internal-storage` profile) | Yes |
| Battery voltage/percentage (averaged SAADC reads, EMA smoothing, 11-point LiPo curve) | Yes |
| USB CDC-ACM (text CLI + binary companion framing) | Yes |
| BLE Nordic UART Service companion | Yes |
| Quectel L76K GNSS (idle-delimited UART DMA + event-driven wake) | Yes |
| 1200-baud DFU touch, UF2 drag-and-drop, and Legacy Buttonless BLE DFU OTA (OTAFIX bootloader) | Yes |

Radio parameters (frequency, bandwidth, spreading factor, coding rate, TX
power) are configurable from the app or CLI, persisted, and applied without a
reboot. Cancel-safe RX duty-cycle/sniff mode is supported across SF7-12
(including SF9-12 16-symbol preambles), backed by a 60 s RX inactivity
watchdog. See [boards/thinknode-m1/README.md](boards/thinknode-m1/README.md)
for ThinkNode M1 pinouts and memory mapping.

---

## 2. Mesh protocol (MeshCore v1.18)

### Identity and crypto

- 64-byte expanded MeshCore private key (scalar + nonce prefix).
- 32-byte seed import/export, expanded-key import/export.
- Ed25519 identities, X25519 shared secrets, reserved-key rejection.
- CLI accepts a full 128-hex-character expanded key or a 64-hex seed.
- Hardware seed fallback if no key is stored.
- Identity is preserved across normal DFU/UF2 flashes (the config page is not
  touched by application updates). A factory reset erases it.

### Packet layer

- All 13 OTA payload types: REQ, RESPONSE, TXT_MSG, ACK, ADVERT, GRP_TXT,
  GRP_DATA, ANON_REQ, PATH, TRACE, MULTIPART, CONTROL, RAW_CUSTOM.
- Path hash sizes 1-3 bytes; path validation and hop limits.
- Transport codes / flood scopes, including a persisted default scope.
- 8-byte dedup hash with a 160-entry seen cache.
- Packet version validation, payload bounds, and malformed-frame rejection.

### Messaging

- Direct text messages with ACKs, retries driven by the app, and `PACKET_SENT`
  timing.
- Channel (group) text with flood scope.
- V3 message frames with the real `txt_type` preserved:
  - `0` plain, `1` CLI data, `2` signed, `3` CLI command.
- Repeater/room CLI replies (`txt_type = 1`) are shown in the app's management
  console.
- Signed room posts (`txt_type = 2`): received posts parse the 4-byte signing
  prefix, ACK with the signed hash, and deliver it to the app as the V3
  `extra` field.
- Per-contact `sync_since` advances from signed posts and is used in room
  logins, so rooms do not replay their entire backlog.
- Channel datagrams (`GRP_DATA`): send with `CMD_SEND_CHANNEL_DATA`, receive as
  `PACKET_CHANNEL_DATA_RECV`, and persist in the same FIFO as text.
- Multipart batched ACKs (`multi_acks`, `0` or `1` extra copy spaced 300 ms apart)
  are sent for direct ACKs, forwarded along direct hops with collision-avoidance
  delay, parsed, and deduplicated into the delivery-confirmation path.
- Direct and channel ACKs, path returns, and companion retry support.

### Adverts and contacts

- Signed adverts with kind nibble, location policy, flood scope, feat fields.
- Advert blob storage for contact sharing/export.
- Up to 350 contacts, auto-add policy (chat/repeater/room/sensor), max-hop
  filter, overwrite-oldest, favourites, manual-add mode.
- Contact share/export/import, add/update/remove, lookup by key.
- Path learning (PATH/PATH return) and `reset path`.

### Routing and management

- Direct and flood routing, packet forwarding in repeater/client-repeat mode.
- Flooded unicast and anonymous requests decrypted/consumed by this node are
  not re-relayed (matching MeshCore `routeRecvPacket`); un-decryptable hash
  matches and group/advert/ACK traffic still follow normal forwarding rules.
- Trace route request and response forwarding.
- Path discovery request, response push, and RTT.
- Control data, raw packets, RAW_CUSTOM data.
- Login to repeaters and room servers; status, telemetry, neighbours,
  binary and anonymous requests; connection table, `HAS_CONNECTION`, logout.
- Remote telemetry (LPP) and self battery telemetry.

---

## 3. Companion protocol

### Transports

- **BLE (Nordic UART Service)**, same command set as USB:
  - Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
  - RX (App -> Device): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
  - TX (Device -> App): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- **USB CDC-ACM**:
  - Inbound frame (App -> Radio): `'<'` (`0x3C`) + `u16` LE length + payload
  - Outbound frame (Radio -> App): `'>'` (`0x3E`) + `u16` LE length + payload
  - USB text CLI runs alongside the binary protocol; a text session does not
    receive binary pushes.

### Commands

All **58** commands from the v1.18 `CompanionProtocol.h` are implemented,
including:

- app start / device query / battery / storage
- send direct and channel text, channel data
- contacts: get, add/update, remove, share, export, import, by-key
- channels: get/set
- messages: sync next, no-more
- radio: set params, TX power, tuning, path hash mode, allowed repeat frequencies
- adverts: self advert, name, position
- identity: export/import private key
- signing: start/data/finish
- login/status/telemetry/anon/binary requests, has-connection, logout
- flood scopes: set key, set/get default scope
- auto-add config, stats, custom vars, device PIN
- trace, path discovery, control data, raw packet/data
- local CLI command (`0x42`) and its reply (`0x1D`), echoing optional `XX|` tag prefixes and stripping USB serial prompt markers
- factory reset and reboot

Errors are returned as `[0x01, code]` with codes for unsupported, not found,
table full, bad state, and illegal argument; send commands validate the
channel, type, and payload before reporting success.

### Persistence of delivery

- Text and channel datagrams are appended to flash on receive.
- A record is popped only when the app syncs the message.
- A reboot before sync redelivers the record.
- `clear messages` erases both the RAM inbox and the flash queue.

---

## 4. Transmit scheduling

The radio task owns all transmission timing.

| Lane | Contents | Timing |
|---|---|---|
| Control | ACKs, path returns, admin replies, local sends | Next opportunity after a 30 ms gap |
| Flood | Forwarded floods | Four-slot calendar, one frame per slot |
| Background | Periodic adverts | Only when control and flood work is idle |

- **Slot width** derives from modem airtime: a 128-byte reference frame plus a
  50 ms guard, clamped to 100-1500 ms. It adapts every 30 s within half/double
  the base: wider after stale/cancelled floods, narrower while idle.
- **Slot assignment** is deterministic from the packet hash + node prefix.
  The radio loop re-reads `current_public_key()` each iteration so runtime
  private-key imports update flood placement without a reboot, and rebases
  pending flood slots when slot width or radio settings change.
- **Cancellation**: when a duplicate of a pending forward is heard, or when
  forwarding is turned off, the scheduled slot is cleared. Cancellations are
  remembered briefly for packets still in the radio channel.
- **Staleness & preamble gating**: stale floods (older than 8 slots) are
  evicted before budget checks, and pending transmits are deferred while a
  LoRa preamble is actively being received (up to a bounded hold window).
- **Budget**: the one-hour duty-cycle budget is split 25% control / 75% flood,
  refilled independently with sub-millisecond remainder tracking, so
  management and ACK traffic cannot be starved.
- **Retries**: up to 4 attempts with backoff; failed floods are rescheduled.

Inspect with `stats-tx`; override the slot width with `set tx.slot <ms>`
(`0` = automatic).

---

## 5. Storage and flash wear

Storage is selected at compile time between two statically dispatched profiles:

| Profile | Store | Region | Design |
|---|---|---|---|
| Default (`ExternalStores`) | Config | internal `0xA0000..0xB0000` (64 KB) | `sequential-storage` map: settings + per-channel records |
| | BLE bonds | internal `0xB0000..0xB4000` (16 KB) | `sequential-storage` map keyed by master ID (up to 4 bonds) |
| | Contacts + adverts | QSPI `0x00000..0x40000` (256 KB) | One `sequential-storage` map keyed by public key; advert blobs share it |
| | Messages | QSPI `0x40000..0x140000` (1 MB) | `sequential-storage` FIFO with variable-length records |
| `--features internal-storage` (`InternalStores`) | Config + bonds | internal `0xA0000..0xB4000` (80 KB) | Same internal stores as above |
| | Contacts + adverts | internal `0xB4000..0xD4000` (128 KB) | `sequential-storage` map in internal flash; no QSPI linked |
| | Messages | internal `0xD4000..0xEC000` (96 KB) | `sequential-storage` FIFO in internal flash |

- QSPI bring-up uses a read-only JEDEC probe (no erase/write self-test on
  boot). If the external flash probe fails, external stores are marked
  unavailable and skipped rather than falling back to internal flash.
- A message costs one compact record (~20-60 bytes typical) and one page erase
  per ~100 messages.
- A contact change appends one deterministic 164-byte record instead of
  rewriting the whole table.
- Config setters only save when a value actually changes; boot performs no
  config write.
- `stats-flash` reports the active profile, QSPI availability/outage drops, and
  per-store writes, erases, and queued messages.
- Ferrix uses its own `sequential-storage` layout rather than standard
  MeshCore's LittleFS filesystem. Switching between stock MeshCore and Ferrix
  requires a full flash erase (back up your private key and contacts first);
  normal upgrades between Ferrix releases preserve identity and settings.
- Legacy snapshot banks from older Ferrix firmware are detected and cleared once.

---

## 6. Dashboard and controls

Nine pages, selected with the bottom button; the top button performs the
page action:

0. Mesh / connection status
1. Inbox
2. Radar
3. Radio and packet counters
4. Signal history
5. Position / GPS
6. Off-grid client repeat
7. Settings (LEDs and sound)
8. System (battery, RX duty, path hash, firmware)

The frontlight turns off after an idle period and wakes on any input. The
e-paper panel enters deep sleep when idle, performs a full clear every 16
partial updates to prevent ghosting, and refreshes periodically according to
`refresh.interval` (default 10 minutes; idle telemetry updates are rate-limited
when set to `0`).

### LED patterns

- 3 quick blinks at boot: successful start.
- 3 quick blinks repeating: Rust panic.
- 6 quick blinks repeating: HardFault.
- 10 quick blinks: SoftDevice fault.

---

## 7. CLI

USB text CLI and `CMD_RUN_CLI_COMMAND` (`0x42`) commands include:

```
ver, board, help
get/set name, get role, get/set lat, get/set lon
get public.key, get/set prv.key
get radio, set radio <freq>,<bw>,<sf>,<cr>, get/set freq, tx, sf, bw, cr
get/set path.hash.mode (or hash-mode), get/set repeat (off-grid), get/set sound
get/set leds, get/set refresh.interval, rxduty, af, dutycycle
get/set autoadd, get/set multiacks, get companion
contacts, clear contacts
messages, clear messages, channels
advert, advert.zerohop, gps advert, clock / time, clock sync
battery
stats-radio, stats-flash, stats-qspi, stats-tx, stats-packets, stats-ble,
stats-login, stats-scope, stats-trace, stats-adverts, stats-disc, stats-binary,
stats-core, stats-gps, stats-wdt, stats-power
clear stats
get/set tx.slot
save, reboot, dfu (or start dfu), start ota
```

Examples:

```
set name d1nk4n
set prv.key <128 hex chars>
set tx.slot 400
stats-tx
stats-power
```

---

## 8. Diagnostics

All `stats-*` commands return compact single-line JSON designed to fit within
the 175-byte `PACKET_CLI_REPLY` (`0x1D`) binary companion frame limit:

- `stats-radio` — airtime, RSSI/SNR, transmit wait, lane counts, retries/drops.
- `stats-flash` — active storage profile, QSPI availability/skipped ops,
  per-store saves/erases, queued messages.
- `stats-qspi` (or `qspi`) — QSPI JEDEC ID, status, and flash health.
- `stats-tx` — slot width/occupancy, control/flood/background counts,
  cancelled/stale floods, duplicate-delay histogram.
- `stats-packets` — received/sent flood/direct counters.
- `stats-power` — LFCLK source (`LFXO` vs `LFRC`) and REG1 DC-DC regulator status.
- `stats-gps` (or `gps`) — fix status, satellites, coordinates, and UART activity.
- `stats-wdt` (or `watchdog`) — watchdog feed/timeout telemetry.
- `stats-ble`, `stats-login`, `stats-scope`, `stats-trace`, `stats-adverts`,
  `stats-disc`, `stats-binary`, `stats-core`.
- `clear stats` resets the runtime counters.

---

## 9. Build, test, flash

```sh
# Release build (external QSPI + internal flash profile; runs task arena check)
./tools/build.sh thinknode-m1 build --release

# Internal-flash-only release build (no QSPI)
./tools/build.sh thinknode-m1 build --release --features internal-storage

# Package UF2 image for drag-and-drop flashing
./tools/build_uf2.sh thinknode-m1 release

# Package Nordic/Adafruit DFU .zip for BLE OTA or web/serial flashing
./tools/build_ota.sh thinknode-m1 release

# Host tests (wire codecs, message queue, contact map, scheduler, vectors)
./tools/build.sh thinknode-m1 test

# Flash over USB serial DFU
./tools/flash_serial.sh
```

Every firmware build and release script runs `tools/check_task_arena.py` against
the compiled ELF to ensure all Embassy `TaskPool` allocations fit inside the
static task arena before flashing.

Set `FERRIX_VERSION` (e.g. `v1.18.2`) and optionally `FERRIX_BUILD_DATE` /
`SOURCE_DATE_EPOCH` to override the compiled version/date metadata. Pushing a
`v*` tag triggers [`.github/workflows/release.yml`](.github/workflows/release.yml),
which runs the host test suite, verifies the task arena, and publishes
`ferrix-thinknode-m1-companion.uf2` and `ferrix-thinknode-m1-companion.zip`
(BLE OTA / webflasher / serial DFU package) to GitHub Releases.

To enter the USB bootloader from the CLI, run `dfu` (`start dfu`), double-click
reset, or trigger a 1200-baud touch on the CDC port. To enter BLE OTA DFU mode
on boards with an OTAFIX bootloader, run `start ota` or trigger Legacy
Buttonless DFU from an nRF DFU app.

---

## 10. Current limitations

| Area | Limitation |
|---|---|
| Node roles | Only the `companion` role is implemented; repeater, room server, and sensor node roles are not implemented (`roles/repeater` is a placeholder). Off-grid client-repeat forwarding is supported on allowed frequencies. |
| Storage format | Uses Ferrix's own `sequential-storage` layout rather than standard MeshCore's LittleFS layout; switching to or from stock MeshCore requires a full flash erase. |
| Older app protocol | Message frames are always V3; apps that report protocol < 3 are not supported. |
| Factory reset | Handler implemented; the destructive path has not been exercised on hardware. |
| BLE OTA | Legacy Buttonless BLE DFU requires an OTAFIX-compatible bootloader; boards with a stock non-OTAFIX bootloader must use USB UF2 drag-and-drop or Serial DFU. |

---

## 11. Related documents

- [`README.md`](README.md) — workspace overview, architecture, and tooling.
- [`boards/thinknode-m1/README.md`](boards/thinknode-m1/README.md) — ThinkNode M1 hardware specs, flashing, pinout, and memory map.
- [`.github/workflows/release.yml`](.github/workflows/release.yml) — matrix firmware build and GitHub Release workflow.
