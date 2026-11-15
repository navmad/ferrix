# Ferrix feature guide

Ferrix is a MeshCore-compatible companion firmware for the
Elecrow **ThinkNode M1**: nRF52840, SX1262 LoRa, 1.54" e-paper, two buttons,
battery monitoring, USB CDC and BLE. It targets **MeshCore v1.18** companion
apps and the v1.18 over-the-air protocol.

- Firmware version: `0.1.0-ferrix`
- USB product/vendor: `ThinkNode M1 Companion` / `MeshCore`
- Board identity: `ThinkNode M1`
- Configuration layout: version 1

---

## 1. Hardware support

| Feature | Status |
|---|---|
| nRF52840 + SX1262 (SPI, DIO1, BUSY, TCXO 1.8 V, DCDC, RX boost) | Yes |
| 1.54" 200x200 e-paper (SSD1681) | Yes |
| Two buttons + frontlight idle timeout | Yes |
| Passive buzzer on P0.06 (PWM boot/click/action/message tones) | Yes |
| Battery voltage/percentage (SAADC) | Yes |
| USB CDC-ACM (text CLI + binary companion framing) | Yes |
| BLE Nordic UART Service companion | Yes |
| Quectel L76K GNSS | Yes |
| 1200-baud DFU touch | Yes |

Radio parameters (frequency, bandwidth, spreading factor, coding rate, TX
power) are configurable from the app or CLI, persisted, and applied without a
reboot. RX duty-cycle/sniff mode is supported.

---

## 2. Mesh protocol (MeshCore v1.18)

### Identity and crypto

- 64-byte expanded MeshCore private key (scalar + nonce prefix).
- 32-byte seed import/export, expanded-key import/export.
- Ed25519 identities, X25519 shared secrets, reserved-key rejection.
- CLI accepts a full 128-hex-character expanded key or a 64-hex seed.
- Hardware seed fallback if no key is stored.
- Identity is preserved across normal DFU flashes (the config page is not
  touched by application DFU). A factory reset erases it.

### Packet layer

- All 12 OTA payload types: REQ, RESPONSE, TXT_MSG, ACK, ADVERT, GRP_TXT,
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
- Signed room posts (`txt_type = 2`): the 4-byte signing prefix is parsed,
  ACKed with the signed hash, and delivered to the app as the V3 `extra` field.
- Per-contact `sync_since` advances from signed posts and is used in room
  logins, so rooms do not replay their entire backlog.
- Channel datagrams (`GRP_DATA`): send with `CMD_SEND_CHANNEL_DATA`, receive as
  `PACKET_CHANNEL_DATA_RECV`, and persist in the same FIFO as text.
- Multipart batched ACKs are parsed and fed to the delivery-confirmation path.
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
- Trace route request and response forwarding.
- Path discovery request, response push, and RTT.
- Control data, raw packets, RAW_CUSTOM data.
- Login to repeaters and room servers; status, telemetry, neighbours,
  binary and anonymous requests; connection table, `HAS_CONNECTION`, logout.
- Remote telemetry (LPP) and self battery telemetry.

---

## 3. Companion protocol

### Transports

- USB CDC binary framing: `'<'` + u16 LE length in, `'>'` + u16 LE length out.
- BLE Nordic UART Service, same command set and framing.
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
- local CLI command (`0x42`) and its reply (`0x1D`)
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
  Floods start at the current opportunity.
- **Cancellation**: when a duplicate of a pending forward is heard, or when
  forwarding is turned off, the scheduled slot is cleared. Cancellations are
  remembered briefly for packets still in the radio channel.
- **Staleness**: a flood that misses its grace window (8 slots) is dropped.
- **Budget**: the one-hour duty-cycle budget is split 25% control / 75% flood,
  refilled independently, so management and ACK traffic cannot be starved.
- **Retries**: up to 4 attempts with backoff; failed floods are rescheduled.

Inspect with `stats-tx`; override the slot width with `set tx.slot <ms>`
(`0` = automatic).

---

## 5. Storage and flash wear

| Store | Region | Design |
|---|---|---|
| Config | 4 KB page at `0xEC000` | Single version-1 struct, checksum, change-detected saves |
| Messages | 128 KB at `0xCC000` | Append-only FIFO (`sequential-storage`), variable-length records |
| Contacts | 112 KB at `0xB0000` | Key-value map keyed by public key |
| Adverts | 64 KB at `0xA0000` | Append-only ring of signed advert blobs |

- A message costs one compact record (~20-60 bytes typical) and one page erase
  per ~100 messages.
- A contact change appends ~200 bytes instead of rewriting the whole table.
- Config setters only save when a value actually changes; boot performs no
  config write.
- `stats-flash` reports per-store writes, erases, and queued messages.
- Legacy snapshot banks are detected and cleared once on the first boot of
  this storage format.

---

## 6. Dashboard and controls

Eight pages, selected with the bottom button; the top button performs the
page action:

0. Messages / connection status
1. Recently heard contacts
2. LoRa radio and packet counters
3. Traffic counters
4. GPS status
5. Off-grid (client repeat) status
6. LEDs
7. Power/battery

The frontlight turns off after an idle period and wakes on any input.

### LED patterns

- 3 quick blinks at boot: successful start.
- 3 quick blinks repeating: Rust panic.
- 6 quick blinks repeating: HardFault.
- 10 quick blinks: SoftDevice fault.

---

## 7. CLI

USB text CLI commands include:

```
ver, board, help
get/set name, get public.key, get/set prv.key
get radio, get/set freq, tx, sf, bw, cr
get/set path.hash.mode, get/set repeat (off-grid)
get/set leds, rxduty, af, dutycycle
get/set autoadd, get companion
contacts, clear contacts
messages, clear messages, channels
advert, time, clock sync
battery
stats-radio, stats-flash, stats-tx, stats-packets, stats-ble, stats-login,
stats-scope, stats-trace, stats-adverts, stats-disc, stats-binary, stats-core
clear stats
get/set tx.slot
save, reboot, dfu
```

Examples:

```
set name d1nk4n
set prv.key <128 hex chars>
set tx.slot 400
stats-tx
```

---

## 8. Diagnostics

- `stats-radio` — airtime, RSSI/SNR, transmit wait, lane counts, retries/drops.
- `stats-flash` — per-store saves/erases, queued messages.
- `stats-tx` — slot width/occupancy, control/flood/background counts,
  cancelled/stale floods, duplicate-delay histogram.
- `stats-packets` — received/sent flood/direct counters.
- `stats-ble`, `stats-login`, `stats-scope`, `stats-trace`, `stats-adverts`,
  `stats-disc`, `stats-binary`, `stats-core`.
- `clear stats` resets the runtime counters.

---

## 9. Build, test, flash

```sh
# Build (release is what the flash script uses)
./tools/build.sh thinknode-m1 build --release

# Host tests (wire codecs, message queue, contact map, vectors)
./tools/build.sh thinknode-m1 test

# Flash over USB DFU
./tools/flash_serial.sh
```

To enter the bootloader from the CLI, run `dfu`, or double-click reset. A
1200-baud touch also triggers DFU when the CDC port is responsive.

---

## 10. Current limitations

| Area | Limitation |
|---|---|
| Repeater/room server role | No separate repeater/room firmware yet. The companion forwards only in off-grid client-repeat mode on the reserved frequencies; it is not a remotely managed repeater or room server. |
| Older app protocol | Message frames are always V3; apps that report protocol < 3 are not supported. |
| Multipart | Batched ACKs are received; multi-ACK sending is not implemented. |
| Signed channel text | Signed direct/room posts are supported; signed group-channel text is passed through as a channel message. |
| Region allow/deny lists | Only flood-scope keys and the default scope are implemented; repeater region ACLs are not. |
| Factory reset | Handler implemented; the destructive path has not been exercised on hardware. |
| OTA | Serial DFU only; no Wi-Fi/OTA A/B update page. |
| Wi-Fi/MQTT/telnet | Not present on this board firmware. |

---

## 11. Related documents

- [`report.md`](report.md) — protocol conformance status and reference detail.
