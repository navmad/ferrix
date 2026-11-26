# Ferrix Feature & Command Guide

Ferrix is a Rust (`no_std` + Embassy) **MeshCore v1.18** companion firmware for the **Elecrow ThinkNode M1** and **Seeed Wio Tracker L1 Pro** (nRF52840 + SX1262). It implements the standard MeshCore v1.18 over-the-air and companion protocol (BLE & USB) so it works out of the box with existing v1.18 companion apps.

This guide highlights **what is different in Ferrix compared to stock MeshCore** and documents the available CLI commands.

---

## Differences vs. Stock MeshCore

### Additions & Behavioral Differences

| Area | Difference in Ferrix |
|---|---|
| **Rust / Embassy Runtime** | Written in `no_std` Rust with static allocation (no heap) and build-time task-arena verification. |
| **Storage Layout & Flash Wear** | Uses wear-leveled `sequential-storage` instead of LittleFS. Default profile stores config and BLE bonds in internal flash and contacts/adverts/messages in external 2 MB QSPI flash (probed read-only at boot); an `--features internal-storage` build profile avoids QSPI altogether. Config and contacts write only on actual changes. |
| **Offline Message Persistence** | Unread inbox messages stay in RAM (256-entry FIFO) by default (`msg.persist off`) to avoid routine flash wear. Enabling `set msg.persist on` persists unread messages across reboots while skipping flash writes whenever BLE is connected and reconciling on disconnect or clean reboot. |
| **3-Lane TX Scheduler** | Separates transmits into **Control** (DMs, ACKs, replies), **Flood** (4-slot airtime-adaptive calendar with deterministic per-node placement, over-the-air duplicate cancellation, and stale-flood eviction), and **Background** (adverts when idle). Duty-cycle budget is split 25% Control / 75% Flood, and Control can borrow unused Flood budget. |
| **Starvation-Gated LBT / CAD** | Runs 4-symbol hardware CAD before transmit and defers TX while an incoming preamble is active. Holds full CAD sensitivity during normal bursts, relaxes `detPeak` only after 60 s of continuous starvation, and tightens back after clear checks. |
| **Automatic Stuck-AGC Recovery** | Watches for a deaf/locked SX1262 receiver (prolonged RX silence + frozen idle half-dB RSSI readings) and automatically recalibrates and re-arms receive mode without rebooting (`stats-agc`, `agc-reset`). |
| **Live Noise Floor Sampling** | Samples idle RSSI every 5 s in continuous RX so `stats-radio` and companion stats report live RF noise floor rather than a static value. |
| **RX Duty-Cycle Sniff Mode** | Optional cancel-safe RX duty-cycle mode (`set rxduty on`) supporting SF7–12 (including 16-symbol preambles) with an inactivity watchdog. |
| **ECDH Shared-Secret Cache** | Caches X25519 peer shared secrets in a bounded 64-entry LRU table (computed outside critical sections so BLE and radio interrupts stay responsive), auto-invalidated on key change or contact removal. |
| **Active-Contact Eviction Protection** | Refreshes a peer's `last_seen_epoch` in RAM on inbound DMs, PATHs, and confirmed ACKs without extra flash writes so active chat partners are not evicted when the contact table fills. |
| **Immediate Request Pushes** | Delivers companion request response pushes (`0x85`–`0x8D`) immediately instead of buffering them behind a fixed delay, while keeping a 300 ms reply delay on inbound requests. |
| **Unified 9-Page Dashboard** | Shared 9-screen UI across ThinkNode M1 (200×200 e-paper, configurable `refresh.interval` + periodic anti-ghosting clear) and Wio Tracker L1 Pro (128×64 OLED, 30 s idle sleep). |
| **Extended JSON Diagnostics** | Adds 7 Ferrix-specific `stats-*` commands (`stats-tx`, `stats-agc`, `stats-flash`, `stats-ble`, `stats-gps`, `stats-power`, `stats-wdt`) alongside `stats-core`, `stats-radio`, and `stats-packets`. |
| **Hardware Watchdog & Fault LEDs** | Hardware watchdog (`stats-wdt`) with distinct LED blink patterns for normal boot, panic, HardFault, and SoftDevice fault. |

### Limitations & Incompatibilities vs. Stock MeshCore

| Area | Limitation |
|---|---|
| **Supported Boards** | Currently supports **Elecrow ThinkNode M1** and **Seeed Wio Tracker L1 Pro** only. |
| **Node Roles** | **Companion role only** (plus off-grid client repeat on allowed frequencies). Standalone repeater, room server, and sensor firmwares are not implemented. |
| **Flash Compatibility** | Ferrix's `sequential-storage` layout is **incompatible with stock MeshCore LittleFS**. Switching between stock MeshCore and Ferrix requires a full flash erase — export your private key (`get prv.key`) first. Upgrades between Ferrix versions preserve identity, settings, and contacts. |
| **Older Companion Apps** | Only protocol v3+ companion apps (targeting v1.18) are supported. |
| **BLE OTA Updates** | Wireless BLE OTA DFU (`start ota`) requires an OTAFIX-compatible bootloader; stock bootloaders use USB UF2 drag-and-drop or serial DFU. |

---

## CLI Command Reference

Commands work over **USB serial** (plain text terminal) or from a companion app's CLI console (`CMD_RUN_CLI_COMMAND`). Commands marked *(Ferrix)* are Ferrix-specific extensions.

### Device Info & Identity

| Command | Arguments | What it does |
|---|---|---|
| `help` | — | Prints a summary of available CLI commands. |
| `ver` | — | Shows the running firmware version (e.g. `1.18.4-ferrix`). |
| `board` | — | Shows the board model and hardware line. |
| `get name` | — | Shows this node's callsign / display name. |
| `set name <name>` | 1–31 printable ASCII chars (no `[ ] \ : , ? *`) | Sets this node's name and persists it. |
| `get role` | — | Shows the node role (`companion`). |
| `get public.key` | — | Prints this node's 32-byte public key in hex. |
| `get prv.key` | — | Prints this node's 64-byte expanded private key in hex (for backups). |
| `set prv.key <hex>` | 128 hex chars (expanded key) or 64 hex chars (32-byte seed) | Imports a private key, re-derives the public key, and saves it to flash. |

### Messaging, Contacts & Channels

| Command | Arguments | What it does |
|---|---|---|
| `send <text>` | Up to 160 bytes | Broadcasts a group text message on the default public channel (channel 0). |
| `msg <hex_id> <text>` | `<hex_id>` = 8 hex chars (first 4 bytes of contact's public key) | Queues an encrypted direct message to a known contact. |
| `messages` | — | Lists messages currently in the inbox with RSSI and SNR. |
| `clear messages` | — | Clears the RAM inbox and erases any persisted offline messages in flash. |
| `contacts` | — | Lists discovered contacts with ID, type (`CHAT`/`RPT`/`ROOM`/`SENS`), name, RSSI/SNR, position, and last-seen time. |
| `clear contacts` | — | Removes all contacts from RAM and flash. |
| `channels` | — | Lists configured group channels and their indices. |
| `advert` | — | Queues a flood advert broadcasting your node across the mesh. |
| `advert.zerohop` | — | Sends a zero-hop advert to direct neighbours only (not relayed). |
| `get autoadd` | — | Shows whether auto-adding contacts from adverts is enabled, its type mask, and max-hop limit. |
| `set autoadd <arg>` | `on`, `off`, or `<hex_mask>,<max_hops>` | Controls auto-add filtering. Mask bits: `0x01` overwrite oldest, `0x02` chat, `0x04` repeater, `0x08` room, `0x10` sensor (`0x00`–`0x1F`). `max_hops`: `0` (unlimited) to `64`. |
| `get multiacks` | — | Shows whether duplicate direct ACKs are enabled (`0` or `1`). |
| `set multiacks <on\|off>` | `on`/`off` or `1`/`0` | Enables or disables sending one extra direct ACK copy 300 ms later. |

### Radio & Forwarding

| Command | Arguments | What it does |
|---|---|---|
| `get radio` | — | Prints current LoRa parameters as `freq_MHz,bw_kHz,sf,cr`. |
| `set radio <freq>,<bw>,<sf>,<cr>` | Or `set radio default` | Applies all four LoRa settings immediately without rebooting (`default` = `910.525,62.5,7,5`). |
| `get freq` / `set freq <val>` | MHz (`910.525`), kHz (`910525`), or Hz (`910525000`); 150–960 MHz | Gets or sets the LoRa center frequency. |
| `get bw` / `set bw <val>` | `7.8`, `10.4`, `15.6`, `20.8`, `31.25`, `41.67`, `62.5`, `125`, `250`, `500` (kHz or Hz) | Gets or sets the LoRa bandwidth. |
| `get sf` / `set sf <5..12>` | `5`–`12` | Gets or sets the LoRa spreading factor. |
| `get cr` / `set cr <5..8>` | `5`–`8` (or `4/5`–`4/8`) | Gets or sets the LoRa coding rate. |
| `get tx` / `set tx <-9..22>` | Integer dBm (`-9` to `22`) | Gets or sets the LoRa transmit power. |
| `get repeat` / `set repeat <on\|off>` | `on`/`off` | Gets or toggles off-grid client repeat mode on allowed frequencies. |
| `get path.hash.mode` / `set path.hash.mode <0..2>` | `0` = 1-byte, `1` = 2-byte, `2` = 3-byte | Gets or sets the hop-hash size used in packet paths. |
| `get af` / `set af <0..9>` | Decimal (`0.0`–`9.0`) | Gets or sets the airtime factor governing TX duty cycle. |
| `get dutycycle` / `set dutycycle <10..100>` | Percentage (`10`–`100`) | Gets or sets the TX duty-cycle limit directly. |
| `get rxduty` / `set rxduty <on\|off>` | `on`/`off` *(Ferrix)* | Gets or toggles power-saving RX duty-cycle sniff mode. |
| `get tx.slot` / `set tx.slot <ms>` | `0` = auto, `1`–`5000` ms *(Ferrix)* | Gets or overrides the flood scheduler slot width. |
| `agc-reset` | — *(Ferrix)* | Manually triggers an SX1262 receiver AGC reset and recalibration. |

### GPS & Location

| Command | Arguments | What it does |
|---|---|---|
| `gps on` / `gps off` | — | Powers the onboard L76K GPS module on or off. |
| `gps advert` | — | Shows the advert location policy (`none`, `share`, or `prefs`). |
| `gps advert <mode>` | `none`, `share`, or `prefs` | Sets whether outgoing adverts omit location, share coordinates, or follow app preferences. |
| `get lat` / `set lat <deg>` | `-90.0` to `90.0` | Gets or sets static latitude for adverts. |
| `get lon` / `set lon <deg>` | `-180.0` to `180.0` | Gets or sets static longitude for adverts. |

### Clock, Display, Audio, LEDs & Storage

| Command | Arguments | What it does |
|---|---|---|
| `get time` / `set time <epoch>` | Unix timestamp (seconds) | Gets or sets the device clock. |
| `clock sync` | — | Acknowledges a clock sync check (`OK`). |
| `get battery` | — | Prints battery voltage (mV) and estimated percentage. |
| `get sound` / `set sound <on\|off>` | `on`/`off` *(Ferrix)* | Gets or toggles the passive buzzer. |
| `get leds` / `set leds <on\|off>` | `on`/`off` *(Ferrix)* | Gets or toggles onboard status LEDs. |
| `get refresh.interval` / `set refresh.interval <sec>` | `0` (on update) or `300`–`3600` s *(Ferrix)* | Gets or sets the periodic e-paper refresh interval (default 600 s). |
| `get msg.persist` / `set msg.persist <on\|off>` | `on`/`off` *(Ferrix)* | Gets or toggles persisting unread inbox messages to flash across reboots (default `off`). |
| `save` | — | Flushes pending configuration changes to flash immediately. |

### Diagnostics & Stats

All `stats-*` commands return compact single-line JSON.

| Command | Scope | What it reports |
|---|---|---|
| `stats-core` | Standard | Battery voltage/%, uptime, latched error flags (`errors`), live TX queue depth (`queue_len`), and companion push drops. |
| `stats-radio` | Standard | Live `noise_floor` (dBm), last RSSI/SNR, TX/RX airtime (seconds), TX count, retries, and drops. |
| `stats-packets` | Standard | Received/sent packet counts broken down by flood vs direct, receive errors, and RX drops. |
| `stats-agc` | *(Ferrix)* | Live `noise_floor`, consecutive frozen half-dB idle RSSI readings (`frozen`), and `agc_resets` count. |
| `stats-tx` | *(Ferrix)* | TX slot width, auto/manual mode, slot occupancy, Control/Flood/Background counts, stale/cancelled floods, queue-full drops, and DM ACK stats. |
| `stats-flash` | *(Ferrix)* | Storage/QSPI status (`ready`, `id_mismatch`, `io_error`, `unavailable`, `internal`), 24-bit JEDEC ID, outage drops, and per-store write/erase/queue counts. |
| `stats-ble` | *(Ferrix)* | BLE state, advertised name, connection status, saved bond count, and bond save/recovery counters. |
| `stats-gps` | *(Ferrix)* | GPS power, fix status, satellites, coordinates, altitude, HDOP, age, and UART byte/sentence counts. |
| `stats-power` | *(Ferrix)* | LFCLK source (`LFXO` crystal vs `LFRC`) and REG1/REG0 DC-DC regulator status. |
| `stats-wdt` | *(Ferrix)* | Task heartbeats, watchdog pet count, and last MCU reset reason. |
| `clear stats` | Standard | Resets runtime telemetry counters, AGC counters, and latched error flags. |

### Power, Reboot & Firmware Updates

| Command | What it does |
|---|---|
| `reboot` | Flushes pending storage writes and restarts the device cleanly. |
| `start dfu` | Flushes storage and reboots into the USB UF2/serial bootloader. |
| `start ota` | Flushes storage and reboots into BLE OTA DFU mode (requires an OTAFIX bootloader). |

---

## On-Device Dashboard & Controls

Both boards share the same 9 pages:
**0** Status · **1** Inbox · **2** Radar · **3** Radio · **4** Signal History · **5** Position / GPS · **6** Client Repeat · **7** Settings (LEDs & sound) · **8** System (battery, RX duty, path hash, firmware).

- **ThinkNode M1**: bottom button (`TK`) single-click = next page, double-click = previous page; top button (`FK`) single-click = primary action, double-click = secondary action. E-paper clears ghosting every 16 partial updates and sleeps when idle; frontlight wakes on input.
- **Wio Tracker L1 Pro**: joystick Left/Up = previous page, Right/Down = next page, Press (`OK`) = primary action; user button (`BTN`) = secondary action. OLED sleeps after 30 s idle and wakes on input or incoming messages.

### LED Fault & Boot Patterns

- **3 quick blinks at boot**: normal startup succeeded.
- **3 quick blinks repeating**: firmware panic.
- **6 quick blinks repeating**: HardFault.
- **10 quick blinks**: BLE SoftDevice fault.

---

## Build, Test & Flash

```sh
# Release build (default external QSPI + internal flash profile)
./tools/build.sh thinknode-m1 build --release
./tools/build.sh wio-tracker-l1-pro build --release

# Internal-flash-only release build (no QSPI flash required)
./tools/build.sh thinknode-m1 build --release --features internal-storage

# Package UF2 image for drag-and-drop USB flashing
./tools/build_uf2.sh thinknode-m1 release
./tools/build_uf2.sh wio-tracker-l1-pro release

# Package DFU .zip for BLE OTA or web/serial flashing
./tools/build_ota.sh thinknode-m1 release
./tools/build_ota.sh wio-tracker-l1-pro release

# Run all host unit tests, wire vectors, mesh simulator, and UI golden tests
./tools/build.sh test

# Hardware-in-the-loop USB smoke tests on a connected board
./tools/build.sh thinknode-m1 hil
./tools/build.sh wio-tracker-l1-pro hil

# Flash over USB serial DFU
./tools/flash_serial.sh
```

Set `FERRIX_VERSION` (and optionally `FERRIX_BUILD_DATE` / `SOURCE_DATE_EPOCH`) to override version metadata. Pushing a `v*` tag triggers [`.github/workflows/release.yml`](.github/workflows/release.yml) to build, test, and publish `.uf2` and `.zip` release artifacts for both boards.

To enter the bootloader manually: run `start dfu`, double-click reset, or touch the USB serial port at 1200 baud. For wireless BLE OTA updates (on OTAFIX bootloaders), run `start ota` or trigger Buttonless DFU from an nRF DFU app.

---

## Related Documents

- [`README.md`](README.md) — workspace architecture, crate layout, and developer setup.
- [`boards/thinknode-m1/README.md`](boards/thinknode-m1/README.md) — ThinkNode M1 pinout, memory map, and board notes.
- [`boards/wio-tracker-l1-pro/README.md`](boards/wio-tracker-l1-pro/README.md) — Wio Tracker L1 Pro pinout, controls, and board notes.
- [`.github/workflows/release.yml`](.github/workflows/release.yml) — matrix firmware build and GitHub Release workflow.
