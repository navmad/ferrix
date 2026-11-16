# Ferrix

A native `no_std` embedded Rust firmware workspace for **MeshCore v1.18** LoRa mesh nodes, built on [Embassy](https://embassy.dev/).

> **Firmware identity:** Ferrix reports `1.18.0-ferrix` in version strings and its boot splash while keeping `ThinkNode M1` / `MeshCore` USB and protocol identifiers for drop-in compatibility with official MeshCore companion apps (iOS, Android, Web, and CLI).

For the full protocol matrix, CLI reference, and current limitations, see [FEATURES.md](FEATURES.md).

---

## Highlights

- **MeshCore v1.18 Companion Protocol**: Implements all 58 companion commands over **BLE Nordic UART Service** and **USB CDC-ACM**, plus an interactive USB text CLI.
- **Modular Workspace Architecture**: Cleanly separates pure wire codecs (`meshcore-wire`), protocol/state/storage (`ferrix-core`), async runtime loops (`ferrix-runtime`), HAL traits (`ferrix-hal`), drivers, MCU platform (`platforms/nrf52`), roles, and board packages.
- **Wear-Leveled Persistence**: Powered by `sequential-storage` across internal flash (config & BLE bonds) and external QSPI flash (contacts, adverts & message FIFO), with a compile-time `internal-storage` profile for boards without QSPI.
- **Deterministic TX Scheduler**: Separates control, flood, and background advert lanes with airtime-derived adaptive slot widths, duplicate cancellation, and split duty-cycle budgets.
- **Board Support**: See each board's README for hardware specs, pinouts, and flashing steps (currently [ThinkNode M1](boards/thinknode-m1/README.md): 1.54" SSD1681 e-paper dashboard, L76K GNSS, battery telemetry, and buzzer).

---

## Supported Boards

| Board Target | Firmware Package | MCU | Radio | Board Docs |
| :--- | :--- | :--- | :--- | :--- |
| **Elecrow ThinkNode M1** | `ferrix-thinknode-m1-companion` | nRF52840 | Semtech SX1262 | [boards/thinknode-m1/README.md](boards/thinknode-m1/README.md) |

---

## Workspace Architecture

```text
ferrix/
├── firmware/
│   └── thinknode-m1-companion/  # Binary crate (`ferrix`): binds board + role
├── roles/
│   ├── companion/               # Companion role: identity, transports, app loop
│   └── repeater/                # Repeater role (placeholder, not built yet)
├── boards/
│   └── thinknode-m1/            # ThinkNode M1 BSP: pins, display UI, storage regions, memory.x
├── platforms/
│   └── nrf52/                   # nRF52840 support: SoftDevice BLE, USB CDC, QSPI, flash, WDT
├── drivers/
│   ├── radio/sx1262/            # Semtech SX1262 driver (`ferrix-sx1262`)
│   ├── display/ssd1681/         # SSD1681 e-paper driver (`ferrix-ssd1681`)
│   └── gnss/l76k/               # Quectel L76K NMEA driver (`ferrix-l76k`)
├── crates/
│   ├── meshcore-wire/           # Pure `no_std` MeshCore packet/frame codecs
│   ├── ferrix-hal/              # Platform-agnostic hardware traits and LoRa airtime math
│   ├── ferrix-core/             # Protocol handlers, routing engine, state, storage formats
│   └── ferrix-runtime/          # Role-agnostic async app, radio, and storage event loops
├── vendor/                      # Local patches (`nrf-softdevice`, `lora-phy`, `sequential-storage`)
└── tools/                       # Build, UF2/serial DFU scripts, CLI monitor, and test oracle
```

### Adding a New Board

1. Create a library crate under `boards/<name>/` containing its `memory.x`, pinout, display/UI bindings, storage layout, and a `README.md` documenting specs, pinout, and flashing.
2. Implement the role trait from `roles/<role>/` (e.g., `CompanionBoard`) and the `ferrix-runtime` board traits.
3. Add a binary crate under `firmware/<board>-<role>/` that selects the board and role.
4. Register the crates in the root `Cargo.toml` workspace, `tools/build.sh` / `tools/build_uf2.sh`, and `.github/workflows/release.yml`.

---

## Getting Started

### Prerequisites

- **Rust toolchain** with the Cortex-M4F target and LLVM tools (for `llvm-objcopy`):
  ```bash
  rustup target add thumbv7em-none-eabihf
  rustup component add llvm-tools-preview
  ```
- **For serial DFU flashing**: [`adafruit-nrfutil`](https://github.com/adafruit/Adafruit_nRF52_nrfutil) (`uv tool install adafruit-nrfutil` or `pipx install adafruit-nrfutil`).
- **For SWD debugging (optional)**: [`probe-rs`](https://probe.rs/).

### Build & Host Tests

Because `.cargo/config.toml` defaults to `thumbv7em-none-eabihf`, use `./tools/build.sh` so host unit tests and embedded builds target the right architecture:

```bash
# Release build (default external QSPI + internal flash storage profile)
./tools/build.sh thinknode-m1 build --release

# Release build keeping all stores in internal flash (no QSPI)
./tools/build.sh thinknode-m1 build --release --features internal-storage

# Run host unit tests (wire codecs, storage formats, routing engine, radio math)
./tools/build.sh thinknode-m1 test
```

> **Note:** Check or build workspace targets with `cargo check` from the root (defaults to `firmware/thinknode-m1-companion` for `thumbv7em-none-eabihf`). Avoid `cargo test` without `--target <host-triple>`, as embedded crates do not link `std` or `test`.

---

## Flashing

Flashing steps (UF2 drag-and-drop, serial DFU, or SWD) and board pinouts live in each board's README:

- [Elecrow ThinkNode M1](boards/thinknode-m1/README.md)

---

## Tooling (`tools/`)

- `tools/build.sh` — Unified wrapper for board builds, `probe-rs` runs, and host-target unit tests.
- `tools/build_uf2.sh` & `tools/uf2conv.py` — ELF-to-UF2 packaging for bootloader drag-and-drop flashing.
- `tools/flash_serial.sh` — End-to-end 1200-baud DFU reset, zip creation, and `adafruit-nrfutil` serial flash.
- `tools/cli_monitor.py` — Interactive USB serial monitor for the Ferrix text CLI.
- `tools/oracle/` — Python reference/vector generator (`gen.py`, `selfcheck.py`) used to validate `meshcore-wire` packet and crypto codecs.

---

## Companion Protocol Transports

- **BLE (Nordic UART Service)**
  - Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
  - RX (App -> Device): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`
  - TX (Device -> App): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- **USB CDC-ACM**
  - Inbound frame (App -> Radio): `'<'` (`0x3C`) + `u16` LE length + payload
  - Outbound frame (Radio -> App): `'>'` (`0x3E`) + `u16` LE length + payload
  - Also serves the interactive USB text CLI alongside binary framing.
