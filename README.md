# Ferrix

A native `no_std` embedded Rust firmware workspace for **MeshCore v1.18** LoRa mesh nodes, built on [Embassy](https://embassy.dev/).

> **Firmware identity:** Ferrix reports `<version>-ferrix` (currently `1.18.2-ferrix`) in version strings and its boot splash while keeping `ThinkNode M1` / `MeshCore` USB and protocol identifiers for drop-in compatibility with MeshCore companion apps (iOS, Android, Web, and CLI).

> **Important — Storage format & full erase required:** Ferrix **does not** follow the standard MeshCore on-flash storage format (it uses its own `sequential-storage` layout across internal flash and external QSPI flash rather than MeshCore's LittleFS layout). **Flashing Ferrix onto a device previously running stock MeshCore (or switching back from Ferrix to stock MeshCore) requires a full flash erase** so leftover filesystem/config sectors do not conflict. Export your private key and contacts beforehand if you want to keep your identity. Normal upgrades between Ferrix releases preserve your identity and configuration.

> **Note:** This project is developed with the assistance of AI coding tools.

For the full protocol matrix, CLI reference, and current limitations, see [FEATURES.md](FEATURES.md).

---

## Highlights

- **MeshCore v1.18 Companion Protocol**: Implements all 58 companion commands over **BLE Nordic UART Service** and **USB CDC-ACM**, plus an interactive USB text CLI and `CMD_RUN_CLI_COMMAND` (`0x42`) / `RESP_CODE_CLI_REPLY` (`0x1D`) with `XX|` tag echo and compact JSON `stats-*` telemetry.
- **Modular Workspace Architecture**: Cleanly separates pure wire codecs (`meshcore-wire`), protocol/state/storage (`ferrix-core`), async runtime loops (`ferrix-runtime`), HAL traits (`ferrix-hal`), drivers, MCU platform (`platforms/nrf52`), roles, and board packages.
- **Wear-Leveled Persistence**: Powered by `sequential-storage` across internal flash (config & BLE bonds) and external QSPI flash (contacts, adverts & message FIFO), with a compile-time `internal-storage` profile for boards without QSPI.
- **Deterministic TX Scheduler**: Separates control, flood, and background advert lanes with airtime-derived adaptive slot widths, duplicate cancellation, multipart ACK bursts (`multi_acks`), and split duty-cycle budgets.
- **Power & OTA Support**: Uses the nRF52840 32.768 kHz LFXO crystal, REG1 DC-DC buck regulator, idle e-paper sleep, event-driven GNSS wake, smoothed SAADC battery telemetry, and Legacy Buttonless BLE DFU OTA (for OTAFIX bootloaders) alongside UF2 and serial DFU flashing.
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
└── tools/                       # Build, UF2/OTA/serial DFU scripts, arena check, CLI monitor, oracle
```

### Adding a New Board

1. Create a library crate under `boards/<name>/` containing its `memory.x`, pinout, display/UI bindings, storage layout, and a `README.md` documenting specs, pinout, and flashing.
2. Implement the role trait from `roles/<role>/` (e.g., `CompanionBoard`) and the `ferrix-runtime` board traits.
3. Add a binary crate under `firmware/<board>-<role>/` that selects the board and role.
4. Register the crates in the root `Cargo.toml` workspace, `tools/build.sh` / `tools/build_uf2.sh` / `tools/build_ota.sh`, and `.github/workflows/release.yml`.

---

## Getting Started

### Prerequisites

- **Rust toolchain** with the Cortex-M4F target and LLVM tools (for `llvm-objcopy`):
  ```bash
  rustup target add thumbv7em-none-eabihf
  rustup component add llvm-tools-preview
  ```
- **For serial / BLE OTA DFU packaging**: [`adafruit-nrfutil`](https://github.com/adafruit/Adafruit_nRF52_nrfutil) (`uv tool install adafruit-nrfutil` or `pipx install adafruit-nrfutil`).
- **For SWD debugging (optional)**: [`probe-rs`](https://probe.rs/).

### Build & Host Tests

Because `.cargo/config.toml` defaults to `thumbv7em-none-eabihf`, use `./tools/build.sh` so host unit tests and embedded builds target the right architecture (firmware builds automatically run `tools/check_task_arena.py` to verify all Embassy tasks fit inside the static task arena):

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

> **Before your first flash:** Because Ferrix uses its own on-flash storage format instead of the standard MeshCore layout, perform a **full flash erase** (e.g., using the MeshCore flasher's erase option or an erase UF2) before flashing Ferrix for the first time, and again if you ever return to stock MeshCore. Back up your private key first if you want to restore your node identity.

Prebuilt `.uf2` (drag-and-drop) and `.zip` (Nordic/Adafruit serial & BLE OTA DFU) artifacts are published on GitHub Releases. Detailed flashing steps (UF2 drag-and-drop, serial DFU, or SWD) and board pinouts live in each board's README:

- [Elecrow ThinkNode M1](boards/thinknode-m1/README.md)

---

## Tooling (`tools/`)

- `tools/build.sh` — Unified wrapper for board builds (with Embassy task arena verification), `probe-rs` runs, and host-target unit tests.
- `tools/build_uf2.sh` & `tools/uf2conv.py` — ELF-to-UF2 packaging for bootloader drag-and-drop flashing.
- `tools/build_ota.sh` — Builds and packages a Nordic/Adafruit DFU `.zip` for BLE OTA (nRF Device Firmware Update / OTAFIX bootloader) or web/serial flashing.
- `tools/flash_serial.sh` — End-to-end 1200-baud DFU reset, zip creation, and `adafruit-nrfutil` serial flash.
- `tools/check_task_arena.py` — Inspects DWARF info in the compiled ELF to ensure all Embassy `TaskPool` allocations fit in the static task arena before flashing or releasing.
- `tools/cli_monitor.py` — Interactive USB serial monitor for the Ferrix text CLI.
- `tools/oracle/` — Python reference/vector generator (`gen.py`, `selfcheck.py`) used to validate `meshcore-wire` packet and crypto codecs.
