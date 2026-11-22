# Elecrow ThinkNode M1

Board support package (`ferrix-thinknode-m1`) for the **Elecrow ThinkNode M1** nRF52840 LoRa mesh node.

## Hardware Specifications

| Component | Part / Spec | Notes |
| :--- | :--- | :--- |
| **MCU** | Nordic nRF52840 | ARM Cortex-M4F (`thumbv7em-none-eabihf`), 1 MB flash, 256 KB RAM |
| **Radio** | Semtech SX1262 | SPI (`TWISPI0`), DIO1 IRQ, external TCXO / power control |
| **Display** | 1.54" SSD1681 E-Paper | 200x200 monochrome (`TWISPI1`), frontlight LED |
| **GNSS** | Quectel L76K | UART NMEA (`UARTE0`), standby & reinit control |
| **External Flash** | Macronix MX25R1635F | 2 MB QSPI flash (contacts, adverts & message FIFO) |
| **User I/O** | 2 buttons, 2 LEDs, buzzer | Top action button (`FK`), bottom page button (`TK`), passive buzzer |
| **Power / Telemetry** | Battery ADC + SoftDevice | `SAADC` battery voltage divider on `P0.04` |

---

## Controls

| Input | Action |
| :--- | :--- |
| Top button (`FK`) single click | Primary page action |
| Top button (`FK`) double click | Secondary page action |
| Bottom button (`TK`) single click | Next page |
| Bottom button (`TK`) double click | Previous page |

---

## Building & Testing

```sh
# Release build (external QSPI + internal flash storage)
./tools/build.sh thinknode-m1 build --release

# Release build storing everything in internal flash (no QSPI init)
./tools/build.sh thinknode-m1 build --release --features internal-storage

# Run all host unit tests (shared across workspace crates and board logic)
./tools/build.sh test
```

**Host tests** (`./tools/build.sh test`, also run in CI) include the
`ferrix-thinknode-m1-logic` crate (`logic/`), which holds the board's pure
logic off-target: the 200x200 e-paper UI, single/double-click button mapping,
display refresh/sleep state machine, buzzer patterns, flash layout, and
settings-to-SX1262 mapping.

- The UI is checked against 27 golden frames in `logic/tests/golden/*.txt`
  (200x200, `#` = ink). After an intentional UI change, regenerate and review
  the diff:
  `BLESS=1 cargo test -p ferrix-thinknode-m1-logic --target "$(rustc -vV | sed -n 's/host: //p')" --test snapshots`
- `layout.rs` tests parse `memory.x`, so the linker script and storage regions
  cannot drift.

**Hardware-in-the-loop smoke test** (needs a board on USB, not run in CI):

```sh
./tools/build.sh thinknode-m1 hil                      # auto-detects the port
./tools/build.sh thinknode-m1 hil --port /dev/cu.usbmodemXXXX --gps-timeout 60
./tools/build.sh thinknode-m1 hil --internal-storage   # for --features internal-storage builds
./tools/build.sh thinknode-m1 hil --tx --soak 600      # also transmit one advert; 10 min stability soak
```

Checks clocks/regulators (`xtal(50ppm)` LFXO + REG1 DC-DC), MX25R1635F QSPI
flash (`c22815`), battery ADC, radio settings, the binary companion handshake,
GPS UART traffic, and config persistence across a reboot. Pass `--flash` to
build and flash first; `hil --self-test` verifies the script against a fake
device in CI.

---

## Flashing

> **Before your first flash:** Ferrix uses its own on-flash storage format instead of standard MeshCore's LittleFS layout. Perform a **full flash erase** before flashing Ferrix for the first time, and again if you ever return to stock MeshCore. Back up your private key first if you want to keep your node identity.

### 1. UF2 Drag-and-Drop

Builds the release binary and packages a UF2 image (family ID `0xADA52840`, base `0x26000`) at `target/thumbv7em-none-eabihf/release/ferrix-thinknode-m1.uf2` (or download `ferrix-thinknode-m1-companion-v<version>.uf2` from GitHub Releases):

```sh
./tools/build_uf2.sh thinknode-m1 release
```

1. Connect the node via a USB-C data cable.
2. **Double-click** the recessed reset button (or run `dfu` in the USB text CLI) to mount the `THINKNODE` / `NRF52BOOT` USB drive.
3. Copy the `.uf2` file onto the drive; the board reboots into Ferrix automatically.

### 2. Serial DFU

Auto-detects the ThinkNode M1 USB port, triggers 1200-baud DFU touch, packages a Nordic DFU `.zip` (requires [`adafruit-nrfutil`](https://github.com/adafruit/Adafruit_nRF52_nrfutil)), and flashes over serial:

```sh
./tools/flash_serial.sh

# Or specify the serial port explicitly:
./tools/flash_serial.sh /dev/cu.usbmodem1101
```

### 3. BLE OTA DFU

Builds a Nordic/Adafruit DFU `.zip` targeting SoftDevice ID `0x00B6` (S140 v6.1.1) — or download `ferrix-thinknode-m1-companion-v<version>.zip` from GitHub Releases:

```sh
./tools/build_ota.sh thinknode-m1 release
```

On boards with an OTAFIX-compatible bootloader, enter BLE DFU via `start ota` in the CLI or Legacy Buttonless DFU from an nRF DFU app.

### 4. SWD Debug Probe

Configured in `.cargo/config.toml` for `probe-rs run --chip nRF52840_xxAA`:

```sh
./tools/build.sh thinknode-m1 run --release
```

---

## Pinout (`src/pins.rs`)

| Subsystem | Signal | nRF52840 Pin | Function |
| :--- | :--- | :--- | :--- |
| **Power** | `PERIPH_PWR_EN` | `P0.12` | Peripheral rail power enable (EPD, LoRa, GPS, sensors) |
| **SX1262 LoRa** | `SX126X_SCK` | `P0.19` | SPI Clock (`TWISPI0`) |
| | `SX126X_MOSI` | `P0.22` | SPI MOSI |
| | `SX126X_MISO` | `P0.23` | SPI MISO |
| | `SX126X_CS` | `P0.24` | SPI Chip Select (active low) |
| | `SX126X_RESET` | `P0.25` | Hardware Reset (active low) |
| | `SX126X_BUSY` | `P0.17` | Radio Busy (active high) |
| | `SX126X_DIO1` | `P0.20` | IRQ Event Interrupt (TX/RX Done) |
| | `SX126X_POWER_EN` | `P0.21` | Radio rail / RF switch enable (active high) |
| **1.54" E-Paper (SSD1681)** | `EINK_SCLK` | `P0.31` | SPI Clock (`TWISPI1`) |
| | `EINK_MOSI` | `P0.29` | SPI MOSI |
| | `EINK_CS` | `P0.30` | Display Chip Select (active low) |
| | `EINK_DC` | `P0.28` | Data / Command |
| | `EINK_RST` | `P0.02` | Display Reset (active low) |
| | `EINK_BUSY` | `P0.03` | Display Busy (active high) |
| | `EINK_EN` | `P1.11` | Frontlight Enable (active high) |
| **External QSPI (MX25R1635F)** | `QSPI_SCK` | `P1.14` | QSPI Clock |
| | `QSPI_CSN` | `P1.15` | QSPI Chip Select |
| | `QSPI_IO0`..`IO3` | `P1.12`, `P1.13`, `P0.07`, `P0.05` | Quad I/O Data Lines |
| **GNSS (Quectel L76K)** | `GPS_TX` | `P1.09` | MCU RX (NMEA sentences from GNSS, `UARTE0`) |
| | `GPS_RX` | `P1.08` | MCU TX (Commands to GNSS, `UARTE0`) |
| | `GPS_STANDBY` | `P1.02` | Low-power standby control |
| | `GPS_REINIT` | `P1.05` | Reset / Reinitialization control |
| **User I/O & Telemetry** | `FK_BUTTON` | `P1.07` | Top button (Page Action, pull-up) |
| | `TK_BUTTON` | `P1.10` | Bottom button (Next/Prev Page, pull-up) |
| | `BUZZER` | `P0.06` | Passive buzzer (GPIO square wave) |
| | `DATA_LED` | `P0.13` | Blue Status LED (active high) |
| | `PWR_LED` | `P1.04` | Red Power/Charge LED (active high) |
| | `BAT_ADC` | `P0.04` | Battery Voltage ADC (`SAADC` AIN2) |
| | `MCU_RESET` | `P0.18` | Hardware Reset (double-click enters UF2 DFU) |

---

## Memory Map (`memory.x`)

Matches the factory Adafruit nRF52 UF2 bootloader and Nordic SoftDevice S140 layout:

### Internal nRF52840 Flash & RAM

| Region | Address Range | Purpose |
| :--- | :--- | :--- |
| **MBR + SoftDevice** | `0x00000000` – `0x00026000` | Master Boot Record & SoftDevice S140 |
| **Application Flash** | `0x00026000` – `0x000A0000` | Ferrix firmware (~488 KB) |
| **Config Journal** | `0x000A0000` – `0x000B0000` | `sequential-storage` config map |
| **BLE Bonds** | `0x000B0000` – `0x000B4000` | `sequential-storage` bond map |
| **Bootloader** | `0x000ED000` – `0x00100000` | Adafruit UF2 bootloader & settings |
| **SoftDevice RAM** | `0x20000000` – `0x20006000` | SoftDevice reserved RAM (24 KB) |
| **Application RAM** | `0x20006000` – `0x20040000` | Ferrix application RAM (232 KB) |

### External QSPI Flash (MX25R1635F, 2 MB)

| Offset Range | Size | Purpose |
| :--- | :--- | :--- |
| `0x000000` – `0x040000` | 256 KB | Contacts + advert blobs (`sequential-storage` map) |
| `0x040000` – `0x140000` | 1 MB | Messages (`sequential-storage` FIFO) |
| `0x140000` – `0x200000` | 768 KB | Unused |
