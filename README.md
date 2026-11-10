# Ferrix

A generic, native `no_std` embedded Rust companion firmware for **MeshCore** LoRa mesh nodes.

Built on the **Embassy** asynchronous runtime for the Nordic **nRF52840** ARM Cortex-M4F microcontroller, with a modular board abstraction layer and extensible Semtech transceiver support (`LoraRadio`).

---

## Supported Boards

| Board Target | Feature Flag | MCU | Radio | Display | GNSS / GPS |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Elecrow ThinkNode M1** | `thinknode-m1` (default) | nRF52840 | Semtech SX1262 | 1.54" SSD1681 E-Paper | Quectel L76K |

---

## Architecture

The firmware decouples application logic from board-specific pinouts and radio hardware:

```text
src/
├── main.rs                  <-- Generic application loop & state machine
├── radio/
│   ├── mod.rs               <-- LoraRadio async trait & LoraConfig
│   ├── sx1262.rs            <-- Semtech SX1262 transceiver driver
│   └── commands.rs          <-- Semtech SX126x opcode definitions
├── boards/                  <-- BOARD SUPPORT LAYER
│   ├── mod.rs               <-- Board trait, capability flags, & feature gating
│   ├── thinknode_m1.rs      <-- ThinkNode M1 hardware pinout & peripherals
│   └── rak4631.rs           <-- RAKwireless WisBlock Core pinout
├── companion/               <-- MeshCore USB CDC-ACM & BLE packet framing & protocol
├── display/                 <-- 1.54" SSD1681 driver & dashboard UI (conditional)
├── gps/                     <-- Quectel L76K NMEA driver (conditional)
└── telemetry/               <-- Battery voltage ADC monitoring & health
```

### Adding a New Radio or Board
1. **New Radio**: Implement the `LoraRadio` async trait in [`src/radio/`](file:///Users/navmad/Projects/embedded-projects/ferrix/src/radio).
2. **New Board**: Add a board module to [`src/boards/`](file:///Users/navmad/Projects/embedded-projects/ferrix/src/boards) and define its pinout, radio type, and capability flags (`HAS_DISPLAY`, `HAS_GPS`).

---

## Hardware Pinout Reference (ThinkNode M1)

| Subsystem | Signal Name | nRF52840 Pin | Function / Description |
| :--- | :--- | :--- | :--- |
| **SX1262 LoRa** | `SX126X_SCK` | `P0.19` | SPI Clock |
| | `SX126X_MOSI` | `P0.22` | SPI Master Out Slave In |
| | `SX126X_MISO` | `P0.23` | SPI Master In Slave Out |
| | `SX126X_CS` | `P0.24` | SPI Chip Select (Active Low) |
| | `SX126X_RESET` | `P0.25` | Hardware Reset (Active Low) |
| | `SX126X_BUSY` | `P0.17` | Radio Busy Signal (Active High) |
| | `SX126X_DIO1` | `P0.20` | IRQ Event Interrupt (TX/RX Done) |
| | `SX126X_DIO3` | `P0.21` | RF Switch & TCXO Power Control |
| **1.54" E-Paper (SSD1681)** | `EINK_SCLK` | `P0.31` | SPI Clock |
| | `EINK_MOSI` | `P0.29` | SPI Data (DIN) |
| | `EINK_CS` | `P0.30` | Display Chip Select (Active Low) |
| | `EINK_DC` | `P0.28` | Data / Command (High = Data, Low = Cmd) |
| | `EINK_RST` | `P0.02` | Display Reset (Active Low) |
| | `EINK_BUSY` | `P0.03` | Display Busy (Active High) |
| | `EINK_EN` | `P1.11` | Front-light / Backlight Enable |
| **GPS (Quectel L76K)** | `GPS_TX` | `P1.09` | MCU RX (NMEA sentences from GPS) |
| | `GPS_RX` | `P1.08` | MCU TX (Commands to GPS) |
| | `GPS_STANDBY`| `P1.02` | Low-power standby control |
| | `GPS_REINIT` | `P1.05` | Reset / Reinitialization control |
| **User I/O & Telemetry**| `FK_BUTTON` | `P1.07` | Function Button |
| | `TK_BUTTON` | `P1.10` | Page Turn Button |
| | `MCU_RESET` | `P0.18` | Hardware Reset (Double-click enters DFU) |
| | `DATA_LED` | `P0.13` | Blue Status LED (Active Low) |
| | `PWR_LED` | `P1.04` | Red Power/Charge LED |
| | `BAT_ADC` | `P0.04` | Battery Voltage ADC (AIN2) |

---

## MeshCore Companion Protocol

The firmware connects to official MeshCore companion apps (iOS / Android / Web / CLI):

1. **BLE Transport**:
   - **Service UUID**: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` (Nordic UART Service)
   - **RX Characteristic**: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` (App -> Device)
   - **TX Characteristic**: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` (Device -> App)
2. **USB CDC-ACM Transport**:
   - Inbound Frame (App -> Radio): `<` (`0x3C`) + 2-byte little-endian length + payload
   - Outbound Frame (Radio -> App): `>` (`0x3E`) + 2-byte little-endian length + payload
3. **Core Command Handlers**:
   - `CMD_DEVICE_QUERY (22)` -> `RESP_CODE_DEVICE_INFO (13)`
   - `CMD_APP_START (1)` -> `RESP_CODE_SELF_INFO (5)`
   - `CMD_GET_CONTACTS (4)`
   - `CMD_GET_BAT (19)`
   - `CMD_GET_DEVICE_TIME (5)` / `CMD_SET_DEVICE_TIME (6)`
   - `CMD_SEND_SELF_ADVERT (7)`

---

## Building and Flashing

### 1. Build UF2 Package (Drag-and-Drop)

The build script compiles the release binary and packages it into a UF2 container with family ID `0xADA52840`:

```bash
# Build for ThinkNode M1 (default):
./tools/build_uf2.sh thinknode-m1 release
```

Output UF2 packages are placed in:
`target/thumbv7em-none-eabihf/release/ferrix-<board>.uf2`

### 2. Flash to Device

1. Connect your node to your computer using a USB-C data cable.
2. **Double-click** the reset button on your device.
3. A USB mass-storage drive will mount automatically (e.g. `THINKNODE`, `RAK4631`, or `NRF52BOOT`).
4. Drag and drop the corresponding `.uf2` file onto that drive.
5. The device will flash and reboot into your MeshCore firmware.

### 3. Flash via SWD Debug Probe (Optional)

If using a J-Link, CMSIS-DAP, or ST-Link:
```bash
cargo run --release --features thinknode-m1
```
*(Configured with `probe-rs run --chip nRF52840_xxAA`)*
