# Seeed Wio Tracker L1 Pro

Board support package (`ferrix-wio-tracker-l1-pro`) for the **Seeed Wio Tracker L1 Pro** nRF52840 LoRa mesh node.
The pinout comes from MeshCore's `variants/wio-tracker-l1`; Ferrix uses the same SoftDevice layout as the stock MeshCore firmware.

## Hardware Specifications

| Component | Part / Spec | Notes |
| :--- | :--- | :--- |
| **MCU** | Nordic nRF52840 | ARM Cortex-M4F (`thumbv7em-none-eabihf`), 1 MB flash, 256 KB RAM, LFXO + REG1 DC-DC |
| **Radio** | Semtech SX1262 | SPI (`TWISPI0`), DIO1 IRQ, 1.8 V TCXO from DIO3, DIO2 drives TX, GPIO `RXEN` (P1.08) drives RX |
| **Display** | 1.3" SH1106 OLED | 128x64 monochrome, I2C `0x3D` (`TWISPI1` as TWIM), sleeps after 30 s idle |
| **GNSS** | Quectel L76K | UART NMEA (`UARTE0`, 9600 baud), standby/enable on P1.09, **no reset line** |
| **External Flash** | Puya P25Q16H | 2 MB QSPI flash (contacts, adverts & message FIFO), JEDEC `85 60 15` |
| **User I/O** | User button, 5-way joystick, LED, buzzer | See [Controls](#controls) |
| **Power / Telemetry** | Battery ADC + SoftDevice | `SAADC` on P0.31 (AIN7), 2:1 divider enabled by P0.04 |

## Pinout

| Function | Pin(s) |
| :--- | :--- |
| LoRa SCK / MISO / MOSI | P0.30 / P0.03 / P0.28 |
| LoRa NSS / RESET / BUSY / DIO1 / RXEN | P1.14 / P1.07 / P1.10 / P0.07 / P1.08 |
| OLED SDA / SCL | P0.06 / P0.05 |
| GPS RX (MCU) / TX (MCU) / standby | P0.26 / P0.27 / P1.09 |
| User button | P0.08 |
| Joystick up / down / left / right / press | P1.04 / P0.12 / P0.11 / P1.03 / P1.05 |
| LED / buzzer | P1.01 / P1.00 |
| VBAT ADC / VBAT enable | P0.31 / P0.04 |
| QSPI SCK / CS / IO0-IO3 | P0.21 / P0.25 / P0.20 / P0.24 / P0.22 / P0.23 |

## Controls

| Input | Action |
| :--- | :--- |
| Joystick left / up | Previous page |
| Joystick right / down | Next page |
| Joystick press (`OK`) | Primary page action (see the footer of each page) |
| User button (`BTN`) | Secondary page action |

The 128x64 UI keeps the same nine pages as the ThinkNode (MESH, INBOX, RADAR, RADIO, SIGNAL, POSITION, OFF-GRID, SETTINGS, SYSTEM); the footer shows the `OK` / `BTN` action for the current page.

## Memory map

See `memory.x`. The app is linked at `0x27000`, after the S140 v7.3.0 SoftDevice, and the factory bootloader lives at `0xED000`. Contacts, adverts and messages live on the external QSPI flash; with `--features internal-storage` they move into the free internal window instead.

## Building & Testing

```sh
# Release build (external QSPI + internal flash storage)
./tools/build.sh wio-tracker-l1-pro build --release

# Release build storing everything in internal flash (no QSPI init)
./tools/build.sh wio-tracker-l1-pro build --release --features internal-storage

# Run host unit tests (shared with every board)
./tools/build.sh wio-tracker-l1-pro test
```

## Testing

**Host tests** (`./tools/build.sh wio-tracker-l1-pro test`, also run in CI) include the
`ferrix-wio-tracker-l1-pro-logic` crate (`logic/`), which holds the board's pure logic so it
builds off-target: the OLED UI, joystick/button mapping, display idle/wake/refresh state
machine, buzzer patterns, flash layout and the settings-to-SX1262 mapping. The firmware crate
re-exports these modules, so there is one copy of the code.

- The UI is checked against golden frames in `logic/tests/golden/*.txt` (128x64, `#` = lit).
  After an intentional UI change, regenerate and review the diff:
  `BLESS=1 cargo test -p ferrix-wio-tracker-l1-pro-logic --target "$(rustc -vV | sed -n 's/host: //p')" --test snapshots`
- `layout.rs` tests parse `memory.x`, so the linker script and the storage regions cannot drift.

**Hardware-in-the-loop smoke test** (needs a board on USB, not run in CI):

```sh
./tools/build.sh wio-tracker-l1-pro hil                      # auto-detects the port
./tools/build.sh wio-tracker-l1-pro hil --port /dev/cu.usbmodemXXXX --gps-timeout 60
./tools/build.sh wio-tracker-l1-pro hil --internal-storage   # for --features internal-storage builds
./tools/build.sh wio-tracker-l1-pro hil --tx --soak 600      # also transmit one advert; 10 min stability soak
```

It checks clocks and regulators, QSPI flash, battery, radio settings, the binary companion
handshake, GPS UART traffic, and that a name change survives a real reboot (the original name is
restored). It exercises the hardware items listed as unverified below (GPS wiring, QSPI ID); `--tx`
verifies the SX1262 path. `--flash` builds and flashes first. `hil --self-test` runs the script's
own logic against a fake device and is run in CI.

## Flashing

> **Before your first flash:** Ferrix uses its own on-flash storage format, so do a full flash erase before the first Ferrix flash and again if you return to stock MeshCore. Back up your private key first if you want to keep your node identity.

```sh
# UF2: double-tap reset, then drag the file onto the mounted drive
./tools/build_uf2.sh wio-tracker-l1-pro release

# Nordic/Adafruit DFU package for BLE OTA or serial DFU
./tools/build_ota.sh wio-tracker-l1-pro release
./tools/flash_serial.sh /dev/cu.usbmodemXXXX wio-tracker-l1-pro release
```

The UF2 base address is `0x27000` and the DFU package targets SoftDevice ID `0x0123` (S140 v7.3.0).

## Hardware items still to confirm on a device

These were taken from MeshCore's variant files and have not been verified on hardware:

1. **USB PID.** Ferrix reports Seeed VID `0x2886`, PID `0x1668` (from MeshCore's board definition, where `0x1667` is the bootloader). Check that companion apps detect the port.
2. **GPS wiring and polarity.** MCU RX is P0.26 and MCU TX is P0.27 (`Serial1`); standby/enable is P1.09, assumed active-high. If the `gps` CLI counters show no bytes after enabling GPS, swap the polarity in `src/gps.rs` or the RX/TX pins in `src/pins.rs`.
3. **OLED bus.** The OLED is assumed to be on P0.06 / P0.05 at `0x3D`. A blank screen with a normal boot means the bus or address is wrong (the firmware logs an OLED init failure but keeps running).
4. **RF switch.** If RX sensitivity is poor, check the `RXEN` handling in `src/radio.rs`.
5. **SoftDevice RAM.** If the node shows 10 fast blinks at boot, raise the RAM origin in `memory.x` to `0x20008000`.
6. **BLE OTA DFU name.** The on-screen DFU notice is generic because the bootloader's advertised name is not confirmed.
