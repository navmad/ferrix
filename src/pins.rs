//! Hardware Pin Definitions for Elecrow ThinkNode M1
//!
//! Target MCU: Nordic Semiconductor nRF52840 (ARM Cortex-M4F @ 64 MHz)
//! Subsystems:
//! - Semtech SX1262 LoRa Transceiver (SPI + Interrupts)
//! - 1.54" SSD1681 Monochrome E-Paper Display (SPI + Control)
//! - Quectel L76K Multi-GNSS / GPS Receiver (UARTE)
//! - On-board Buttons, Battery Voltage ADC Sense, and Status LEDs

use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals;

/// Semtech SX1262 LoRa Radio Pin Configuration
pub struct LoraPins {
    /// SPI Clock (P0.19)
    pub sck: peripherals::P0_19,
    /// SPI MOSI - Master Out Slave In (P0.22)
    pub mosi: peripherals::P0_22,
    /// SPI MISO - Master In Slave Out (P0.23)
    pub miso: peripherals::P0_23,
    /// SPI Chip Select / NSS (P0.24, Active Low)
    pub cs: Output<'static>,
    /// Hardware Reset (P0.25, Active Low)
    pub reset: Output<'static>,
    /// Radio Busy status line (P0.17, Active High)
    pub busy: Input<'static>,
    /// Radio DIO1 Interrupt line (P0.20, TX/RX Done, Timeout)
    pub dio1: Input<'static>,
    /// Radio DIO3 / RF Switch / TCXO Power Control (P0.21)
    pub rf_sw: Output<'static>,
}

/// 1.54" 200x200 Monochrome E-Paper Display (SSD1681) Pin Configuration
pub struct DisplayPins {
    /// SPI Clock (P0.31)
    pub sclk: peripherals::P0_31,
    /// SPI MOSI / DIN (P0.29)
    pub mosi: peripherals::P0_29,
    /// Display Chip Select (P0.30, Active Low)
    pub cs: Output<'static>,
    /// Data / Command selection (P0.28, High = Data, Low = Command)
    pub dc: Output<'static>,
    /// Display Reset (P0.02, Active Low)
    pub reset: Output<'static>,
    /// Display Busy indicator (P0.03, High = Busy)
    pub busy: Input<'static>,
    /// Front-light / E-Ink backlight enable (P1.11, High = On)
    pub backlight: Output<'static>,
}

/// Quectel L76K GNSS / GPS Module Pin Configuration
pub struct GpsPins {
    /// MCU Transmit -> GPS Receive (P1.08)
    pub tx: peripherals::P1_08,
    /// MCU Receive <- GPS Transmit (P1.09)
    pub rx: peripherals::P1_09,
    /// GPS Standby control line (P1.02)
    pub standby: Output<'static>,
    /// GPS Reinitialization / Reset line (P1.05)
    pub reinit: Output<'static>,
}

/// Physical User Controls (Buttons)
pub struct ButtonPins {
    /// Function Button (FK / P1.07, Active Low with internal pull-up)
    pub function_btn: Input<'static>,
    /// Page Turn Button (TK / P1.10, Active Low with internal pull-up)
    pub page_btn: Input<'static>,
}

/// Status and Telemetry Pins
pub struct TelemetryPins {
    /// Blue Data / Status LED (P0.13, Active Low)
    pub blue_led: Output<'static>,
    /// Red Power / Charging LED (P1.04, Active Low)
    pub red_led: Output<'static>,
}

/// Combined Board Peripherals struct
pub struct BoardPins {
    pub lora: LoraPins,
    pub display: DisplayPins,
    pub gps: GpsPins,
    pub buttons: ButtonPins,
    pub telemetry: TelemetryPins,
    pub spim0: peripherals::TWISPI0,
    pub spim1: peripherals::TWISPI1,
    pub uarte0: peripherals::UARTE0,
    pub saadc: peripherals::SAADC,
    pub bat_adc_pin: peripherals::P0_04,
}

impl BoardPins {
    /// Initialize all hardware pins according to ThinkNode M1 hardware specs.
    pub fn init(p: embassy_nrf::Peripherals) -> Self {
        // --- LoRa SX1262 Pins ---
        let lora_cs = Output::new(p.P0_24, Level::High, OutputDrive::Standard);
        let lora_reset = Output::new(p.P0_25, Level::High, OutputDrive::Standard);
        let lora_busy = Input::new(p.P0_17, Pull::None);
        let lora_dio1 = Input::new(p.P0_20, Pull::None);
        let lora_rf_sw = Output::new(p.P0_21, Level::Low, OutputDrive::Standard);

        let lora = LoraPins {
            sck: p.P0_19,
            mosi: p.P0_22,
            miso: p.P0_23,
            cs: lora_cs,
            reset: lora_reset,
            busy: lora_busy,
            dio1: lora_dio1,
            rf_sw: lora_rf_sw,
        };

        // --- E-Paper Display Pins ---
        let epd_cs = Output::new(p.P0_30, Level::High, OutputDrive::Standard);
        let epd_dc = Output::new(p.P0_28, Level::Low, OutputDrive::Standard);
        let epd_reset = Output::new(p.P0_02, Level::High, OutputDrive::Standard);
        let epd_busy = Input::new(p.P0_03, Pull::None);
        let epd_backlight = Output::new(p.P1_11, Level::Low, OutputDrive::Standard);

        let display = DisplayPins {
            sclk: p.P0_31,
            mosi: p.P0_29,
            cs: epd_cs,
            dc: epd_dc,
            reset: epd_reset,
            busy: epd_busy,
            backlight: epd_backlight,
        };

        // --- GPS Pins ---
        let gps_standby = Output::new(p.P1_02, Level::Low, OutputDrive::Standard);
        let gps_reinit = Output::new(p.P1_05, Level::High, OutputDrive::Standard);

        let gps = GpsPins {
            tx: p.P1_08,
            rx: p.P1_09,
            standby: gps_standby,
            reinit: gps_reinit,
        };

        // --- Buttons ---
        let function_btn = Input::new(p.P1_07, Pull::Up);
        let page_btn = Input::new(p.P1_10, Pull::Up);

        let buttons = ButtonPins {
            function_btn,
            page_btn,
        };

        // --- Telemetry & LEDs ---
        let blue_led = Output::new(p.P0_13, Level::High, OutputDrive::Standard);
        let red_led = Output::new(p.P1_04, Level::High, OutputDrive::Standard);

        let telemetry = TelemetryPins {
            blue_led,
            red_led,
        };

        Self {
            lora,
            display,
            gps,
            buttons,
            telemetry,
            spim0: p.TWISPI0,
            spim1: p.TWISPI1,
            uarte0: p.UARTE0,
            saadc: p.SAADC,
            bat_adc_pin: p.P0_04,
        }
    }
}
