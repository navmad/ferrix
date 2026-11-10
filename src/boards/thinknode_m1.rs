//! Elecrow ThinkNode M1 Board Support Package
//!
//! Target MCU: Nordic Semiconductor nRF52840 (ARM Cortex-M4F @ 64 MHz)
//! Subsystems:
//! - Semtech SX1262 LoRa Transceiver (SPI0)
//! - 1.54" SSD1681 Monochrome E-Paper Display (SPI1)
//! - Quectel L76K Multi-GNSS / GPS Receiver (UARTE0)
//! - User Buttons (FK, TK), Battery ADC (P0.04), and Dual Status LEDs

use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals;

use crate::boards::Board;
use crate::companion::protocol::{CAP_BLE, CAP_CLIENT, CAP_EPD, CAP_GPS};
use crate::input::{InputDevice, InputEvent};

/// Semtech SX1262 LoRa Radio Pin Configuration
pub struct LoraPins {
    pub sck: peripherals::P0_19,
    pub mosi: peripherals::P0_22,
    pub miso: peripherals::P0_23,
    pub cs: Output<'static>,
    pub reset: Output<'static>,
    pub busy: Input<'static>,
    pub dio1: Input<'static>,
    pub rf_sw: Output<'static>,
}

/// 1.54" 200x200 Monochrome E-Paper Display (SSD1681) Pin Configuration
pub struct DisplayPins {
    pub sclk: peripherals::P0_31,
    pub mosi: peripherals::P0_29,
    pub cs: Output<'static>,
    pub dc: Output<'static>,
    pub reset: Output<'static>,
    pub busy: Input<'static>,
    pub backlight: Output<'static>,
}

/// Quectel L76K GNSS / GPS Module Pin Configuration
pub struct GpsPins {
    pub tx: peripherals::P1_08,
    pub rx: peripherals::P1_09,
    pub standby: Output<'static>,
    pub reinit: Output<'static>,
}

/// Physical User Controls (ThinkNode M1 Buttons mapping to semantic InputEvents)
pub struct ThinkNodeInputs {
    pub function_btn: Input<'static>,
    pub page_btn: Input<'static>,
    last_fk: bool,
    last_tk: bool,
}

impl ThinkNodeInputs {
    pub fn new(function_btn: Input<'static>, page_btn: Input<'static>) -> Self {
        Self {
            function_btn,
            page_btn,
            last_fk: false,
            last_tk: false,
        }
    }
}

impl InputDevice for ThinkNodeInputs {
    async fn poll_event(&mut self) -> Option<InputEvent> {
        let fk_active = self.function_btn.is_low();
        let tk_active = self.page_btn.is_low();

        // Edge detection for Function button (FK) -> ActionPrimary
        if fk_active && !self.last_fk {
            self.last_fk = true;
            return Some(InputEvent::ActionPrimary);
        } else if !fk_active {
            self.last_fk = false;
        }

        // Edge detection for Page button (TK) -> NextPage
        if tk_active && !self.last_tk {
            self.last_tk = true;
            return Some(InputEvent::NextPage);
        } else if !tk_active {
            self.last_tk = false;
        }

        None
    }
}

/// Status and Telemetry Pins
pub struct TelemetryPins {
    pub blue_led: Output<'static>,
    pub red_led: Output<'static>,
}

/// Combined Board Peripherals struct for ThinkNode M1
pub struct BoardPins {
    pub lora: LoraPins,
    pub display: Option<DisplayPins>,
    pub gps: Option<GpsPins>,
    pub inputs: ThinkNodeInputs,
    pub telemetry: TelemetryPins,
    pub power_en: Output<'static>,
    pub spim0: peripherals::TWISPI0,
    pub spim1: peripherals::TWISPI1,
    pub uarte0: peripherals::UARTE0,
    pub saadc: peripherals::SAADC,
    pub bat_adc_pin: peripherals::P0_04,
}

pub struct ThinkNodeM1;

impl Board for ThinkNodeM1 {
    const NAME: &'static str = "ThinkNode M1";
    const HAS_DISPLAY: bool = true;
    const HAS_GPS: bool = true;
    const HAS_BATTERY_ADC: bool = true;
    const CAPABILITIES: u32 = CAP_CLIENT | CAP_GPS | CAP_EPD | CAP_BLE;

    type Pins = BoardPins;

    fn init(p: embassy_nrf::Peripherals) -> Self::Pins {
        // --- Peripheral Power Enable (P0.12) ---
        // Powers on EPD display, LoRa, GPS, and sensors
        let power_en = Output::new(p.P0_12, Level::High, OutputDrive::Standard);

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
        // Backlight (P1.11): Active-high (High = ON)
        let epd_backlight = Output::new(p.P1_11, Level::High, OutputDrive::Standard);

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

        // --- Buttons & User Inputs ---
        // P1.07 is PIN_BUTTON2 (Main / Page Turn button)
        // P1.10 is PIN_BUTTON1 (Aux / Function button)
        let page_btn = Input::new(p.P1_07, Pull::Up);
        let function_btn = Input::new(p.P1_10, Pull::Up);
        let inputs = ThinkNodeInputs::new(function_btn, page_btn);

        // --- Telemetry & LEDs ---
        // Blue LED (DATA_LED, P0.13) is active-high: Low = OFF, High = ON
        let blue_led = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);
        // Red LED (PWR_LED, P1.04) is active-high: Low = OFF, High = ON
        let red_led = Output::new(p.P1_04, Level::Low, OutputDrive::Standard);

        let telemetry = TelemetryPins {
            blue_led,
            red_led,
        };

        BoardPins {
            lora,
            display: Some(display),
            gps: Some(gps),
            inputs,
            telemetry,
            power_en,
            spim0: p.TWISPI0,
            spim1: p.TWISPI1,
            uarte0: p.UARTE0,
            saadc: p.SAADC,
            bat_adc_pin: p.P0_04,
        }
    }
}

