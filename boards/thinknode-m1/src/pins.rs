//! ThinkNode M1 pinout and peripheral construction.
//!
//! This module only describes the nRF52840 pins and the timing of the two
//! buttons. It does not own board identity, power telemetry or startup logic.

use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pull};
use embassy_nrf::peripherals;
use embassy_time::Timer;
use ferrix_runtime::app::{InputEvent, INPUT_EVENT_CHANNEL};

embassy_nrf::bind_interrupts!(pub(crate) struct BoardIrqs {
    TWISPI0 => embassy_nrf::spim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => embassy_nrf::spim::InterruptHandler<peripherals::TWISPI1>;
    SAADC => embassy_nrf::saadc::InterruptHandler;
});

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

/// Physical user controls mapped to semantic input events.
pub struct ThinkNodeInputs {
    pub function_btn: Input<'static>,
    pub page_btn: Input<'static>,
}

impl ThinkNodeInputs {
    pub fn new(function_btn: Input<'static>, page_btn: Input<'static>) -> Self {
        Self {
            function_btn,
            page_btn,
        }
    }

    /// Wait for the next debounced semantic input event.
    pub async fn wait_event(&mut self) -> Option<InputEvent> {
        use embassy_futures::select::{select, Either};

        let first = select(
            self.function_btn.wait_for_falling_edge(),
            self.page_btn.wait_for_falling_edge(),
        )
        .await;
        let is_function = matches!(first, Either::First(_));

        // Let contact bounce settle, then look for a second click of the
        // same button within the double-click window.
        Timer::after_millis(40).await;
        let double = if is_function {
            matches!(
                select(
                    self.function_btn.wait_for_falling_edge(),
                    Timer::after_millis(280)
                )
                .await,
                Either::First(_)
            )
        } else {
            matches!(
                select(
                    self.page_btn.wait_for_falling_edge(),
                    Timer::after_millis(280)
                )
                .await,
                Either::First(_)
            )
        };

        Some(if is_function {
            if double {
                InputEvent::ActionSecondary
            } else {
                InputEvent::ActionPrimary
            }
        } else if double {
            InputEvent::PrevPage
        } else {
            InputEvent::NextPage
        })
    }
}

/// Background button task: blocks on GPIOTE edges and forwards semantic events.
#[embassy_executor::task]
pub async fn thinknode_input_task(mut inputs: ThinkNodeInputs) -> ! {
    loop {
        if let Some(event) = inputs.wait_event().await {
            let _ = INPUT_EVENT_CHANNEL.send(event).await;
        }
    }
}

/// Status and Telemetry Pins
pub struct TelemetryPins {
    pub blue_led: Output<'static>,
    pub red_led: Output<'static>,
}

/// Combined board peripherals for ThinkNode M1.
pub struct Pins {
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
    pub usbd: peripherals::USBD,
    pub qspi: ferrix_nrf52::qspi::QspiPins,
    pub wdt: peripherals::WDT,
    /// Passive buzzer on P0.06, driven as a GPIO square wave.
    pub buzzer: Output<'static>,
}

/// Initialize the ThinkNode M1 pinout and peripherals.
pub fn init(p: embassy_nrf::Peripherals) -> Pins {
    // --- Peripheral Power Enable (P0.12) ---
    // Powers on EPD display, LoRa, GPS, and sensors
    let power_en = Output::new(p.P0_12, Level::High, OutputDrive::Standard);

    // --- LoRa SX1262 Pins ---
    let lora_cs = Output::new(p.P0_24, Level::High, OutputDrive::Standard);
    let lora_reset = Output::new(p.P0_25, Level::High, OutputDrive::Standard);
    let lora_busy = Input::new(p.P0_17, Pull::None);
    let lora_dio1 = Input::new(p.P0_20, Pull::None);
    // P0.21 is SX126X_POWER_EN (active-high power enable for the SX1262 rail)
    let lora_rf_sw = Output::new(p.P0_21, Level::High, OutputDrive::Standard);

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

    // --- Buttons & User Inputs ---
    // P1.07 is the top button: page action.
    // P1.10 is the bottom button: next page.
    let function_btn = Input::new(p.P1_07, Pull::Up);
    let page_btn = Input::new(p.P1_10, Pull::Up);
    let inputs = ThinkNodeInputs::new(function_btn, page_btn);

    // --- Telemetry & LEDs ---
    // Blue LED (DATA_LED, P0.13) is active-high: Low = OFF, High = ON
    let blue_led = Output::new(p.P0_13, Level::Low, OutputDrive::Standard);
    // Red LED (PWR_LED, P1.04) is active-high: Low = OFF, High = ON
    let red_led = Output::new(p.P1_04, Level::Low, OutputDrive::Standard);

    let telemetry = TelemetryPins { blue_led, red_led };

    // Passive buzzer on P0.06. The app toggles it for tones.
    let buzzer = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);

    // --- External QSPI Flash (MX25R1635F) ---
    let qspi = ferrix_nrf52::qspi::QspiPins {
        qspi: p.QSPI,
        sck: p.P1_14,
        csn: p.P1_15,
        io0: p.P1_12,
        io1: p.P1_13,
        io2: p.P0_07,
        io3: p.P0_05,
    };

    Pins {
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
        usbd: p.USBD,
        qspi,
        wdt: p.WDT,
        buzzer,
    }
}
