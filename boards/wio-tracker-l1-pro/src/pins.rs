//! Wio Tracker L1 Pro pinout and peripheral construction.
//!
//! This module only describes the nRF52840 pins and the six-switch input
//! cluster. It does not own board identity, power telemetry or startup logic.
//!
//! Pin numbers come from MeshCore's `variants/wio-tracker-l1` (Arduino pin
//! indexes resolved through its `variant.cpp` port map).

use embassy_futures::select::select_array;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin as _, Pull};
use embassy_nrf::peripherals;
use embassy_time::Timer;
use ferrix_l76k::NoPin;
use ferrix_nrf52::gnss::GnssPins;
use ferrix_runtime::app::{InputEvent, INPUT_EVENT_CHANNEL};

embassy_nrf::bind_interrupts!(pub(crate) struct BoardIrqs {
    TWISPI0 => embassy_nrf::spim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => embassy_nrf::twim::InterruptHandler<peripherals::TWISPI1>;
    SAADC => embassy_nrf::saadc::InterruptHandler;
});

/// Semtech SX1262 LoRa radio pins.
pub struct LoraPins {
    pub sck: peripherals::P0_30,
    pub mosi: peripherals::P0_28,
    pub miso: peripherals::P0_03,
    pub cs: Output<'static>,
    pub reset: Output<'static>,
    pub busy: Input<'static>,
    pub dio1: Input<'static>,
    /// RF-switch RX enable (SX126X_RXEN). TX is switched by DIO2.
    pub rxen: Output<'static>,
}

/// SH1106 OLED on I2C (TWIM1).
pub struct OledPins {
    pub twim: peripherals::TWISPI1,
    pub sda: peripherals::P0_06,
    pub scl: peripherals::P0_05,
}

/// GNSS wiring: L76K with standby/enable but no reset line.
pub type GpsPins = GnssPins<Output<'static>, NoPin>;

/// The user button and five-way joystick.
pub struct WioInputs {
    pub user: Input<'static>,
    pub up: Input<'static>,
    pub down: Input<'static>,
    pub left: Input<'static>,
    pub right: Input<'static>,
    pub press: Input<'static>,
}

/// Map a switch index (the order used by [`WioInputs::wait_event`]) to the
/// semantic event it produces.
///
/// - joystick left/up: previous page
/// - joystick right/down: next page
/// - joystick press: primary page action (FK1)
/// - user button: secondary page action (FK2)
pub(crate) fn event_for_switch(index: usize) -> InputEvent {
    match index {
        0 => InputEvent::ActionSecondary,
        1 | 3 => InputEvent::PrevPage,
        2 | 4 => InputEvent::NextPage,
        _ => InputEvent::ActionPrimary,
    }
}

impl WioInputs {
    /// Wait for the next debounced switch press.
    pub async fn wait_event(&mut self) -> InputEvent {
        loop {
            let Self {
                user,
                up,
                down,
                left,
                right,
                press,
            } = self;
            let (_, index) = select_array([
                user.wait_for_falling_edge(),
                up.wait_for_falling_edge(),
                down.wait_for_falling_edge(),
                left.wait_for_falling_edge(),
                right.wait_for_falling_edge(),
                press.wait_for_falling_edge(),
            ])
            .await;

            // Let contact bounce settle, then require the switch to still be
            // held so a glitch does not turn into an event.
            Timer::after_millis(40).await;
            let still_low = match index {
                0 => self.user.is_low(),
                1 => self.up.is_low(),
                2 => self.down.is_low(),
                3 => self.left.is_low(),
                4 => self.right.is_low(),
                _ => self.press.is_low(),
            };
            if still_low {
                return event_for_switch(index);
            }
        }
    }
}

/// Background input task: blocks on GPIOTE edges and forwards semantic events.
#[embassy_executor::task]
pub async fn wio_input_task(mut inputs: WioInputs) -> ! {
    loop {
        let event = inputs.wait_event().await;
        let _ = INPUT_EVENT_CHANNEL.send(event).await;
    }
}

/// Combined board peripherals for the Wio Tracker L1 Pro.
pub struct Pins {
    pub lora: LoraPins,
    pub oled: OledPins,
    pub gps: GpsPins,
    pub inputs: WioInputs,
    /// Status / TX LED (P1.01, active-high).
    pub led: Output<'static>,
    /// Battery-divider enable (P0.04), held high.
    pub vbat_en: Output<'static>,
    pub spim0: peripherals::TWISPI0,
    pub uarte0: peripherals::UARTE0,
    pub saadc: peripherals::SAADC,
    pub bat_adc_pin: peripherals::P0_31,
    pub usbd: peripherals::USBD,
    pub qspi: ferrix_nrf52::qspi::QspiPins,
    pub wdt: peripherals::WDT,
    /// Passive buzzer on P1.00, driven as a GPIO square wave.
    pub buzzer: Output<'static>,
}

/// Initialize the Wio Tracker L1 Pro pinout and peripherals.
pub fn init(p: embassy_nrf::Peripherals) -> Pins {
    // --- LoRa SX1262 ---
    let lora = LoraPins {
        sck: p.P0_30,
        mosi: p.P0_28,
        miso: p.P0_03,
        cs: Output::new(p.P1_14, Level::High, OutputDrive::Standard),
        reset: Output::new(p.P1_07, Level::High, OutputDrive::Standard),
        busy: Input::new(p.P1_10, Pull::None),
        dio1: Input::new(p.P0_07, Pull::None),
        rxen: Output::new(p.P1_08, Level::Low, OutputDrive::Standard),
    };

    // --- OLED (SH1106, I2C) ---
    let oled = OledPins {
        twim: p.TWISPI1,
        sda: p.P0_06,
        scl: p.P0_05,
    };

    // --- GPS (L76K). Held off until the dashboard enables it. ---
    // MCU RX is P0.26 (Serial1 RX), MCU TX is P0.27. P1.09 is standby/enable.
    let gps = GnssPins {
        rx: p.P0_26.degrade(),
        tx: p.P0_27.degrade(),
        standby: Output::new(p.P1_09, Level::Low, OutputDrive::Standard),
        reset: NoPin,
        timer1: p.TIMER1,
        ppi_ch0: p.PPI_CH0,
        ppi_ch1: p.PPI_CH1,
    };

    // --- Buttons: all active-low with pull-ups ---
    let inputs = WioInputs {
        user: Input::new(p.P0_08, Pull::Up),
        up: Input::new(p.P1_04, Pull::Up),
        down: Input::new(p.P0_12, Pull::Up),
        left: Input::new(p.P0_11, Pull::Up),
        right: Input::new(p.P1_03, Pull::Up),
        press: Input::new(p.P1_05, Pull::Up),
    };

    // --- LED, battery enable, buzzer ---
    let led = Output::new(p.P1_01, Level::Low, OutputDrive::Standard);
    let vbat_en = Output::new(p.P0_04, Level::High, OutputDrive::Standard);
    let buzzer = Output::new(p.P1_00, Level::Low, OutputDrive::Standard);

    // --- External QSPI flash (P25Q16H) ---
    let qspi = ferrix_nrf52::qspi::QspiPins {
        qspi: p.QSPI,
        sck: p.P0_21.degrade(),
        csn: p.P0_25.degrade(),
        io0: p.P0_20.degrade(),
        io1: p.P0_24.degrade(),
        io2: p.P0_22.degrade(),
        io3: p.P0_23.degrade(),
    };

    Pins {
        lora,
        oled,
        gps,
        inputs,
        led,
        vbat_en,
        spim0: p.TWISPI0,
        uarte0: p.UARTE0,
        saadc: p.SAADC,
        bat_adc_pin: p.P0_31,
        usbd: p.USBD,
        qspi,
        wdt: p.WDT,
        buzzer,
    }
}
