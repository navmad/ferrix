//! ThinkNode M1 board support package.
//!
//! This crate wires the nRF52840 peripherals, the SX1262 radio, the SSD1681
//! panel, the L76K GNSS and the companion transports together. It implements
//! the role-facing [`CompanionBoard`] trait; the firmware image in
//! `firmware/thinknode-m1-companion` only chooses the role.

#![no_std]

pub mod app;
pub mod display;
pub mod gps;
pub mod host;
pub mod pins;
pub mod radio;
pub mod storage;

use core::sync::atomic::{AtomicU16, Ordering};

use defmt_rtt as _;

use embassy_executor::Spawner;
use embassy_nrf::gpio::Output;
use embassy_nrf::peripherals;
use embassy_nrf::saadc;
use embassy_nrf::spim::{self, Spim};
use embassy_time::{Delay, Duration, Timer};
use ferrix_core::companion::cli::CliHost;
use ferrix_core::companion::protocol::{
    CompanionHost, CompanionNode, CAP_BLE, CAP_CLIENT, CAP_EPD, CAP_GPS,
};
use ferrix_core::engine::EngineContext;
use ferrix_nrf52::buzzer::PassiveBuzzer;
use ferrix_role_companion::CompanionBoard;
use ferrix_runtime::radio::ChannelRadio;
use heapless::String;
use static_cell::StaticCell;

use crate::app::ThinkNodeApp;
use crate::display::{EpdFrameBuffer, PanelConfig, Ssd1681, ThinkNodeDisplay};
use crate::host::BOARD_ENGINE_CONTEXT;
use crate::host::BOARD_HOST;
use crate::pins::{BoardIrqs, DisplayPins, LoraPins, Pins, TelemetryPins};
use crate::radio::sx1262_task;

/// Hardware model reported to MeshCore companion apps.
pub const NAME: &str = "ThinkNode M1";
/// Whether this board has a display.
pub const HAS_DISPLAY: bool = true;
/// Whether this board has a GNSS receiver.
pub const HAS_GPS: bool = true;
/// Whether this board has a battery ADC.
pub const HAS_BATTERY_ADC: bool = true;
/// Protocol capability bits.
pub const CAPABILITIES: u32 = CAP_CLIENT | CAP_GPS | CAP_EPD | CAP_BLE;

pub use ferrix_nrf52::seed::hardware_seed;

static EPD_FRAMEBUFFER: StaticCell<EpdFrameBuffer> = StaticCell::new();

// ---------------------------------------------------------------------------
// Battery telemetry
// ---------------------------------------------------------------------------

/// Latest measured battery voltage. 0 means "not sampled / no battery".
static BATTERY_MV: AtomicU16 = AtomicU16::new(0);

pub use ferrix_core::battery::MIN_PLAUSIBLE_BATTERY_MV;

pub fn set_battery_millivolts(mv: u16) {
    if mv >= MIN_PLAUSIBLE_BATTERY_MV {
        BATTERY_MV.store(mv, Ordering::Relaxed);
    }
}

pub fn battery_millivolts() -> u16 {
    BATTERY_MV.load(Ordering::Relaxed)
}

pub fn battery_percentage() -> u8 {
    ferrix_core::battery::millivolts_to_percentage(battery_millivolts())
}

/// ThinkNode M1 battery ADC: AIN2 (P0.04), 150K+150K divider (2:1), SAADC
/// gain 1/6 with the internal 0.6 V reference.
///
/// `7236 / 4096` matches the board's `vbat-mv-multiplier` (2:1 divider,
/// 3.6 V full scale, +0.5% SAADC gain trim).
pub fn adc_raw_to_battery_millivolts(raw: u16) -> u16 {
    let mv = (raw as u32 * 7236) / 4096;
    mv.min(u16::MAX as u32) as u16
}

// ---------------------------------------------------------------------------
// Fatal-error indicators
//
// The red LED (P1.04) on the ThinkNode M1 is driven by the charger IC, so it
// cannot be used as a firmware status. Use the MCU-controlled blue LED (P0.13)
// instead. Patterns (all on blue):
//   - normal boot:  3 fast blinks, then the slow main-loop heartbeat
//   - panic:        3 quick blinks, repeated forever
//   - HardFault:    6 quick blinks, repeated forever
//   - SoftDevice:  10 quick blinks, repeated forever
// ---------------------------------------------------------------------------

const P0_BASE: u32 = 0x5000_0000;
const P0_PIN_CNF13: *mut u32 = (P0_BASE + 0x504 + 4 * 13) as *mut u32;
const P0_OUTSET: *mut u32 = (P0_BASE + 0x508) as *mut u32;
const P0_OUTCLR: *mut u32 = (P0_BASE + 0x50C) as *mut u32;
const P0_DIRSET: *mut u32 = (P0_BASE + 0x518) as *mut u32;
const BLUE_LED: u32 = 1 << 13;

/// ~35 ms at 64 MHz. Used instead of a timer so it still works when the
/// scheduler, the SoftDevice or interrupts are wedged.
#[inline(never)]
fn fatal_delay() {
    cortex_m::asm::delay(2_000_000);
}

fn blue_on() {
    unsafe { P0_OUTSET.write_volatile(BLUE_LED) }
}

fn blue_off() {
    unsafe { P0_OUTCLR.write_volatile(BLUE_LED) }
}

fn blue_init() {
    unsafe {
        // Dir=output, input buffer disconnected.
        P0_PIN_CNF13.write_volatile(0x0000_0003);
        P0_DIRSET.write_volatile(BLUE_LED);
    }
}

fn fatal_blink_burst(count: u32) {
    blue_init();
    for _ in 0..count {
        blue_on();
        fatal_delay();
        blue_off();
        fatal_delay();
    }
    // Gap between bursts.
    for _ in 0..4 {
        fatal_delay();
    }
}

/// Repeat a burst forever. Used for fatal faults that cannot recover.
pub(crate) fn fatal_blink_forever(count: u32) -> ! {
    loop {
        fatal_blink_burst(count);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    fatal_blink_forever(3)
}

#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    fatal_blink_forever(6)
}

/// Persistent state the role needs across the startup phases.
pub struct ThinkNodeBoard {
    pins: Option<Pins>,
    _power_en: Option<Output<'static>>,
    telemetry: Option<TelemetryPins>,
    battery_adc: Option<saadc::Saadc<'static, 1>>,
    display: Option<ThinkNodeDisplay>,
    buzzer: Option<PassiveBuzzer<'static>>,
    radio: Option<ChannelRadio>,
    spim0: Option<peripherals::TWISPI0>,
    lora_pins: Option<LoraPins>,
    usbd: Option<peripherals::USBD>,
    qspi: Option<ferrix_nrf52::qspi::QspiPins>,
}

impl ThinkNodeBoard {
    /// Take the nRF peripherals and hold the board pin configuration.
    pub fn new(p: embassy_nrf::Peripherals) -> Self {
        Self {
            pins: Some(crate::pins::init(p)),
            _power_en: None,
            telemetry: None,
            battery_adc: None,
            display: None,
            buzzer: None,
            radio: None,
            spim0: None,
            lora_pins: None,
            usbd: None,
            qspi: None,
        }
    }

    fn build_display(&mut self, spim1: peripherals::TWISPI1, pins: DisplayPins) {
        let mut config = spim::Config::default();
        config.frequency = spim::Frequency::M4;
        config.mode = spim::MODE_0;

        let spim1 = Spim::new_txonly(spim1, BoardIrqs, pins.sclk, pins.mosi, config);
        let spi = embedded_hal_bus::spi::ExclusiveDevice::new(spim1, pins.cs, Delay).unwrap();
        let epd = Ssd1681::new(
            spi,
            pins.dc,
            pins.reset,
            pins.busy,
            Some(pins.backlight),
            Delay,
            PanelConfig::THINKNODE_M1,
        );
        let fb = EPD_FRAMEBUFFER.init(EpdFrameBuffer::new());
        self.display = Some(ThinkNodeDisplay::new(epd, fb));
    }
}

impl CompanionBoard for ThinkNodeBoard {
    type App = ThinkNodeApp;
    type Radio = ChannelRadio;

    fn name(&self) -> &'static str {
        NAME
    }

    fn capabilities(&self) -> u32 {
        CAPABILITIES
    }

    fn usb_ids(&self) -> (u16, u16) {
        (0x239A, 0x8029)
    }

    fn usb_manufacturer(&self) -> &'static str {
        "MeshCore"
    }

    fn usb_product(&self) -> &'static str {
        "ThinkNode M1 Companion"
    }

    fn ble_model(&self) -> &'static str {
        NAME
    }

    fn companion_host(&self) -> &'static dyn CompanionHost {
        &BOARD_HOST
    }

    fn cli_host(&self) -> &'static dyn CliHost {
        &BOARD_HOST
    }

    fn engine_context(&self) -> &'static dyn EngineContext {
        &BOARD_ENGINE_CONTEXT
    }

    fn hardware_seed(&self) -> [u8; 32] {
        hardware_seed()
    }

    fn forwarding_enabled(&self) -> bool {
        ferrix_core::state::forwarding_enabled()
    }

    async fn init(&mut self, spawner: Spawner) -> (String<32>, Option<[u8; 64]>) {
        let pins = self.pins.take().expect("board peripherals already taken");
        let Pins {
            lora,
            display: display_pins,
            gps,
            inputs,
            telemetry,
            power_en,
            spim0,
            spim1,
            uarte0,
            saadc,
            bat_adc_pin,
            usbd,
            qspi,
            wdt,
            buzzer: buzzer_pin,
        } = pins;

        self._power_en = Some(power_en);
        self.buzzer = Some(PassiveBuzzer::new(buzzer_pin));

        let wdt_handle = ferrix_nrf52::watchdog::start(wdt);
        spawner
            .spawn(ferrix_nrf52::watchdog::watchdog_task(wdt_handle))
            .unwrap();

        // The GPS task owns the L76K UART and control pins; it holds the module
        // in reset/sleep until the dashboard enables it.
        if let Some(gps) = gps {
            spawner.spawn(crate::gps::gps_task(uarte0, gps)).unwrap();
        }

        spawner
            .spawn(crate::pins::thinknode_input_task(inputs))
            .unwrap();

        // Battery ADC: ThinkNode M1 AIN2 (P0.04), 150K+150K divider (2:1).
        let mut battery_adc = saadc::Saadc::new(
            saadc,
            BoardIrqs,
            saadc::Config::default(),
            [saadc::ChannelConfig::single_ended(bat_adc_pin)],
        );

        // Take one sample before the companion/display start so a freshly
        // booted device never reports 0 mV to the app.
        let mut battery_buf = [0i16; 1];
        if HAS_BATTERY_ADC {
            match embassy_futures::select::select(
                battery_adc.sample(&mut battery_buf),
                Timer::after_millis(100),
            )
            .await
            {
                embassy_futures::select::Either::First(_) => {
                    let raw = battery_buf[0].max(0) as u16;
                    set_battery_millivolts(adc_raw_to_battery_millivolts(raw));
                }
                embassy_futures::select::Either::Second(_) => {}
            }
        }

        let (node_name, custom_key) = crate::storage::init_config();
        crate::storage::load();

        if let Some(disp) = display_pins {
            self.build_display(spim1, disp);
        }

        self.telemetry = Some(telemetry);
        self.battery_adc = Some(battery_adc);
        self.spim0 = Some(spim0);
        self.lora_pins = Some(lora);
        self.usbd = Some(usbd);
        self.qspi = Some(qspi);

        (node_name, custom_key)
    }

    fn publish_identity(&mut self, public_key: [u8; 32], private_key: [u8; 64]) {
        ferrix_core::state::set_identity(public_key, private_key);
    }

    fn start_transports(
        &mut self,
        spawner: Spawner,
        public_key: [u8; 32],
        private_key: [u8; 64],
    ) {
        let usbd = self.usbd.take().expect("USB peripheral already taken");
        let usb_driver = ferrix_nrf52::usb::driver(usbd);
        let (vid, pid) = self.usb_ids();
        let (usb_dev, cdc_class) = ferrix_nrf52::usb::init_usb(
            usb_driver,
            vid,
            pid,
            self.usb_manufacturer(),
            self.usb_product(),
        );

        spawner
            .spawn(ferrix_nrf52::usb::usb_task(usb_dev))
            .unwrap();
        spawner
            .spawn(ferrix_nrf52::usb::cdc_acm_task(
                cdc_class,
                public_key,
                private_key,
                self.name(),
                self.capabilities(),
                self.cli_host(),
            ))
            .unwrap();
        spawner
            .spawn(ferrix_nrf52::ble::contact_task())
            .unwrap();

        ferrix_nrf52::ble::init_ble_companion(
            spawner,
            public_key,
            private_key,
            self.name(),
            self.capabilities(),
            self.ble_model(),
            self.cli_host(),
            self.qspi.take().expect("QSPI pins already taken"),
            crate::fatal_blink_forever,
            crate::storage::spawn_storage,
        );
    }

    async fn start_ui(&mut self, node: &CompanionNode<'_>) {
        if ferrix_core::state::buzzer_enabled() {
            if let Some(buzzer) = &mut self.buzzer {
                buzzer.melody(&[(1_800, 70), (2_400, 90)], 30).await;
            }
        }
        if let Some(display) = &mut self.display {
            defmt::info!("Initializing display and drawing Ferrix boot splash...");
            display.init().await.ok();
            display.render_boot(node).await.ok();
            Timer::after(Duration::from_secs(2)).await;
            // Presses during the splash are intentionally ignored.
            while ferrix_runtime::app::INPUT_EVENT_CHANNEL.try_receive().is_ok() {}
            display
                .render_dashboard(node, &ferrix_runtime::app::UiState::default())
                .await
                .ok();
            defmt::info!("Dashboard rendered successfully!");
        }
    }

    fn start_radio(&mut self, spawner: Spawner) {
        let spim0 = self.spim0.take().expect("SPIM0 already taken");
        let lora = self.lora_pins.take().expect("LoRa pins already taken");

        let mut config = spim::Config::default();
        config.frequency = spim::Frequency::M8;
        config.mode = spim::MODE_0;

        let spim0 = Spim::new(
            spim0,
            BoardIrqs,
            lora.sck,
            lora.miso,
            lora.mosi,
            config,
        );

        spawner
            .spawn(sx1262_task(
                spim0,
                lora.cs,
                lora.reset,
                lora.busy,
                lora.dio1,
                lora.rf_sw,
            ))
            .unwrap();

        self.radio = Some(ChannelRadio::new());
    }

    fn take_app(&mut self) -> Self::App {
        let telemetry = self.telemetry.take().expect("telemetry already taken");
        ThinkNodeApp::new(
            telemetry.blue_led,
            self.display.take(),
            self.battery_adc.take().expect("battery ADC already taken"),
            self.buzzer.take().expect("buzzer already taken"),
        )
    }

    fn take_radio(&mut self) -> Self::Radio {
        self.radio.take().expect("radio already taken")
    }
}

#[cfg(test)]
mod battery_tests {
    use super::*;

    #[test]
    fn adc_raw_to_battery_millivolts() {
        // Full-scale 3.6 V at the pin, 2:1 divider -> ~7.2 V battery.
        assert_eq!(adc_raw_to_battery_millivolts(4095), 7234);
        assert_eq!(adc_raw_to_battery_millivolts(0), 0);
        // ~4.2 V battery -> ~2377 counts.
        assert_eq!(adc_raw_to_battery_millivolts(2377), 4199);
        // 3.7 V battery.
        assert_eq!(adc_raw_to_battery_millivolts(2094), 3699);
    }
}
