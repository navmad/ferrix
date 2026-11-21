//! ThinkNode M1 board support package.
//!
//! This crate wires the nRF52840 peripherals, the SX1262 radio, the SSD1681
//! panel, the L76K GNSS and the companion transports together. It implements
//! the role-facing [`CompanionBoard`] trait; the firmware image in
//! `firmware/thinknode-m1-companion` only chooses the role.

#![no_std]
#![deny(unsafe_code)]

pub mod app;
pub mod display;
pub mod gps;
pub mod host;
pub mod pins;
pub mod radio;
pub mod storage;

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
pub use ferrix_thinknode_m1_logic::config::NAME;
/// Whether this board has a display.
pub const HAS_DISPLAY: bool = true;
/// Whether this board has a GNSS receiver.
pub const HAS_GPS: bool = true;
/// Whether this board has a battery ADC.
pub const HAS_BATTERY_ADC: bool = true;
/// Protocol capability bits.
pub const CAPABILITIES: u32 = CAP_CLIENT | CAP_GPS | CAP_EPD | CAP_BLE;

/// Board startup configuration for ThinkNode M1.
///
/// ThinkNode M1 populates the `REG1` (`VDD -> DEC4`) buck inductor (while
/// `REG0` stays in normal/LDO mode) and a `32.768 kHz` LFXO crystal (`XL1`/`XL2`).
pub const STARTUP_CONFIG: ferrix_nrf52::startup::StartupConfig =
    ferrix_nrf52::startup::StartupConfig {
        dcdc_reg1: true,
        lfclk: ferrix_nrf52::startup::Lfclk::ExternalXtal(
            ferrix_nrf52::startup::LfclkAccuracy::Ppm50,
        ),
    };

pub use ferrix_nrf52::seed::hardware_seed;

static EPD_FRAMEBUFFER: StaticCell<EpdFrameBuffer> = StaticCell::new();
static EPD_PREV_FRAMEBUFFER: StaticCell<EpdFrameBuffer> = StaticCell::new();

// ---------------------------------------------------------------------------
// Battery telemetry
// ---------------------------------------------------------------------------

pub use ferrix_nrf52::battery::{
    battery_millivolts, battery_percentage, set_battery_millivolts, MIN_PLAUSIBLE_BATTERY_MV,
};

pub use ferrix_thinknode_m1_logic::config::BATTERY_ADC_SCALE;

/// Averaged battery sample in millivolts; see `ferrix_nrf52::battery`.
pub async fn sample_battery_adc_mv(adc: &mut saadc::Saadc<'static, 1>) -> Option<u16> {
    if !HAS_BATTERY_ADC {
        return None;
    }
    ferrix_nrf52::battery::sample_adc_mv(adc, BATTERY_ADC_SCALE).await
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

const BLUE_FAULT_LED: ferrix_nrf52::fault::FaultLed = ferrix_nrf52::fault::FaultLed::p0(13);

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    ferrix_nrf52::fault::blink_pin_forever(
        BLUE_FAULT_LED,
        ferrix_thinknode_m1_logic::config::PANIC_BLINKS,
    )
}

#[allow(unsafe_code)]
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    ferrix_nrf52::fault::blink_pin_forever(
        BLUE_FAULT_LED,
        ferrix_thinknode_m1_logic::config::HARDFAULT_BLINKS,
    )
}

/// Persistent state the role needs across the startup phases.
pub struct ThinkNodeBoard {
    pins: Option<Pins>,
    _power_en: Option<Output<'static>>,
    telemetry: Option<TelemetryPins>,
    battery_adc: Option<saadc::Saadc<'static, 1>>,
    display: Option<ThinkNodeDisplay>,
    app_fb: Option<&'static mut EpdFrameBuffer>,
    backlight: Option<Output<'static>>,
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
        if !ferrix_core::command::register_push_sink(&ferrix_nrf52::usb::USB_PUSH_SINK)
            || !ferrix_core::command::register_push_sink(&ferrix_nrf52::ble::BLE_PUSH_SINK)
        {
            defmt::warn!("push transport table is full or duplicated");
        }
        Self {
            pins: Some(crate::pins::init(p)),
            _power_en: None,
            telemetry: None,
            battery_adc: None,
            display: None,
            app_fb: None,
            backlight: None,
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
            None,
            Delay,
            PanelConfig::THINKNODE_M1,
        );
        let fb = EPD_FRAMEBUFFER.init(EpdFrameBuffer::new());
        let prev_fb = EPD_PREV_FRAMEBUFFER.init(EpdFrameBuffer::new());
        self.display = Some(ThinkNodeDisplay::new(epd, fb, prev_fb, pins.backlight));
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
        ferrix_thinknode_m1_logic::config::USB_IDS
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

    #[inline(never)]
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
            nvmc,
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

        // Take an averaged sample before the companion/display start so a
        // freshly booted device never reports 0 mV or single-sample ADC noise.
        if let Some(mv) = sample_battery_adc_mv(&mut battery_adc).await {
            set_battery_millivolts(mv);
        }

        let (node_name, custom_key) = {
            // Safe NVMC reader lives only during boot, before SoftDevice takes
            // ownership of internal flash.
            let flash_reader = ferrix_nrf52::flash::NvmcReader::new(nvmc);
            crate::storage::capture_softdevice_vectors(&flash_reader);
            let (node_name, custom_key) = crate::storage::init_config(&flash_reader);
            // A node without an imported key gets a random identity on first
            // boot, persisted immediately. Factory reset therefore rotates it.
            let custom_key = match custom_key {
                Some(key) => Some(key),
                None => {
                    let identity = meshcore_wire::LocalIdentity::from_seed(
                        node_name.as_str(),
                        &ferrix_nrf52::seed::random_seed(),
                    );
                    let key = identity.private_key_bytes();
                    crate::storage::set_custom_priv_key(key);
                    crate::storage::request_save_config();
                    Some(key)
                }
            };
            crate::storage::load(&flash_reader);
            (node_name, custom_key)
        };

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

    #[inline(never)]
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

        ferrix_nrf52::storage::set_qspi(self.qspi.take());
        ferrix_nrf52::ble::init_ble_companion(
            spawner,
            public_key,
            private_key,
            self.name(),
            self.capabilities(),
            self.ble_model(),
            self.cli_host(),
            Some(BLUE_FAULT_LED),
            crate::storage::spawn_storage,
        );
    }

    #[inline(never)]
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
            display.sleep().await.ok();
            defmt::info!("Dashboard rendered successfully!");
        }
    }

    #[inline(never)]
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

        if let Some(buzzer) = self.buzzer.take() {
            spawner.spawn(crate::app::buzzer_task(buzzer)).unwrap();
        }
        if let Some(display) = self.display.take() {
            let (panel, fb, backlight) = display.into_parts();
            spawner
                .spawn(crate::display::display_task(panel))
                .unwrap();
            self.app_fb = Some(fb);
            self.backlight = Some(backlight);
        }

        self.radio = Some(ChannelRadio::new());
    }

    fn take_app(&mut self) -> Self::App {
        let telemetry = self.telemetry.take().expect("telemetry already taken");
        ThinkNodeApp::new(
            telemetry.blue_led,
            self.app_fb.take(),
            self.backlight.take(),
            self.battery_adc.take().expect("battery ADC already taken"),
        )
    }

    fn take_radio(&mut self) -> Self::Radio {
        self.radio.take().expect("radio already taken")
    }
}
