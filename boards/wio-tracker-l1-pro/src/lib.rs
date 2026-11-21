//! Seeed Wio Tracker L1 Pro board support package.
//!
//! This crate wires the nRF52840 peripherals, the SX1262 radio, the SH1106
//! OLED, the L76K GNSS and the companion transports together. It implements
//! the role-facing [`CompanionBoard`] trait; the firmware image in
//! `firmware/wio-tracker-l1-pro-companion` only chooses the role.

#![no_std]

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
use embassy_nrf::twim::{self, Twim};
use embassy_time::{Duration, Timer};
use ferrix_core::companion::cli::CliHost;
use ferrix_core::companion::protocol::{CompanionHost, CompanionNode, CAP_BLE, CAP_CLIENT, CAP_GPS};
use ferrix_core::engine::EngineContext;
use ferrix_nrf52::buzzer::PassiveBuzzer;
use ferrix_nrf52::fatal::{FatalLed, Port};
use ferrix_role_companion::CompanionBoard;
use ferrix_runtime::radio::ChannelRadio;
use ferrix_sh1106::{Framebuffer, Sh1106};
use heapless::String;
use static_cell::StaticCell;

use crate::app::WioApp;
use crate::display::WioDisplay;
use crate::host::{BOARD_ENGINE_CONTEXT, BOARD_HOST};
use crate::pins::{BoardIrqs, LoraPins, OledPins, Pins};
use crate::radio::sx1262_task;

/// Hardware model reported to MeshCore companion apps.
pub const NAME: &str = "Wio Tracker L1 Pro";
/// Whether this board has a battery ADC.
pub const HAS_BATTERY_ADC: bool = true;
/// Protocol capability bits. The OLED has no dedicated capability bit.
pub const CAPABILITIES: u32 = CAP_CLIENT | CAP_GPS | CAP_BLE;

/// Board startup configuration for the Wio Tracker L1 Pro: DC/DC on REG1 and a
/// 32.768 kHz LFXO crystal (MeshCore builds with `USE_LFXO`).
pub const STARTUP_CONFIG: ferrix_nrf52::startup::StartupConfig =
    ferrix_nrf52::startup::StartupConfig {
        dcdc_reg1: true,
        lfclk: ferrix_nrf52::startup::Lfclk::ExternalXtal(
            ferrix_nrf52::startup::LfclkAccuracy::Ppm20,
        ),
    };

pub use ferrix_nrf52::seed::hardware_seed;

static OLED_FRAMEBUFFER: StaticCell<Framebuffer> = StaticCell::new();

// ---------------------------------------------------------------------------
// Battery telemetry
// ---------------------------------------------------------------------------

pub use ferrix_nrf52::battery::{
    battery_millivolts, battery_percentage, set_battery_millivolts, MIN_PLAUSIBLE_BATTERY_MV,
};

/// Wio Tracker L1 Pro battery ADC: AIN7 (P0.31), 2:1 divider enabled by P0.04,
/// SAADC gain 1/6 with the internal 0.6 V reference (3.6 V full scale).
///
/// `7200 / 4096` = 2 (divider) x 3600 mV, matching MeshCore's
/// `ADC_MULTIPLIER` / `AREF_VOLTAGE`.
pub const BATTERY_ADC_SCALE: u32 = 7200;

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
// The status LED (P1.01) is the only MCU-controlled LED. Patterns:
//   - normal boot:  slow main-loop heartbeat
//   - panic:        3 quick blinks, repeated forever
//   - HardFault:    6 quick blinks, repeated forever
//   - SoftDevice:  10 quick blinks, repeated forever
// ---------------------------------------------------------------------------

const FATAL_LED: FatalLed = FatalLed {
    port: Port::P1,
    pin: 1,
    active_high: true,
};

/// Repeat a burst forever. Used for fatal faults that cannot recover.
pub(crate) fn fatal_blink_forever(count: u32) -> ! {
    FATAL_LED.blink_forever(count)
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
pub struct WioTrackerBoard {
    pins: Option<Pins>,
    _vbat_en: Option<Output<'static>>,
    led: Option<Output<'static>>,
    battery_adc: Option<saadc::Saadc<'static, 1>>,
    display: Option<WioDisplay>,
    app_fb: Option<&'static mut Framebuffer>,
    buzzer: Option<PassiveBuzzer<'static>>,
    radio: Option<ChannelRadio>,
    spim0: Option<peripherals::TWISPI0>,
    lora_pins: Option<LoraPins>,
    usbd: Option<peripherals::USBD>,
    qspi: Option<ferrix_nrf52::qspi::QspiPins>,
}

impl WioTrackerBoard {
    /// Take the nRF peripherals and hold the board pin configuration.
    pub fn new(p: embassy_nrf::Peripherals) -> Self {
        if !ferrix_core::command::register_push_sink(&ferrix_nrf52::usb::USB_PUSH_SINK)
            || !ferrix_core::command::register_push_sink(&ferrix_nrf52::ble::BLE_PUSH_SINK)
        {
            defmt::warn!("push transport table is full or duplicated");
        }
        Self {
            pins: Some(crate::pins::init(p)),
            _vbat_en: None,
            led: None,
            battery_adc: None,
            display: None,
            app_fb: None,
            buzzer: None,
            radio: None,
            spim0: None,
            lora_pins: None,
            usbd: None,
            qspi: None,
        }
    }

    fn build_display(&mut self, oled: OledPins) {
        let mut config = twim::Config::default();
        config.frequency = twim::Frequency::K400;

        let i2c = Twim::new(oled.twim, BoardIrqs, oled.sda, oled.scl, config);
        let fb = OLED_FRAMEBUFFER.init(Framebuffer::new());
        let panel = Sh1106::new(i2c, Sh1106::<Twim<'static, peripherals::TWISPI1>>::ADDRESS_WIO_TRACKER_L1);
        self.display = Some(WioDisplay::new(panel, fb));
    }
}

impl CompanionBoard for WioTrackerBoard {
    type App = WioApp;
    type Radio = ChannelRadio;

    fn name(&self) -> &'static str {
        NAME
    }

    fn capabilities(&self) -> u32 {
        CAPABILITIES
    }

    /// Seeed VID with the application PID from MeshCore's board definition
    /// (`hwids[1]`; `0x1667` is the bootloader).
    fn usb_ids(&self) -> (u16, u16) {
        (0x2886, 0x1668)
    }

    fn usb_manufacturer(&self) -> &'static str {
        "MeshCore"
    }

    fn usb_product(&self) -> &'static str {
        "Wio Tracker L1 Pro Companion"
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
            oled,
            gps,
            inputs,
            led,
            vbat_en,
            spim0,
            uarte0,
            saadc,
            bat_adc_pin,
            usbd,
            qspi,
            wdt,
            buzzer: buzzer_pin,
        } = pins;

        self._vbat_en = Some(vbat_en);
        self.buzzer = Some(PassiveBuzzer::new(buzzer_pin));

        let wdt_handle = ferrix_nrf52::watchdog::start(wdt);
        spawner
            .spawn(ferrix_nrf52::watchdog::watchdog_task(wdt_handle))
            .unwrap();

        // The GPS task owns the L76K UART and control pins; it holds the module
        // off until the dashboard enables it.
        spawner.spawn(crate::gps::gps_task(uarte0, gps)).unwrap();

        spawner
            .spawn(crate::pins::wio_input_task(inputs))
            .unwrap();

        // Battery ADC: AIN7 (P0.31) behind the 2:1 divider enabled by P0.04.
        let mut battery_adc = saadc::Saadc::new(
            saadc,
            BoardIrqs,
            saadc::Config::default(),
            [saadc::ChannelConfig::single_ended(bat_adc_pin)],
        );

        // Let the divider settle, then take an averaged sample before the
        // companion/display start so a freshly booted device never reports
        // 0 mV or single-sample ADC noise.
        Timer::after_millis(10).await;
        if let Some(mv) = sample_battery_adc_mv(&mut battery_adc).await {
            set_battery_millivolts(mv);
        }

        let (node_name, custom_key) = crate::storage::init_config();
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
        crate::storage::load();

        self.build_display(oled);

        self.led = Some(led);
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

        ferrix_nrf52::storage::set_qspi(self.qspi.take());
        ferrix_nrf52::ble::init_ble_companion(
            spawner,
            public_key,
            private_key,
            self.name(),
            self.capabilities(),
            self.ble_model(),
            self.cli_host(),
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
            defmt::info!("Initializing OLED and drawing Ferrix boot splash...");
            if display.init().await.is_err() {
                defmt::warn!("OLED init failed (check I2C wiring/address)");
            }
            display.render_boot(node).await.ok();
            Timer::after(Duration::from_secs(2)).await;
            // Presses during the splash are intentionally ignored.
            while ferrix_runtime::app::INPUT_EVENT_CHANNEL.try_receive().is_ok() {}
            display
                .render_dashboard(node, &ferrix_runtime::app::UiState::default())
                .await
                .ok();
        }
    }

    fn start_radio(&mut self, spawner: Spawner) {
        let spim0 = self.spim0.take().expect("SPIM0 already taken");
        let lora = self.lora_pins.take().expect("LoRa pins already taken");

        let mut config = spim::Config::default();
        config.frequency = spim::Frequency::M8;
        config.mode = spim::MODE_0;

        let spim0 = Spim::new(spim0, BoardIrqs, lora.sck, lora.miso, lora.mosi, config);

        spawner
            .spawn(sx1262_task(
                spim0,
                lora.cs,
                lora.reset,
                lora.busy,
                lora.dio1,
                lora.rxen,
            ))
            .unwrap();

        if let Some(buzzer) = self.buzzer.take() {
            spawner.spawn(crate::app::buzzer_task(buzzer)).unwrap();
        }
        if let Some(display) = self.display.take() {
            let (panel, fb) = display.into_parts();
            spawner
                .spawn(crate::display::display_task(panel))
                .unwrap();
            self.app_fb = Some(fb);
        }

        self.radio = Some(ChannelRadio::new());
    }

    fn take_app(&mut self) -> Self::App {
        WioApp::new(
            self.led.take().expect("LED already taken"),
            self.app_fb.take(),
            self.battery_adc.take().expect("battery ADC already taken"),
        )
    }

    fn take_radio(&mut self) -> Self::Radio {
        self.radio.take().expect("radio already taken")
    }
}
