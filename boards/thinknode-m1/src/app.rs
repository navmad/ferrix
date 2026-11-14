//! ThinkNode M1 implementation of the runtime application board interface.

use defmt::info;
use embassy_nrf::gpio::Output;
use embassy_nrf::saadc;
use embassy_nrf::peripherals::TWISPI1;
use embassy_time::{Duration, Instant, Timer};
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_runtime::app::{AppBoard, UiState};

use crate::display::{NodeDisplay, ThinkNodeDisplay};
use crate::gps;

/// Seconds the display frontlight stays on after the last button press.
const BACKLIGHT_IDLE_SECS: u64 = 15;

/// Board-side devices used by the application loop.
pub struct ThinkNodeApp {
    led: Output<'static>,
    display: Option<ThinkNodeDisplay<'static, TWISPI1>>,
    battery_adc: saadc::Saadc<'static, 1>,
    battery_buf: [i16; 1],
    backlight_on: bool,
    epd_active: bool,
    backlight_deadline: Instant,
}

impl ThinkNodeApp {
    pub fn new(
        led: Output<'static>,
        display: Option<ThinkNodeDisplay<'static, TWISPI1>>,
        battery_adc: saadc::Saadc<'static, 1>,
    ) -> Self {
        Self {
            led,
            display,
            battery_adc,
            battery_buf: [0i16; 1],
            backlight_on: true,
            epd_active: true,
            backlight_deadline: Instant::now()
                .checked_add(Duration::from_secs(BACKLIGHT_IDLE_SECS))
                .unwrap(),
        }
    }
}

/// Drive the blue status LED unless the user disabled LEDs in the UI.
pub fn set_blue_led(led: &mut Output<'static>, on: bool) {
    if on && !ferrix_core::state::leds_disabled() {
        led.set_high();
    } else {
        led.set_low();
    }
}

async fn advert_feedback(led: &mut Output<'static>) {
    for _ in 0..2 {
        set_blue_led(led, true);
        Timer::after_millis(40).await;
        set_blue_led(led, false);
        Timer::after_millis(40).await;
    }
}

impl AppBoard for ThinkNodeApp {
    fn page_count(&self) -> u8 {
        crate::display::ui::PAGE_COUNT
    }

    fn has_display(&self) -> bool {
        self.display.is_some()
    }

    fn has_battery_adc(&self) -> bool {
        crate::board::HAS_BATTERY_ADC
    }

    fn gps_enabled(&self) -> bool {
        gps::gps_enabled()
    }

    fn set_gps_enabled(&mut self, on: bool) {
        gps::set_gps_enabled(on);
        info!("Input: GPS power {}", if on { "on" } else { "off" });
    }

    fn set_led(&mut self, on: bool) {
        set_blue_led(&mut self.led, on);
    }

    async fn advert_feedback(&mut self) {
        advert_feedback(&mut self.led).await;
    }

    async fn render(&mut self, node: &CompanionNode<'_>, ui: &UiState, full: bool) {
        if let Some(d) = &mut self.display {
            if full {
                d.render_dashboard_full(node, ui).await.ok();
            } else {
                d.render_dashboard(node, ui).await.ok();
            }
        }
    }

    async fn input_wake(&mut self) {
        self.backlight_on = true;
        self.epd_active = true;
        self.backlight_deadline = Instant::now()
            .checked_add(Duration::from_secs(BACKLIGHT_IDLE_SECS))
            .unwrap();
        if let Some(d) = &mut self.display {
            d.set_backlight(true);
        }
    }

    async fn tick(&mut self, node: &CompanionNode<'_>, ui: &UiState, tick: u32) {
        if self.backlight_on && Instant::now() >= self.backlight_deadline {
            if let Some(d) = &mut self.display {
                d.set_backlight(false);
            }
            self.backlight_on = false;
            self.epd_active = false;
        }

        if tick % 30 == 0 && self.epd_active {
            self.render(node, ui, false).await;
        }
    }

    async fn sample_battery(&mut self) -> Option<u16> {
        if !crate::board::HAS_BATTERY_ADC {
            return None;
        }
        self.battery_adc.sample(&mut self.battery_buf).await;
        let raw = self.battery_buf[0].max(0) as u16;
        let mv = crate::telemetry::adc_raw_to_battery_millivolts(raw);
        if mv >= crate::telemetry::MIN_PLAUSIBLE_BATTERY_MV {
            crate::telemetry::set_battery_millivolts(mv);
            Some(mv)
        } else {
            None
        }
    }

    fn set_leds_disabled(&mut self, disabled: bool) {
        crate::companion::usb::set_leds_disabled(disabled);
    }
}
