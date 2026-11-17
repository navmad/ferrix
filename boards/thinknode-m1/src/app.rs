//! ThinkNode M1 implementation of the runtime application board interface.

use defmt::info;
use embassy_futures::select::{select, Either};
use embassy_nrf::gpio::Output;
use embassy_nrf::saadc;
use embassy_time::{Duration, Instant, Timer};
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_nrf52::buzzer::PassiveBuzzer;
use ferrix_runtime::app::{AppBoard, UiSound, UiState};

use crate::display::ThinkNodeDisplay;
use crate::gps;

/// Seconds the display frontlight stays on after the last button press.
const BACKLIGHT_IDLE_SECS: u64 = 15;
/// 2 s ticks between idle EPD fast refreshes (300 * 2 s = 10 minutes).
const IDLE_REFRESH_TICKS: u32 = 300;

/// Board-side devices used by the application loop.
pub struct ThinkNodeApp {
    led: Output<'static>,
    display: Option<ThinkNodeDisplay>,
    battery_adc: saadc::Saadc<'static, 1>,
    battery_buf: [i16; 1],
    buzzer: PassiveBuzzer<'static>,
    backlight_on: bool,
    backlight_deadline: Instant,
}

impl ThinkNodeApp {
    pub fn new(
        led: Output<'static>,
        display: Option<ThinkNodeDisplay>,
        battery_adc: saadc::Saadc<'static, 1>,
        buzzer: PassiveBuzzer<'static>,
    ) -> Self {
        Self {
            led,
            display,
            battery_adc,
            battery_buf: [0i16; 1],
            buzzer,
            backlight_on: false,
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
        crate::HAS_BATTERY_ADC
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
            if !self.backlight_on {
                d.sleep().await.ok();
            }
        }
    }

    async fn input_wake(&mut self) {
        self.backlight_on = true;
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
                d.sleep().await.ok();
            }
            self.backlight_on = false;
        }

        if tick % IDLE_REFRESH_TICKS == 0 {
            self.render(node, ui, false).await;
        }
    }

    async fn sample_battery(&mut self) -> Option<u16> {
        if !crate::HAS_BATTERY_ADC {
            return None;
        }
        match select(
            self.battery_adc.sample(&mut self.battery_buf),
            Timer::after_millis(100),
        )
        .await
        {
            Either::First(_) => {}
            Either::Second(_) => return None,
        }
        let raw = self.battery_buf[0].max(0) as u16;
        let mv = crate::adc_raw_to_battery_millivolts(raw);
        if mv >= crate::MIN_PLAUSIBLE_BATTERY_MV {
            crate::set_battery_millivolts(mv);
            Some(mv)
        } else {
            None
        }
    }

    fn set_leds_disabled(&mut self, disabled: bool) {
        crate::host::set_leds_disabled(disabled);
    }

    async fn beep(&mut self, sound: UiSound) {
        if !ferrix_core::state::buzzer_enabled() {
            return;
        }
        match sound {
            UiSound::Click => self.buzzer.beep(2_200, 15).await,
            UiSound::Action => self.buzzer.beep(1_800, 35).await,
            UiSound::Notify => {
                self.buzzer.melody(&[(2_700, 70), (3_200, 90)], 50).await;
            }
        }
    }
}
