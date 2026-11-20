//! Wio Tracker L1 implementation of the runtime application board interface.

use defmt::info;
use embassy_nrf::gpio::Output;
use embassy_nrf::saadc;
use embassy_time::{Duration, Instant, Timer};
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_nrf52::buzzer::PassiveBuzzer;
use ferrix_runtime::app::{AppBoard, UiSound, UiState};

use crate::display::WioDisplay;
use crate::gps;

/// Seconds the OLED stays on after the last button press.
const DISPLAY_IDLE_SECS: u64 = 30;

/// Board-side devices used by the application loop.
pub struct WioApp {
    led: Output<'static>,
    display: Option<WioDisplay>,
    battery_adc: saadc::Saadc<'static, 1>,
    battery_filter: ferrix_core::battery::BatteryMvFilter,
    buzzer: PassiveBuzzer<'static>,
    display_on: bool,
    display_deadline: Instant,
    last_periodic_refresh: Instant,
}

impl WioApp {
    pub fn new(
        led: Output<'static>,
        display: Option<WioDisplay>,
        battery_adc: saadc::Saadc<'static, 1>,
        buzzer: PassiveBuzzer<'static>,
    ) -> Self {
        let now = Instant::now();
        let mut battery_filter = ferrix_core::battery::BatteryMvFilter::new();
        let initial_mv = crate::battery_millivolts();
        if initial_mv >= crate::MIN_PLAUSIBLE_BATTERY_MV {
            battery_filter.update(initial_mv);
        }
        Self {
            led,
            display,
            battery_adc,
            battery_filter,
            buzzer,
            display_on: true,
            display_deadline: now
                .checked_add(Duration::from_secs(DISPLAY_IDLE_SECS))
                .unwrap(),
            last_periodic_refresh: now,
        }
    }
}

/// Drive the status LED unless the user disabled LEDs in the UI.
pub fn set_status_led(led: &mut Output<'static>, on: bool) {
    if on && !ferrix_core::state::leds_disabled() {
        led.set_high();
    } else {
        led.set_low();
    }
}

async fn advert_feedback(led: &mut Output<'static>) {
    for _ in 0..2 {
        set_status_led(led, true);
        Timer::after_millis(40).await;
        set_status_led(led, false);
        Timer::after_millis(40).await;
    }
}

impl AppBoard for WioApp {
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
        set_status_led(&mut self.led, on);
    }

    async fn advert_feedback(&mut self) {
        advert_feedback(&mut self.led).await;
    }

    async fn render(&mut self, node: &CompanionNode<'_>, ui: &UiState, _full: bool) {
        self.last_periodic_refresh = Instant::now();
        if let Some(d) = &mut self.display {
            // The OLED has no refresh cost and its RAM keeps updating while
            // the panel is off, so a wake always shows current content.
            d.render_dashboard(node, ui).await.ok();
        }
    }

    async fn input_wake(&mut self) {
        self.display_deadline = Instant::now()
            .checked_add(Duration::from_secs(DISPLAY_IDLE_SECS))
            .unwrap();
        if !self.display_on {
            if let Some(d) = &mut self.display {
                d.set_on(true).await.ok();
            }
            self.display_on = true;
        }
    }

    async fn tick(&mut self, node: &CompanionNode<'_>, ui: &UiState, _tick: u32) {
        if self.display_on && Instant::now() >= self.display_deadline {
            if let Some(d) = &mut self.display {
                d.set_on(false).await.ok();
            }
            self.display_on = false;
        }

        let interval_secs = ferrix_core::state::display_refresh_interval_secs();
        if interval_secs > 0
            && self.last_periodic_refresh.elapsed() >= Duration::from_secs(interval_secs as u64)
        {
            self.render(node, ui, false).await;
        }
    }

    async fn sample_battery(&mut self) -> Option<u16> {
        let raw_mv = crate::sample_battery_adc_mv(&mut self.battery_adc).await?;
        let mv = self.battery_filter.update(raw_mv);
        crate::set_battery_millivolts(mv);
        Some(mv)
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

    async fn prepare_ota_dfu(&mut self, node: &CompanionNode<'_>, _ui: &UiState) {
        if let Some(d) = &mut self.display {
            d.set_on(true).await.ok();
            d.render_ota_dfu(node).await.ok();
        }
        self.display_on = true;
    }

    fn reboot_to_ota_dfu(&mut self) -> ! {
        let sd_active =
            ferrix_core::state::BLE_STATUS.load(core::sync::atomic::Ordering::Relaxed) >= 3;
        ferrix_nrf52::reset::reboot_to_bootloader_mode(
            sd_active,
            ferrix_nrf52::reset::BootloaderMode::BleOta,
        )
    }
}
