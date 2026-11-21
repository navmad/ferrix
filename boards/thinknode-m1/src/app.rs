//! ThinkNode M1 implementation of the runtime application board interface.

use defmt::info;
use embassy_nrf::gpio::Output;
use embassy_nrf::saadc;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::TrySendError;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_nrf52::buzzer::PassiveBuzzer;
use ferrix_runtime::app::{AppBoard, UiSound, UiState};

use crate::display::{DisplayCmd, EpdFrameBuffer, DISPLAY_CMD, DISPLAY_RETURN};
use crate::gps;

/// Seconds the display frontlight stays on after the last button press.
const BACKLIGHT_IDLE_SECS: u64 = 15;

/// Sound requests played off the radio RX/TX loop by [`buzzer_task`].
pub static BUZZER_SIGNAL: Signal<CriticalSectionRawMutex, UiSound> = Signal::new();

/// Background task that drives the passive buzzer without stalling `app::run`.
#[embassy_executor::task]
pub async fn buzzer_task(mut buzzer: PassiveBuzzer<'static>) -> ! {
    loop {
        let sound = BUZZER_SIGNAL.wait().await;
        if !ferrix_core::state::buzzer_enabled() {
            continue;
        }
        match sound {
            UiSound::Click => buzzer.beep(2_200, 15).await,
            UiSound::Action => buzzer.beep(1_800, 35).await,
            UiSound::Notify => {
                buzzer.melody(&[(2_700, 70), (3_200, 90)], 50).await;
            }
        }
    }
}

/// Board-side devices used by the application loop.
pub struct ThinkNodeApp {
    led: Output<'static>,
    fb: Option<&'static mut EpdFrameBuffer>,
    backlight: Option<Output<'static>>,
    has_display: bool,
    battery_adc: saadc::Saadc<'static, 1>,
    battery_filter: ferrix_core::battery::BatteryMvFilter,
    backlight_on: bool,
    backlight_deadline: Instant,
    last_periodic_refresh: Instant,
}

impl ThinkNodeApp {
    pub fn new(
        led: Output<'static>,
        fb: Option<&'static mut EpdFrameBuffer>,
        backlight: Option<Output<'static>>,
        battery_adc: saadc::Saadc<'static, 1>,
    ) -> Self {
        let now = Instant::now();
        let has_display = fb.is_some();
        let mut battery_filter = ferrix_core::battery::BatteryMvFilter::new();
        let initial_mv = crate::battery_millivolts();
        if initial_mv >= crate::MIN_PLAUSIBLE_BATTERY_MV {
            battery_filter.update(initial_mv);
        }
        Self {
            led,
            fb,
            backlight,
            has_display,
            battery_adc,
            battery_filter,
            backlight_on: false,
            backlight_deadline: now
                .checked_add(Duration::from_secs(BACKLIGHT_IDLE_SECS))
                .unwrap(),
            last_periodic_refresh: now,
        }
    }

    fn set_backlight(&mut self, enabled: bool) {
        if let Some(pin) = &mut self.backlight {
            if enabled {
                pin.set_high();
            } else {
                pin.set_low();
            }
        }
    }

    fn reclaim_fb(&mut self) {
        if self.fb.is_none() {
            if let Ok(fb) = DISPLAY_RETURN.try_receive() {
                self.fb = Some(fb);
            }
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

impl AppBoard for ThinkNodeApp {
    fn page_count(&self) -> u8 {
        crate::display::ui::PAGE_COUNT
    }

    fn has_display(&self) -> bool {
        self.has_display
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

    fn advert_feedback(&mut self) {
        set_blue_led(&mut self.led, true);
    }

    fn render(&mut self, node: &CompanionNode<'_>, ui: &UiState, full: bool) -> bool {
        if !self.has_display {
            return true;
        }
        self.reclaim_fb();
        let Some(fb) = self.fb.take() else {
            return false;
        };
        crate::display::ui::render_dashboard(node, ui, fb);
        let cmd = DisplayCmd::Flush {
            fb,
            full,
            sleep_after: !self.backlight_on,
        };
        match DISPLAY_CMD.try_send(cmd) {
            Ok(()) => {
                self.last_periodic_refresh = Instant::now();
                true
            }
            Err(TrySendError::Full(DisplayCmd::Flush { fb, .. })) => {
                self.fb = Some(fb);
                false
            }
            Err(TrySendError::Full(DisplayCmd::Sleep)) => unreachable!(),
        }
    }

    fn input_wake(&mut self) {
        self.backlight_on = true;
        self.backlight_deadline = Instant::now()
            .checked_add(Duration::from_secs(BACKLIGHT_IDLE_SECS))
            .unwrap();
        self.set_backlight(true);
    }

    fn tick(&mut self, node: &CompanionNode<'_>, ui: &UiState, _tick: u32) {
        self.reclaim_fb();
        if self.backlight_on && Instant::now() >= self.backlight_deadline {
            self.set_backlight(false);
            if self.has_display {
                let _ = DISPLAY_CMD.try_send(DisplayCmd::Sleep);
            }
            self.backlight_on = false;
        }

        let interval_secs = ferrix_core::state::display_refresh_interval_secs();
        if interval_secs > 0
            && self.last_periodic_refresh.elapsed() >= Duration::from_secs(interval_secs as u64)
        {
            let _ = self.render(node, ui, false);
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

    fn beep(&mut self, sound: UiSound) {
        if ferrix_core::state::buzzer_enabled() {
            BUZZER_SIGNAL.signal(sound);
        }
    }

    async fn prepare_ota_dfu(&mut self, node: &CompanionNode<'_>, _ui: &UiState) {
        self.backlight_on = false;
        self.set_backlight(false);
        if !self.has_display {
            return;
        }
        let fb = match self.fb.take() {
            Some(fb) => fb,
            None => DISPLAY_RETURN.receive().await,
        };
        crate::display::ui::render_ota_dfu(node, fb);
        DISPLAY_CMD
            .send(DisplayCmd::Flush {
                fb,
                full: true,
                sleep_after: true,
            })
            .await;
        // Wait for the OTA notice to finish painting before resetting.
        self.fb = Some(DISPLAY_RETURN.receive().await);
    }

    fn reboot_to_ota_dfu(&mut self) -> ! {
        let sd_active = ferrix_core::state::BLE_STATUS.load(core::sync::atomic::Ordering::Relaxed) >= 3;
        ferrix_nrf52::reset::reboot_to_bootloader_mode(
            sd_active,
            ferrix_nrf52::reset::BootloaderMode::BleOta,
        )
    }
}
