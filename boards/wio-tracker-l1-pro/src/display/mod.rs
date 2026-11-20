//! Display subsystem for the Wio Tracker L1.
//!
//! The SH1106 controller and the 128x64 [`Framebuffer`] live in
//! `ferrix-sh1106`; this module owns the `WioDisplay` wrapper used by the
//! application loop and the compact UI in [`ui`].

pub mod ui;

use embassy_nrf::peripherals::TWISPI1;
use embassy_nrf::twim::Twim;
use embassy_time::{with_timeout, Duration};
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_runtime::app::UiState;
use ferrix_sh1106::{Error, Framebuffer, Sh1106};

/// Bound on one frame flush so a stuck I2C bus cannot stall the app loop.
const FLUSH_TIMEOUT: Duration = Duration::from_millis(500);

/// Concrete driver type for the Wio Tracker L1's panel.
pub type WioOled = Sh1106<Twim<'static, TWISPI1>>;

/// Wio Tracker L1 display wrapper.
pub struct WioDisplay {
    oled: WioOled,
    fb: &'static mut Framebuffer,
    on: bool,
}

impl WioDisplay {
    pub fn new(oled: WioOled, fb: &'static mut Framebuffer) -> Self {
        Self { oled, fb, on: true }
    }

    /// Initialise the panel controller; the panel is on and blank afterwards.
    pub async fn init(&mut self) -> Result<(), Error> {
        self.on = true;
        match with_timeout(FLUSH_TIMEOUT, self.oled.init()).await {
            Ok(r) => r,
            Err(_) => Err(Error::I2c),
        }
    }

    /// Turn the panel on or off. Frame RAM keeps updating while off, so the
    /// next wake shows current content.
    pub async fn set_on(&mut self, on: bool) -> Result<(), Error> {
        if on == self.on {
            return Ok(());
        }
        self.on = on;
        match with_timeout(FLUSH_TIMEOUT, self.oled.set_on(on)).await {
            Ok(r) => r,
            Err(_) => Err(Error::I2c),
        }
    }

    async fn flush(&mut self) -> Result<(), Error> {
        match with_timeout(FLUSH_TIMEOUT, self.oled.flush(self.fb.as_bytes())).await {
            Ok(r) => r,
            Err(_) => Err(Error::I2c),
        }
    }

    /// Render and flush the active dashboard page.
    pub async fn render_dashboard(
        &mut self,
        node: &CompanionNode<'_>,
        ui: &UiState,
    ) -> Result<(), Error> {
        ui::render_dashboard(node, ui, self.fb);
        self.flush().await
    }

    /// Render the boot splash.
    pub async fn render_boot(&mut self, node: &CompanionNode<'_>) -> Result<(), Error> {
        ui::render_boot(node, self.fb);
        self.flush().await
    }

    /// Render the BLE OTA DFU notice.
    pub async fn render_ota_dfu(&mut self, node: &CompanionNode<'_>) -> Result<(), Error> {
        ui::render_ota_dfu(node, self.fb);
        self.flush().await
    }
}
