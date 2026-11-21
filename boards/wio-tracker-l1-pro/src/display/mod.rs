//! Display subsystem for the Wio Tracker L1 Pro.
//!
//! The SH1106 controller and the 128x64 [`Framebuffer`] live in
//! `ferrix-sh1106`; this module owns the `WioDisplay` wrapper used by the
//! application loop and the compact UI in [`ui`].

pub mod ui;

use embassy_nrf::peripherals::TWISPI1;
use embassy_nrf::twim::Twim;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{with_timeout, Duration};
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_runtime::app::UiState;
use ferrix_sh1106::{Error, Framebuffer, Sh1106};

/// Bound on one frame flush so a stuck I2C bus cannot stall the display task.
const FLUSH_TIMEOUT: Duration = Duration::from_millis(500);

/// Concrete driver type for the Wio Tracker L1 Pro's panel.
pub type WioOled = Sh1106<Twim<'static, TWISPI1>>;

/// Commands sent from the application loop to [`display_task`].
pub enum DisplayCmd {
    /// Flush the rendered framebuffer to the panel, then return it via
    /// [`DISPLAY_RETURN`].
    Flush { fb: &'static mut Framebuffer },
    /// Turn the OLED panel on or off without flushing.
    SetOn(bool),
}

/// Queue of display refresh/power requests processed off the RX/TX loop.
pub static DISPLAY_CMD: Channel<CriticalSectionRawMutex, DisplayCmd, 2> = Channel::new();

/// Returns ownership of the staging framebuffer once [`display_task`] finishes
/// flushing it to the SH1106 panel.
pub static DISPLAY_RETURN: Channel<CriticalSectionRawMutex, &'static mut Framebuffer, 1> =
    Channel::new();

/// Hardware-facing half of the panel owned by [`display_task`] after boot.
pub struct DisplayPanel {
    oled: WioOled,
    on: bool,
}

impl DisplayPanel {
    async fn set_on(&mut self, on: bool) -> Result<(), Error> {
        if on == self.on {
            return Ok(());
        }
        self.on = on;
        match with_timeout(FLUSH_TIMEOUT, self.oled.set_on(on)).await {
            Ok(r) => r,
            Err(_) => Err(Error::I2c),
        }
    }

    async fn flush(&mut self, fb: &Framebuffer) -> Result<(), Error> {
        match with_timeout(FLUSH_TIMEOUT, self.oled.flush(fb.as_bytes())).await {
            Ok(r) => r,
            Err(_) => Err(Error::I2c),
        }
    }
}

/// Background task that drives SH1106 I2C flushes and power state without
/// stalling LoRa RX/TX or companion commands.
#[embassy_executor::task]
pub async fn display_task(mut panel: DisplayPanel) -> ! {
    loop {
        match DISPLAY_CMD.receive().await {
            DisplayCmd::Flush { fb } => {
                let _ = panel.flush(fb).await;
                DISPLAY_RETURN.send(fb).await;
            }
            DisplayCmd::SetOn(on) => {
                let _ = panel.set_on(on).await;
            }
        }
    }
}

/// Wio Tracker L1 Pro display wrapper used during synchronous boot UI setup.
pub struct WioDisplay {
    panel: DisplayPanel,
    fb: &'static mut Framebuffer,
}

impl WioDisplay {
    pub fn new(oled: WioOled, fb: &'static mut Framebuffer) -> Self {
        Self {
            panel: DisplayPanel { oled, on: true },
            fb,
        }
    }

    /// Split into the background panel task handle and staging framebuffer once
    /// boot splash rendering is complete.
    pub fn into_parts(self) -> (DisplayPanel, &'static mut Framebuffer) {
        (self.panel, self.fb)
    }

    /// Initialise the panel controller; the panel is on and blank afterwards.
    pub async fn init(&mut self) -> Result<(), Error> {
        self.panel.on = true;
        match with_timeout(FLUSH_TIMEOUT, self.panel.oled.init()).await {
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
        self.panel.flush(self.fb).await
    }

    /// Render the boot splash.
    pub async fn render_boot(&mut self, node: &CompanionNode<'_>) -> Result<(), Error> {
        ui::render_boot(node, self.fb);
        self.panel.flush(self.fb).await
    }
}
