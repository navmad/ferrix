//! Display subsystem for the ThinkNode M1.
//!
//! The SSD1681 controller lives in `ferrix-ssd1681`; this module owns the
//! board's 200x200 framebuffer, dashboard rendering and the `NodeDisplay`
//! adapter used by the application loop.

pub mod framebuffer;
pub mod ui;

pub use ferrix_ssd1681::{PanelConfig, Ssd1681};
pub use framebuffer::{EpdFrameBuffer, EPD_HEIGHT, EPD_WIDTH};
pub use ui::render_dashboard;

use embassy_nrf::gpio::{Input, Output};
use embassy_nrf::peripherals::TWISPI1;
use embassy_nrf::spim::Spim;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use ferrix_core::companion::protocol::CompanionNode;
use ferrix_runtime::app::UiState;

type EpdSpi = ExclusiveDevice<Spim<'static, TWISPI1>, Output<'static>, Delay>;

/// Concrete driver type for the ThinkNode panel and its pins.
pub type ThinkNodeEpd = Ssd1681<
    EpdSpi,
    Output<'static>,
    Output<'static>,
    Input<'static>,
    Output<'static>,
    Delay,
>;

/// Common display interface for MeshCore nodes.
#[allow(async_fn_in_trait)]
pub trait NodeDisplay {
    type Error: core::fmt::Debug;

    /// Initialize the display controller.
    async fn init(&mut self) -> Result<(), Self::Error>;

    /// Render and flush the dashboard (fast differential refresh).
    async fn render_dashboard(
        &mut self,
        node: &CompanionNode<'_>,
        ui: &UiState,
    ) -> Result<(), Self::Error>;

    /// Render the full dashboard with a full OTP clearing refresh.
    async fn render_dashboard_full(
        &mut self,
        node: &CompanionNode<'_>,
        ui: &UiState,
    ) -> Result<(), Self::Error>;

    /// Render the boot splash.
    async fn render_boot(&mut self, node: &CompanionNode<'_>) -> Result<(), Self::Error>;

    /// Put the display into ultra-low-power sleep.
    async fn sleep(&mut self) -> Result<(), Self::Error>;
}

/// ThinkNode M1 display wrapper implementing `NodeDisplay`.
pub struct ThinkNodeDisplay {
    epd: ThinkNodeEpd,
    fb: &'static mut EpdFrameBuffer,
}

impl ThinkNodeDisplay {
    pub fn new(epd: ThinkNodeEpd, fb: &'static mut EpdFrameBuffer) -> Self {
        Self { epd, fb }
    }

    /// Turn the frontlight/backlight on or off.
    pub fn set_backlight(&mut self, enabled: bool) {
        let _ = self.epd.set_backlight(enabled);
    }
}

impl NodeDisplay for ThinkNodeDisplay {
    type Error = ferrix_ssd1681::Error;

    async fn init(&mut self) -> Result<(), Self::Error> {
        self.epd.init().await
    }

    async fn render_dashboard(
        &mut self,
        node: &CompanionNode<'_>,
        ui: &UiState,
    ) -> Result<(), Self::Error> {
        ui::render_dashboard(node, ui, self.fb);
        self.epd.display_frame_fast(&self.fb.buffer).await
    }

    async fn render_dashboard_full(
        &mut self,
        node: &CompanionNode<'_>,
        ui: &UiState,
    ) -> Result<(), Self::Error> {
        ui::render_dashboard(node, ui, self.fb);
        self.epd.display_frame_full(&self.fb.buffer).await
    }

    async fn render_boot(&mut self, node: &CompanionNode<'_>) -> Result<(), Self::Error> {
        ui::render_boot(node, self.fb);
        self.epd.display_frame_full(&self.fb.buffer).await
    }

    async fn sleep(&mut self) -> Result<(), Self::Error> {
        self.epd.sleep().await
    }
}
