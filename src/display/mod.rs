//! Display Subsystem & NodeDisplay Trait
//!
//! Provides a unified display interface for rendering MeshCore UI screens
//! across different display technologies (E-Paper, OLED, TFT).

pub mod epd;
pub mod ui;

pub use epd::{EpdFrameBuffer, Ssd1681};
pub use ui::render_dashboard;

use crate::companion::protocol::CompanionNode;

/// Common Display Interface for MeshCore Nodes
#[allow(async_fn_in_trait)]
pub trait NodeDisplay {
    type Error: core::fmt::Debug;

    /// Initialize the display controller
    async fn init(&mut self) -> Result<(), Self::Error>;

    /// Render and flush the dashboard to the screen (fast differential refresh)
    async fn render_dashboard(&mut self, node: &CompanionNode) -> Result<(), Self::Error>;

    /// Render full dashboard with full OTP clearing refresh (for boot / calibration)
    async fn render_dashboard_full(&mut self, node: &CompanionNode) -> Result<(), Self::Error>;

    /// Put the display into ultra-low-power sleep
    async fn sleep(&mut self) -> Result<(), Self::Error>;
}

/// ThinkNode M1 SSD1681 E-Paper Display wrapper implementing NodeDisplay
pub struct ThinkNodeDisplay<'a, T: embassy_nrf::spim::Instance> {
    epd: Ssd1681<'a, T>,
    fb: &'static mut EpdFrameBuffer,
}

impl<'a, T: embassy_nrf::spim::Instance> ThinkNodeDisplay<'a, T> {
    pub fn new(epd: Ssd1681<'a, T>, fb: &'static mut EpdFrameBuffer) -> Self {
        Self { epd, fb }
    }

    /// Turn the frontlight/backlight on or off.
    pub fn set_backlight(&mut self, enabled: bool) {
        self.epd.set_backlight(enabled);
    }
}

impl<'a, T: embassy_nrf::spim::Instance> NodeDisplay for ThinkNodeDisplay<'a, T> {
    type Error = ();

    async fn init(&mut self) -> Result<(), Self::Error> {
        self.epd.init().await
    }

    async fn render_dashboard(&mut self, node: &CompanionNode) -> Result<(), Self::Error> {
        ui::render_dashboard(node, self.fb);
        self.epd.display_frame_fast(self.fb).await
    }

    async fn render_dashboard_full(&mut self, node: &CompanionNode) -> Result<(), Self::Error> {
        ui::render_dashboard(node, self.fb);
        self.epd.display_frame_full(self.fb).await
    }

    async fn sleep(&mut self) -> Result<(), Self::Error> {
        self.epd.sleep().await
    }
}
