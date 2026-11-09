//! E-Paper Display Subsystem Module

pub mod epd;
pub mod ui;

pub use epd::{EpdFrameBuffer, Ssd1681};
pub use ui::render_dashboard;
