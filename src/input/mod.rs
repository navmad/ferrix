//! Semantic User Input Abstraction for ferrix
//!
//! Decouples physical input hardware (pushbuttons, 5-way D-pads, rotary encoders,
//! touch buttons) from application actions.

/// Semantic user input actions across any hardware platform
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InputEvent {
    /// Move to the next dashboard / telemetry page
    NextPage,
    /// Move to the previous dashboard / telemetry page
    PrevPage,
    /// Primary action (e.g. transmit beacon, confirm, select)
    ActionPrimary,
    /// Secondary action (e.g. toggle backlight, cancel)
    ActionSecondary,
    /// Directional navigation / scrolling
    Up,
    Down,
    /// Power or display sleep toggle
    PowerToggle,
}

/// Asynchronous user input device trait
#[allow(async_fn_in_trait)]
pub trait InputDevice {
    /// Poll for the next debounced semantic input event
    async fn poll_event(&mut self) -> Option<InputEvent>;
}

