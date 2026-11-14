//! Semantic user input abstraction.
//!
//! The event type and channel live in `ferrix-runtime` so the application loop
//! can wait on them; the board only maps its buttons to these events.

pub use ferrix_runtime::app::{InputEvent, INPUT_EVENT_CHANNEL};

/// Asynchronous user input device trait.
#[allow(async_fn_in_trait)]
pub trait InputDevice {
    /// Poll for the next debounced semantic input event.
    async fn poll_event(&mut self) -> Option<InputEvent>;

    /// Wait for the next edge without polling.
    async fn wait_event(&mut self) -> Option<InputEvent>;
}
