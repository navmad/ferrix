//! Two-button single/double-click mapping for ThinkNode M1.

use ferrix_runtime::app::InputEvent;

/// Contact-bounce settle window after the first falling edge.
pub const DEBOUNCE_MS: u64 = 40;

/// Maximum interval after debounce to recognize a second click of the same button.
pub const DOUBLE_CLICK_MS: u64 = 280;

/// Physical buttons on the ThinkNode M1.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Button {
    /// Top button (P1.07): page actions (FK1 / FK2).
    Function,
    /// Bottom button (P1.10): page navigation (TK / TKx2).
    Page,
}

/// Map a debounced button press (single or double click) to its semantic event.
///
/// - Function single click: primary page action (FK1)
/// - Function double click: secondary page action (FK2)
/// - Page single click: next page (TK)
/// - Page double click: previous page (TKx2)
pub fn event_for_click(button: Button, double: bool) -> InputEvent {
    match (button, double) {
        (Button::Function, false) => InputEvent::ActionPrimary,
        (Button::Function, true) => InputEvent::ActionSecondary,
        (Button::Page, false) => InputEvent::NextPage,
        (Button::Page, true) => InputEvent::PrevPage,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_button_click_maps_as_documented() {
        assert_eq!(
            event_for_click(Button::Function, false),
            InputEvent::ActionPrimary
        );
        assert_eq!(
            event_for_click(Button::Function, true),
            InputEvent::ActionSecondary
        );
        assert_eq!(event_for_click(Button::Page, false), InputEvent::NextPage);
        assert_eq!(event_for_click(Button::Page, true), InputEvent::PrevPage);
    }

    #[test]
    fn all_four_gestures_produce_distinct_events() {
        let events = [
            event_for_click(Button::Function, false),
            event_for_click(Button::Function, true),
            event_for_click(Button::Page, false),
            event_for_click(Button::Page, true),
        ];
        for i in 0..events.len() {
            for j in i + 1..events.len() {
                assert_ne!(events[i], events[j], "gestures {i} and {j} collide");
            }
        }
    }

    #[test]
    fn timing_windows_are_sane() {
        assert!(DEBOUNCE_MS >= 10 && DEBOUNCE_MS <= 100);
        assert!(DOUBLE_CLICK_MS > DEBOUNCE_MS);
        assert!(DOUBLE_CLICK_MS <= 500);
    }
}
