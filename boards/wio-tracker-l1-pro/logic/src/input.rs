//! Switch-index to semantic-event mapping for the joystick and user button.

use ferrix_runtime::app::InputEvent;

/// Number of physical switches the input task watches.
pub const SWITCH_COUNT: usize = 6;

/// Map a switch index (the order used by the board's `WioInputs::wait_event`)
/// to the semantic event it produces.
///
/// - joystick left/up: previous page
/// - joystick right/down: next page
/// - joystick press: primary page action (FK1)
/// - user button: secondary page action (FK2)
pub fn event_for_switch(index: usize) -> InputEvent {
    match index {
        0 => InputEvent::ActionSecondary,
        1 | 3 => InputEvent::PrevPage,
        2 | 4 => InputEvent::NextPage,
        _ => InputEvent::ActionPrimary,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_switch_maps_as_documented() {
        let expected = [
            InputEvent::ActionSecondary,
            InputEvent::PrevPage,
            InputEvent::NextPage,
            InputEvent::PrevPage,
            InputEvent::NextPage,
            InputEvent::ActionPrimary,
        ];
        assert_eq!(expected.len(), SWITCH_COUNT);
        for (i, want) in expected.iter().enumerate() {
            assert_eq!(event_for_switch(i), *want, "switch {i}");
        }
    }

    #[test]
    fn out_of_range_index_is_primary_action() {
        assert_eq!(event_for_switch(SWITCH_COUNT), InputEvent::ActionPrimary);
        assert_eq!(event_for_switch(usize::MAX), InputEvent::ActionPrimary);
    }

    #[test]
    fn navigation_is_symmetric() {
        let prev = (0..SWITCH_COUNT)
            .filter(|&i| event_for_switch(i) == InputEvent::PrevPage)
            .count();
        let next = (0..SWITCH_COUNT)
            .filter(|&i| event_for_switch(i) == InputEvent::NextPage)
            .count();
        assert_eq!(prev, next);
    }
}
