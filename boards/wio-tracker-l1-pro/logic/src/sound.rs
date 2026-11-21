//! Buzzer patterns for each UI sound.

use ferrix_runtime::app::UiSound;

/// One tone: frequency in Hz and duration in ms.
pub type Tone = (u32, u64);

/// What the buzzer task plays for a sound.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pattern {
    /// A single beep.
    Beep(Tone),
    /// Several tones separated by `gap_ms` of silence.
    Melody { tones: &'static [Tone], gap_ms: u64 },
}

const NOTIFY: [Tone; 2] = [(2_700, 70), (3_200, 90)];

pub fn pattern(sound: UiSound) -> Pattern {
    match sound {
        UiSound::Click => Pattern::Beep((2_200, 15)),
        UiSound::Action => Pattern::Beep((1_800, 35)),
        UiSound::Notify => Pattern::Melody {
            tones: &NOTIFY,
            gap_ms: 50,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: [UiSound; 3] = [UiSound::Click, UiSound::Action, UiSound::Notify];

    #[test]
    fn patterns_match_the_shipped_tones() {
        assert_eq!(pattern(UiSound::Click), Pattern::Beep((2_200, 15)));
        assert_eq!(pattern(UiSound::Action), Pattern::Beep((1_800, 35)));
        assert_eq!(
            pattern(UiSound::Notify),
            Pattern::Melody {
                tones: &[(2_700, 70), (3_200, 90)],
                gap_ms: 50
            }
        );
    }

    #[test]
    fn tones_are_audible_and_short() {
        for sound in ALL {
            let single;
            let tones: &[Tone] = match pattern(sound) {
                Pattern::Beep(t) => {
                    single = [t];
                    &single
                }
                Pattern::Melody { tones, .. } => tones,
            };
            assert!(!tones.is_empty());
            for &(hz, ms) in tones {
                assert!((500..=5_000).contains(&hz), "{sound:?} {hz} Hz");
                assert!((1..=200).contains(&ms), "{sound:?} {ms} ms");
            }
        }
    }

    #[test]
    fn sounds_are_distinguishable() {
        assert_ne!(pattern(UiSound::Click), pattern(UiSound::Action));
        assert_ne!(pattern(UiSound::Action), pattern(UiSound::Notify));
    }
}
