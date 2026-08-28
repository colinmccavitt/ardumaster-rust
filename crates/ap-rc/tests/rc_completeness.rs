//! FW-019 completeness: RC_Channel surfaces already on main vs remaining.

use ap_rc::completeness::{
    completeness_counts, completeness_has, completeness_unique_names, on_main_items,
    remaining_items, this_slice_items, PortStatus, RC_COMPLETENESS,
};

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "PWM scaling + deadzone",
    "aux-function switch latch",
    "FS_THR / failsafe throttle",
    "RCMAP / RC trim persist",
    "RC_OVERRIDE / GCS override timeout",
    "option-switch 2-pos vs 3-pos PWM ranges",
    "FLTMODE_CH six-position PWM decode",
    "FLTMODE1-6 six-position mapping",
    "INITIAL_MODE / boot-mode-from-switch",
    "RC_OPTIONS bitfield decode/apply",
    "RC_SPEED / PWM update-rate",
    "RC_REVERSED / per-channel reverse",
];

const THIS_SLICE: &[&str] = &["completeness table"];

const REMAINING: &[&str] = &[
    "full RCn_OPTION aux-function table",
    "HAL raw PWM I/O",
    "hardware RC protocols",
    "log-replay",
];

#[test]
fn completeness_table_matches_main_versus_remaining() {
    assert!(completeness_unique_names());
    assert_eq!(
        RC_COMPLETENESS.len(),
        ON_MAIN.len() + THIS_SLICE.len() + REMAINING.len()
    );
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, ON_MAIN.len());
    assert_eq!(this_slice, THIS_SLICE.len());
    assert_eq!(remaining, REMAINING.len());
    for name in ON_MAIN {
        assert!(
            completeness_has(name, PortStatus::OnMain),
            "{name} must stay listed as already on main"
        );
    }
    for name in THIS_SLICE {
        assert!(
            completeness_has(name, PortStatus::ThisSlice),
            "{name} must be the closing-slice row"
        );
    }
    for name in REMAINING {
        assert!(
            completeness_has(name, PortStatus::Remaining),
            "{name} is remaining / out of scope for the SITL stub close"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}
