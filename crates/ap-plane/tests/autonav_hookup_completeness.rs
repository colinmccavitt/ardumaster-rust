//! FW-023 completeness: modes/hookups already on main vs remaining.

use ap_plane::autonav_completeness::{
    completeness_counts, completeness_has, completeness_unique_names, on_main_items,
    remaining_items, this_slice_items, PortStatus, AUTONAV_COMPLETENESS,
};

/// Hookups already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "AUTO enter/navigate",
    "RTL enter/navigate",
    "LOITER enter/navigate",
    "GUIDED enter/navigate",
    "TAKEOFF enter/navigate",
    "AUTOLAND enter-and-stage",
    "AVOID_ADSB enter/navigate",
    "AUTO mission-complete",
    "RTL climb-then-home",
    "AUTO NAV_LOITER_TO_ALT",
    "GUIDED remaining-leg",
    "FW-022 assisted modes",
];

const THIS_SLICE: &[&str] = &["RTL_AUTOLAND", "completeness table"];

const REMAINING: &[&str] = &[
    "QLAND / VTOL modes",
    "RTL switch_QRTL",
    "GUIDED offboard slew",
    "log-replay",
];

#[test]
fn completeness_table_matches_main_versus_remaining() {
    assert!(completeness_unique_names());
    assert_eq!(
        AUTONAV_COMPLETENESS.len(),
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
            "{name} is remaining / out of scope for the fixed-wing stub close"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}
