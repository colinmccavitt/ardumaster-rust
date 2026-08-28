//! FW-024 completeness: AP_Mission command surfaces already on main vs remaining.

use ap_mission::completeness::{
    completeness_counts, completeness_has, completeness_unique_names, on_main_items,
    remaining_items, this_slice_items, PortStatus, MISSION_COMPLETENESS,
};

/// Surfaces already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "Mission_Command storage",
    "NAV_WAYPOINT verify-distance / reached-wp",
    "NAV_LOITER_UNLIM do/verify",
    "NAV_LOITER_TURNS do/verify",
    "NAV_LOITER_TIME do/verify",
    "NAV_LOITER_TO_ALT do/verify",
    "NAV_CONTINUE_AND_CHANGE_ALT do/verify",
    "NAV_LAND do/verify",
    "DO_JUMP / jump-to-seq",
    "DO_CHANGE_SPEED / airspeed-groundspeed-throttle",
    "DO_SET_HOME / current-or-specified-LLA",
    "DO_SET_ROI / point-camera-at-location",
    "mission_scheduler_hookup",
    "mission_alt_offset_glue_hookup",
];

const THIS_SLICE: &[&str] = &["completeness table"];

/// Documented-deferred gaps — do not invent stubs that already exist.
const REMAINING: &[&str] = &[
    "DO_DIGICAM",
    "DO_MOUNT",
    "conditionals",
    "remaining nav/do commands",
    "log-replay",
];

#[test]
fn completeness_table_matches_main_versus_remaining() {
    assert!(completeness_unique_names());
    assert_eq!(
        MISSION_COMPLETENESS.len(),
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
            "{name} is remaining / documented-deferred for the SITL stub close"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
}
