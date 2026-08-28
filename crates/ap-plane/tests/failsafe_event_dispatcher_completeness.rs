//! FW-027 completeness: failsafe dispatcher already on main vs remaining
//! `events.cpp` / `failsafe.cpp` gaps.

use ap_plane::failsafe_action_hookup::{
    short_failsafe_action, FailsafeActionLong, FailsafeActionResult, FailsafeActionShort,
};
use ap_plane::failsafe_event_dispatcher_completeness::{
    completeness_counts, completeness_has, completeness_unique_names,
    emergency_landing_long_failsafe_action, emergency_landing_short_failsafe_action,
    long_emergency_landing_applies, on_main_items, remaining_items,
    short_emergency_landing_applies, this_slice_items, FailsafePortItem, PortStatus,
    FAILSAFE_DISPATCHER_COMPLETENESS, FENCE_ACTION_AUTOLAND_OR_RTL, Q_OPTIONS_FS_QRTL,
    Q_OPTIONS_FS_RTL,
};
use ap_plane::fence_failsafe_hookup::FenceAction;
use ap_plane::mode_table::ModeNumber;

/// Hookups already on main — do not redo these slices.
const ON_MAIN: &[&str] = &[
    "FS_THR / THR_FS_VALUE",
    "rc_failsafe_scheduler_hookup",
    "FS_SHORT_ACTN / FS_LONG_ACTN",
    "FS_GCS_ENABL",
    "FS_BATT_ENABLE",
    "FS_LONG_TIMEOUT",
    "FS_SHORT_TIMEOUT",
    "terrain failsafe",
    "geofence FENCE_ACTION",
    "failsafe_in_landing_sequence",
    "failsafe off-event recovery",
    "failsafe_check heartbeat",
    "ARSPD_FBW_MIN",
    "CIRCLE/TAKEOFF/RTL no-short-action",
];

const THIS_SLICE: &[&str] = &["emergency-landing override", "completeness table"];

const REMAINING: &[&str] = &[
    "Q_OPTIONS FS_RTL / FS_QRTL",
    "FENCE_ACTION 8 AUTOLAND-or-RTL",
];

#[test]
fn completeness_table_matches_main_versus_remaining() {
    assert!(completeness_unique_names());
    assert_eq!(
        FAILSAFE_DISPATCHER_COMPLETENESS.len(),
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
            "{name} is a remaining events.cpp / failsafe.cpp gap"
        );
    }
    assert_eq!(on_main_items().count(), ON_MAIN.len());
    assert_eq!(this_slice_items().count(), THIS_SLICE.len());
    assert_eq!(remaining_items().count(), REMAINING.len());
    for item in FAILSAFE_DISPATCHER_COMPLETENESS {
        let FailsafePortItem { name, status, note } = item;
        assert!(!name.is_empty(), "catalog row missing a name");
        assert!(!note.is_empty(), "{name} missing an upstream note");
        let _ = status;
    }
}

#[test]
fn circle_takeoff_rtl_already_take_no_short_action() {
    for mode in [ModeNumber::Circle, ModeNumber::Takeoff, ModeNumber::Rtl] {
        assert!(
            !short_emergency_landing_applies(mode),
            "{mode:?} is ShortGroup::Never — emergency landing must not rewrite it"
        );
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::Fbwa),
            FailsafeActionResult::Continue
        );
        assert_eq!(
            emergency_landing_short_failsafe_action(mode, FailsafeActionShort::Fbwa, true),
            FailsafeActionResult::Continue,
            "emergency landing must not invent a short action for {mode:?}"
        );
    }
}

#[test]
fn emergency_landing_overrides_stick_short_and_stick_or_hold_long() {
    for mode in [
        ModeNumber::Manual,
        ModeNumber::Stabilize,
        ModeNumber::Acro,
        ModeNumber::FlyByWireA,
        ModeNumber::Autotune,
        ModeNumber::FlyByWireB,
        ModeNumber::Cruise,
        ModeNumber::Training,
    ] {
        assert!(short_emergency_landing_applies(mode));
        assert!(long_emergency_landing_applies(mode));
        assert_eq!(
            emergency_landing_short_failsafe_action(mode, FailsafeActionShort::Circle, true),
            FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
        );
        assert_eq!(
            emergency_landing_long_failsafe_action(mode, FailsafeActionLong::Rtl, true, true),
            FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
        );
    }

    for mode in [
        ModeNumber::Circle,
        ModeNumber::Loiter,
        ModeNumber::Thermal,
        ModeNumber::Takeoff,
    ] {
        assert!(!short_emergency_landing_applies(mode));
        assert!(long_emergency_landing_applies(mode));
        assert_eq!(
            emergency_landing_long_failsafe_action(mode, FailsafeActionLong::Rtl, true, true),
            FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
        );
    }

    assert_eq!(
        emergency_landing_short_failsafe_action(
            ModeNumber::Manual,
            FailsafeActionShort::Disabled,
            true
        ),
        FailsafeActionResult::Continue
    );
    assert_eq!(
        emergency_landing_long_failsafe_action(
            ModeNumber::Rtl,
            FailsafeActionLong::Rtl,
            true,
            true
        ),
        FailsafeActionResult::Continue,
        "RTL long group does not consult emergency_landing"
    );
    assert_eq!(
        emergency_landing_short_failsafe_action(
            ModeNumber::QStabilize,
            FailsafeActionShort::Circle,
            true
        ),
        FailsafeActionResult::Switch(ModeNumber::QLand),
        "Q_OPTIONS RTL/QRTL is remaining — Q modes still default QLAND"
    );
}

#[test]
fn remaining_q_options_and_fence_action_8_are_still_stubs() {
    assert_eq!(Q_OPTIONS_FS_QRTL, 1 << 5);
    assert_eq!(Q_OPTIONS_FS_RTL, 1 << 20);
    assert_eq!(FENCE_ACTION_AUTOLAND_OR_RTL, 8);
    assert_eq!(
        FenceAction::from_param(FENCE_ACTION_AUTOLAND_OR_RTL),
        None,
        "FENCE_ACTION 8 AUTOLAND-or-RTL is remaining — do not claim it this slice"
    );
    assert!(completeness_has(
        "Q_OPTIONS FS_RTL / FS_QRTL",
        PortStatus::Remaining
    ));
    assert!(completeness_has(
        "FENCE_ACTION 8 AUTOLAND-or-RTL",
        PortStatus::Remaining
    ));
}
