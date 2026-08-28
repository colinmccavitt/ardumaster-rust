//! FS_SHORT_ACTN / FS_LONG_ACTN action table, upstream events.cpp.

use ap_plane::failsafe_action_hookup::{
    long_failsafe_action, short_failsafe_action, FailsafeActionLong, FailsafeActionResult,
    FailsafeActionShort,
};
use ap_plane::mode_table::ModeNumber;

#[test]
fn short_and_long_param_values_match_upstream_defines() {
    assert_eq!(
        FailsafeActionShort::from_param(0),
        Some(FailsafeActionShort::BestGuess)
    );
    assert_eq!(
        FailsafeActionShort::from_param(1),
        Some(FailsafeActionShort::Circle)
    );
    assert_eq!(
        FailsafeActionShort::from_param(2),
        Some(FailsafeActionShort::Fbwa)
    );
    assert_eq!(
        FailsafeActionShort::from_param(3),
        Some(FailsafeActionShort::Disabled)
    );
    assert_eq!(
        FailsafeActionShort::from_param(4),
        Some(FailsafeActionShort::Fbwb)
    );
    assert_eq!(FailsafeActionShort::from_param(5), None);
    assert_eq!(
        FailsafeActionShort::default_param(),
        FailsafeActionShort::BestGuess
    );

    assert_eq!(
        FailsafeActionLong::from_param(0),
        Some(FailsafeActionLong::Continue)
    );
    assert_eq!(
        FailsafeActionLong::from_param(1),
        Some(FailsafeActionLong::Rtl)
    );
    assert_eq!(
        FailsafeActionLong::from_param(2),
        Some(FailsafeActionLong::Glide)
    );
    assert_eq!(
        FailsafeActionLong::from_param(3),
        Some(FailsafeActionLong::Parachute)
    );
    assert_eq!(
        FailsafeActionLong::from_param(4),
        Some(FailsafeActionLong::Auto)
    );
    assert_eq!(
        FailsafeActionLong::from_param(5),
        Some(FailsafeActionLong::Autoland)
    );
    assert_eq!(FailsafeActionLong::from_param(6), None);
    assert_eq!(
        FailsafeActionLong::default_param(),
        FailsafeActionLong::Continue
    );
}

#[test]
fn short_stick_modes_circle_unless_fbw() {
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
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::BestGuess),
            FailsafeActionResult::Switch(ModeNumber::Circle)
        );
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::Circle),
            FailsafeActionResult::Switch(ModeNumber::Circle)
        );
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::Fbwa),
            FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
        );
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::Fbwb),
            FailsafeActionResult::Switch(ModeNumber::FlyByWireB)
        );
    }
}

#[test]
fn short_auto_like_bestguess_continues() {
    for mode in [
        ModeNumber::Auto,
        ModeNumber::Autoland,
        ModeNumber::AvoidAdsb,
        ModeNumber::Guided,
        ModeNumber::Loiter,
        ModeNumber::Thermal,
    ] {
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::BestGuess),
            FailsafeActionResult::Continue
        );
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::Circle),
            FailsafeActionResult::Switch(ModeNumber::Circle)
        );
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::Fbwa),
            FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
        );
    }
}

#[test]
fn short_circle_rtl_takeoff_never_change() {
    for mode in [
        ModeNumber::Circle,
        ModeNumber::Takeoff,
        ModeNumber::Rtl,
        ModeNumber::Initialising,
    ] {
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::Circle),
            FailsafeActionResult::Continue
        );
        assert_eq!(
            short_failsafe_action(mode, FailsafeActionShort::Fbwa),
            FailsafeActionResult::Continue
        );
    }
}

#[test]
fn short_disabled_never_enters_the_event() {
    assert!(!FailsafeActionShort::Disabled.is_enabled());
    assert!(FailsafeActionShort::BestGuess.is_enabled());
    assert_eq!(
        short_failsafe_action(ModeNumber::Manual, FailsafeActionShort::Disabled),
        FailsafeActionResult::Continue
    );
    assert_eq!(
        short_failsafe_action(ModeNumber::Auto, FailsafeActionShort::Disabled),
        FailsafeActionResult::Continue
    );
}

#[test]
fn long_stick_continue_is_rtl_auto_continue_stays() {
    assert_eq!(
        long_failsafe_action(ModeNumber::Manual, FailsafeActionLong::Continue, true),
        FailsafeActionResult::Switch(ModeNumber::Rtl)
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Stabilize, FailsafeActionLong::Rtl, true),
        FailsafeActionResult::Switch(ModeNumber::Rtl)
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Cruise, FailsafeActionLong::Glide, true),
        FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Auto, FailsafeActionLong::Continue, true),
        FailsafeActionResult::Continue
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Guided, FailsafeActionLong::Rtl, true),
        FailsafeActionResult::Switch(ModeNumber::Rtl)
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Auto, FailsafeActionLong::Glide, true),
        FailsafeActionResult::Switch(ModeNumber::FlyByWireA)
    );
}

#[test]
fn long_parachute_auto_and_autoland() {
    assert_eq!(
        long_failsafe_action(ModeNumber::Manual, FailsafeActionLong::Parachute, true),
        FailsafeActionResult::Parachute
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Auto, FailsafeActionLong::Parachute, true),
        FailsafeActionResult::Parachute
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Loiter, FailsafeActionLong::Auto, true),
        FailsafeActionResult::Switch(ModeNumber::Auto)
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Manual, FailsafeActionLong::Autoland, true),
        FailsafeActionResult::Switch(ModeNumber::Autoland)
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Manual, FailsafeActionLong::Autoland, false),
        FailsafeActionResult::Switch(ModeNumber::Rtl)
    );
}

#[test]
fn long_rtl_only_honors_auto_and_autoland() {
    assert_eq!(
        long_failsafe_action(ModeNumber::Rtl, FailsafeActionLong::Continue, true),
        FailsafeActionResult::Continue
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Rtl, FailsafeActionLong::Rtl, true),
        FailsafeActionResult::Continue
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Rtl, FailsafeActionLong::Auto, true),
        FailsafeActionResult::Switch(ModeNumber::Auto)
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Rtl, FailsafeActionLong::Autoland, false),
        FailsafeActionResult::Switch(ModeNumber::Autoland)
    );
    assert_eq!(
        long_failsafe_action(ModeNumber::Autoland, FailsafeActionLong::Rtl, true),
        FailsafeActionResult::Continue
    );
}
