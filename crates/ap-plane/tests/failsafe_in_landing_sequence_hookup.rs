//! Landing-sequence gate for RC short/long failsafe.
//!
//! Upstream `Plane::failsafe_in_landing_sequence` in `ArduPlane/events.cpp`.
//! AUTO / AUTOLAND skip the existing `FS_SHORT_ACTN` / `FS_LONG_ACTN` table
//! when the vehicle is in a landing sequence.

use ap_plane::failsafe_action_hookup::{
    long_failsafe_action, short_failsafe_action, FailsafeActionLong, FailsafeActionResult,
    FailsafeActionShort,
};
use ap_plane::failsafe_in_landing_sequence_hookup::{
    failsafe_in_landing_sequence, gated_long_failsafe_action, gated_short_failsafe_action,
    skip_rc_failsafe_in_landing_sequence, LandingSequenceInputs,
};
use ap_plane::mode_table::ModeNumber;

fn land() -> LandingSequenceInputs {
    LandingSequenceInputs::land_stage()
}

fn cruise() -> LandingSequenceInputs {
    LandingSequenceInputs::default()
}

#[test]
fn landing_sequence_is_land_or_mission_flag_or_vtol() {
    assert!(!failsafe_in_landing_sequence(&cruise()));
    assert!(failsafe_in_landing_sequence(&land()));
    assert!(failsafe_in_landing_sequence(
        &LandingSequenceInputs::mission_flag()
    ));
    assert!(failsafe_in_landing_sequence(
        &LandingSequenceInputs::vtol_land()
    ));
    assert!(!failsafe_in_landing_sequence(&LandingSequenceInputs {
        mission_in_landing_sequence: false,
        vtol_land_sequence: false,
        ..cruise()
    }));
}

#[test]
fn auto_and_autoland_skip_short_and_long_in_landing_sequence() {
    for mode in [ModeNumber::Auto, ModeNumber::Autoland] {
        assert!(skip_rc_failsafe_in_landing_sequence(mode, &land()));
        assert!(!skip_rc_failsafe_in_landing_sequence(mode, &cruise()));
        assert_eq!(
            gated_short_failsafe_action(mode, FailsafeActionShort::Circle, &land()),
            FailsafeActionResult::Continue
        );
        assert_eq!(
            gated_long_failsafe_action(mode, FailsafeActionLong::Rtl, true, &land()),
            FailsafeActionResult::Continue
        );
    }
}

#[test]
fn auto_outside_landing_sequence_still_uses_the_action_table() {
    assert_eq!(
        gated_short_failsafe_action(ModeNumber::Auto, FailsafeActionShort::Circle, &cruise()),
        short_failsafe_action(ModeNumber::Auto, FailsafeActionShort::Circle)
    );
    assert_eq!(
        gated_long_failsafe_action(ModeNumber::Auto, FailsafeActionLong::Rtl, true, &cruise()),
        long_failsafe_action(ModeNumber::Auto, FailsafeActionLong::Rtl, true)
    );
    assert_eq!(
        gated_short_failsafe_action(ModeNumber::Auto, FailsafeActionShort::Circle, &cruise()),
        FailsafeActionResult::Switch(ModeNumber::Circle)
    );
    assert_eq!(
        gated_long_failsafe_action(ModeNumber::Auto, FailsafeActionLong::Rtl, true, &cruise()),
        FailsafeActionResult::Switch(ModeNumber::Rtl)
    );
}

#[test]
fn stick_and_guided_modes_do_not_use_the_landing_sequence_gate() {
    for mode in [
        ModeNumber::Manual,
        ModeNumber::FlyByWireA,
        ModeNumber::Guided,
        ModeNumber::AvoidAdsb,
        ModeNumber::Loiter,
    ] {
        assert!(!skip_rc_failsafe_in_landing_sequence(mode, &land()));
        assert_eq!(
            gated_short_failsafe_action(mode, FailsafeActionShort::Circle, &land()),
            short_failsafe_action(mode, FailsafeActionShort::Circle)
        );
        assert_eq!(
            gated_long_failsafe_action(mode, FailsafeActionLong::Rtl, true, &land()),
            long_failsafe_action(mode, FailsafeActionLong::Rtl, true)
        );
    }
}

#[test]
fn mission_flag_and_vtol_land_also_gate_auto() {
    let mission = LandingSequenceInputs::mission_flag();
    let vtol = LandingSequenceInputs::vtol_land();
    assert_eq!(
        gated_short_failsafe_action(ModeNumber::Auto, FailsafeActionShort::Fbwa, &mission),
        FailsafeActionResult::Continue
    );
    assert_eq!(
        gated_long_failsafe_action(ModeNumber::Auto, FailsafeActionLong::Glide, true, &vtol),
        FailsafeActionResult::Continue
    );
}
