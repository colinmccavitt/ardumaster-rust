use ap_plane::mode_glue_hookup::{
    mode_glue_tick, resolve_effective_stick_mixing, ModeGlueInputs,
};
use ap_plane::mode_run::StickMixing;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

#[test]
fn vtol_yaw_applies_only_in_vtol_modes() {
    assert_eq!(
        resolve_effective_stick_mixing(
            ModeNumber::QHover,
            Some(StickMixing::VtolYaw),
        ),
        Some(StickMixing::VtolYaw),
    );
    assert_eq!(
        resolve_effective_stick_mixing(
            ModeNumber::FlyByWireB,
            Some(StickMixing::VtolYaw),
        ),
        None,
    );
}

#[test]
fn fbw_stick_mixing_dropped_in_vtol_modes() {
    assert_eq!(
        resolve_effective_stick_mixing(
            ModeNumber::QHover,
            Some(StickMixing::Fbw),
        ),
        None,
    );
    assert_eq!(
        resolve_effective_stick_mixing(
            ModeNumber::FlyByWireB,
            Some(StickMixing::Fbw),
        ),
        Some(StickMixing::Fbw),
    );
}

#[test]
fn mode_entry_zeros_pilot_throttle_in_auto_mode() {
    let out = mode_glue_tick(&ModeGlueInputs {
        control_mode: ModeNumber::Auto.as_number(),
        throttle_suppressed: true,
        pilot_throttle: 75.0,
        ..ModeGlueInputs::default()
    });
    assert!(out.throttle_zeroed_by_mode_entry);
    assert_eq!(out.pilot_throttle, 0.0);
}

#[test]
fn mode_entry_leaves_manual_throttle_alone() {
    let out = mode_glue_tick(&ModeGlueInputs {
        control_mode: ModeNumber::Stabilize.as_number(),
        throttle_suppressed: true,
        pilot_throttle: 75.0,
        ..ModeGlueInputs::default()
    });
    assert!(!out.throttle_zeroed_by_mode_entry);
    assert_eq!(out.pilot_throttle, 75.0);
}

#[test]
fn mode_glue_passes_through_when_not_suppressed() {
    let out = mode_glue_tick(&ModeGlueInputs {
        control_mode: ModeNumber::Auto.as_number(),
        throttle_suppressed: false,
        pilot_throttle: 60.0,
        stick_mixing: Some(StickMixing::Fbw),
        ..ModeGlueInputs::default()
    });
    assert_eq!(out.pilot_throttle, 60.0);
    assert_eq!(out.effective_stick_mixing, Some(StickMixing::Fbw));
}

#[test]
fn restore_pilot_throttle_when_transition_clears_suppression() {
    use ap_plane::mode_glue_hookup::restore_pilot_throttle_on_transition_clear;

    let (throttle, restored) = restore_pilot_throttle_on_transition_clear(
        true,
        false,
        0.0,
        75.0,
    );
    assert!(restored);
    assert_eq!(throttle, 75.0);
}

#[test]
fn restore_skips_when_still_suppressed() {
    use ap_plane::mode_glue_hookup::restore_pilot_throttle_on_transition_clear;

    let (throttle, restored) = restore_pilot_throttle_on_transition_clear(
        true,
        true,
        0.0,
        75.0,
    );
    assert!(!restored);
    assert_eq!(throttle, 0.0);
}

#[test]
fn restore_skips_when_throttle_already_nonzero() {
    use ap_plane::mode_glue_hookup::restore_pilot_throttle_on_transition_clear;

    let (throttle, restored) = restore_pilot_throttle_on_transition_clear(
        true,
        false,
        50.0,
        75.0,
    );
    assert!(!restored);
    assert_eq!(throttle, 50.0);
}
