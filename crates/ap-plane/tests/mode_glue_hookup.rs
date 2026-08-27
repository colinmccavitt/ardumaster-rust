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

#[test]
fn mode_glue_restore_tick_applies_pilot_throttle() {
    use ap_plane::mode_glue_hookup::{mode_glue_restore_tick, ModeGlueRestoreInputs};

    let out = mode_glue_restore_tick(&ModeGlueRestoreInputs {
        transition_cleared: true,
        throttle_suppressed: false,
        current_throttle: 0.0,
        pilot_throttle: 75.0,
    });
    assert!(out.restored);
    assert_eq!(out.pilot_throttle, 75.0);
}

#[test]
fn mode_glue_restore_tick_skips_when_still_suppressed() {
    use ap_plane::mode_glue_hookup::{mode_glue_restore_tick, ModeGlueRestoreInputs};

    let out = mode_glue_restore_tick(&ModeGlueRestoreInputs {
        transition_cleared: true,
        throttle_suppressed: true,
        current_throttle: 0.0,
        pilot_throttle: 75.0,
    });
    assert!(!out.restored);
    assert_eq!(out.pilot_throttle, 0.0);
}

#[test]
fn mode_glue_set_servos_tick_restores_then_keeps_pilot_throttle() {
    use ap_plane::landing_hookup::ServoOutputState;
    use ap_plane::mode_glue_hookup::{mode_glue_set_servos_tick, ModeGlueSetServosInputs};
    use ap_plane::mode_table::{BuildFeatures, ModeNumber};
    use ap_plane::rc_failsafe_scheduler_hookup::RcChannelConfig;
    use ap_plane::yaw_throttle_glue_hookup::PilotThrottleGlueInputs;

    let out = mode_glue_set_servos_tick(
        ServoOutputState {
            throttle_scaled: 0.0,
            ..ServoOutputState::default()
        },
        &ModeGlueSetServosInputs {
            control_mode: ModeNumber::Auto.as_number(),
            features: BuildFeatures::default(),
            transition_cleared: true,
            throttle_suppressed: false,
            current_throttle: 0.0,
            pilot_throttle: PilotThrottleGlueInputs {
                throttle_pwm: Some(1750),
                throttle_cfg: RcChannelConfig::default(),
                ..PilotThrottleGlueInputs::default()
            },
        },
    );
    assert!(out.throttle_restored);
    assert!(out.clear_throttle_zeroed);
    assert!(!out.mode_entry_applied);
    assert!((out.servos.throttle_scaled - 79.1).abs() < 0.5);
    assert_eq!(out.stabilize_throttle, Some(out.servos.throttle_scaled));
}

#[test]
fn mode_glue_set_servos_tick_zeros_when_still_suppressed() {
    use ap_plane::landing_hookup::ServoOutputState;
    use ap_plane::mode_glue_hookup::{mode_glue_set_servos_tick, ModeGlueSetServosInputs};
    use ap_plane::mode_table::{BuildFeatures, ModeNumber};
    use ap_plane::rc_failsafe_scheduler_hookup::RcChannelConfig;
    use ap_plane::yaw_throttle_glue_hookup::PilotThrottleGlueInputs;

    let out = mode_glue_set_servos_tick(
        ServoOutputState {
            throttle_scaled: 60.0,
            ..ServoOutputState::default()
        },
        &ModeGlueSetServosInputs {
            control_mode: ModeNumber::Auto.as_number(),
            features: BuildFeatures::default(),
            transition_cleared: false,
            throttle_suppressed: true,
            current_throttle: 60.0,
            pilot_throttle: PilotThrottleGlueInputs {
                throttle_pwm: Some(1750),
                throttle_cfg: RcChannelConfig::default(),
                ..PilotThrottleGlueInputs::default()
            },
        },
    );
    assert!(!out.throttle_restored);
    assert!(out.mode_entry_applied);
    assert_eq!(out.servos.throttle_scaled, 0.0);
    assert_eq!(out.stabilize_throttle, None);
}

#[test]
fn mode_glue_update_control_tick_zeros_auto_entry_throttle() {
    use ap_plane::mode_glue_hookup::{
        mode_glue_update_control_tick, ModeGlueUpdateControlInputs,
    };
    use ap_plane::mode_run::StickMixing;
    use ap_plane::mode_table::{BuildFeatures, ModeNumber};
    use ap_plane::rc_failsafe_scheduler_hookup::RcChannelConfig;
    use ap_plane::yaw_throttle_glue_hookup::PilotThrottleGlueInputs;

    let out = mode_glue_update_control_tick(&ModeGlueUpdateControlInputs {
        pilot_throttle: PilotThrottleGlueInputs {
            throttle_pwm: Some(2000),
            throttle_cfg: RcChannelConfig::default(),
            ..PilotThrottleGlueInputs::default()
        },
        control_mode: ModeNumber::Auto.as_number(),
        features: BuildFeatures::default(),
        stick_mixing: Some(StickMixing::Fbw),
        throttle_suppressed: true,
    });
    assert!(out.throttle_zeroed_by_mode_entry);
    assert_eq!(out.pilot_throttle, 0.0);
    assert_eq!(out.effective_stick_mixing, Some(StickMixing::Fbw));
}

#[test]
fn mode_glue_update_control_tick_passes_pilot_throttle_when_not_suppressed() {
    use ap_plane::mode_glue_hookup::{
        mode_glue_update_control_tick, ModeGlueUpdateControlInputs,
    };
    use ap_plane::mode_run::StickMixing;
    use ap_plane::mode_table::{BuildFeatures, ModeNumber};
    use ap_plane::rc_failsafe_scheduler_hookup::RcChannelConfig;
    use ap_plane::yaw_throttle_glue_hookup::PilotThrottleGlueInputs;

    let out = mode_glue_update_control_tick(&ModeGlueUpdateControlInputs {
        pilot_throttle: PilotThrottleGlueInputs {
            throttle_pwm: Some(2000),
            throttle_cfg: RcChannelConfig::default(),
            ..PilotThrottleGlueInputs::default()
        },
        control_mode: ModeNumber::FlyByWireB.as_number(),
        features: BuildFeatures::default(),
        stick_mixing: Some(StickMixing::Fbw),
        throttle_suppressed: false,
    });
    assert!(!out.throttle_zeroed_by_mode_entry);
    assert!((out.pilot_throttle - 100.0).abs() < 0.1);
}

#[test]
fn mode_glue_set_servos_tick_maps_rc_throttle_before_restore() {
    use ap_plane::landing_hookup::ServoOutputState;
    use ap_plane::mode_glue_hookup::{mode_glue_set_servos_tick, ModeGlueSetServosInputs};
    use ap_plane::mode_table::{BuildFeatures, ModeNumber};
    use ap_plane::rc_failsafe_scheduler_hookup::RcChannelConfig;
    use ap_plane::yaw_throttle_glue_hookup::PilotThrottleGlueInputs;

    let out = mode_glue_set_servos_tick(
        ServoOutputState {
            throttle_scaled: 0.0,
            ..ServoOutputState::default()
        },
        &ModeGlueSetServosInputs {
            control_mode: ModeNumber::FlyByWireB.as_number(),
            features: BuildFeatures::default(),
            transition_cleared: true,
            throttle_suppressed: false,
            current_throttle: 0.0,
            pilot_throttle: PilotThrottleGlueInputs {
                throttle_pwm: Some(2000),
                throttle_cfg: RcChannelConfig::default(),
                ..PilotThrottleGlueInputs::default()
            },
        },
    );
    assert!(out.throttle_restored);
    assert!((out.servos.throttle_scaled - 100.0).abs() < 0.1);
}


#[test]
fn mode_glue_stabilize_tick_mixes_vtol_yaw_into_rudder() {
    use ap_plane::mode_glue_hookup::{mode_glue_stabilize_tick, ModeGlueStabilizeInputs};
    use ap_plane::mode_run::StickMixing;

    let out = mode_glue_stabilize_tick(
        0.0,
        &ModeGlueStabilizeInputs {
            stick_mixing: Some(StickMixing::VtolYaw),
            yaw_norm_dz: 0.5,
            rudder_limit_scaled: 1000.0,
        },
    );
    assert!((out.rudder_scaled - 500.0).abs() < 0.1);
}

#[test]
fn mode_glue_stabilize_tick_skips_non_vtol_mixing() {
    use ap_plane::mode_glue_hookup::{mode_glue_stabilize_tick, ModeGlueStabilizeInputs};
    use ap_plane::mode_run::StickMixing;

    let out = mode_glue_stabilize_tick(
        250.0,
        &ModeGlueStabilizeInputs {
            stick_mixing: Some(StickMixing::Fbw),
            yaw_norm_dz: 1.0,
            rudder_limit_scaled: 4500.0,
        },
    );
    assert_eq!(out.rudder_scaled, 250.0);
}

