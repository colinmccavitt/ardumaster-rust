use ap_plane::mode_run::PilotThrottleSource;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::throttle_context_hookup::{throttle_context_tick, ThrottleContextInputs};

#[test]
fn manual_mode_uses_manual_overrides() {
    let out = throttle_context_tick(&ThrottleContextInputs {
        control_mode: ModeNumber::Manual.as_number(),
        ..ThrottleContextInputs::default()
    });
    assert!(!out.use_throttle_limits);
    assert!(!out.use_battery_compensation);
}

#[test]
fn stabilize_keeps_limits_without_battery_comp() {
    let out = throttle_context_tick(&ThrottleContextInputs {
        control_mode: ModeNumber::Stabilize.as_number(),
        ..ThrottleContextInputs::default()
    });
    assert!(out.use_throttle_limits);
    assert!(!out.use_battery_compensation);
}

#[test]
fn fbwb_applies_battery_compensation() {
    let out = throttle_context_tick(&ThrottleContextInputs {
        control_mode: ModeNumber::FlyByWireB.as_number(),
        ..ThrottleContextInputs::default()
    });
    assert!(out.use_throttle_limits);
    assert!(out.use_battery_compensation);
}

#[test]
fn thr_pass_stab_selects_direct_pilot_throttle() {
    let out = throttle_context_tick(&ThrottleContextInputs {
        throttle_passthru_stabilize: true,
        ..ThrottleContextInputs::default()
    });
    assert_eq!(out.pilot_throttle_source, PilotThrottleSource::Direct);
}

#[test]
fn vtol_mode_defers_limits_to_quadplane() {
    let mut features = BuildFeatures::default();
    features.quadplane = true;
    let out = throttle_context_tick(&ThrottleContextInputs {
        control_mode: ModeNumber::QHover.as_number(),
        features,
        allow_forward_throttle_in_vtol: false,
        ..ThrottleContextInputs::default()
    });
    assert!(!out.use_throttle_limits);
    assert!(!out.use_battery_compensation);
}
