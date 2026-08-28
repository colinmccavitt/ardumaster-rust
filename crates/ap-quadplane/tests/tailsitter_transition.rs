//! Tailsitter transition pitch / throttle ramp — upstream
//! `Tailsitter_Transition` pitch slew and `Tailsitter::output` throttle.

use ap_quadplane::transition::{
    TransitionKind, TransitionRamp, GAIN_SCALING_MIN_DEFAULT, PITCH_CD_LIMIT,
    THROTTLE_SCALE_MAX_DEFAULT, TRANSITION_ANGLE_FW_DEFAULT, TRANSITION_ANGLE_VTOL_DEFAULT,
    TRANSITION_RATE_FW_DEFAULT, TRANSITION_RATE_VTOL_DEFAULT, TRANSITION_THROTTLE_VTOL_DEFAULT,
};

#[test]
fn groupinfo_defaults_match_upstream() {
    let ramp = TransitionRamp::new();
    assert_eq!(ramp.angle_fw(), TRANSITION_ANGLE_FW_DEFAULT);
    assert_eq!(ramp.angle_fw(), 45);
    assert_eq!(ramp.angle_vtol(), TRANSITION_ANGLE_VTOL_DEFAULT);
    assert_eq!(ramp.angle_vtol(), 0);
    assert!((ramp.rate_fw() - TRANSITION_RATE_FW_DEFAULT).abs() < f32::EPSILON);
    assert!((ramp.rate_fw() - 50.0).abs() < f32::EPSILON);
    assert!((ramp.rate_vtol() - TRANSITION_RATE_VTOL_DEFAULT).abs() < f32::EPSILON);
    assert!((ramp.rate_vtol() - 50.0).abs() < f32::EPSILON);
    assert!((ramp.throttle_vtol() - TRANSITION_THROTTLE_VTOL_DEFAULT).abs() < f32::EPSILON);
    assert!((ramp.throttle_vtol() - (-1.0)).abs() < f32::EPSILON);
    assert!((ramp.throttle_scale_max() - THROTTLE_SCALE_MAX_DEFAULT).abs() < f32::EPSILON);
    assert!((ramp.throttle_scale_max() - 2.0).abs() < f32::EPSILON);
    assert!((ramp.gain_scaling_min() - GAIN_SCALING_MIN_DEFAULT).abs() < f32::EPSILON);
    assert!((ramp.gain_scaling_min() - 0.4).abs() < f32::EPSILON);
}

#[test]
fn ang_vt_zero_falls_back_to_angle() {
    let ramp = TransitionRamp::new();
    assert_eq!(ramp.get_transition_angle_vtol(), ramp.angle_fw());
    assert_eq!(ramp.get_transition_angle_vtol(), 45);
}

#[test]
fn explicit_ang_vt_wins() {
    let mut ramp = TransitionRamp::new();
    ramp.set_angle_vtol(70);
    assert_eq!(ramp.get_transition_angle_vtol(), 70);
    assert_eq!(ramp.angle_fw(), 45);
}

#[test]
fn fw_pitch_ramps_down_at_rat_fw() {
    // 50 deg/s * 1000 ms * 0.1 = 5000 centidegrees.
    let ramp = TransitionRamp::new();
    let pitch = ramp.pitch_cd(TransitionKind::ToFw, 0.0, 1000, false);
    assert_eq!(pitch, -5000);
}

#[test]
fn vtol_pitch_ramps_up_at_rat_vt() {
    let ramp = TransitionRamp::new();
    let pitch = ramp.pitch_cd(TransitionKind::ToVtol, 0.0, 1000, false);
    assert_eq!(pitch, 5000);
}

#[test]
fn inverted_fw_ramps_the_other_way() {
    let ramp = TransitionRamp::new();
    let pitch = ramp.pitch_cd(TransitionKind::ToFw, 0.0, 1000, true);
    assert_eq!(pitch, 5000);
}

#[test]
fn pitch_demand_clamps_to_plus_minus_85_deg() {
    let mut ramp = TransitionRamp::new();
    ramp.set_rate_fw(500.0);
    ramp.set_rate_vtol(500.0);
    // 500 deg/s * 2000 ms * 0.1 = 100_000 cd, well past the limit.
    assert_eq!(
        ramp.pitch_cd(TransitionKind::ToFw, 0.0, 2000, false),
        -PITCH_CD_LIMIT
    );
    assert_eq!(
        ramp.pitch_cd(TransitionKind::ToVtol, 0.0, 2000, false),
        PITCH_CD_LIMIT
    );
}

#[test]
fn fw_complete_requires_pitch_past_angle() {
    let ramp = TransitionRamp::new();
    // ANGLE is 45 deg = 4500 cd. Equality is not enough (`>` not `>=`).
    assert!(!ramp.angle_complete(TransitionKind::ToFw, -4500));
    assert!(!ramp.angle_complete(TransitionKind::ToFw, 4500));
    assert!(ramp.angle_complete(TransitionKind::ToFw, -4501));
    assert!(ramp.angle_complete(TransitionKind::ToFw, 4501));
}

#[test]
fn vtol_complete_uses_ang_vt_fallback() {
    let mut ramp = TransitionRamp::new();
    // ANG_VT is 0, so ANGLE (45) is the threshold.
    assert!(!ramp.angle_complete(TransitionKind::ToVtol, 4500));
    assert!(ramp.angle_complete(TransitionKind::ToVtol, 4501));

    ramp.set_angle_vtol(60);
    assert!(!ramp.angle_complete(TransitionKind::ToVtol, 6000));
    assert!(ramp.angle_complete(TransitionKind::ToVtol, 6001));
}

#[test]
fn vtol_throttle_negative_uses_hover_max_cruise() {
    let ramp = TransitionRamp::new();
    // Default THR_VT is -1.
    let thr = ramp.throttle(TransitionKind::ToVtol, 0.35, 50.0, 0.1);
    assert!((thr - 0.50).abs() < 1e-6);

    let thr_hover_wins = ramp.throttle(TransitionKind::ToVtol, 0.60, 40.0, 0.1);
    assert!((thr_hover_wins - 0.60).abs() < 1e-6);
}

#[test]
fn vtol_throttle_explicit_percent_is_used() {
    let mut ramp = TransitionRamp::new();
    ramp.set_throttle_vtol(80.0);
    let thr = ramp.throttle(TransitionKind::ToVtol, 0.35, 50.0, 0.1);
    assert!((thr - 0.80).abs() < 1e-6);
}

#[test]
fn vtol_throttle_zero_is_used_not_hover() {
    // `is_negative(0)` is false, so THR_VT 0 is a real demand.
    let mut ramp = TransitionRamp::new();
    ramp.set_throttle_vtol(0.0);
    let thr = ramp.throttle(TransitionKind::ToVtol, 0.35, 50.0, 0.1);
    assert!(thr.abs() < 1e-6);
}

#[test]
fn vtol_throttle_is_capped_at_one() {
    let mut ramp = TransitionRamp::new();
    ramp.set_throttle_vtol(150.0);
    let thr = ramp.throttle(TransitionKind::ToVtol, 0.35, 50.0, 0.1);
    assert!((thr - 1.0).abs() < 1e-6);
}

#[test]
fn fw_throttle_is_hover_max_current() {
    let ramp = TransitionRamp::new();
    assert!((ramp.throttle(TransitionKind::ToFw, 0.35, 50.0, 0.20) - 0.35).abs() < 1e-6);
    assert!((ramp.throttle(TransitionKind::ToFw, 0.35, 50.0, 0.70) - 0.70).abs() < 1e-6);
}

#[test]
fn throttle_scaler_is_gscmax_when_throttle_is_zero() {
    let ramp = TransitionRamp::new();
    assert!((ramp.throttle_scaler(0.35, 0.0) - THROTTLE_SCALE_MAX_DEFAULT).abs() < 1e-6);
}

#[test]
fn throttle_scaler_is_hover_over_throttle_clamped() {
    let ramp = TransitionRamp::new();
    // hover 0.4 / throttle 0.2 = 2.0, at the GSCMAX ceiling.
    assert!((ramp.throttle_scaler(0.4, 0.2) - 2.0).abs() < 1e-6);
    // hover 0.4 / throttle 0.1 = 4.0, clamped to GSCMAX 2.
    assert!((ramp.throttle_scaler(0.4, 0.1) - 2.0).abs() < 1e-6);
    // hover 0.4 / throttle 2.0 = 0.2, clamped up to GSCMIN 0.4.
    assert!((ramp.throttle_scaler(0.4, 2.0) - 0.4).abs() < 1e-6);
    // hover 0.4 / throttle 0.5 = 0.8, in range.
    assert!((ramp.throttle_scaler(0.4, 0.5) - 0.8).abs() < 1e-6);
}
