//! VTOL_Assist angle-error trigger.

use ap_vtol_assist::{
    angle_check_enabled, evaluate_angle, AngleSample, AssistState, VtolAssist,
    ALLOWED_ENVELOPE_ERROR_DEG, ASSIST_ANGLE_DEFAULT,
};

/// Default `ROLL_LIMIT_DEG` / `PTCH_LIM_*` from Plane-4.7.0 `config.h`.
fn sample(roll_deg: f32, pitch_deg: f32, nav_roll_cd: i32, nav_pitch_cd: i32) -> AngleSample {
    AngleSample::new(
        roll_deg,
        pitch_deg,
        nav_roll_cd,
        nav_pitch_cd,
        45.0,
        20.0,
        -25.0,
    )
}

fn gated(speed: f32, angle: i8) -> VtolAssist {
    let mut assist = VtolAssist::new();
    assist.set_speed(speed);
    assist.set_angle(angle);
    assist
}

#[test]
fn allowed_envelope_slack_matches_upstream() {
    assert_eq!(ALLOWED_ENVELOPE_ERROR_DEG as i32, 5);
    assert_eq!(ASSIST_ANGLE_DEFAULT, 30);
}

#[test]
fn default_angle_is_live_once_speed_gate_opens() {
    let closed = VtolAssist::new();
    assert_eq!(closed.angle(), ASSIST_ANGLE_DEFAULT);
    assert!(!angle_check_enabled(&closed));

    let assist = gated(8.0, ASSIST_ANGLE_DEFAULT);
    assert!(assist.should_check());
    assert!(angle_check_enabled(&assist));
}

#[test]
fn outside_envelope_and_large_error_requests_angle_assist() {
    let assist = gated(8.0, 30);
    // |roll| 60 > 45+5, nav roll 0 → error 60 >= 30.
    let s = sample(60.0, 0.0, 0, 0);
    assert!(!s.inside_envelope());
    assert!(!s.inside_angle_error(assist.angle()));
    assert!(s.trigger(assist.angle()));

    let d = evaluate_angle(&assist, s);
    assert!(d.angle_assist());
    assert!(!d.force_assist());
    assert!(d.requested());
}

#[test]
fn pitch_outside_envelope_and_large_error_requests() {
    let assist = gated(8.0, 30);
    // pitch 40 >= 20+5, nav pitch 0 → error 40 >= 30.
    let s = sample(0.0, 40.0, 0, 0);
    assert!(!s.inside_envelope());
    assert!(!s.inside_angle_error(assist.angle()));

    let d = evaluate_angle(&assist, s);
    assert!(d.angle_assist());
    assert!(d.requested());
}

#[test]
fn large_error_inside_envelope_does_not_request() {
    let assist = gated(8.0, 30);
    // |roll| 40 <= 50 (inside envelope) but error 40 >= 30.
    let s = sample(40.0, 0.0, 0, 0);
    assert!(s.inside_envelope());
    assert!(!s.inside_angle_error(assist.angle()));
    assert!(!s.trigger(assist.angle()));

    let d = evaluate_angle(&assist, s);
    assert!(!d.angle_assist());
    assert!(!d.requested());
}

#[test]
fn outside_envelope_but_close_to_nav_does_not_request() {
    let assist = gated(8.0, 30);
    // roll 60 (outside envelope), nav 55 deg → error 5 < 30.
    let s = sample(60.0, 0.0, 5500, 0);
    assert!(!s.inside_envelope());
    assert!(s.inside_angle_error(assist.angle()));
    assert!(!s.trigger(assist.angle()));

    let d = evaluate_angle(&assist, s);
    assert!(!d.angle_assist());
    assert!(!d.requested());
}

#[test]
fn error_equal_to_assist_angle_is_outside() {
    let assist = gated(8.0, 30);
    // Upstream uses `< angle`, so error == 30 is not inside.
    // roll 60, nav 30 deg (3000 cd) → error 30; also outside envelope.
    let s = sample(60.0, 0.0, 3000, 0);
    assert!(!s.inside_envelope());
    assert!(!s.inside_angle_error(30));
    assert!(s.trigger(30));

    let d = evaluate_angle(&assist, s);
    assert!(d.angle_assist());
    assert!(d.requested());
}

#[test]
fn error_just_below_assist_angle_is_inside() {
    let assist = gated(8.0, 30);
    // roll 60, nav 30.1 deg → error 29.9 < 30; outside envelope but no trigger.
    let s = sample(60.0, 0.0, 3010, 0);
    assert!(!s.inside_envelope());
    assert!(s.inside_angle_error(30));

    let d = evaluate_angle(&assist, s);
    assert!(!d.angle_assist());
    assert!(!d.requested());
}

#[test]
fn roll_at_limit_plus_slack_stays_inside_envelope() {
    // Upstream `|roll| <= roll_limit + 5`.
    let at = sample(50.0, 0.0, 0, 0);
    assert!(at.inside_envelope());

    let over = sample(50.1, 0.0, 0, 0);
    assert!(!over.inside_envelope());
}

#[test]
fn pitch_at_limit_plus_slack_is_outside_envelope() {
    // Upstream pitch uses `<` / `>`, not `<=` / `>=`.
    let at_max = sample(0.0, 25.0, 0, 0);
    assert!(!at_max.inside_envelope());

    let under_max = sample(0.0, 24.9, 0, 0);
    assert!(under_max.inside_envelope());

    let at_min = sample(0.0, -30.0, 0, 0);
    assert!(!at_min.inside_envelope());

    let above_min = sample(0.0, -29.9, 0, 0);
    assert!(above_min.inside_envelope());
}

#[test]
fn zero_or_negative_assist_angle_disables_trigger() {
    let mut assist = gated(8.0, 0);
    assert!(!angle_check_enabled(&assist));

    let s = sample(60.0, 40.0, 0, 0);
    let d = evaluate_angle(&assist, s);
    assert!(!d.angle_assist());
    assert!(!d.requested());

    assist.set_angle(-1);
    assert!(!angle_check_enabled(&assist));
    let d = evaluate_angle(&assist, s);
    assert!(!d.angle_assist());
    assert!(!d.requested());
}

#[test]
fn closed_speed_gate_clears_angle() {
    let mut assist = VtolAssist::new();
    assist.set_angle(30);
    assert!(!assist.should_check());
    assert!(!angle_check_enabled(&assist));

    let d = evaluate_angle(&assist, sample(60.0, 40.0, 0, 0));
    assert!(!d.angle_assist());
    assert!(!d.force_assist());
    assert!(!d.requested());
}

#[test]
fn disabled_state_clears_every_flag() {
    let mut assist = gated(8.0, 30);
    assist.set_state(AssistState::AssistDisabled);

    let d = evaluate_angle(&assist, sample(60.0, 40.0, 0, 0));
    assert!(!d.force_assist());
    assert!(!d.angle_assist());
    assert!(!d.requested());
}

#[test]
fn force_enable_requests_when_angle_is_healthy() {
    let mut assist = gated(8.0, 30);
    assist.set_state(AssistState::ForceEnabled);

    let healthy = sample(0.0, 0.0, 0, 0);
    assert!(healthy.inside_envelope());
    assert!(healthy.inside_angle_error(30));

    let d = evaluate_angle(&assist, healthy);
    assert!(d.force_assist());
    assert!(!d.angle_assist());
    assert!(d.requested());
}

#[test]
fn force_enable_requests_when_speed_gate_is_closed() {
    let mut assist = VtolAssist::new();
    assist.set_state(AssistState::ForceEnabled);

    let d = evaluate_angle(&assist, sample(60.0, 40.0, 0, 0));
    assert!(d.force_assist());
    assert!(!d.angle_assist());
    assert!(d.requested());
}

#[test]
fn force_plus_open_gate_keeps_angle() {
    let mut assist = gated(8.0, 30);
    assist.set_state(AssistState::ForceEnabled);

    let d = evaluate_angle(&assist, sample(60.0, 40.0, 0, 0));
    assert!(d.force_assist());
    assert!(d.angle_assist());
    assert!(d.requested());
}
