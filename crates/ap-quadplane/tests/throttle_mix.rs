//! QuadPlane throttle mix / tilt-wait — upstream
//! `update_throttle_mix` and TIMER `tilt_fwd_complete` before
//! forward flight.

use ap_quadplane::throttle::{
    allow_update_throttle_mix, tilt_fwd_complete, timer_may_complete, ThrottleMix, ThrottleMixView,
    LAND_CHECK_ACCEL_MOVING, LAND_CHECK_ANGLE_ERROR_DEG, LAND_CHECK_LARGE_ANGLE_CD,
};
use ap_quadplane::transition_fsm::{SltTransition, TransitionState, Q_TRANSITION_MS_DEFAULT};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn land_check_constants_match_upstream() {
    assert!((LAND_CHECK_ANGLE_ERROR_DEG - 30.0).abs() < 0.01);
    assert!((LAND_CHECK_LARGE_ANGLE_CD - 1500.0).abs() < 0.01);
    assert!((LAND_CHECK_ACCEL_MOVING - 3.0).abs() < 0.01);
}

#[test]
fn allow_update_false_while_assisted_in_transition() {
    // Upstream: `!(assisted_flight && (AIRSPEED_WAIT || TIMER))`.
    assert!(!allow_update_throttle_mix(true, true));
    assert!(allow_update_throttle_mix(false, true));
    assert!(allow_update_throttle_mix(true, false));
    assert!(allow_update_throttle_mix(false, false));
}

#[test]
fn mix_holds_when_transition_owns_mix() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.allow_update = false;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Hold);
}

#[test]
fn mix_min_when_disarmed() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.armed = false;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Min);
}

#[test]
fn man_throttle_zero_without_airmode_is_min() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.vtol_man_throttle = true;
    view.throttle_input = 0.0;
    view.air_mode_active = false;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Min);
}

#[test]
fn man_throttle_positive_is_man() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.vtol_man_throttle = true;
    view.throttle_input = 0.4;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Man);
}

#[test]
fn man_throttle_zero_with_airmode_is_man() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.vtol_man_throttle = true;
    view.throttle_input = 0.0;
    view.air_mode_active = true;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Man);
}

#[test]
fn auto_hover_uses_mix_max() {
    // descent_not_demanded (vel_U >= 0) forces mix max.
    let qp = available_qp();
    assert_eq!(
        qp.update_throttle_mix(&ThrottleMixView::hover()),
        ThrottleMix::Max
    );
}

#[test]
fn auto_descent_small_attitude_uses_mix_min() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.vel_desired_u_ms = -0.5;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Min);
}

#[test]
fn auto_large_angle_request_uses_mix_max() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.vel_desired_u_ms = -0.5;
    view.roll_target_cd = 1600.0;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Max);
}

#[test]
fn auto_large_angle_error_uses_mix_max() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.vel_desired_u_ms = -0.5;
    view.att_error_deg = 31.0;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Max);
}

#[test]
fn auto_accel_moving_uses_mix_max() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.vel_desired_u_ms = -0.5;
    view.accel_ef_filt_len = 3.1;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Max);
}

#[test]
fn land_sequence_forces_max_until_final() {
    let qp = available_qp();
    let mut view = ThrottleMixView::hover();
    view.vel_desired_u_ms = -1.0;
    view.in_vtol_land_sequence = true;
    view.in_vtol_land_final = false;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Max);

    view.in_vtol_land_final = true;
    assert_eq!(qp.update_throttle_mix(&view), ThrottleMix::Min);
}

#[test]
fn tilt_wait_blocks_forward_flight_until_angle_achieved() {
    // SLT / no tilt: complete immediately.
    assert!(tilt_fwd_complete(false, true, false));
    assert!(QuadPlane::tilt_fwd_complete(false, true, false));
    // Non-continuous tilt: tilt_angle_achieved is true.
    assert!(tilt_fwd_complete(true, false, false));
    // Continuous tilt still slewing: wait.
    assert!(!tilt_fwd_complete(true, true, false));
    // Continuous tilt on the commanded angle: go.
    assert!(tilt_fwd_complete(true, true, true));
}

#[test]
fn timer_may_complete_requires_tilt_and_dwell() {
    assert!(!timer_may_complete(true, false));
    assert!(!timer_may_complete(false, true));
    assert!(timer_may_complete(true, true));
}

#[test]
fn slt_timer_waits_for_tilt_before_done() {
    let mut fsm = SltTransition::new();
    fsm.enter_timer();
    // `enter_timer` leaves `transition_low_airspeed_ms` at 0, so a now
    // of DEFAULT+1 expires the constrained `Q_TRANSITION_MS` dwell.
    let expired = u32::from(Q_TRANSITION_MS_DEFAULT as u16) + 1;
    fsm.update_timer(expired, tilt_fwd_complete(true, true, false));
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    assert!(!fsm.complete());

    fsm.update_timer(expired, tilt_fwd_complete(true, true, true));
    assert_eq!(fsm.transition_state(), TransitionState::Done);
    assert!(fsm.complete());
}
