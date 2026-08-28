//! QRTL `_enter` — home/rally, climb cone, QLAND-instead-of-RTL.
//!
//! Upstream `ArduPlane/mode_qrtl.cpp` `ModeQRTL::_enter`.

use ap_quadplane::landing::Q_LAND_FINAL_ALT_DEFAULT_M;
use ap_quadplane::mode_qrtl::{
    calc_best_rally_or_home, qrtl_climb_cone_target_alt_m, qrtl_enter, qrtl_min_climb_m,
    qrtl_vtol_return_radius_m, ModeQrtl, QrtlDestination, QrtlEnterAction, QrtlEnterView,
    QrtlSubMode, MODE_QRTL, Q_RTL_ALT_DEFAULT_M, Q_RTL_ALT_MIN_DEFAULT_M, RTL_RADIUS_DEFAULT_M,
    WP_LOITER_RAD_DEFAULT_M,
};
use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

fn dirty_for_mode_enter(qp: &mut QuadPlane) {
    qp.set_lean_angle_max_cd(4500);
    qp.set_guided_wait_takeoff(true);
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    qp.poscontrol_mut().set_correction_ne_m(4.0, -2.0);
}

fn approx(a: f32, b: f32) -> bool {
    let d = a - b;
    d > -0.05 && d < 0.05
}

#[test]
fn qrtl_mode_number_and_predicates_match_upstream() {
    assert_eq!(MODE_QRTL, 21);
    assert_eq!(ModeQrtl::mode_number(), MODE_QRTL);
    assert!(ModeQrtl::is_vtol_mode());
    assert!(!ModeQrtl::is_vtol_man_mode());
    assert!(ModeQrtl::does_auto_throttle());
}

#[test]
fn qrtl_defaults_match_upstream_params() {
    assert!(approx(Q_RTL_ALT_DEFAULT_M, 15.0));
    assert!(approx(Q_RTL_ALT_MIN_DEFAULT_M, 10.0));
    assert!(approx(WP_LOITER_RAD_DEFAULT_M, 60.0));
    assert!(approx(RTL_RADIUS_DEFAULT_M, 0.0));
    assert!(approx(Q_LAND_FINAL_ALT_DEFAULT_M, 6.0));
}

#[test]
fn vtol_return_radius_is_one_and_a_half_times_larger_abs_radius() {
    assert!(approx(
        qrtl_vtol_return_radius_m(WP_LOITER_RAD_DEFAULT_M, RTL_RADIUS_DEFAULT_M),
        90.0
    ));
    assert!(approx(qrtl_vtol_return_radius_m(-80.0, 40.0), 120.0));
    assert!(approx(qrtl_vtol_return_radius_m(30.0, -100.0), 150.0));
}

#[test]
fn min_climb_is_constrained_between_land_final_and_qrtl_alt() {
    assert!(approx(
        qrtl_min_climb_m(10.0, Q_LAND_FINAL_ALT_DEFAULT_M, 15.0),
        10.0
    ));
    assert!(approx(qrtl_min_climb_m(3.0, 6.0, 15.0), 6.0));
    assert!(approx(qrtl_min_climb_m(20.0, 6.0, 15.0), 15.0));
}

#[test]
fn climb_cone_is_full_qrtl_alt_outside_radius() {
    let min_climb = qrtl_min_climb_m(10.0, 6.0, 15.0);
    assert!(approx(
        qrtl_climb_cone_target_alt_m(15.0, 200.0, 90.0, min_climb),
        15.0
    ));
}

#[test]
fn climb_cone_scales_inside_radius_but_not_below_min_climb() {
    let min_climb = qrtl_min_climb_m(10.0, 6.0, 15.0);
    // 15 * (50 / 90) ≈ 8.33, floored at min_climb 10.
    assert!(approx(
        qrtl_climb_cone_target_alt_m(15.0, 50.0, 90.0, min_climb),
        10.0
    ));
    // 15 * (80 / 90) ≈ 13.33, above min_climb.
    assert!(approx(
        qrtl_climb_cone_target_alt_m(15.0, 80.0, 90.0, min_climb),
        15.0 * (80.0 / 90.0)
    ));
}

#[test]
fn home_wins_when_no_rally() {
    let (dest, dist) = calc_best_rally_or_home(200.0, None, true);
    assert_eq!(dest, QrtlDestination::Home);
    assert!(approx(dist, 200.0));
}

#[test]
fn closer_rally_wins_over_home() {
    let (dest, dist) = calc_best_rally_or_home(200.0, Some(80.0), true);
    assert_eq!(dest, QrtlDestination::Rally);
    assert!(approx(dist, 80.0));
}

#[test]
fn farther_rally_loses_when_home_is_included() {
    let (dest, dist) = calc_best_rally_or_home(80.0, Some(200.0), true);
    assert_eq!(dest, QrtlDestination::Home);
    assert!(approx(dist, 80.0));
}

#[test]
fn farther_rally_wins_when_home_is_excluded() {
    let (dest, dist) = calc_best_rally_or_home(80.0, Some(200.0), false);
    assert_eq!(dest, QrtlDestination::Rally);
    assert!(approx(dist, 200.0));
}

#[test]
fn equal_distances_keep_home_when_included() {
    let (dest, dist) = calc_best_rally_or_home(100.0, Some(100.0), true);
    assert_eq!(dest, QrtlDestination::Home);
    assert!(approx(dist, 100.0));
}

#[test]
fn guided_wait_takeoff_enters_qland_instead_of_qrtl() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);

    let result = qrtl_enter(&mut qp, QrtlEnterView::new());

    assert!(result.accepted);
    assert_eq!(result.action, QrtlEnterAction::QLandInstead);
    assert!(!result.do_rtl);
    assert!(!result.position1);
    assert!(!qp.guided_wait_takeoff());
    assert!(qp.guided_wait_takeoff_on_mode_enter());
    assert_eq!(qp.lean_angle_max_cd(), 0);
    assert!(qp.poscontrol().mode_enter_cleared());
}

#[test]
fn vtol_below_cone_far_from_home_climbs_before_return() {
    let mut qp = available_qp();
    let result = qrtl_enter(&mut qp, QrtlEnterView::new());

    assert!(result.accepted);
    assert_eq!(result.action, QrtlEnterAction::Climb);
    assert_eq!(result.submode, QrtlSubMode::Climb);
    assert_eq!(result.dest, QrtlDestination::Home);
    assert!(approx(result.dist_m, 200.0));
    assert!(approx(result.radius_m, 90.0));
    assert!(approx(result.climb_target_alt_m, 15.0));
    assert!(approx(result.dist_to_climb_m, 10.0));
    assert_eq!(result.climb_next_wp_alt_cm, 1500);
    assert_eq!(result.rtl_alt_abs_cm, 1500);
    assert!(!result.do_rtl);
    assert!(!result.poscontrol_init_approach);
    assert!(!result.position1);
    assert_eq!(qp.poscontrol().state(), PositionControlState::None);
}

#[test]
fn vtol_above_cone_far_from_home_does_rtl() {
    let mut qp = available_qp();
    let result = qrtl_enter(&mut qp, QrtlEnterView::far_above_cone());

    assert_eq!(result.action, QrtlEnterAction::Rtl);
    assert_eq!(result.submode, QrtlSubMode::Rtl);
    assert!(result.do_rtl);
    assert!(result.poscontrol_init_approach);
    assert!(!result.position1);
    assert_eq!(result.rtl_alt_abs_cm, 1500);
    assert!(result.slow_descent);
}

#[test]
fn vtol_above_cone_inside_radius_jumps_to_position1() {
    let mut qp = available_qp();
    let result = qrtl_enter(&mut qp, QrtlEnterView::close_above_cone());

    assert_eq!(result.action, QrtlEnterAction::Rtl);
    assert!(result.do_rtl);
    assert!(result.position1);
    assert_eq!(qp.poscontrol().state(), PositionControlState::Position1);
    // Close-in uses MIN(QRTL alt, current abs) = MIN(1500, 1200).
    assert_eq!(result.rtl_alt_abs_cm, 1200);
    assert!(!result.slow_descent);
}

#[test]
fn closer_rally_selects_rally_and_still_climbs() {
    let mut qp = available_qp();
    let mut view = QrtlEnterView::new();
    view.rally_dist_m = Some(80.0);
    let result = qrtl_enter(&mut qp, view);

    assert_eq!(result.dest, QrtlDestination::Rally);
    assert_eq!(result.action, QrtlEnterAction::Climb);
    assert!(approx(result.dist_m, 80.0));
    // Cone at 80/90 of 15 m ≈ 13.33; AGL 5 → still climb.
    assert!(result.dist_to_climb_m > 0.0);
}

#[test]
fn forward_flight_skips_climb_and_does_rtl() {
    let mut qp = available_qp();
    let result = qrtl_enter(&mut qp, QrtlEnterView::forward_flight());

    assert_eq!(result.action, QrtlEnterAction::Rtl);
    assert_eq!(result.submode, QrtlSubMode::Rtl);
    assert!(result.do_rtl);
    assert!(result.poscontrol_init_approach);
    assert!(!result.position1);
    assert_eq!(result.rtl_alt_abs_cm, 1500);
}

#[test]
fn qrtl_enter_calls_mode_enter_when_not_guided_wait() {
    let mut qp = available_qp();
    qp.set_lean_angle_max_cd(4500);
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);

    let result = qrtl_enter(&mut qp, QrtlEnterView::new());

    assert_eq!(result.action, QrtlEnterAction::Climb);
    assert_eq!(qp.lean_angle_max_cd(), 0);
    assert!(qp.poscontrol().mode_enter_cleared());
    assert!(!qp.guided_wait_takeoff_on_mode_enter());
}
