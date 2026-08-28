//! QRTL `_enter` plus `run()` climb-then-return.
//!
//! Upstream `ArduPlane/mode_qrtl.cpp` `ModeQRTL::_enter` / `run`.

use ap_quadplane::landing::Q_LAND_FINAL_ALT_DEFAULT_M;
use ap_quadplane::auto_vtol::VerifyLandView;
use ap_quadplane::mode_qrtl::{
    calc_best_rally_or_home, mode_qrtl_surfaces_complete, qrtl_allows_throttle_nudging,
    qrtl_climb_cone_target_alt_m, qrtl_climb_finished, qrtl_copy_home_alt, qrtl_enter,
    qrtl_land_handoff, qrtl_min_climb_m, qrtl_run, qrtl_should_verify_land, qrtl_stick_mixing_fbw,
    qrtl_update, qrtl_update_target_altitude, qrtl_vtol_return_radius_m, ModeQrtl,
    QrtlDestination, QrtlEnterAction, QrtlEnterView, QrtlPortStatus, QrtlRunAction, QrtlRunView,
    QrtlSubMode, QrtlTargetAltView, MODE_QRTL, MODE_QRTL_CPP_SURFACES, Q_RTL_ALT_DEFAULT_M,
    Q_RTL_ALT_MIN_DEFAULT_M, Q_WP_SPD_UP_DEFAULT_MS, RTL_ALTITUDE_DEFAULT_M, RTL_RADIUS_DEFAULT_M,
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
    assert!(!ModeQrtl::pre_arm_checks());
}

#[test]
fn qrtl_defaults_match_upstream_params() {
    assert!(approx(Q_RTL_ALT_DEFAULT_M, 15.0));
    assert!(approx(Q_RTL_ALT_MIN_DEFAULT_M, 10.0));
    assert!(approx(WP_LOITER_RAD_DEFAULT_M, 60.0));
    assert!(approx(RTL_RADIUS_DEFAULT_M, 0.0));
    assert!(approx(Q_LAND_FINAL_ALT_DEFAULT_M, 6.0));
    assert!(approx(Q_WP_SPD_UP_DEFAULT_MS, 2.5));
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

#[test]
fn climb_finished_when_stopping_point_is_above_or_unreadable() {
    assert!(!qrtl_climb_finished(Some(-5.0)));
    assert!(!qrtl_climb_finished(Some(0.0)));
    assert!(qrtl_climb_finished(Some(0.1)));
    assert!(qrtl_climb_finished(None));
}

#[test]
fn qrtl_run_climb_holds_xy_and_climbs_at_wp_speed_up() {
    let mut qp = available_qp();
    let out = qrtl_run(&mut qp, QrtlRunView::climbing());

    assert_eq!(out.action, QrtlRunAction::Climb);
    assert_eq!(out.submode, QrtlSubMode::Climb);
    assert!(approx(out.climb_rate_ms, Q_WP_SPD_UP_DEFAULT_MS));
    assert!(out.xy_hold);
    assert!(out.tilt_assigned);
    assert!(out.weathervane);
    assert!(out.z_controller);
    assert!(out.fw_stabilize);
    assert!(!out.ne_externally_limited);
    assert!(!out.do_rtl);
    assert!(!out.poscontrol_init_approach);
    assert!(!out.position1);
    assert!(!out.vtol_position_controller);
    assert_eq!(qp.poscontrol().state(), PositionControlState::None);
}

#[test]
fn qrtl_run_climb_then_return_far_from_home() {
    let mut qp = available_qp();
    let out = qrtl_run(&mut qp, QrtlRunView::climb_done_far());

    assert_eq!(out.action, QrtlRunAction::ClimbThenReturn);
    assert_eq!(out.submode, QrtlSubMode::Rtl);
    assert_eq!(out.dest, QrtlDestination::Home);
    assert!(approx(out.dist_m, 200.0));
    assert!(approx(out.radius_m, 90.0));
    assert!(approx(out.climb_rate_ms, Q_WP_SPD_UP_DEFAULT_MS));
    assert!(out.xy_hold);
    assert!(out.do_rtl);
    assert!(out.poscontrol_init_approach);
    assert!(!out.position1);
    assert!(!out.vtol_position_controller);
    assert!(out.fw_stabilize);
    assert_eq!(out.rtl_alt_abs_cm, 1500);
    assert!(!out.slow_descent);
    assert_eq!(qp.poscontrol().state(), PositionControlState::None);
}

#[test]
fn qrtl_run_climb_then_return_close_in_jumps_to_position1() {
    let mut qp = available_qp();
    let out = qrtl_run(&mut qp, QrtlRunView::climb_done_close());

    assert_eq!(out.action, QrtlRunAction::ClimbThenReturn);
    assert_eq!(out.submode, QrtlSubMode::Rtl);
    assert!(out.do_rtl);
    assert!(out.position1);
    assert_eq!(qp.poscontrol().state(), PositionControlState::Position1);
    // Close-in uses MIN(QRTL alt, climb WP abs) = MIN(1500, 1200).
    assert_eq!(out.rtl_alt_abs_cm, 1200);
}

#[test]
fn qrtl_run_climb_done_with_failed_height_lookup_heads_home() {
    let mut qp = available_qp();
    let mut view = QrtlRunView::climbing();
    view.stopping_height_above_next_wp_m = None;
    let out = qrtl_run(&mut qp, view);

    assert_eq!(out.action, QrtlRunAction::ClimbThenReturn);
    assert_eq!(out.submode, QrtlSubMode::Rtl);
    assert!(out.do_rtl);
}

#[test]
fn qrtl_run_already_returning_uses_vtol_position_controller() {
    let mut qp = available_qp();
    let out = qrtl_run(&mut qp, QrtlRunView::returning());

    assert_eq!(out.action, QrtlRunAction::Return);
    assert_eq!(out.submode, QrtlSubMode::Rtl);
    assert!(out.vtol_position_controller);
    assert!(out.fw_stabilize);
    assert!(!out.xy_hold);
    assert!(!out.do_rtl);
    assert!(!out.z_controller);
    assert!(!out.position1);
    assert!(!out.copy_home_alt);
    assert!(!out.verify_vtol_land);
    assert!(!out.stick_mixing_fbw);
}

#[test]
fn qrtl_run_tailsitter_fw_pullup_skips_climb() {
    let mut qp = available_qp();
    let out = qrtl_run(&mut qp, QrtlRunView::tailsitter_fw_transition());

    assert_eq!(out.action, QrtlRunAction::FwControllers);
    assert!(!out.fw_stabilize);
    assert!(!out.xy_hold);
    assert!(!out.do_rtl);
    assert!(!out.vtol_position_controller);
}

#[test]
fn qrtl_run_climb_marks_ne_limited_when_vtol_attitude_limited() {
    let mut qp = available_qp();
    let mut view = QrtlRunView::climbing();
    view.vtol_roll_pitch_limited = true;
    let out = qrtl_run(&mut qp, view);

    assert_eq!(out.action, QrtlRunAction::Climb);
    assert!(out.ne_externally_limited);
}

#[test]
fn qrtl_run_climb_then_return_uses_closer_rally_for_radius() {
    let mut qp = available_qp();
    let mut view = QrtlRunView::climb_done_far();
    view.rally_dist_m = Some(40.0);
    let out = qrtl_run(&mut qp, view);

    assert_eq!(out.dest, QrtlDestination::Rally);
    assert!(approx(out.dist_m, 40.0));
    assert!(out.position1);
    assert_eq!(qp.poscontrol().state(), PositionControlState::Position1);
}


#[test]
fn qrtl_land_handoff_starts_verify_at_position2() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::Position2);
    let land = VerifyLandView::hover_over(1_000, 1500, 0.5);
    let out = qrtl_land_handoff(&mut qp, Some(land));

    assert!(!qrtl_copy_home_alt(PositionControlState::Position2));
    assert!(qrtl_should_verify_land(PositionControlState::Position2));
    assert!(!out.copy_home_alt);
    assert!(out.verify_vtol_land);
    assert!(!out.stick_mixing_fbw);
    assert!(out.land.entered_descend);
    assert_eq!(qp.poscontrol().state(), PositionControlState::LandDescend);
}

#[test]
fn qrtl_land_handoff_past_position2_copies_home_alt() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::LandDescend);
    let out = qrtl_land_handoff(&mut qp, None);

    assert!(qrtl_copy_home_alt(PositionControlState::LandDescend));
    assert!(out.copy_home_alt);
    assert!(out.verify_vtol_land);
    assert!(!out.stick_mixing_fbw);
}

#[test]
fn qrtl_run_rtl_approach_mixes_sticks_and_does_not_land() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    let out = qrtl_run(&mut qp, QrtlRunView::returning());

    assert_eq!(out.action, QrtlRunAction::Return);
    assert!(out.vtol_position_controller);
    assert!(out.stick_mixing_fbw);
    assert!(!out.copy_home_alt);
    assert!(!out.verify_vtol_land);
    assert!(qrtl_stick_mixing_fbw(PositionControlState::Approach));
    assert!(qrtl_allows_throttle_nudging(
        QrtlSubMode::Rtl,
        PositionControlState::Approach
    ));
    assert!(!qrtl_allows_throttle_nudging(
        QrtlSubMode::Climb,
        PositionControlState::Approach
    ));
    assert!(!qrtl_allows_throttle_nudging(
        QrtlSubMode::Rtl,
        PositionControlState::Position1
    ));
}

#[test]
fn qrtl_update_delegates_to_qstabilize() {
    let out = qrtl_update();
    assert!(out.used_qstabilize);
}

#[test]
fn qrtl_target_altitude_uses_base_mode_outside_approach() {
    let out = qrtl_update_target_altitude(QrtlTargetAltView::not_approach());
    assert!(out.used_base_mode);
    assert!(approx(out.offset_up_m, 0.0));
}

#[test]
fn qrtl_target_altitude_holds_rtl_delta_when_far() {
    let out = qrtl_update_target_altitude(QrtlTargetAltView::approach_far());
    assert!(!out.used_base_mode);
    // RTL_ALTITUDE 100 - Q_RTL_ALT 15 = 85 m, still outside rad_max.
    assert!(approx(out.offset_up_m, RTL_ALTITUDE_DEFAULT_M - Q_RTL_ALT_DEFAULT_M));
}

#[test]
fn mode_qrtl_cpp_surfaces_are_complete() {
    assert!(mode_qrtl_surfaces_complete());
    assert_eq!(MODE_QRTL_CPP_SURFACES.len(), 7);
    let names: [&str; 7] = [
        "_enter",
        "update",
        "run",
        "run land handoff",
        "update_target_altitude",
        "allows_throttle_nudging",
        "get_VTOL_return_radius",
    ];
    for (i, row) in MODE_QRTL_CPP_SURFACES.iter().enumerate() {
        assert_eq!(row.name, names[i]);
        assert!(
            row.status == QrtlPortStatus::OnMain || row.status == QrtlPortStatus::ThisSlice
        );
    }
    assert_eq!(MODE_QRTL_CPP_SURFACES[0].status, QrtlPortStatus::OnMain);
    assert_eq!(MODE_QRTL_CPP_SURFACES[1].status, QrtlPortStatus::ThisSlice);
    assert_eq!(MODE_QRTL_CPP_SURFACES[3].status, QrtlPortStatus::ThisSlice);
    assert_eq!(MODE_QRTL_CPP_SURFACES[6].status, QrtlPortStatus::OnMain);
}
