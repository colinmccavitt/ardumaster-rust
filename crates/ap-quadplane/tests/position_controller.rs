//! Leftover VTOL position / takeoff / waypoint controller stub.

use ap_quadplane::poscontrol::PositionControlState;
use ap_quadplane::position_controller::{
    airbrake_exit_to_position1, approach_stop_distance_m, position1_enters_position2,
    waypoint_refresh_destination, ControllerKind, HeightControl, HorizontalAction,
    PositionControllerView, TakeoffControllerView, TakeoffWait, WaypointControllerView,
    POSITION2_DIST_THRESHOLD_M, POSITION2_TARGET_SPEED_MS,
};
use ap_quadplane::quadplane_completeness::{completeness_counts, completeness_has, PortStatus};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn vtol_position_controller_setup_fail_and_none_to_position1() {
    let mut off = QuadPlane::new();
    let r = off.vtol_position_controller(PositionControllerView::approach_far(100));
    assert_eq!(r.kind, ControllerKind::None);
    assert_eq!(r.state, PositionControlState::None);
    assert!(!off.available());

    let mut qp = available_qp();
    qp.poscontrol_mut().set_state(PositionControlState::None);
    let r = qp.vtol_position_controller(PositionControllerView::approach_far(40));
    assert_eq!(r.kind, ControllerKind::Position);
    assert!(r.last_run_updated);
    assert!(r.flow_of_control);
    assert_eq!(r.horizontal, HorizontalAction::EnterPosition1);
    assert_eq!(r.state, PositionControlState::Position1);
    assert_eq!(qp.position_controllers().last_run_ms(), 40);
    assert!(r.logged_qpos);
}

#[test]
fn vtol_position_controller_approach_airbrake_and_position2() {
    let mut qp = available_qp();
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    let close = PositionControllerView::approach_close(200);
    assert!(
        close.wp_distance_m
            < approach_stop_distance_m(close.stopping_distance_m, close.closing_speed_ms)
    );
    let r = qp.vtol_position_controller(close);
    assert_eq!(r.horizontal, HorizontalAction::EnterAirbrake);
    assert_eq!(r.state, PositionControlState::Airbrake);
    assert!(!r.would_hold_hover);
    assert_eq!(r.height, HeightControl::RelaxZ);

    let mut hold = close;
    hold.now_ms = 201;
    hold.wp_distance_m = 200.0;
    hold.aspeed_ms = 20.0;
    qp.position_controllers_mut().set_state_start_ms(200);
    let r = qp.vtol_position_controller(hold);
    assert_eq!(r.horizontal, HorizontalAction::Hold);
    assert!(r.would_hold_hover);
    assert!(r.suppress_z);

    let mut tailsit = close;
    tailsit.tailsitter_enabled = true;
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    let r = qp.vtol_position_controller(tailsit);
    assert_eq!(r.horizontal, HorizontalAction::EnterPosition1);
    assert_eq!(r.state, PositionControlState::Position1);

    qp.poscontrol_mut()
        .set_state(PositionControlState::Airbrake);
    qp.position_controllers_mut().set_state_start_ms(0);
    let mut air = PositionControllerView::approach_far(2000);
    air.aspeed_ms = 5.0;
    air.aspeed_threshold_ms = 10.0;
    assert!(airbrake_exit_to_position1(
        air.aspeed_ms,
        air.aspeed_threshold_ms,
        air.heading_err_deg,
        air.closing_speed_ms,
        air.desired_closing_speed_ms,
        air.roll_err_cd,
        air.pitch_err_cd
    ));
    let r = qp.vtol_position_controller(air);
    assert_eq!(r.horizontal, HorizontalAction::EnterPosition1);
    assert_eq!(r.state, PositionControlState::Position1);

    qp.poscontrol_mut()
        .set_state(PositionControlState::Position1);
    let mut pos1 = PositionControllerView::approach_far(3000);
    pos1.wp_distance_m = 9.0;
    pos1.rel_groundspeed_sq = 0.0;
    pos1.tilt_angle_achieved = true;
    assert!(position1_enters_position2(
        pos1.wp_distance_m,
        pos1.tilt_angle_achieved,
        pos1.rel_groundspeed_sq
    ));
    let r = qp.vtol_position_controller(pos1);
    assert_eq!(r.horizontal, HorizontalAction::EnterPosition2);
    assert_eq!(r.state, PositionControlState::Position2);
    assert!(!qp.poscontrol().pilot_correction_done());
}

#[test]
fn takeoff_and_waypoint_controller_gates() {
    let mut qp = available_qp();
    let disarmed = qp.takeoff_controller(TakeoffControllerView {
        armed: false,
        ..TakeoffControllerView::climbing(10, 0.0)
    });
    assert_eq!(disarmed.kind, ControllerKind::TakeoffWait);
    assert_eq!(disarmed.wait, TakeoffWait::Disarmed);
    assert!(!disarmed.setup_target);

    qp.set_guided_takeoff(true);
    let tilt = qp.takeoff_controller(TakeoffControllerView {
        now_ms: 50,
        armed: true,
        spool_unlimited: false,
        in_guided: true,
        tiltrotor_enabled: true,
        tiltrotor_fully_up: false,
        motor_check_passed: true,
        rudder_arm_wait: false,
        alt_m: 0.0,
        navalt_min_m: 0.0,
    });
    assert_eq!(tilt.wait, TakeoffWait::Tilt);
    assert_eq!(qp.position_controllers().takeoff_start_time_ms(), 50);

    let climb = qp.takeoff_controller(TakeoffControllerView::climbing(100, 1.0));
    assert_eq!(climb.kind, ControllerKind::Takeoff);
    assert_eq!(climb.wait, TakeoffWait::None);
    assert!(climb.setup_target);
    assert!(!climb.no_navigation);

    let mut nav = TakeoffControllerView::climbing(200, 1.0);
    nav.navalt_min_m = 5.0;
    let below = qp.takeoff_controller(nav);
    assert!(below.no_navigation);

    let first = qp.waypoint_controller(WaypointControllerView {
        now_ms: 10,
        same_loc_as_last: false,
    });
    assert_eq!(first.kind, ControllerKind::Waypoint);
    assert!(first.refreshed_destination);
    assert!(first.setup_target);

    let fresh = qp.waypoint_controller(WaypointControllerView {
        now_ms: 20,
        same_loc_as_last: true,
    });
    assert!(!fresh.refreshed_destination);
    assert!(!waypoint_refresh_destination(true, 20, 10));

    qp.position_controllers_mut().set_last_loiter_ms(0);
    let stale = qp.waypoint_controller(WaypointControllerView {
        now_ms: 501,
        same_loc_as_last: true,
    });
    assert!(stale.refreshed_destination);
}

#[test]
fn catalog_marks_controllers_this_slice_and_leaves_other_rows() {
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, 18);
    assert_eq!(this_slice, 1);
    assert_eq!(remaining, 0);
    assert!(completeness_has(
        "position / takeoff / waypoint controllers",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "assisted-flight latch extras",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "land-sequence predicates",
        PortStatus::OnMain
    ));
    assert_eq!(POSITION2_DIST_THRESHOLD_M as i32, 10);
    assert_eq!(POSITION2_TARGET_SPEED_MS as i32, 3);
    let qp = available_qp();
    assert!(qp.available());
    assert!(!qp.in_assisted_flight());
    assert_eq!(qp.logging().qpos_writes(), 0);
}
