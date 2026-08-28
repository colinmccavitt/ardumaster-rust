//! Leftover thrust_loss_check / run_esc_calibration / takeoff_failure_scalar stub.

use ap_quadplane::quadplane_completeness::{
    completeness_counts, completeness_has, esc_cal_passthrough, takeoff_failure_scalar_armed,
    takeoff_failure_time_limit_ms, takeoff_failure_timed_out, thrust_loss_already_engaged_or_idle,
    thrust_loss_disabled, thrust_loss_not_descending, thrust_loss_option_is_set,
    thrust_loss_throttle_not_saturated, thrust_loss_throttle_too_low, thrust_loss_tilt_too_steep,
    thrust_loss_vtol_only_skip, PortStatus, ThrustLossOption, THRUST_LOSS_ANGLE_ERROR_DEG,
    THRUST_LOSS_THROTTLE_MIN, THRUST_LOSS_THROTTLE_SAT, THRUST_LOSS_TILT_LIMIT_DEG,
};
use ap_quadplane::thrust_loss::{
    EscCalView, ThrustLossView, Q_ESC_CAL_DEFAULT, Q_THRST_LOSS_OPT_DEFAULT,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn thrust_loss_check_clears_on_reset_and_engages_after_one_second() {
    let mut qp = available_qp();
    assert_eq!(qp.thrust_loss().options(), Q_THRST_LOSS_OPT_DEFAULT);
    assert_eq!(qp.thrust_loss().counter(), 0);

    let cleared = qp.thrust_loss_check(ThrustLossView::inactive());
    assert!(cleared.cleared);
    assert!(!cleared.engaged);
    assert_eq!(cleared.counter, 0);

    let first = qp.thrust_loss_check(ThrustLossView::losing());
    assert!(!first.cleared);
    assert!(!first.engaged);
    assert_eq!(first.counter, 1);
    assert_eq!(qp.thrust_loss().counter(), 1);

    let engaged = qp.thrust_loss_check(ThrustLossView::losing());
    assert!(engaged.engaged);
    assert_eq!(engaged.counter, 0);
    assert_eq!(qp.thrust_loss().counter(), 0);
}

#[test]
fn thrust_loss_check_respects_options_and_reject_gates() {
    let mut qp = available_qp();
    qp.set_thrust_loss_options(ThrustLossOption::Disabled.as_i32());
    assert!(qp.thrust_loss_option_is_set(ThrustLossOption::Disabled));
    assert!(thrust_loss_disabled(qp.thrust_loss().options()));
    let disabled = qp.thrust_loss_check(ThrustLossView::losing());
    assert!(disabled.cleared);
    assert_eq!(qp.thrust_loss().counter(), 0);

    qp.set_thrust_loss_options(ThrustLossOption::VtolOnly.as_i32());
    let mut fw = ThrustLossView::losing();
    fw.in_vtol_mode = false;
    assert!(thrust_loss_vtol_only_skip(
        qp.thrust_loss().options(),
        false
    ));
    assert!(!thrust_loss_vtol_only_skip(
        qp.thrust_loss().options(),
        true
    ));
    let skipped = qp.thrust_loss_check(fw);
    assert!(skipped.cleared);

    qp.set_thrust_loss_options(0);
    let mut idle = ThrustLossView::losing();
    idle.armed = false;
    assert!(thrust_loss_already_engaged_or_idle(
        false, false, true, true
    ));
    assert!(qp.thrust_loss_check(idle).cleared);

    let mut steep = ThrustLossView::losing();
    steep.att_target_xy_rad_len_sq = 1.0;
    assert!(thrust_loss_tilt_too_steep(1.0));
    assert!(!thrust_loss_tilt_too_steep(0.0));
    assert!(qp.thrust_loss_check(steep).cleared);

    let mut low = ThrustLossView::losing();
    low.throttle_in = 0.2;
    low.throttle_upper = true;
    assert!(thrust_loss_throttle_too_low(0.2));
    assert!(!thrust_loss_throttle_too_low(THRUST_LOSS_THROTTLE_MIN));
    assert!(qp.thrust_loss_check(low).cleared);

    let mut unsaturated = ThrustLossView::losing();
    unsaturated.throttle_in = 0.5;
    unsaturated.throttle_upper = false;
    assert!(thrust_loss_throttle_not_saturated(0.5, false));
    assert!(!thrust_loss_throttle_not_saturated(
        THRUST_LOSS_THROTTLE_SAT,
        false
    ));
    assert!(qp.thrust_loss_check(unsaturated).cleared);

    let mut climb = ThrustLossView::losing();
    climb.vel_ned_z = 0.0;
    assert!(thrust_loss_not_descending(true, 0.0));
    assert!(!thrust_loss_not_descending(true, 0.1));
    assert!(qp.thrust_loss_check(climb).cleared);
}

#[test]
fn run_esc_calibration_disarmed_clears_and_modes_set_passthrough() {
    let mut qp = available_qp();
    assert_eq!(qp.esc_calibration(), Q_ESC_CAL_DEFAULT);
    assert!(!qp.esc_cal_notify());

    qp.set_esc_calibration(1);
    let off = qp.run_esc_calibration(EscCalView::disarmed());
    assert!(!off.notify);
    assert_eq!(off.passthrough as i32, 0);
    assert!(!qp.esc_cal_notify());

    let start = qp.run_esc_calibration(EscCalView::armed_mid());
    assert!(start.started);
    assert!(start.notify);
    assert_eq!((start.passthrough * 100.0) as i32, 50);
    assert!(qp.esc_cal_notify());

    let again = qp.run_esc_calibration(EscCalView::armed_mid());
    assert!(!again.started);
    assert_eq!((esc_cal_passthrough(1, true, 50.0) * 100.0) as i32, 50);

    qp.set_esc_calibration(2);
    let full = qp.run_esc_calibration(EscCalView::armed_mid());
    assert_eq!(full.passthrough as i32, 1);
    assert_eq!(esc_cal_passthrough(2, true, 0.0) as i32, 1);
    assert_eq!(esc_cal_passthrough(1, false, 80.0) as i32, 0);

    qp.set_esc_calibration(2);
    assert!(qp.esc_calibration_reset_on_setup_defaults());
    assert_eq!(qp.esc_calibration(), 0);
    assert!(!qp.esc_calibration_reset_on_setup_defaults());
}

#[test]
fn takeoff_failure_scalar_timeout_and_time_limit() {
    let mut qp = available_qp();
    assert!(!takeoff_failure_scalar_armed(qp.takeoff_failure_scalar()));
    assert!(!qp.takeoff_failure_timed_out(10_000));
    assert_eq!(takeoff_failure_time_limit_ms(1.0, 0.0), 5000);
    assert_eq!(takeoff_failure_time_limit_ms(10.0, 1.0), 10_000);
    assert!(takeoff_failure_timed_out(1.0, 5001, 5000));
    assert!(!takeoff_failure_timed_out(0.0, 5001, 5000));
    assert!(!takeoff_failure_timed_out(1.0, 5000, 5000));

    qp.set_takeoff_failure_scalar(1.0);
    assert!(takeoff_failure_scalar_armed(qp.takeoff_failure_scalar()));
    assert!(qp.takeoff_failure_timed_out(1));
}

#[test]
fn catalog_marks_thrust_loss_this_slice_and_leaves_tecs() {
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, 18);
    assert_eq!(this_slice, 1);
    assert_eq!(remaining, 0);
    assert!(completeness_has(
        "guided / QRTL / RTL_MODE",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "thrust-loss / ESC-cal / takeoff-failure",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "TECS / stick-mix / stopping-distance leftovers",
        PortStatus::ThisSlice
    ));
    assert_eq!(THRUST_LOSS_TILT_LIMIT_DEG as i32, 15);
    assert_eq!(THRUST_LOSS_ANGLE_ERROR_DEG as i32, 30);
    assert_eq!((THRUST_LOSS_THROTTLE_MIN * 100.0) as i32, 25);
    assert_eq!((THRUST_LOSS_THROTTLE_SAT * 100.0) as i32, 90);
    assert!(thrust_loss_option_is_set(
        ThrustLossOption::Disabled.as_i32(),
        ThrustLossOption::Disabled
    ));
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
    assert_eq!(qp.esc_calibration(), 0);
    assert!(!qp.takeoff_failure_timed_out(0));
}
