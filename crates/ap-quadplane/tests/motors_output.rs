//! Leftover motors_output / hold_hover / hold_stabilize / set_armed stub.

use ap_quadplane::air_mode::AirMode;
use ap_quadplane::motors_output::{
    DesiredSpoolState, MotorsOutputAction, MotorsOutputView, SetArmedView,
};
use ap_quadplane::poscontrol::THROTTLE_WAIT_INPUT_MIN;
use ap_quadplane::quadplane_completeness::{
    att_control_relax_stale, climb_rate_ms_from_cms, completeness_counts, completeness_has,
    hold_stabilize_ground_idle, hold_stabilize_should_boost, leftover_option_is_set,
    motors_inactive, motors_output_skip_tailsitter_transition, motors_were_active, LeftoverQOption,
    PortStatus, ATT_CONTROL_RELAX_MS, MOTORS_ACTIVE_THROTTLE, MOTORS_INACTIVE_MS,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn set_armed_is_noop_until_setup_then_latches_guided_and_throttle_wait() {
    let mut qp = QuadPlane::with_enable(1);
    qp.set_armed(true, SetArmedView::guided());
    assert!(!qp.motors_armed());
    assert!(!qp.guided_wait_takeoff());
    assert!(!qp.throttle_wait());

    assert!(qp.setup());
    qp.set_armed(true, SetArmedView::guided());
    assert!(qp.motors_armed());
    assert!(qp.guided_wait_takeoff());
    assert!(qp.throttle_wait());

    qp.set_armed(true, SetArmedView::new());
    assert!(qp.motors_armed());
    assert!(qp.guided_wait_takeoff());
    assert!(qp.throttle_wait());

    qp.set_throttle_wait(false);
    qp.set_air_mode(AirMode::On);
    qp.set_armed(false, SetArmedView::new());
    assert!(!qp.motors_armed());
    assert!(!qp.throttle_wait());
}

#[test]
fn set_armed_skips_throttle_wait_in_air_mode_and_clears_guided_wait() {
    let mut qp = available_qp();
    qp.set_air_mode(AirMode::On);
    qp.set_throttle_wait(false);
    qp.set_armed(
        true,
        SetArmedView {
            in_guided: true,
            throttle_input: 0,
            is_flying: false,
        },
    );
    assert!(qp.motors_armed());
    assert!(qp.guided_wait_takeoff());
    assert!(!qp.throttle_wait());

    qp.set_armed(false, SetArmedView::guided());
    assert!(!qp.motors_armed());
    assert!(!qp.guided_wait_takeoff());

    qp.set_air_mode(AirMode::Off);
    qp.set_armed(
        true,
        SetArmedView {
            in_guided: false,
            throttle_input: THROTTLE_WAIT_INPUT_MIN,
            is_flying: false,
        },
    );
    assert!(!qp.throttle_wait());
}

#[test]
fn hold_stabilize_grounds_idle_without_air_mode_and_boosts_unless_tailsitter_assist() {
    let mut qp = available_qp();
    let idle = qp.hold_stabilize(0.0);
    assert_eq!(idle.desired_spool, DesiredSpoolState::GroundIdle);
    assert!(idle.attitude_relaxed);
    assert!(!idle.should_boost);
    assert!(hold_stabilize_ground_idle(0.0, false));
    assert!(!hold_stabilize_ground_idle(0.0, true));

    qp.set_air_mode(AirMode::On);
    let air = qp.hold_stabilize(0.0);
    assert_eq!(air.desired_spool, DesiredSpoolState::ThrottleUnlimited);
    assert!(!air.attitude_relaxed);
    assert!(air.should_boost);

    let flying = qp.hold_stabilize(0.4);
    assert_eq!(flying.desired_spool, DesiredSpoolState::ThrottleUnlimited);
    assert!(!flying.attitude_relaxed);
    assert!(hold_stabilize_should_boost(false, true));
    assert!(!hold_stabilize_should_boost(true, true));
    assert!(hold_stabilize_should_boost(true, false));
}

#[test]
fn hold_hover_sets_unlimited_spool_and_climb_rate() {
    let mut qp = available_qp();
    let hold = qp.hold_hover(250.0);
    assert_eq!(hold.desired_spool, DesiredSpoolState::ThrottleUnlimited);
    assert_eq!(
        qp.motors_output_state().desired_spool(),
        DesiredSpoolState::ThrottleUnlimited
    );
    assert!((hold.climb_rate_ms - climb_rate_ms_from_cms(250.0)).abs() < 0.0001);
    assert!((qp.motors_output_state().climb_rate_cms() - 250.0).abs() < 0.0001);
}

#[test]
fn motors_output_gates_delay_disarmed_esc_tailsitter_then_latches_active() {
    let mut qp = available_qp();
    qp.set_options(LeftoverQOption::DelayArming.as_i32());
    assert!(leftover_option_is_set(
        qp.options(),
        LeftoverQOption::DelayArming
    ));

    let mut view = MotorsOutputView::armed_output(5_000);
    view.arming_delay_active = true;
    let delay = qp.motors_output(view);
    assert_eq!(delay.action, MotorsOutputAction::DelayArming);
    assert_eq!(delay.desired_spool, DesiredSpoolState::ShutDown);
    assert!(delay.motors_output_ran);
    assert!(!delay.rate_controller_ran);

    qp.set_options(0);
    view.arming_delay_active = false;
    view.armed_and_safety_off = false;
    let disarmed = qp.motors_output(view);
    assert_eq!(disarmed.action, MotorsOutputAction::Disarmed);
    assert_eq!(disarmed.desired_spool, DesiredSpoolState::ShutDown);

    view.armed_and_safety_off = true;
    view.esc_calibration_qstabilize = true;
    let esc = qp.motors_output(view);
    assert_eq!(esc.action, MotorsOutputAction::EscCalibration);
    assert!(!esc.motors_output_ran);

    view.esc_calibration_qstabilize = false;
    view.tailsitter_in_vtol_transition = true;
    assert!(motors_output_skip_tailsitter_transition(true, false));
    assert!(!motors_output_skip_tailsitter_transition(true, true));
    let ts = qp.motors_output(view);
    assert_eq!(ts.action, MotorsOutputAction::TailsitterTransition);
    assert!(!ts.motors_output_ran);

    view.tailsitter_in_vtol_transition = false;
    view.now_ms = 10_000;
    view.motors_throttle = 0.5;
    let out = qp.motors_output(view);
    assert_eq!(out.action, MotorsOutputAction::Output);
    assert!(out.motors_output_ran);
    assert!(out.rate_controller_ran);
    assert!(out.attitude_relaxed);
    assert!(out.motors_inactive);
    assert_eq!(qp.motors_output_state().last_motors_active_ms(), 10_000);
    assert_eq!(qp.motors_output_state().last_att_control_ms(), 10_000);

    view.now_ms = 10_050;
    view.motors_throttle = 0.0;
    let quiet = qp.motors_output(view);
    assert!(!quiet.motors_inactive);
    assert!(!quiet.attitude_relaxed);
    assert_eq!(qp.motors_output_state().last_motors_active_ms(), 10_000);
    assert!(motors_inactive(10_000 + MOTORS_INACTIVE_MS + 1, 10_000));
    assert!(!motors_inactive(10_000 + MOTORS_INACTIVE_MS, 10_000));
    assert!(att_control_relax_stale(
        10_000 + ATT_CONTROL_RELAX_MS + 1,
        10_000
    ));
    assert!(motors_were_active(MOTORS_ACTIVE_THROTTLE + 0.001, false));
    assert!(motors_were_active(0.0, true));
    assert!(!motors_were_active(MOTORS_ACTIVE_THROTTLE, false));
}

#[test]
fn catalog_marks_motors_output_this_slice_and_leaves_other_rows() {
    let (on_main, this_slice, remaining) = completeness_counts();
    assert_eq!(on_main, 18);
    assert_eq!(this_slice, 1);
    assert_eq!(remaining, 0);
    assert!(completeness_has(
        "motors_output / hold / set_armed",
        PortStatus::OnMain
    ));
    assert!(completeness_has(
        "land-sequence predicates",
        PortStatus::OnMain
    ));
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
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    assert!(qp.available());
    assert!(!qp.motors_armed());
    assert_eq!(
        qp.motors_output_state().desired_spool(),
        DesiredSpoolState::ShutDown
    );
}
