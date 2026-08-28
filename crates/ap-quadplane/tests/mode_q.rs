//! QSTABILIZE / QHOVER / QACRO `_enter` + `run()` — upstream
//! `mode_qstabilize.cpp` / `mode_qhover.cpp` / `mode_qacro.cpp`.

use ap_quadplane::mode_q::{
    qacro_enter, qacro_run, qhover_enter, qhover_run, qstabilize_enter, qstabilize_run,
    QAcroEnterState, QHoverEnterState, QHoverEnterView, QManualMode, QManualRunAction, QManualRunView,
    QManualSpool, MODE_QACRO, MODE_QHOVER, MODE_QSTABILIZE, Q_ACRO_PITCH_RATE_DEFAULT,
    Q_ACRO_ROLL_RATE_DEFAULT, Q_ACRO_YAW_RATE_DEFAULT,
};
use ap_quadplane::poscontrol::{PositionControlState, THROTTLE_WAIT_INPUT_MIN};
use ap_quadplane::transition_fsm::{SltTransition, TransitionState};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

fn dirty_for_mode_enter(qp: &mut QuadPlane) {
    qp.set_lean_angle_max_cd(4500);
    qp.set_throttle_wait(true);
    qp.set_guided_wait_takeoff(true);
    qp.poscontrol_mut()
        .set_state(PositionControlState::Approach);
    qp.poscontrol_mut().set_correction_ne_m(4.0, -2.0);
}

#[test]
fn q_manual_mode_numbers_match_upstream() {
    assert_eq!(MODE_QSTABILIZE, 17);
    assert_eq!(MODE_QHOVER, 18);
    assert_eq!(MODE_QACRO, 23);
    assert_eq!(QManualMode::Stabilize.mode_number(), MODE_QSTABILIZE);
    assert_eq!(QManualMode::Hover.mode_number(), MODE_QHOVER);
    assert_eq!(QManualMode::Acro.mode_number(), MODE_QACRO);
    assert_eq!(
        QManualMode::from_number(17),
        Some(QManualMode::Stabilize)
    );
    assert_eq!(QManualMode::from_number(18), Some(QManualMode::Hover));
    assert_eq!(QManualMode::from_number(23), Some(QManualMode::Acro));
    assert_eq!(QManualMode::from_number(19), None);
}

#[test]
fn q_manual_modes_are_vtol_man_modes() {
    for mode in [
        QManualMode::Stabilize,
        QManualMode::Hover,
        QManualMode::Acro,
    ] {
        assert!(mode.is_vtol_mode());
        assert!(mode.is_vtol_man_mode());
    }
    assert!(QManualMode::Stabilize.is_vtol_man_throttle());
    assert!(!QManualMode::Hover.is_vtol_man_throttle());
    assert!(QManualMode::Acro.is_vtol_man_throttle());
}

#[test]
fn qstabilize_enter_calls_mode_enter_and_clears_throttle_wait() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);

    assert!(qstabilize_enter(&mut qp));

    assert!(!qp.throttle_wait());
    assert_eq!(qp.lean_angle_max_cd(), 0);
    assert!(qp.poscontrol().mode_enter_cleared());
    assert!(!qp.guided_wait_takeoff());
    assert!(qp.guided_wait_takeoff_on_mode_enter());
}

#[test]
fn qhover_enter_parked_idle_sets_throttle_wait() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);
    qp.set_throttle_wait(false);
    let mut state = QHoverEnterState::new();

    assert!(qhover_enter(
        &mut qp,
        QHoverEnterView::parked_idle(),
        &mut state
    ));

    assert!(qp.throttle_wait());
    assert!(state.d_speed_accel_set);
    assert!(state.d_correction_set);
    assert!(state.climb_rate_zeroed);
    assert_eq!(qp.lean_angle_max_cd(), 0);
    assert!(qp.poscontrol().mode_enter_cleared());
}

#[test]
fn qhover_enter_clears_wait_when_stick_or_flying() {
    let mut qp = available_qp();
    qp.set_throttle_wait(true);
    let mut state = QHoverEnterState::new();
    assert!(qhover_enter(
        &mut qp,
        QHoverEnterView::new(THROTTLE_WAIT_INPUT_MIN, false),
        &mut state
    ));
    assert!(!qp.throttle_wait());

    qp.set_throttle_wait(true);
    assert!(qhover_enter(
        &mut qp,
        QHoverEnterView::new(0, true),
        &mut state
    ));
    assert!(!qp.throttle_wait());
}

#[test]
fn qacro_enter_clears_wait_and_forces_transition_complete() {
    let mut qp = available_qp();
    dirty_for_mode_enter(&mut qp);
    let mut transition = SltTransition::new();
    assert_eq!(transition.transition_state(), TransitionState::AirspeedWait);
    let mut state = QAcroEnterState::new();

    assert!(qacro_enter(&mut qp, &mut transition, &mut state));

    assert!(!qp.throttle_wait());
    assert!(transition.complete());
    assert_eq!(transition.transition_state(), TransitionState::Done);
    assert!(!transition.in_forced_transition());
    assert!(state.attitude_relaxed);
    assert!(state.yaw_rate_tc_cleared);
    assert!(state.acro_quat_latched);
    assert_eq!(qp.lean_angle_max_cd(), 0);
    assert!(qp.poscontrol().mode_enter_cleared());
}

#[test]
fn qstabilize_and_qacro_do_not_use_init_throttle_wait() {
    // Parked idle would set throttle_wait if these called
    // init_throttle_wait. They force false instead.
    let mut qp = available_qp();
    qp.set_throttle_wait(true);
    assert!(qstabilize_enter(&mut qp));
    assert!(!qp.throttle_wait());

    qp.set_throttle_wait(true);
    let mut transition = SltTransition::new();
    let mut state = QAcroEnterState::new();
    assert!(qacro_enter(&mut qp, &mut transition, &mut state));
    assert!(!qp.throttle_wait());
}


#[test]
fn qstabilize_run_uses_hold_stabilize() {
    let view = QManualRunView::flying();
    let out = qstabilize_run(&view);
    assert_eq!(out.action, QManualRunAction::HoldStabilize);
    assert!(out.tilt_assigned);
    assert_eq!(out.spool, QManualSpool::Unchanged);
    assert!(!out.d_relaxed);
    assert!(out.acro_rates.is_none());
}

#[test]
fn qstabilize_run_esc_cal_and_tailsitter_fw() {
    let mut esc = QManualRunView::flying();
    esc.esc_calibration = 1;
    let out = qstabilize_run(&esc);
    assert_eq!(out.action, QManualRunAction::EscCalibration);
    assert!(out.tilt_assigned);

    let fw = QManualRunView::tailsitter_fw_transition();
    let out = qstabilize_run(&fw);
    assert_eq!(out.action, QManualRunAction::FwControllers);
    assert!(!out.tilt_assigned);
}

#[test]
fn qhover_run_hold_hover_vs_throttle_wait() {
    let mut qp = available_qp();
    let view = QManualRunView::flying();

    qp.set_throttle_wait(false);
    let flying = qhover_run(&qp, &view);
    assert_eq!(flying.action, QManualRunAction::HoldHover);
    assert!(flying.tilt_assigned);
    assert_eq!(flying.spool, QManualSpool::ThrottleUnlimited);
    assert!(!flying.d_relaxed);

    qp.set_throttle_wait(true);
    let wait = qhover_run(&qp, &view);
    assert_eq!(wait.action, QManualRunAction::ThrottleWait);
    assert!(!wait.tilt_assigned);
    assert_eq!(wait.spool, QManualSpool::GroundIdle);
    assert!(wait.d_relaxed);

    let fw = qhover_run(&qp, &QManualRunView::tailsitter_fw_transition());
    assert_eq!(fw.action, QManualRunAction::FwControllers);
}

#[test]
fn qacro_run_rates_vs_throttle_wait() {
    let mut qp = available_qp();
    let mut view = QManualRunView::flying();
    view.roll_norm = 0.5;
    view.pitch_norm = -0.25;
    view.rudder_norm = 1.0;

    qp.set_throttle_wait(false);
    let flying = qacro_run(&qp, &view);
    assert_eq!(flying.action, QManualRunAction::AcroRates);
    assert!(!flying.tilt_assigned);
    assert_eq!(flying.spool, QManualSpool::ThrottleUnlimited);
    assert!(!flying.d_relaxed);
    let rates = flying.acro_rates.expect("acro rates");
    assert_eq!(rates.roll_cds, 0.5 * Q_ACRO_ROLL_RATE_DEFAULT * 100.0);
    assert_eq!(rates.pitch_cds, -0.25 * Q_ACRO_PITCH_RATE_DEFAULT * 100.0);
    assert_eq!(rates.yaw_cds, 1.0 * Q_ACRO_YAW_RATE_DEFAULT * 100.0);
    assert!(!rates.locking);

    qp.set_throttle_wait(true);
    let wait = qacro_run(&qp, &view);
    assert_eq!(wait.action, QManualRunAction::ThrottleWait);
    assert_eq!(wait.spool, QManualSpool::GroundIdle);
    assert!(!wait.d_relaxed);
    assert!(wait.acro_rates.is_none());
}

#[test]
fn qacro_run_tailsitter_swaps_roll_yaw() {
    let mut qp = available_qp();
    qp.set_throttle_wait(false);
    let mut view = QManualRunView::flying();
    view.tailsitter_enabled = true;
    view.acro_locking = true;
    view.roll_norm = 0.5;
    view.pitch_norm = 0.25;
    view.rudder_norm = -1.0;

    let out = qacro_run(&qp, &view);
    assert_eq!(out.action, QManualRunAction::AcroRates);
    let rates = out.acro_rates.expect("acro rates");
    // tailsitter: roll = rudder * yaw_rate * 100; yaw = -roll * roll_rate * 100
    assert_eq!(rates.roll_cds, -1.0 * Q_ACRO_YAW_RATE_DEFAULT * 100.0);
    assert_eq!(rates.pitch_cds, 0.25 * Q_ACRO_PITCH_RATE_DEFAULT * 100.0);
    assert_eq!(rates.yaw_cds, -0.5 * Q_ACRO_ROLL_RATE_DEFAULT * 100.0);
    assert!(rates.locking);
}
