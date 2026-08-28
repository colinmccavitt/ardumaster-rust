//! QSTABILIZE / QHOVER / QACRO `_enter` — upstream
//! `mode_qstabilize.cpp` / `mode_qhover.cpp` / `mode_qacro.cpp`.

use ap_quadplane::mode_q::{
    qacro_enter, qhover_enter, qstabilize_enter, QAcroEnterState, QHoverEnterState, QHoverEnterView,
    QManualMode, MODE_QACRO, MODE_QHOVER, MODE_QSTABILIZE,
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
