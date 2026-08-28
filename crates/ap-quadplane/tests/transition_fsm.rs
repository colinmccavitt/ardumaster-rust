//! QuadPlane SLT transition FSM — upstream `ArduPlane/transition.h`
//! `SLT_Transition::State` and the AIR / VTOL / TRANSITION phase
//! `get_mav_vtol_state` reports.

use ap_quadplane::air_mode::MavVtolState;
use ap_quadplane::transition_fsm::{
    back_transition_time_s, constrain_transition_time_ms, stopping_distance_m, SltTransition,
    TransitionPhase, TransitionState, Q_TRANS_DECEL_DEFAULT, Q_TRANSITION_MS_DEFAULT,
    Q_TRANSITION_MS_MAX, Q_TRANSITION_MS_MIN,
};
use ap_quadplane::QuadPlane;

fn available_qp() -> QuadPlane {
    let mut qp = QuadPlane::with_enable(1);
    assert!(qp.setup());
    qp
}

#[test]
fn slt_state_discriminants_match_upstream() {
    assert_eq!(TransitionState::AirspeedWait as u8, 0);
    assert_eq!(TransitionState::Timer as u8, 1);
    assert_eq!(TransitionState::Done as u8, 2);
}

#[test]
fn new_zero_inits_to_airspeed_wait() {
    let fsm = SltTransition::new();
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    assert_eq!(fsm.get_log_transition_state(), 0);
    assert!(!fsm.complete());
    assert!(fsm.in_transition());
    assert!(!fsm.in_forced_transition());
    assert_eq!(fsm.transition_start_ms(), 0);
    assert_eq!(fsm.transition_low_airspeed_ms(), 0);
    assert_eq!(fsm.transition_time_ms(), Q_TRANSITION_MS_DEFAULT);
    assert_eq!(fsm.transition_decel_mss(), Q_TRANS_DECEL_DEFAULT);
}

#[test]
fn complete_is_only_done() {
    let mut fsm = SltTransition::new();
    assert!(!fsm.complete());
    fsm.enter_timer();
    assert!(!fsm.complete());
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    assert_eq!(fsm.get_log_transition_state(), 1);
    assert!(fsm.in_transition());
    fsm.force_transition_complete();
    assert!(fsm.complete());
    assert!(!fsm.in_transition());
    assert_eq!(fsm.get_log_transition_state(), 2);
}

#[test]
fn restart_returns_to_airspeed_wait() {
    let mut fsm = SltTransition::new();
    fsm.force_transition_complete();
    fsm.restart();
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    assert!(fsm.in_transition());
    assert!(!fsm.complete());
}

#[test]
fn force_transition_complete_clears_timers_and_forced_latch() {
    let mut fsm = SltTransition::new();
    fsm.enter_timer();
    fsm.force_transition_complete();
    assert_eq!(fsm.transition_state(), TransitionState::Done);
    assert!(!fsm.in_forced_transition());
    assert_eq!(fsm.transition_start_ms(), 0);
    assert_eq!(fsm.transition_low_airspeed_ms(), 0);
}

#[test]
fn vtol_update_parked_goes_done() {
    let mut fsm = SltTransition::new();
    fsm.vtol_update(true, false);
    assert_eq!(fsm.transition_state(), TransitionState::Done);
    assert!(fsm.complete());
    assert!(!fsm.in_forced_transition());
    assert_eq!(fsm.transition_start_ms(), 0);
    assert_eq!(fsm.transition_low_airspeed_ms(), 0);
}

#[test]
fn vtol_update_flying_arms_airspeed_wait() {
    let mut fsm = SltTransition::new();
    fsm.force_transition_complete();
    fsm.vtol_update(false, true);
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    assert!(fsm.in_transition());
}

#[test]
fn phase_is_air_vtol_or_transition() {
    let mut fsm = SltTransition::new();
    assert_eq!(fsm.phase(true), TransitionPhase::Vtol);
    assert_eq!(fsm.phase(false), TransitionPhase::Transition);
    fsm.enter_timer();
    assert_eq!(fsm.phase(false), TransitionPhase::Transition);
    fsm.force_transition_complete();
    assert_eq!(fsm.phase(false), TransitionPhase::Air);
    assert_eq!(fsm.phase(true), TransitionPhase::Vtol);
}

#[test]
fn get_mav_vtol_state_maps_phase() {
    let mut fsm = SltTransition::new();
    assert_eq!(fsm.get_mav_vtol_state(true), MavVtolState::Mc);
    assert_eq!(fsm.get_mav_vtol_state(false), MavVtolState::TransitionToFw);
    fsm.enter_timer();
    assert_eq!(fsm.get_mav_vtol_state(false), MavVtolState::TransitionToFw);
    fsm.force_transition_complete();
    assert_eq!(fsm.get_mav_vtol_state(false), MavVtolState::Fw);
    assert_eq!(fsm.get_mav_vtol_state(true), MavVtolState::Mc);
}

#[test]
fn active_frwd_needs_assist_and_open_slt_and_no_airbrake() {
    let mut fsm = SltTransition::new();
    assert!(!fsm.active_frwd(false, false));
    assert!(fsm.active_frwd(true, false));
    assert!(!fsm.active_frwd(true, true));
    fsm.enter_timer();
    assert!(fsm.active_frwd(true, false));
    fsm.force_transition_complete();
    assert!(!fsm.active_frwd(true, false));
}

#[test]
fn quadplane_in_transition_needs_available() {
    let fsm = SltTransition::new();
    let qp = QuadPlane::with_enable(1);
    assert!(!qp.available());
    assert!(!qp.in_transition(&fsm));
    let qp = available_qp();
    assert!(qp.in_transition(&fsm));
    let mut done = SltTransition::new();
    done.force_transition_complete();
    assert!(!qp.in_transition(&done));
}

#[test]
fn q_transition_ms_default_and_constrain() {
    assert_eq!(Q_TRANSITION_MS_DEFAULT, 5000);
    assert_eq!(Q_TRANSITION_MS_MIN, 500);
    assert_eq!(Q_TRANSITION_MS_MAX, 30000);
    assert_eq!(constrain_transition_time_ms(5000), 5000);
    assert_eq!(constrain_transition_time_ms(500), 500);
    assert_eq!(constrain_transition_time_ms(30000), 30000);
    assert_eq!(constrain_transition_time_ms(100), 500);
    assert_eq!(constrain_transition_time_ms(-1), 500);
    assert_eq!(constrain_transition_time_ms(i16::MAX), 30000);
    let mut fsm = SltTransition::new();
    assert_eq!(fsm.timer_duration_ms(), 5000);
    fsm.set_transition_time_ms(100);
    assert_eq!(fsm.timer_duration_ms(), 500);
}

#[test]
fn airspeed_wait_lasts_until_airspeed_not_q_transition_ms() {
    let mut fsm = SltTransition::new();
    fsm.update_airspeed_wait(1, false, 0.0, 10.0, false);
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    assert_eq!(fsm.transition_start_ms(), 1);
    // Well past Q_TRANSITION_MS with no airspeed: still waiting.
    fsm.update_airspeed_wait(1 + 5_000 + 5_000, false, 0.0, 10.0, false);
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    assert_eq!(fsm.transition_start_ms(), 1);
    fsm.update_airspeed_wait(20_000, true, 9.0, 10.0, false);
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    fsm.update_airspeed_wait(21_000, true, 10.0, 10.0, false);
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    fsm.update_airspeed_wait(22_000, true, 12.0, 10.0, true);
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    fsm.update_airspeed_wait(23_000, true, 12.0, 10.0, false);
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    assert_eq!(fsm.transition_low_airspeed_ms(), 23_000);
}

#[test]
fn timer_completes_after_constrained_q_transition_ms() {
    let mut fsm = SltTransition::new();
    fsm.update_airspeed_wait(1_000, true, 12.0, 10.0, false);
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    assert_eq!(fsm.transition_low_airspeed_ms(), 1_000);
    // Strict `>` — equal to the dwell is still TIMER.
    fsm.update_timer(1_000 + fsm.timer_duration_ms(), true);
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    fsm.update_timer(1_000 + fsm.timer_duration_ms() + 1, false);
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    fsm.update_timer(1_000 + fsm.timer_duration_ms() + 1, true);
    assert!(fsm.complete());
    assert_eq!(fsm.transition_start_ms(), 0);
    assert_eq!(fsm.transition_low_airspeed_ms(), 0);
}

#[test]
fn custom_q_transition_ms_and_assist_back() {
    let mut fsm = SltTransition::new();
    fsm.set_transition_time_ms(1000);
    fsm.update_forward_timing(100, true, 20.0, 10.0, false, true);
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    fsm.update_forward_timing(1_100, true, 20.0, 10.0, false, true);
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    fsm.update_forward_timing(1_101, true, 8.0, 10.0, true, true);
    assert_eq!(fsm.transition_state(), TransitionState::AirspeedWait);
    fsm.update_forward_timing(1_200, true, 20.0, 10.0, false, true);
    assert_eq!(fsm.transition_state(), TransitionState::Timer);
    fsm.update_forward_timing(2_201, true, 20.0, 10.0, false, true);
    assert!(fsm.complete());
}

#[test]
fn q_trans_decel_stopping_distance_and_back_time() {
    assert_eq!(Q_TRANS_DECEL_DEFAULT, 2.0);
    let fsm = SltTransition::new();
    assert_eq!(fsm.transition_decel_mss(), 2.0);
    // v = 10 m/s → v² = 100 → 100 / (2 * 2) = 25 m; t = 10 / 2 = 5 s.
    assert_eq!(fsm.stopping_distance_m(100.0), 25.0);
    assert_eq!(fsm.back_transition_time_s(10.0), 5.0);
    assert_eq!(stopping_distance_m(100.0, 2.0), 25.0);
    assert_eq!(back_transition_time_s(10.0, 2.0), 5.0);
    let mut fsm = SltTransition::new();
    fsm.set_transition_decel_mss(4.0);
    assert_eq!(fsm.stopping_distance_m(100.0), 12.5);
    assert_eq!(fsm.back_transition_time_s(10.0), 2.5);
}
