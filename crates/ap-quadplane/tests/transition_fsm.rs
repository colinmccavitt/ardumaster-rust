//! QuadPlane SLT transition FSM — upstream `ArduPlane/transition.h`
//! `SLT_Transition::State` and the AIR / VTOL / TRANSITION phase
//! `get_mav_vtol_state` reports.

use ap_quadplane::air_mode::MavVtolState;
use ap_quadplane::transition_fsm::{SltTransition, TransitionPhase, TransitionState};
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
