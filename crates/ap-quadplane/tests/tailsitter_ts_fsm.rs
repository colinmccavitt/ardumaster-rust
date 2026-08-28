//! Tailsitter_Transition FSM + leftover complete predicates —
//! upstream `transition_fw_complete` / `transition_vtol_complete`
//! (roll-error, 1.5× timeout, disarmed, vectored zero-throttle) and
//! `update` / `VTOL_update` / `show_vtol_view` / `get_mav_vtol_state`.
//!
//! The pitch / throttle *ramp* stays in `tailsitter_transition.rs`.
//! This file is the state machine that calls it.

use ap_quadplane::air_mode::MavVtolState;
use ap_quadplane::tailsitter::{
    roll_error_limit_cd, CompleteReason, Tailsitter, TailsitterConfig, TailsitterTransition,
    TailsitterTransitionState, TransitionCompleteSample, LAST_VTOL_MODE_MS, ROLL_ERROR_FLOOR_CD,
    ROLL_ERROR_MARGIN_CD, TRANSITION_TIMEOUT_SCALE, VTOL_ZERO_GROUNDSPEED_MS, VTOL_ZERO_THROTTLE,
};
use ap_quadplane::transition::{
    PITCH_CD_LIMIT, TRANSITION_ANGLE_FW_DEFAULT, TRANSITION_RATE_FW_DEFAULT,
};

#[test]
fn roll_error_limit_is_max_of_floor_and_limit_plus_margin() {
    assert_eq!(roll_error_limit_cd(4500), 4500 + ROLL_ERROR_MARGIN_CD);
    assert_eq!(roll_error_limit_cd(0), ROLL_ERROR_FLOOR_CD);
    assert_eq!(roll_error_limit_cd(8000), 8500);
}

#[test]
fn new_matches_post_setup_force_complete() {
    let ts = TailsitterTransition::new();
    assert_eq!(ts.state(), TailsitterTransitionState::Done);
    assert!(ts.complete());
    assert!(!ts.active_frwd());
    assert_eq!(ts.get_log_transition_state(), 2);
    assert_eq!(ts.fw_limit_start_ms(), 0);
    assert_eq!(ts.vtol_limit_start_ms(), 0);
}

#[test]
fn disarmed_completes_fw_and_vtol_immediately() {
    let ts = TailsitterTransition::new();
    let mut s = TransitionCompleteSample::armed_level();
    s.armed_and_safety_off = false;
    assert_eq!(
        ts.transition_fw_complete(&s),
        Some(CompleteReason::Disarmed)
    );
    assert_eq!(
        ts.transition_vtol_complete(&s),
        Some(CompleteReason::Disarmed)
    );
}

#[test]
fn fw_pitch_past_angle_completes_equality_does_not() {
    let mut ts = TailsitterTransition::new();
    ts.restart(0, 0.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.pitch_cd = -4500;
    assert_eq!(ts.transition_fw_complete(&s), None);
    s.pitch_cd = -4501;
    assert_eq!(ts.transition_fw_complete(&s), Some(CompleteReason::Pitch));
}

#[test]
fn fw_roll_error_uses_limit_plus_margin() {
    // roll_limit 4500 → threshold 5000. Equality is not enough.
    let mut ts = TailsitterTransition::new();
    ts.restart(0, 0.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.roll_cd = 5000;
    assert_eq!(ts.transition_fw_complete(&s), None);
    s.roll_cd = 5001;
    assert_eq!(
        ts.transition_fw_complete(&s),
        Some(CompleteReason::RollError)
    );
    s.roll_cd = -5001;
    assert_eq!(
        ts.transition_fw_complete(&s),
        Some(CompleteReason::RollError)
    );
}

#[test]
fn fw_timeout_is_one_point_five_times_angle_over_rate() {
    // (45 + 0) / 50 * 1500 = 1350 ms. `>` so 1350 is still waiting.
    let mut ts = TailsitterTransition::new();
    ts.restart(100, 0.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = 100 + 1350;
    assert_eq!(ts.transition_fw_complete(&s), None);
    s.now_ms = 100 + 1351;
    assert_eq!(ts.transition_fw_complete(&s), Some(CompleteReason::Timeout));
}

#[test]
fn fw_timeout_grows_with_initial_pitch() {
    // (45 + 45) / 50 * 1500 = 2700 ms when start pitch is 4500 cd.
    let mut ts = TailsitterTransition::new();
    ts.restart(0, 4500.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = 2700;
    assert_eq!(ts.transition_fw_complete(&s), None);
    s.now_ms = 2701;
    assert_eq!(ts.transition_fw_complete(&s), Some(CompleteReason::Timeout));
}

#[test]
fn vtol_pitch_uses_ang_vt_fallback() {
    let mut ts = TailsitterTransition::new();
    ts.force_transition_complete(0, 0);
    let mut s = TransitionCompleteSample::armed_level();
    s.pitch_cd = 4500;
    assert_eq!(ts.transition_vtol_complete(&s), None);
    s.pitch_cd = 4501;
    assert_eq!(ts.transition_vtol_complete(&s), Some(CompleteReason::Pitch));

    ts.ramp_mut().set_angle_vtol(60);
    s.pitch_cd = 6000;
    assert_eq!(ts.transition_vtol_complete(&s), None);
    s.pitch_cd = 6001;
    assert_eq!(ts.transition_vtol_complete(&s), Some(CompleteReason::Pitch));
}

#[test]
fn vtol_zero_throttle_vectored_and_slow() {
    let ts = TailsitterTransition::new();
    let mut s = TransitionCompleteSample::armed_level();
    s.is_vectored = true;
    s.pilot_throttle = VTOL_ZERO_THROTTLE - 0.001;
    s.groundspeed_ms = VTOL_ZERO_GROUNDSPEED_MS - 0.1;
    assert_eq!(
        ts.transition_vtol_complete(&s),
        Some(CompleteReason::ZeroThrottle)
    );

    s.groundspeed_ms = VTOL_ZERO_GROUNDSPEED_MS;
    assert_eq!(ts.transition_vtol_complete(&s), None);

    s.groundspeed_ms = 0.0;
    s.pilot_throttle = VTOL_ZERO_THROTTLE;
    assert_eq!(ts.transition_vtol_complete(&s), None);

    s.pilot_throttle = 0.0;
    s.is_vectored = false;
    assert_eq!(ts.transition_vtol_complete(&s), None);
}

#[test]
fn vtol_inverted_roll_folds_through_180() {
    // inverted: roll_cd = 18000 - |roll|. Level inverted (18000) → 0, no error.
    let ts = TailsitterTransition::new();
    let mut s = TransitionCompleteSample::armed_level();
    s.fly_inverted = true;
    s.roll_cd = 18000;
    assert_eq!(ts.transition_vtol_complete(&s), None);
    // Upright sensor while inverted-flag: 18000 - 0 = 18000 > 5000.
    s.roll_cd = 0;
    assert_eq!(
        ts.transition_vtol_complete(&s),
        Some(CompleteReason::RollError)
    );
}

#[test]
fn vtol_timeout_is_angle_minus_initial_over_rate() {
    // (45 - 0) / 50 * 1500 = 1350. force_complete stamps vtol start.
    let mut ts = TailsitterTransition::new();
    ts.force_transition_complete(200, 0);
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = 200 + 1350;
    assert_eq!(ts.transition_vtol_complete(&s), None);
    s.now_ms = 200 + 1351;
    assert_eq!(
        ts.transition_vtol_complete(&s),
        Some(CompleteReason::Timeout)
    );
}

#[test]
fn show_vtol_view_hides_vtol_wait_and_keeps_fw_wait() {
    let mut ts = TailsitterTransition::new();
    assert!(!ts.show_vtol_view(false));
    assert!(ts.show_vtol_view(true));

    ts.restart(0, 0.0);
    assert!(ts.show_vtol_view(false));
    assert!(ts.show_vtol_view(true));

    // Enter ANGLE_WAIT_VTOL via a 1 s gap.
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = LAST_VTOL_MODE_MS + 1;
    s.pitch_cd = 0;
    let _ = ts.vtol_update(&s, 0.0);
    assert_eq!(ts.state(), TailsitterTransitionState::AngleWaitVtol);
    assert!(!ts.show_vtol_view(true));
    assert!(!ts.show_vtol_view(false));
}

#[test]
fn mav_vtol_state_matches_upstream_switch() {
    let mut ts = TailsitterTransition::new();
    assert_eq!(ts.get_mav_vtol_state(false), MavVtolState::Fw);
    assert_eq!(ts.get_mav_vtol_state(true), MavVtolState::Fw);

    ts.restart(0, 0.0);
    assert_eq!(ts.get_mav_vtol_state(false), MavVtolState::TransitionToFw);
    assert_eq!(ts.get_mav_vtol_state(true), MavVtolState::Mc);

    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = LAST_VTOL_MODE_MS + 1;
    let _ = ts.vtol_update(&s, 0.0);
    assert_eq!(ts.state(), TailsitterTransitionState::AngleWaitVtol);
    assert_eq!(ts.get_mav_vtol_state(true), MavVtolState::TransitionToMc);
}

#[test]
fn is_in_fw_flight_needs_enabled_fw_and_done() {
    let ts = TailsitterTransition::new();
    assert!(ts.is_in_fw_flight(true, false));
    assert!(!ts.is_in_fw_flight(false, false));
    assert!(!ts.is_in_fw_flight(true, true));

    let sit = Tailsitter::setup(TailsitterConfig::tailsitter_frame());
    assert!(sit.enabled());
    assert!(sit.is_in_fw_flight(false, true));
    assert!(!sit.is_in_fw_flight(true, true));
    assert!(!sit.is_in_fw_flight(false, false));
}

#[test]
fn restart_clamps_initial_pitch_and_enters_fw_wait() {
    let mut ts = TailsitterTransition::new();
    ts.restart(50, 20_000.0);
    assert_eq!(ts.state(), TailsitterTransitionState::AngleWaitFw);
    assert!(ts.active_frwd());
    assert_eq!(ts.fw_transition_start_ms(), 50);
    // 20000 clamps to 8500; timeout uses the clamped value.
    // (45 + 85) / 50 * 1500 ≈ 3900. Stay inside / past that window
    // by a few ms so f32 rounding of `0.01` cannot flip the compare.
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = 50 + 3800;
    assert_eq!(ts.transition_fw_complete(&s), None);
    s.now_ms = 50 + 4000;
    assert_eq!(ts.transition_fw_complete(&s), Some(CompleteReason::Timeout));
}

#[test]
fn force_complete_clears_fw_limit_and_stamps_vtol_start() {
    let mut ts = TailsitterTransition::new();
    ts.restart(0, 0.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = 2000;
    s.pitch_cd = -5000;
    let out = ts.update(&s, false, 0.35, 0.2);
    assert!(out.start_fw_limit);
    assert_eq!(ts.fw_limit_start_ms(), 2000);

    ts.force_transition_complete(3000, PITCH_CD_LIMIT + 1000);
    assert!(ts.complete());
    assert_eq!(ts.fw_limit_start_ms(), 0);
    assert_eq!(ts.vtol_transition_start_ms(), 3000);
}

#[test]
fn update_ramps_pitch_and_holds_hover_max_current() {
    let mut ts = TailsitterTransition::new();
    ts.restart(0, 0.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = 1000;
    let out = ts.update(&s, false, 0.35, 0.20);
    assert!(out.use_synthetic_airspeed);
    assert!(out.assisted_flight);
    assert_eq!(out.nav_pitch_cd, Some(-5000));
    assert_eq!(out.nav_roll_cd, Some(0));
    assert!(out.completed.is_none());
    let thr = out.throttle.expect("waiting throttle");
    assert!((thr - 0.35).abs() < 1e-6);

    let out_cur = ts.update(&s, false, 0.35, 0.70);
    let thr_cur = out_cur.throttle.expect("waiting throttle");
    assert!((thr_cur - 0.70).abs() < 1e-6);
}

#[test]
fn update_inverted_flips_fw_ramp() {
    let mut ts = TailsitterTransition::new();
    ts.restart(0, 0.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = 1000;
    let out = ts.update(&s, true, 0.35, 0.2);
    assert_eq!(out.nav_pitch_cd, Some(5000));
}

#[test]
fn update_complete_uses_entry_synthetic_flag() {
    let mut ts = TailsitterTransition::new();
    ts.restart(0, 0.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = 2000;
    s.pitch_cd = -5000;
    let out = ts.update(&s, false, 0.35, 0.2);
    assert_eq!(out.completed, Some(CompleteReason::Pitch));
    assert!(out.use_synthetic_airspeed);
    assert!(out.start_fw_limit);
    assert_eq!(ts.state(), TailsitterTransitionState::Done);
    assert_eq!(ts.fw_limit_start_ms(), 2000);

    // Next cycle is already DONE — no synthetic, no assist.
    let out2 = ts.update(&s, false, 0.35, 0.2);
    assert!(!out2.use_synthetic_airspeed);
    assert!(!out2.assisted_flight);
    assert!(out2.completed.is_none());
}

#[test]
fn update_disarmed_complete_does_not_start_fw_limit() {
    let mut ts = TailsitterTransition::new();
    ts.restart(0, 0.0);
    let mut s = TransitionCompleteSample::armed_level();
    s.armed_and_safety_off = false;
    s.now_ms = 10;
    let out = ts.update(&s, false, 0.35, 0.2);
    assert_eq!(out.completed, Some(CompleteReason::Disarmed));
    assert!(!out.start_fw_limit);
    assert_eq!(ts.fw_limit_start_ms(), 0);
}

#[test]
fn vtol_update_enters_wait_after_one_second_gap() {
    let mut ts = TailsitterTransition::new();
    let mut s = TransitionCompleteSample::armed_level();

    // First VTOL cycle: last is 0, now 100 — gap is not `> 1000`, so
    // stay DONE then restart() into ANGLE_WAIT_FW.
    s.now_ms = 100;
    let out = ts.vtol_update(&s, 0.0);
    assert!(!out.still_waiting);
    assert_eq!(ts.state(), TailsitterTransitionState::AngleWaitFw);
    assert_eq!(ts.last_vtol_mode_ms(), 100);

    // Equality on the 1 s window does not re-enter the VTOL wait.
    s.now_ms = 100 + LAST_VTOL_MODE_MS;
    let out = ts.vtol_update(&s, 0.0);
    assert!(!out.still_waiting);
    assert_eq!(ts.state(), TailsitterTransitionState::AngleWaitFw);

    // FW restamp so the VTOL timeout is fresh when we come back.
    ts.force_transition_complete(s.now_ms, 0);

    // More than 1 s away from last_vtol_mode_ms → ANGLE_WAIT_VTOL.
    s.now_ms = 100 + LAST_VTOL_MODE_MS + LAST_VTOL_MODE_MS + 1;
    let out = ts.vtol_update(&s, 0.0);
    assert!(out.still_waiting);
    assert!(out.assisted_flight);
    assert_eq!(ts.state(), TailsitterTransitionState::AngleWaitVtol);
    assert_eq!(ts.last_vtol_mode_ms(), s.now_ms);
}

#[test]
fn vtol_update_complete_restarts_fw_and_stamps_vtol_limit() {
    let mut ts = TailsitterTransition::new();
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = LAST_VTOL_MODE_MS + 1;
    s.pitch_cd = 5000;
    let out = ts.vtol_update(&s, 1200.0);
    assert_eq!(out.completed, Some(CompleteReason::Pitch));
    assert!(out.start_vtol_limit);
    assert!(!out.still_waiting);
    assert_eq!(ts.vtol_limit_start_ms(), s.now_ms);
    assert_eq!(ts.state(), TailsitterTransitionState::AngleWaitFw);
    assert_eq!(ts.fw_transition_start_ms(), s.now_ms);
}

#[test]
fn vtol_update_disarmed_complete_skips_vtol_limit() {
    let mut ts = TailsitterTransition::new();
    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = LAST_VTOL_MODE_MS + 1;
    s.armed_and_safety_off = false;
    let out = ts.vtol_update(&s, 0.0);
    assert_eq!(out.completed, Some(CompleteReason::Disarmed));
    assert!(!out.start_vtol_limit);
    assert_eq!(ts.vtol_limit_start_ms(), 0);
    assert_eq!(ts.state(), TailsitterTransitionState::AngleWaitFw);
}

#[test]
fn allow_weathervane_waits_for_vtol_leftover() {
    let mut ts = TailsitterTransition::new();
    assert!(ts.allow_weathervane(false));
    assert!(!ts.allow_weathervane(true));

    let mut s = TransitionCompleteSample::armed_level();
    s.now_ms = LAST_VTOL_MODE_MS + 1;
    s.pitch_cd = 5000;
    let _ = ts.vtol_update(&s, 0.0);
    assert!(!ts.allow_weathervane(false));
}

#[test]
fn set_fw_roll_pitch_raises_nose_in_vtol_transition() {
    let mut ts = TailsitterTransition::new();
    ts.force_transition_complete(0, 0);
    let mut pitch = 0;
    let mut roll = 1500;
    ts.set_fw_roll_pitch(&mut pitch, &mut roll, 1000, true);
    assert_eq!(pitch, 5000);
    assert_eq!(roll, 0);
}

#[test]
fn set_fw_roll_pitch_restamps_when_done() {
    let mut ts = TailsitterTransition::new();
    let mut pitch = 2000;
    let mut roll = 800;
    ts.set_fw_roll_pitch(&mut pitch, &mut roll, 400, false);
    assert_eq!(ts.vtol_transition_start_ms(), 400);
    assert_eq!(pitch, 2000);
    assert_eq!(roll, 800);
}

#[test]
fn timeout_scale_and_angle_defaults_match_upstream() {
    assert!((TRANSITION_TIMEOUT_SCALE - 1500.0).abs() < f32::EPSILON);
    assert_eq!(TRANSITION_ANGLE_FW_DEFAULT, 45);
    assert!((TRANSITION_RATE_FW_DEFAULT - 50.0).abs() < f32::EPSILON);
}
