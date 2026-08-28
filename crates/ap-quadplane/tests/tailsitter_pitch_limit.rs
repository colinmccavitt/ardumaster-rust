//! Tailsitter post-transition pitch-forward / pitch-down leftover —
//! upstream `set_VTOL_roll_pitch_limit` and the `fw_limit_*` half of
//! `set_FW_roll_pitch`.
//!
//! `*_start_ms == 0` is the idle sentinel (same as upstream). Tests
//! stamp a non-zero start so they exercise the leftover, not the
//! idle early-return.

use ap_quadplane::tailsitter::{in_vtol_transition, PitchLimit, LAST_VTOL_MODE_MS};
use ap_quadplane::transition::{
    PITCH_CD_LIMIT, TRANSITION_RATE_FW_DEFAULT, TRANSITION_RATE_VTOL_DEFAULT,
};

#[test]
fn groupinfo_rates_match_upstream() {
    let lim = PitchLimit::new();
    assert!((lim.rate_vtol() - TRANSITION_RATE_VTOL_DEFAULT).abs() < f32::EPSILON);
    assert!((lim.rate_fw() - TRANSITION_RATE_FW_DEFAULT).abs() < f32::EPSILON);
    assert_eq!(lim.vtol_limit_start_ms(), 0);
    assert_eq!(lim.fw_limit_start_ms(), 0);
}

#[test]
fn vtol_limit_inactive_when_not_started() {
    let mut lim = PitchLimit::new();
    let mut roll = 1200;
    let mut pitch = 0;
    assert!(!lim.set_vtol_roll_pitch_limit(&mut roll, &mut pitch, 1_000));
    assert_eq!(roll, 1200);
    assert_eq!(pitch, 0);
}

#[test]
fn vtol_limit_clears_once_change_passes_zero() {
    // 50 deg/s * 1000 ms * 0.1 = 5000 cd > |4000|.
    let mut lim = PitchLimit::new();
    lim.start_vtol(100, 4000.0);
    let mut roll = 0;
    let mut pitch = 0;
    assert!(!lim.set_vtol_roll_pitch_limit(&mut roll, &mut pitch, 1_100));
    assert_eq!(lim.vtol_limit_start_ms(), 0);
    assert_eq!(pitch, 0);
}

#[test]
fn vtol_positive_initial_holds_pitch_up() {
    // start 6000 at t=50, 200 ms later → change 1000, leftover 5000.
    // Demand 2000 is below the leftover, so hold 5000 and zero roll.
    // Stamp stays 50: a successful hold does not clear the leftover.
    let mut lim = PitchLimit::new();
    lim.start_vtol(50, 6000.0);
    let mut roll = 1500;
    let mut pitch = 2000;
    assert!(lim.set_vtol_roll_pitch_limit(&mut roll, &mut pitch, 250));
    assert_eq!(lim.vtol_limit_start_ms(), 50);
    assert_eq!(roll, 0);
    assert_eq!(pitch, 5000);
}

#[test]
fn vtol_positive_initial_clears_when_demand_already_higher() {
    // leftover 5000, demand 5500 is already more nose-up than the leftover.
    let mut lim = PitchLimit::new();
    lim.start_vtol(10, 6000.0);
    let mut roll = 800;
    let mut pitch = 5500;
    assert!(!lim.set_vtol_roll_pitch_limit(&mut roll, &mut pitch, 210));
    assert_eq!(lim.vtol_limit_start_ms(), 0);
    assert_eq!(roll, 800);
    assert_eq!(pitch, 5500);
}

#[test]
fn vtol_negative_initial_holds_pitch_down() {
    // start -6000, 200 ms → leftover -5000.
    // Demand -2000 is above the leftover, so hold -5000.
    let mut lim = PitchLimit::new();
    lim.start_vtol(50, -6000.0);
    let mut roll = 900;
    let mut pitch = -2000;
    assert!(lim.set_vtol_roll_pitch_limit(&mut roll, &mut pitch, 250));
    assert_eq!(lim.vtol_limit_start_ms(), 50);
    assert_eq!(roll, 0);
    assert_eq!(pitch, -5000);
}

#[test]
fn fw_start_clamps_initial_pitch() {
    let mut lim = PitchLimit::new();
    lim.start_fw(100, 20_000.0);
    // leftover = 8500 - 200*50*0.1 = 7500. Demand 0 is below, so hold.
    let mut roll = 400;
    let mut pitch = 0;
    assert!(lim.apply_fw_pitch_down_limit(&mut roll, &mut pitch, 300));
    assert_eq!(lim.fw_limit_start_ms(), 100);
    assert_eq!(roll, 0);
    assert_eq!(pitch, PITCH_CD_LIMIT - 1000);
}

#[test]
fn fw_limit_clears_at_or_past_zero() {
    let mut lim = PitchLimit::new();
    lim.start_fw(100, 4500.0);
    // 1000 ms → leftover 4500 - 5000 = -500 <= 0, clear.
    let mut roll = 100;
    let mut pitch = 2000;
    assert!(!lim.apply_fw_pitch_down_limit(&mut roll, &mut pitch, 1_100));
    assert_eq!(lim.fw_limit_start_ms(), 0);
    assert_eq!(roll, 100);
    assert_eq!(pitch, 2000);
}

#[test]
fn fw_limit_never_holds_a_smaller_pitch_than_demand() {
    // leftover 3500, demand 4000 is already higher → clear, do not pull down.
    let mut lim = PitchLimit::new();
    lim.start_fw(100, 4500.0);
    let mut roll = 250;
    let mut pitch = 4000;
    assert!(!lim.apply_fw_pitch_down_limit(&mut roll, &mut pitch, 300));
    assert_eq!(lim.fw_limit_start_ms(), 0);
    assert_eq!(roll, 250);
    assert_eq!(pitch, 4000);
}

#[test]
fn fw_limit_holds_when_demand_is_below_leftover() {
    let mut lim = PitchLimit::new();
    lim.start_fw(100, 4500.0);
    let mut roll = 250;
    let mut pitch = 2000;
    assert!(lim.apply_fw_pitch_down_limit(&mut roll, &mut pitch, 300));
    assert_eq!(lim.fw_limit_start_ms(), 100);
    assert_eq!(roll, 0);
    assert_eq!(pitch, 3500);
}

#[test]
fn allow_stick_mixing_blocks_vtol_pitch_up() {
    let lim = PitchLimit::new();
    assert!(!lim.allow_stick_mixing(true, false));
    assert!(lim.allow_stick_mixing(false, true));
}

#[test]
fn allow_stick_mixing_blocks_fw_level_off() {
    let mut lim = PitchLimit::new();
    lim.start_fw(10, 4000.0);
    assert!(!lim.allow_stick_mixing(false, true));
    assert!(lim.allow_stick_mixing(false, false));
}

#[test]
fn in_vtol_transition_requires_enable_and_vtol_mode() {
    assert!(!in_vtol_transition(false, true, true, 0, 0));
    assert!(!in_vtol_transition(true, false, true, 0, 0));
    assert!(in_vtol_transition(true, true, true, 0, 0));
}

#[test]
fn in_vtol_transition_now_zero_skips_last_mode_window() {
    // now == 0 is the allow_stick_mixing call; the 1 s window is ignored.
    assert!(!in_vtol_transition(true, true, false, 0, 0));
}

#[test]
fn in_vtol_transition_window_after_leaving_fw() {
    assert_eq!(LAST_VTOL_MODE_MS, 1000);
    // last VTOL mode was 1001 ms ago → still treated as in transition.
    assert!(in_vtol_transition(true, true, false, 2001, 1000));
    // 1000 ms exactly is not enough (`>` not `>=`).
    assert!(!in_vtol_transition(true, true, false, 2000, 1000));
}
