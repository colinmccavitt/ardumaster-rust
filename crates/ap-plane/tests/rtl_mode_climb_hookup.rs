//! RTL mode hookup for climb-then-home remaining-leg.

use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::rtl_mode_hookup::{rtl_mode_climb_tick, RtlModeClimbInputs};

fn climb_inp() -> RtlModeClimbInputs {
    RtlModeClimbInputs {
        control_mode: ModeNumber::Rtl.as_number(),
        features: BuildFeatures::default(),
        done_climb: false,
        climb_before_turn: true,
        rtl_climb_min_m: 0,
        current_alt_cm: 8_000,
        next_wp_alt_cm: 10_000,
        prev_wp_alt_cm: 8_000,
    }
}

#[test]
fn rtl_mode_climb_holds_roll_until_rtl_alt() {
    let out = rtl_mode_climb_tick(&climb_inp());
    assert!(out.applied);
    assert!(out.climb_gated);
    assert!(!out.done_climb);
    assert!(out.constrain_roll);
    assert!(!out.setup_remaining_leg);
}

#[test]
fn rtl_mode_climb_complete_starts_remaining_home_leg() {
    let mut inp = climb_inp();
    inp.current_alt_cm = 10_100;
    let out = rtl_mode_climb_tick(&inp);
    assert!(out.applied);
    assert!(out.climb_gated);
    assert!(out.done_climb);
    assert!(!out.constrain_roll);
    assert!(out.setup_remaining_leg);
}

#[test]
fn rtl_mode_climb_equal_rtl_alt_is_still_climbing() {
    let mut inp = climb_inp();
    inp.current_alt_cm = inp.next_wp_alt_cm;
    let out = rtl_mode_climb_tick(&inp);
    assert!(out.applied);
    assert!(!out.done_climb);
    assert!(out.constrain_roll);
    assert!(!out.setup_remaining_leg);
}

#[test]
fn rtl_mode_climb_min_holds_until_climbed() {
    let mut inp = climb_inp();
    inp.climb_before_turn = false;
    inp.rtl_climb_min_m = 50;
    inp.prev_wp_alt_cm = 8_000;
    inp.current_alt_cm = 12_999;
    let out = rtl_mode_climb_tick(&inp);
    assert!(out.applied);
    assert!(out.climb_gated);
    assert!(!out.done_climb);
    assert!(out.constrain_roll);
    assert!(!out.setup_remaining_leg);
}

#[test]
fn rtl_mode_climb_min_complete_starts_remaining_home_leg() {
    let mut inp = climb_inp();
    inp.climb_before_turn = false;
    inp.rtl_climb_min_m = 50;
    inp.prev_wp_alt_cm = 8_000;
    inp.current_alt_cm = 13_100;
    let out = rtl_mode_climb_tick(&inp);
    assert!(out.applied);
    assert!(out.done_climb);
    assert!(!out.constrain_roll);
    assert!(out.setup_remaining_leg);
}

#[test]
fn rtl_mode_climb_before_turn_overrides_climb_min() {
    let mut inp = climb_inp();
    inp.climb_before_turn = true;
    inp.rtl_climb_min_m = 10;
    inp.prev_wp_alt_cm = 8_000;
    inp.current_alt_cm = 9_500;
    inp.next_wp_alt_cm = 10_000;
    let out = rtl_mode_climb_tick(&inp);
    assert!(out.applied);
    assert!(out.climb_gated);
    assert!(!out.done_climb);
    assert!(out.constrain_roll);
    assert!(!out.setup_remaining_leg);
}

#[test]
fn rtl_mode_climb_no_gate_without_options() {
    let mut inp = climb_inp();
    inp.climb_before_turn = false;
    inp.rtl_climb_min_m = 0;
    let out = rtl_mode_climb_tick(&inp);
    assert!(out.applied);
    assert!(!out.climb_gated);
    assert!(!out.done_climb);
    assert!(!out.constrain_roll);
    assert!(!out.setup_remaining_leg);
}

#[test]
fn rtl_mode_climb_already_done_does_not_setup_again() {
    let mut inp = climb_inp();
    inp.done_climb = true;
    inp.current_alt_cm = 12_000;
    let out = rtl_mode_climb_tick(&inp);
    assert!(out.applied);
    assert!(out.done_climb);
    assert!(!out.constrain_roll);
    assert!(!out.setup_remaining_leg);
}

#[test]
fn rtl_mode_climb_skips_other_modes() {
    let mut inp = climb_inp();
    inp.control_mode = ModeNumber::Auto.as_number();
    let out = rtl_mode_climb_tick(&inp);
    assert!(!out.applied);
    assert!(!out.climb_gated);
    assert!(!out.done_climb);
    assert!(!out.constrain_roll);
    assert!(!out.setup_remaining_leg);
}

#[test]
fn main_loop_rtl_climb_then_home() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Rtl.as_number();
    vehicle.home_is_set = true;
    vehicle.rtl_climb_before_turn = true;
    vehicle.rtl_current_alt_cm = 8_000;
    vehicle.rtl_next_wp_alt_cm = 10_000;
    vehicle.rtl_prev_wp_alt_cm = 8_000;
    vehicle.rtl_done_climb = true;

    vehicle.update_control_mode();

    assert!(vehicle.rtl_mode_nav_applied);
    assert!(vehicle.rtl_mode_started);
    assert!(vehicle.rtl_mode_climb_applied);
    assert!(vehicle.rtl_climb_gated);
    assert!(!vehicle.rtl_done_climb);
    assert!(vehicle.rtl_climb_constrain_roll);
    assert!(!vehicle.rtl_setup_remaining_leg);

    vehicle.rtl_current_alt_cm = 10_100;
    vehicle.update_control_mode();

    assert!(vehicle.rtl_mode_climb_applied);
    assert!(!vehicle.rtl_mode_started);
    assert!(vehicle.rtl_done_climb);
    assert!(!vehicle.rtl_climb_constrain_roll);
    assert!(vehicle.rtl_setup_remaining_leg);
}

#[test]
fn main_loop_rtl_climb_skips_loiter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Loiter.as_number();
    vehicle.rtl_climb_before_turn = true;
    vehicle.rtl_current_alt_cm = 12_000;
    vehicle.rtl_next_wp_alt_cm = 10_000;

    vehicle.update_control_mode();

    assert!(!vehicle.rtl_mode_climb_applied);
    assert!(!vehicle.rtl_setup_remaining_leg);
    assert!(!vehicle.rtl_climb_constrain_roll);
    assert!(vehicle.loiter_mode_nav_applied);
}
