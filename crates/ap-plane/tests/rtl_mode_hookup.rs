//! RTL mode hookup for home-loiter navigate.

use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::rtl_mode_hookup::{rtl_mode_nav_tick, RtlModeNavInputs};

fn rtl_inp() -> RtlModeNavInputs {
    RtlModeNavInputs {
        control_mode: ModeNumber::Rtl.as_number(),
        features: BuildFeatures::default(),
        mode_just_entered: true,
        home_is_set: true,
        rtl_radius_m: 200,
    }
}

#[test]
fn rtl_mode_nav_starts_on_enter() {
    let out = rtl_mode_nav_tick(&rtl_inp());
    assert!(out.applied);
    assert!(out.started);
    assert!(out.allow_loiter);
    assert!(out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 200);
}

#[test]
fn rtl_mode_nav_resumes_when_already_in_rtl() {
    let mut inp = rtl_inp();
    inp.mode_just_entered = false;
    let out = rtl_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.allow_loiter);
}

#[test]
fn rtl_mode_nav_skips_other_modes() {
    let mut inp = rtl_inp();
    inp.control_mode = ModeNumber::Auto.as_number();
    let out = rtl_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.allow_loiter);
    assert!(!out.direction_set);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn rtl_mode_nav_blocks_loiter_without_home() {
    let mut inp = rtl_inp();
    inp.home_is_set = false;
    let out = rtl_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.started);
    assert!(!out.allow_loiter);
}

#[test]
fn rtl_mode_nav_decodes_ccw_radius() {
    let mut inp = rtl_inp();
    inp.rtl_radius_m = -150;
    let out = rtl_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.direction_set);
    assert!(out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 150);
}

#[test]
fn rtl_mode_nav_zero_radius_leaves_direction() {
    let mut inp = rtl_inp();
    inp.rtl_radius_m = 0;
    let out = rtl_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn main_loop_starts_rtl_loiter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Rtl.as_number();
    vehicle.home_is_set = true;
    vehicle.rtl_radius_m = -180;

    vehicle.update_control_mode();

    assert!(vehicle.rtl_mode_nav_applied);
    assert!(vehicle.rtl_mode_started);
    assert!(vehicle.rtl_mode_loiter_allowed);
    assert!(vehicle.rtl_loiter_ccw);
    assert_eq!(vehicle.rtl_loiter_radius_m, 180);
    assert!(!vehicle.auto_mode_mission_applied);
    assert!(!vehicle.auto_mode_mission_started);
    assert!(!vehicle.circle_mode_nav_applied);
    assert!(!vehicle.thermal_mode_nav_applied);
}

#[test]
fn main_loop_blocks_rtl_loiter_without_home() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Rtl.as_number();
    vehicle.home_is_set = false;
    vehicle.tracked_control_mode = ModeNumber::Rtl.as_number();
    vehicle.rtl_radius_m = 90;

    vehicle.update_control_mode();

    assert!(vehicle.rtl_mode_nav_applied);
    assert!(!vehicle.rtl_mode_started);
    assert!(!vehicle.rtl_mode_loiter_allowed);
    assert_eq!(vehicle.rtl_loiter_radius_m, 90);
    assert!(!vehicle.rtl_loiter_ccw);
}
