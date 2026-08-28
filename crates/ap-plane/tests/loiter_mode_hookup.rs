//! LOITER mode hookup for location-hold navigate.

use ap_plane::loiter_mode_hookup::{loiter_mode_nav_tick, LoiterModeNavInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn loiter_inp() -> LoiterModeNavInputs {
    LoiterModeNavInputs {
        control_mode: ModeNumber::Loiter.as_number(),
        features: BuildFeatures::default(),
        mode_just_entered: true,
        wp_loiter_rad_m: 200,
        stick_mixing_enabled: false,
        loiter_alt_control: false,
    }
}

#[test]
fn loiter_mode_nav_starts_on_enter() {
    let out = loiter_mode_nav_tick(&loiter_inp());
    assert!(out.applied);
    assert!(out.started);
    assert!(out.allow_loiter);
    assert!(out.direction_set);
    assert!(!out.loiter_ccw);
    assert!(!out.alt_control);
    assert_eq!(out.loiter_radius_m, 200);
}

#[test]
fn loiter_mode_nav_resumes_when_already_in_loiter() {
    let mut inp = loiter_inp();
    inp.mode_just_entered = false;
    let out = loiter_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.allow_loiter);
}

#[test]
fn loiter_mode_nav_skips_other_modes() {
    let mut inp = loiter_inp();
    inp.control_mode = ModeNumber::Rtl.as_number();
    let out = loiter_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.allow_loiter);
    assert!(!out.direction_set);
    assert!(!out.alt_control);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn loiter_mode_nav_decodes_ccw_radius() {
    let mut inp = loiter_inp();
    inp.wp_loiter_rad_m = -150;
    let out = loiter_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.direction_set);
    assert!(out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 150);
}

#[test]
fn loiter_mode_nav_zero_radius_leaves_direction() {
    let mut inp = loiter_inp();
    inp.wp_loiter_rad_m = 0;
    let out = loiter_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.allow_loiter);
    assert!(!out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn loiter_mode_nav_alt_control_needs_stick_mixing() {
    let mut inp = loiter_inp();
    inp.loiter_alt_control = true;
    let out = loiter_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.alt_control);

    inp.stick_mixing_enabled = true;
    let out = loiter_mode_nav_tick(&inp);
    assert!(out.alt_control);
}

#[test]
fn main_loop_starts_loiter_hold() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Loiter.as_number();
    vehicle.wp_loiter_rad_m = -180;
    vehicle.loiter_alt_control_enabled = true;

    vehicle.update_control_mode();

    assert!(vehicle.loiter_mode_nav_applied);
    assert!(vehicle.loiter_mode_started);
    assert!(vehicle.loiter_mode_loiter_allowed);
    assert!(vehicle.loiter_ccw);
    assert_eq!(vehicle.loiter_radius_m, 180);
    assert!(vehicle.loiter_alt_control);
    assert!(!vehicle.rtl_mode_nav_applied);
    assert!(!vehicle.auto_mode_mission_applied);
    assert!(!vehicle.circle_mode_nav_applied);
    assert!(!vehicle.thermal_mode_nav_applied);
}

#[test]
fn main_loop_resumes_loiter_without_reenter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Loiter.as_number();
    vehicle.tracked_control_mode = ModeNumber::Loiter.as_number();
    vehicle.wp_loiter_rad_m = 90;
    vehicle.stick_mixing = None;

    vehicle.update_control_mode();

    assert!(vehicle.loiter_mode_nav_applied);
    assert!(!vehicle.loiter_mode_started);
    assert!(vehicle.loiter_mode_loiter_allowed);
    assert_eq!(vehicle.loiter_radius_m, 90);
    assert!(!vehicle.loiter_ccw);
    assert!(!vehicle.loiter_alt_control);
}
