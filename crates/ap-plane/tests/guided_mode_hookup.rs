//! GUIDED mode hookup for current-location loiter navigate.

use ap_plane::guided_mode_hookup::{guided_mode_nav_tick, GuidedModeNavInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn guided_inp() -> GuidedModeNavInputs {
    GuidedModeNavInputs {
        control_mode: ModeNumber::Guided.as_number(),
        features: BuildFeatures::default(),
        mode_just_entered: true,
        active_radius_m: 250,
        wp_loiter_rad_m: 200,
        guided_ccw: false,
    }
}

#[test]
fn guided_mode_nav_starts_on_enter() {
    let out = guided_mode_nav_tick(&guided_inp());
    assert!(out.applied);
    assert!(out.started);
    assert!(out.allow_loiter);
    assert!(out.clear_throttle_passthru);
    assert!(out.direction_set);
    assert!(!out.loiter_ccw);
    // _enter resets active_radius_m so update_loiter uses WP_LOITER_RAD.
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn guided_mode_nav_resumes_when_already_in_guided() {
    let mut inp = guided_inp();
    inp.mode_just_entered = false;
    let out = guided_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.allow_loiter);
    assert!(!out.clear_throttle_passthru);
    assert!(out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 250);
}

#[test]
fn guided_mode_nav_skips_other_modes() {
    let mut inp = guided_inp();
    inp.control_mode = ModeNumber::Loiter.as_number();
    let out = guided_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.allow_loiter);
    assert!(!out.direction_set);
    assert!(!out.clear_throttle_passthru);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn guided_mode_nav_enter_decodes_ccw_from_wp_loiter_rad() {
    let mut inp = guided_inp();
    inp.wp_loiter_rad_m = -150;
    let out = guided_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.started);
    assert!(out.direction_set);
    assert!(out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn guided_mode_nav_zero_wp_radius_leaves_direction_on_enter() {
    let mut inp = guided_inp();
    inp.wp_loiter_rad_m = 0;
    let out = guided_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn guided_mode_nav_applies_set_radius_direction_after_enter() {
    let mut inp = guided_inp();
    inp.mode_just_entered = false;
    inp.active_radius_m = 180;
    inp.guided_ccw = true;
    let out = guided_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.direction_set);
    assert!(out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 180);
}

#[test]
fn guided_mode_nav_zero_active_radius_leaves_direction() {
    let mut inp = guided_inp();
    inp.mode_just_entered = false;
    inp.active_radius_m = 0;
    inp.guided_ccw = true;
    let out = guided_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.allow_loiter);
    assert!(!out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn main_loop_starts_guided_loiter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Guided.as_number();
    vehicle.wp_loiter_rad_m = -180;
    vehicle.guided_active_radius_m = 250;
    vehicle.guided_throttle_passthru = true;

    vehicle.update_control_mode();

    assert!(vehicle.guided_mode_nav_applied);
    assert!(vehicle.guided_mode_started);
    assert!(vehicle.guided_mode_loiter_allowed);
    assert!(vehicle.guided_loiter_ccw);
    assert_eq!(vehicle.guided_loiter_radius_m, 0);
    assert_eq!(vehicle.guided_active_radius_m, 0);
    assert!(!vehicle.guided_throttle_passthru);
    assert!(!vehicle.loiter_mode_nav_applied);
    assert!(!vehicle.rtl_mode_nav_applied);
    assert!(!vehicle.auto_mode_mission_applied);
    assert!(!vehicle.circle_mode_nav_applied);
    assert!(!vehicle.thermal_mode_nav_applied);
}

#[test]
fn main_loop_resumes_guided_without_reenter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Guided.as_number();
    vehicle.tracked_control_mode = ModeNumber::Guided.as_number();
    vehicle.guided_active_radius_m = 90;
    vehicle.guided_loiter_ccw = true;
    vehicle.guided_throttle_passthru = true;

    vehicle.update_control_mode();

    assert!(vehicle.guided_mode_nav_applied);
    assert!(!vehicle.guided_mode_started);
    assert!(vehicle.guided_mode_loiter_allowed);
    assert_eq!(vehicle.guided_loiter_radius_m, 90);
    assert!(vehicle.guided_loiter_ccw);
    assert!(vehicle.guided_throttle_passthru);
}
