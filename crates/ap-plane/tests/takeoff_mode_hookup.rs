//! TAKEOFF mode hookup for climb-then-loiter navigate.

use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::takeoff_mode_hookup::{
    takeoff_mode_nav_tick, TakeoffModeNavInputs, TKOFF_ALT_DEFAULT_M, TKOFF_DIST_DEFAULT_M,
};

fn takeoff_inp() -> TakeoffModeNavInputs {
    TakeoffModeNavInputs {
        control_mode: ModeNumber::Takeoff.as_number(),
        features: BuildFeatures::default(),
        mode_just_entered: true,
        home_is_set: true,
        current_loc_initialised: true,
        target_alt_m: TKOFF_ALT_DEFAULT_M,
        target_dist_m: TKOFF_DIST_DEFAULT_M,
        wp_loiter_rad_m: 200,
    }
}

#[test]
fn takeoff_mode_nav_starts_on_enter() {
    let out = takeoff_mode_nav_tick(&takeoff_inp());
    assert!(out.applied);
    assert!(out.started);
    assert!(out.allow_setup);
    assert!(out.allow_loiter);
    assert!(out.direction_set);
    assert!(!out.loiter_ccw);
    // navigate always calls update_loiter(0); WP_LOITER_RAD is the default.
    assert_eq!(out.loiter_radius_m, 0);
    assert_eq!(out.target_alt_m, TKOFF_ALT_DEFAULT_M);
    assert_eq!(out.target_dist_m, TKOFF_DIST_DEFAULT_M);
}

#[test]
fn takeoff_mode_nav_resumes_when_already_in_takeoff() {
    let mut inp = takeoff_inp();
    inp.mode_just_entered = false;
    let out = takeoff_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.allow_setup);
    assert!(out.allow_loiter);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn takeoff_mode_nav_skips_other_modes() {
    let mut inp = takeoff_inp();
    inp.control_mode = ModeNumber::Guided.as_number();
    let out = takeoff_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.allow_setup);
    assert!(!out.allow_loiter);
    assert!(!out.direction_set);
    assert_eq!(out.loiter_radius_m, 0);
    assert_eq!(out.target_alt_m, 0);
    assert_eq!(out.target_dist_m, 0);
}

#[test]
fn takeoff_mode_nav_blocks_setup_without_home() {
    let mut inp = takeoff_inp();
    inp.home_is_set = false;
    let out = takeoff_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.started);
    assert!(!out.allow_setup);
    assert!(out.allow_loiter);
}

#[test]
fn takeoff_mode_nav_blocks_setup_without_loc() {
    let mut inp = takeoff_inp();
    inp.current_loc_initialised = false;
    let out = takeoff_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.started);
    assert!(!out.allow_setup);
    assert!(out.allow_loiter);
}

#[test]
fn takeoff_mode_nav_decodes_ccw_from_wp_loiter_rad() {
    let mut inp = takeoff_inp();
    inp.wp_loiter_rad_m = -150;
    let out = takeoff_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.direction_set);
    assert!(out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn takeoff_mode_nav_zero_radius_leaves_direction() {
    let mut inp = takeoff_inp();
    inp.wp_loiter_rad_m = 0;
    let out = takeoff_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.allow_loiter);
    assert!(!out.direction_set);
    assert!(!out.loiter_ccw);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn main_loop_starts_takeoff_loiter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Takeoff.as_number();
    vehicle.wp_loiter_rad_m = -180;
    vehicle.takeoff_target_alt_m = 60;
    vehicle.takeoff_target_dist_m = 250;

    vehicle.update_control_mode();

    assert!(vehicle.takeoff_mode_nav_applied);
    assert!(vehicle.takeoff_mode_started);
    assert!(vehicle.takeoff_mode_setup_allowed);
    assert!(vehicle.takeoff_mode_loiter_allowed);
    assert!(vehicle.takeoff_loiter_ccw);
    assert_eq!(vehicle.takeoff_loiter_radius_m, 0);
    assert_eq!(vehicle.takeoff_target_alt_m, 60);
    assert_eq!(vehicle.takeoff_target_dist_m, 250);
    assert!(!vehicle.guided_mode_nav_applied);
    assert!(!vehicle.loiter_mode_nav_applied);
    assert!(!vehicle.rtl_mode_nav_applied);
    assert!(!vehicle.auto_mode_mission_applied);
    assert!(!vehicle.circle_mode_nav_applied);
    assert!(!vehicle.thermal_mode_nav_applied);
}

#[test]
fn main_loop_resumes_takeoff_without_reenter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Takeoff.as_number();
    vehicle.tracked_control_mode = ModeNumber::Takeoff.as_number();
    vehicle.wp_loiter_rad_m = 90;
    vehicle.home_is_set = false;

    vehicle.update_control_mode();

    assert!(vehicle.takeoff_mode_nav_applied);
    assert!(!vehicle.takeoff_mode_started);
    assert!(!vehicle.takeoff_mode_setup_allowed);
    assert!(vehicle.takeoff_mode_loiter_allowed);
    assert_eq!(vehicle.takeoff_loiter_radius_m, 0);
    assert!(!vehicle.takeoff_loiter_ccw);
}

#[test]
fn main_loop_takeoff_blocks_setup_without_loc() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Takeoff.as_number();
    vehicle.current_loc_initialised = false;

    vehicle.update_control_mode();

    assert!(vehicle.takeoff_mode_nav_applied);
    assert!(vehicle.takeoff_mode_started);
    assert!(!vehicle.takeoff_mode_setup_allowed);
    assert!(vehicle.takeoff_mode_loiter_allowed);
}
