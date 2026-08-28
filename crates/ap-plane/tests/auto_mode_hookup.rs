//! AUTO mode hookup for mission start / advance.

use ap_math::location::{AltFrame, Location};
use ap_plane::auto_mode_hookup::{auto_mode_mission_tick, AutoModeMissionInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn auto_inp() -> AutoModeMissionInputs {
    AutoModeMissionInputs {
        control_mode: ModeNumber::Auto.as_number(),
        features: BuildFeatures::default(),
        mode_just_entered: true,
        mission_running: false,
        home_is_set: true,
        waypoint_count: 2,
        current_index: 3,
    }
}

fn wp(lat: i32, lng: i32) -> Location {
    Location::new_with_alt(lat, lng, 10_000, AltFrame::Absolute)
}

#[test]
fn auto_mode_mission_starts_on_enter() {
    let out = auto_mode_mission_tick(&auto_inp());
    assert!(out.applied);
    assert!(out.started);
    assert!(out.mission_running);
    assert!(out.allow_advance);
    assert_eq!(out.current_index, 0);
}

#[test]
fn auto_mode_mission_resumes_when_already_running() {
    let mut inp = auto_inp();
    inp.mission_running = true;
    inp.current_index = 2;
    let out = auto_mode_mission_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(out.mission_running);
    assert_eq!(out.current_index, 2);
}

#[test]
fn auto_mode_mission_skips_other_modes() {
    let mut inp = auto_inp();
    inp.control_mode = ModeNumber::Rtl.as_number();
    let out = auto_mode_mission_tick(&inp);
    assert!(!out.applied);
    assert!(!out.started);
    assert!(!out.allow_advance);
    assert_eq!(out.current_index, 3);
}

#[test]
fn auto_mode_mission_skips_empty_mission() {
    let mut inp = auto_inp();
    inp.waypoint_count = 0;
    let out = auto_mode_mission_tick(&inp);
    assert!(out.applied);
    assert!(!out.started);
    assert!(!out.mission_running);
    assert!(!out.allow_advance);
}

#[test]
fn auto_mode_mission_blocks_advance_without_home() {
    let mut inp = auto_inp();
    inp.home_is_set = false;
    let out = auto_mode_mission_tick(&inp);
    assert!(out.applied);
    assert!(out.started);
    assert!(out.mission_running);
    assert!(!out.allow_advance);
}

#[test]
fn main_loop_starts_mission_in_auto() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Auto.as_number();
    vehicle.home_is_set = true;
    let target = wp(-35_000_000, 149_000_000);
    let mut far = target;
    far.offset(2000.0, 0.0);
    vehicle.mission_inputs.waypoint_count = 2;
    vehicle.mission_inputs.waypoints = [
        target,
        far,
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
    ];
    vehicle.mission_inputs.current_loc = far;
    vehicle.mission_inputs.wp_radius_m = 100.0;

    vehicle.update_control_mode();

    assert!(vehicle.auto_mode_mission_applied);
    assert!(vehicle.auto_mode_mission_started);
    assert!(vehicle.mission.running);
    assert_eq!(vehicle.mission.current_index, 0);
    assert!(!vehicle.auto_mode_mission_advanced);
    assert!(!vehicle.mission_advanced);
    assert!(!vehicle.thermal_mode_nav_applied);
    assert!(!vehicle.circle_mode_nav_applied);
}

#[test]
fn main_loop_advances_mission_item_in_auto() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Auto.as_number();
    vehicle.tracked_control_mode = ModeNumber::Auto.as_number();
    vehicle.home_is_set = true;
    vehicle.mission.running = true;
    vehicle.mission.current_index = 0;
    let target = wp(-35_000_000, 149_000_000);
    let mut near = target;
    near.offset(50.0, 0.0);
    let mut far = target;
    far.offset(2000.0, 0.0);
    vehicle.mission_inputs.control_mode = ModeNumber::Auto.as_number();
    vehicle.mission_inputs.waypoint_count = 2;
    vehicle.mission_inputs.waypoints = [
        target,
        far,
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
    ];
    vehicle.mission_inputs.current_loc = near;
    vehicle.mission_inputs.wp_radius_m = 100.0;

    vehicle.update_control_mode();

    assert!(vehicle.auto_mode_mission_applied);
    assert!(!vehicle.auto_mode_mission_started);
    assert!(vehicle.auto_mode_mission_advanced);
    assert!(vehicle.mission_advanced);
    assert_eq!(vehicle.mission.current_index, 1);
    assert!(vehicle.mission.running);
}
