//! AUTO mode hookup for mission-complete / landing handoff.

use ap_math::location::{AltFrame, Location};
use ap_plane::auto_mode_hookup::{
    auto_mode_complete_tick, AutoModeCompleteInputs, MODE_REASON_MISSION_END,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn complete_inp() -> AutoModeCompleteInputs {
    AutoModeCompleteInputs {
        control_mode: ModeNumber::Auto.as_number(),
        features: BuildFeatures::default(),
        mission_running: true,
        mission_complete: true,
        current_nav_is_land: false,
    }
}

fn wp(lat: i32, lng: i32) -> Location {
    Location::new_with_alt(lat, lng, 10_000, AltFrame::Absolute)
}

#[test]
fn auto_mode_complete_switches_to_rtl() {
    let out = auto_mode_complete_tick(&complete_inp());
    assert!(out.applied);
    assert!(out.switch_to_rtl);
    assert!(!out.allow_land);
    assert_eq!(out.reason, MODE_REASON_MISSION_END);
}

#[test]
fn auto_mode_complete_lands_instead_of_rtl() {
    let mut inp = complete_inp();
    inp.current_nav_is_land = true;
    let out = auto_mode_complete_tick(&inp);
    assert!(out.applied);
    assert!(!out.switch_to_rtl);
    assert!(out.allow_land);
    assert_eq!(out.reason, 0);
}

#[test]
fn auto_mode_without_running_mission_switches_to_rtl() {
    let mut inp = complete_inp();
    inp.mission_running = false;
    inp.mission_complete = false;
    let out = auto_mode_complete_tick(&inp);
    assert!(out.applied);
    assert!(out.switch_to_rtl);
    assert!(!out.allow_land);
    assert_eq!(out.reason, MODE_REASON_MISSION_END);
}

#[test]
fn auto_mode_running_land_cmd_hands_off_to_landing() {
    let mut inp = complete_inp();
    inp.mission_complete = false;
    inp.current_nav_is_land = true;
    let out = auto_mode_complete_tick(&inp);
    assert!(out.applied);
    assert!(!out.switch_to_rtl);
    assert!(out.allow_land);
    assert_eq!(out.reason, 0);
}

#[test]
fn auto_mode_running_mission_does_not_handoff() {
    let mut inp = complete_inp();
    inp.mission_complete = false;
    let out = auto_mode_complete_tick(&inp);
    assert!(out.applied);
    assert!(!out.switch_to_rtl);
    assert!(!out.allow_land);
    assert_eq!(out.reason, 0);
}

#[test]
fn auto_mode_complete_skips_other_modes() {
    let mut inp = complete_inp();
    inp.control_mode = ModeNumber::Rtl.as_number();
    let out = auto_mode_complete_tick(&inp);
    assert!(!out.applied);
    assert!(!out.switch_to_rtl);
    assert!(!out.allow_land);
    assert_eq!(out.reason, 0);
}

#[test]
fn main_loop_auto_complete_requests_rtl() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Auto.as_number();
    vehicle.tracked_control_mode = ModeNumber::Auto.as_number();
    vehicle.home_is_set = true;
    vehicle.mission.running = true;
    vehicle.mission.complete = false;
    vehicle.mission.current_index = 0;
    let target = wp(-35_000_000, 149_000_000);
    let mut near = target;
    near.offset(50.0, 0.0);
    vehicle.mission_inputs.control_mode = ModeNumber::Auto.as_number();
    vehicle.mission_inputs.waypoint_count = 1;
    vehicle.mission_inputs.waypoints = [
        target,
        Location::new(0, 0),
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

    assert!(vehicle.auto_mode_complete_applied);
    assert!(vehicle.mission.complete);
    assert!(vehicle.auto_mode_switch_to_rtl);
    assert!(!vehicle.auto_mode_land_handoff);
    assert!(!vehicle.auto_mode_mission_started);
}

#[test]
fn main_loop_auto_complete_lands_when_nav_is_land() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Auto.as_number();
    vehicle.tracked_control_mode = ModeNumber::Auto.as_number();
    vehicle.home_is_set = true;
    vehicle.mission.running = true;
    vehicle.mission.complete = true;
    vehicle.auto_current_nav_is_land = true;

    vehicle.update_control_mode();

    assert!(vehicle.auto_mode_complete_applied);
    assert!(!vehicle.auto_mode_switch_to_rtl);
    assert!(vehicle.auto_mode_land_handoff);
}

#[test]
fn main_loop_auto_complete_skips_rtl_mode() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Rtl.as_number();
    vehicle.home_is_set = true;
    vehicle.mission.complete = true;

    vehicle.update_control_mode();

    assert!(!vehicle.auto_mode_complete_applied);
    assert!(!vehicle.auto_mode_switch_to_rtl);
    assert!(!vehicle.auto_mode_land_handoff);
    assert!(vehicle.rtl_mode_nav_applied);
}
