//! AUTO mode hookup for loiter-to-alt complete / resume-AUTO.

use ap_math::location::{AltFrame, Location};
use ap_plane::auto_mode_hookup::{
    auto_mode_loiter_to_alt_tick, AutoModeLoiterToAltInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn loiter_to_alt_inp() -> AutoModeLoiterToAltInputs {
    AutoModeLoiterToAltInputs {
        control_mode: ModeNumber::Auto.as_number(),
        features: BuildFeatures::default(),
        current_nav_is_loiter_to_alt: true,
        condition_value: 0,
        loiter_sum_cd: 200,
        reached_target_alt: false,
        unable_to_achieve_target_alt: false,
        heading_lined_up: false,
        next_nav_cmd_available: true,
        cmd_p1_radius_m: 80,
    }
}

fn wp(lat: i32, lng: i32) -> Location {
    Location::new_with_alt(lat, lng, 10_000, AltFrame::Absolute)
}

#[test]
fn auto_mode_loiter_to_alt_holds_until_alt() {
    let out = auto_mode_loiter_to_alt_tick(&loiter_to_alt_inp());
    assert!(out.applied);
    assert!(out.allow_loiter);
    assert!(!out.complete);
    assert!(!out.alt_reached);
    assert!(!out.reset_sum_cd);
    assert_eq!(out.condition_value, 0);
    assert_eq!(out.loiter_radius_m, 80);
}

#[test]
fn auto_mode_loiter_to_alt_needs_sum_cd() {
    let mut inp = loiter_to_alt_inp();
    inp.loiter_sum_cd = 1;
    inp.reached_target_alt = true;
    inp.heading_lined_up = true;
    let out = auto_mode_loiter_to_alt_tick(&inp);
    assert!(out.applied);
    assert!(!out.complete);
    assert!(!out.alt_reached);
    assert_eq!(out.condition_value, 0);
}

#[test]
fn auto_mode_loiter_to_alt_complete_resumes_auto() {
    let mut inp = loiter_to_alt_inp();
    inp.reached_target_alt = true;
    inp.heading_lined_up = true;
    let out = auto_mode_loiter_to_alt_tick(&inp);
    assert!(out.applied);
    assert!(out.allow_loiter);
    assert!(out.alt_reached);
    assert!(out.reset_sum_cd);
    assert!(out.complete);
    assert_eq!(out.condition_value, 1);
}

#[test]
fn auto_mode_loiter_to_alt_unable_still_latches_alt() {
    let mut inp = loiter_to_alt_inp();
    inp.unable_to_achieve_target_alt = true;
    inp.heading_lined_up = false;
    let out = auto_mode_loiter_to_alt_tick(&inp);
    assert!(out.applied);
    assert!(out.alt_reached);
    assert!(out.reset_sum_cd);
    assert!(!out.complete);
    assert_eq!(out.condition_value, 1);
}

#[test]
fn auto_mode_loiter_to_alt_waits_for_heading() {
    let mut inp = loiter_to_alt_inp();
    inp.condition_value = 1;
    inp.heading_lined_up = false;
    let out = auto_mode_loiter_to_alt_tick(&inp);
    assert!(out.applied);
    assert!(!out.alt_reached);
    assert!(!out.reset_sum_cd);
    assert!(!out.complete);
    assert_eq!(out.condition_value, 1);
}

#[test]
fn auto_mode_loiter_to_alt_heading_then_complete() {
    let mut inp = loiter_to_alt_inp();
    inp.condition_value = 1;
    inp.heading_lined_up = true;
    let out = auto_mode_loiter_to_alt_tick(&inp);
    assert!(out.applied);
    assert!(!out.alt_reached);
    assert!(!out.reset_sum_cd);
    assert!(out.complete);
}

#[test]
fn auto_mode_loiter_to_alt_no_next_nav_completes() {
    let mut inp = loiter_to_alt_inp();
    inp.reached_target_alt = true;
    inp.next_nav_cmd_available = false;
    inp.heading_lined_up = false;
    let out = auto_mode_loiter_to_alt_tick(&inp);
    assert!(out.applied);
    assert!(out.complete);
    assert!(out.alt_reached);
}

#[test]
fn auto_mode_loiter_to_alt_skips_other_modes() {
    let mut inp = loiter_to_alt_inp();
    inp.control_mode = ModeNumber::Loiter.as_number();
    inp.reached_target_alt = true;
    inp.heading_lined_up = true;
    let out = auto_mode_loiter_to_alt_tick(&inp);
    assert!(!out.applied);
    assert!(!out.complete);
    assert!(!out.allow_loiter);
    assert_eq!(out.loiter_radius_m, 0);
}

#[test]
fn auto_mode_loiter_to_alt_skips_other_nav() {
    let mut inp = loiter_to_alt_inp();
    inp.current_nav_is_loiter_to_alt = false;
    inp.reached_target_alt = true;
    inp.heading_lined_up = true;
    let out = auto_mode_loiter_to_alt_tick(&inp);
    assert!(!out.applied);
    assert!(!out.complete);
    assert!(!out.allow_loiter);
}

#[test]
fn main_loop_auto_loiter_to_alt_resumes_next_item() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Auto.as_number();
    vehicle.tracked_control_mode = ModeNumber::Auto.as_number();
    vehicle.home_is_set = true;
    vehicle.mission.running = true;
    vehicle.mission.complete = false;
    vehicle.mission.current_index = 0;
    vehicle.auto_current_nav_is_loiter_to_alt = true;
    vehicle.auto_condition_value = 0;
    vehicle.auto_loiter_sum_cd = 250;
    vehicle.auto_loiter_reached_target_alt = false;
    vehicle.auto_loiter_heading_lined_up = false;
    vehicle.auto_loiter_next_nav_available = true;
    vehicle.auto_loiter_to_alt_radius_m = 90;
    vehicle.mission_inputs.control_mode = ModeNumber::Auto.as_number();
    vehicle.mission_inputs.waypoint_count = 2;
    vehicle.mission_inputs.wp_radius_m = 30.0;
    let first = wp(-35_000_000, 149_000_000);
    let second = wp(-35_010_000, 149_010_000);
    let mut here = first;
    here.offset(400.0, 0.0);
    vehicle.mission_inputs.waypoints = [
        first,
        second,
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
        Location::new(0, 0),
    ];
    vehicle.mission_inputs.current_loc = here;

    vehicle.update_control_mode();

    assert!(vehicle.auto_mode_loiter_to_alt_applied);
    assert!(vehicle.auto_mode_loiter_to_alt_allow_loiter);
    assert!(!vehicle.auto_mode_loiter_to_alt_complete);
    assert_eq!(vehicle.auto_condition_value, 0);
    assert_eq!(vehicle.mission.current_index, 0);

    vehicle.auto_loiter_reached_target_alt = true;
    vehicle.auto_loiter_heading_lined_up = true;
    vehicle.update_control_mode();

    assert!(vehicle.auto_mode_loiter_to_alt_applied);
    assert!(vehicle.auto_loiter_to_alt_alt_reached);
    assert!(vehicle.auto_loiter_to_alt_reset_sum_cd);
    assert!(vehicle.auto_mode_loiter_to_alt_complete);
    assert_eq!(vehicle.auto_condition_value, 1);
    assert_eq!(vehicle.auto_loiter_sum_cd, 0);
    assert_eq!(vehicle.mission.current_index, 1);
    assert!(vehicle.auto_mode_mission_advanced);
    assert!(!vehicle.mission.complete);
    assert!(!vehicle.auto_mode_switch_to_rtl);
}

#[test]
fn main_loop_auto_loiter_to_alt_skips_loiter_mode() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Loiter.as_number();
    vehicle.auto_current_nav_is_loiter_to_alt = true;
    vehicle.auto_loiter_sum_cd = 250;
    vehicle.auto_loiter_reached_target_alt = true;
    vehicle.auto_loiter_heading_lined_up = true;

    vehicle.update_control_mode();

    assert!(!vehicle.auto_mode_loiter_to_alt_applied);
    assert!(!vehicle.auto_mode_loiter_to_alt_complete);
    assert!(vehicle.loiter_mode_nav_applied);
}
