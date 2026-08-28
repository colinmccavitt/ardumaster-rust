//! GUIDED mode hookup for altitude/location remaining-leg.

use ap_plane::guided_mode_hookup::{guided_mode_update_tick, GuidedModeUpdateInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn update_inp() -> GuidedModeUpdateInputs {
    GuidedModeUpdateInputs {
        control_mode: ModeNumber::Guided.as_number(),
        features: BuildFeatures::default(),
        location_update: true,
        altitude_update: false,
        terrain_alt: false,
    }
}

#[test]
fn guided_mode_location_update_starts_remaining_leg() {
    let out = guided_mode_update_tick(&update_inp());
    assert!(out.applied);
    assert!(out.set_guided_wp);
    assert!(out.setup_remaining_leg);
    assert!(out.convert_abs_alt);
    assert!(!out.copy_next_wp_alt);
    assert!(!out.reset_offset_altitude);
}

#[test]
fn guided_mode_terrain_location_skips_abs_conversion() {
    let mut inp = update_inp();
    inp.terrain_alt = true;
    let out = guided_mode_update_tick(&inp);
    assert!(out.applied);
    assert!(out.set_guided_wp);
    assert!(out.setup_remaining_leg);
    assert!(!out.convert_abs_alt);
}

#[test]
fn guided_mode_altitude_update_copies_next_wp_alt() {
    let mut inp = update_inp();
    inp.location_update = false;
    inp.altitude_update = true;
    let out = guided_mode_update_tick(&inp);
    assert!(out.applied);
    assert!(!out.set_guided_wp);
    assert!(!out.setup_remaining_leg);
    assert!(out.convert_abs_alt);
    assert!(out.copy_next_wp_alt);
    assert!(out.reset_offset_altitude);
}

#[test]
fn guided_mode_terrain_altitude_copies_without_abs_conversion() {
    let mut inp = update_inp();
    inp.location_update = false;
    inp.altitude_update = true;
    inp.terrain_alt = true;
    let out = guided_mode_update_tick(&inp);
    assert!(out.applied);
    assert!(out.copy_next_wp_alt);
    assert!(out.reset_offset_altitude);
    assert!(!out.convert_abs_alt);
}

#[test]
fn guided_mode_idle_guided_does_not_setup_remaining_leg() {
    let mut inp = update_inp();
    inp.location_update = false;
    let out = guided_mode_update_tick(&inp);
    assert!(out.applied);
    assert!(!out.set_guided_wp);
    assert!(!out.setup_remaining_leg);
    assert!(!out.convert_abs_alt);
    assert!(!out.copy_next_wp_alt);
    assert!(!out.reset_offset_altitude);
}

#[test]
fn guided_mode_location_and_altitude_update_together() {
    let mut inp = update_inp();
    inp.altitude_update = true;
    let out = guided_mode_update_tick(&inp);
    assert!(out.applied);
    assert!(out.set_guided_wp);
    assert!(out.setup_remaining_leg);
    assert!(out.convert_abs_alt);
    assert!(out.copy_next_wp_alt);
    assert!(out.reset_offset_altitude);
}

#[test]
fn guided_mode_update_skips_other_modes() {
    let mut inp = update_inp();
    inp.control_mode = ModeNumber::Loiter.as_number();
    let out = guided_mode_update_tick(&inp);
    assert!(!out.applied);
    assert!(!out.set_guided_wp);
    assert!(!out.setup_remaining_leg);
    assert!(!out.convert_abs_alt);
    assert!(!out.copy_next_wp_alt);
    assert!(!out.reset_offset_altitude);
}

#[test]
fn main_loop_guided_location_update_remaining_leg() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Guided.as_number();
    vehicle.tracked_control_mode = ModeNumber::Guided.as_number();
    vehicle.guided_location_update = true;

    vehicle.update_control_mode();

    assert!(vehicle.guided_mode_nav_applied);
    assert!(!vehicle.guided_mode_started);
    assert!(vehicle.guided_mode_update_applied);
    assert!(vehicle.guided_set_guided_wp);
    assert!(vehicle.guided_setup_remaining_leg);
    assert!(vehicle.guided_convert_abs_alt);
    assert!(!vehicle.guided_copy_next_wp_alt);
    assert!(!vehicle.guided_reset_offset_altitude);
    assert!(!vehicle.guided_location_update);
}

#[test]
fn main_loop_guided_altitude_update_remaining_leg() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Guided.as_number();
    vehicle.tracked_control_mode = ModeNumber::Guided.as_number();
    vehicle.guided_altitude_update = true;

    vehicle.update_control_mode();

    assert!(vehicle.guided_mode_update_applied);
    assert!(!vehicle.guided_set_guided_wp);
    assert!(!vehicle.guided_setup_remaining_leg);
    assert!(vehicle.guided_convert_abs_alt);
    assert!(vehicle.guided_copy_next_wp_alt);
    assert!(vehicle.guided_reset_offset_altitude);
    assert!(!vehicle.guided_altitude_update);
}

#[test]
fn main_loop_guided_update_skips_loiter() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Loiter.as_number();
    vehicle.guided_location_update = true;
    vehicle.guided_altitude_update = true;

    vehicle.update_control_mode();

    assert!(!vehicle.guided_mode_update_applied);
    assert!(!vehicle.guided_setup_remaining_leg);
    assert!(!vehicle.guided_copy_next_wp_alt);
    assert!(vehicle.guided_location_update);
    assert!(vehicle.guided_altitude_update);
    assert!(vehicle.loiter_mode_nav_applied);
}
