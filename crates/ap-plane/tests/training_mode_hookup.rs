//! Training mode hookup for envelope-limit nav demands.

use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::training_mode_hookup::{training_mode_nav_tick, TrainingModeNavInputs};

fn training_inp() -> TrainingModeNavInputs {
    TrainingModeNavInputs {
        control_mode: ModeNumber::Training.as_number(),
        features: BuildFeatures::default(),
        roll_sensor_cd: 0,
        pitch_sensor_cd: 0,
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
    }
}

#[test]
fn training_mode_nav_inside_limits_zeros_and_marks_manual() {
    let out = training_mode_nav_tick(&training_inp());
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 0);
    assert_eq!(out.nav_pitch_cd, 0);
    assert!(out.training_manual_roll);
    assert!(out.training_manual_pitch);
}

#[test]
fn training_mode_nav_holds_roll_at_limit() {
    let mut inp = training_inp();
    inp.roll_sensor_cd = 5000;
    let out = training_mode_nav_tick(&inp);
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 4500);
    assert!(!out.training_manual_roll);
    assert!(out.training_manual_pitch);
}

#[test]
fn training_mode_nav_holds_negative_roll_at_limit() {
    let mut inp = training_inp();
    inp.roll_sensor_cd = -4800;
    let out = training_mode_nav_tick(&inp);
    assert_eq!(out.nav_roll_cd, -4500);
    assert!(!out.training_manual_roll);
}

#[test]
fn training_mode_nav_holds_pitch_at_limits() {
    let mut inp = training_inp();
    inp.pitch_sensor_cd = 3000;
    let hi = training_mode_nav_tick(&inp);
    assert_eq!(hi.nav_pitch_cd, 2500);
    assert!(!hi.training_manual_pitch);

    inp.pitch_sensor_cd = -2500;
    let lo = training_mode_nav_tick(&inp);
    assert_eq!(lo.nav_pitch_cd, -2000);
    assert!(!lo.training_manual_pitch);
}

#[test]
fn training_mode_nav_inverts_held_pitch_when_inverted() {
    let mut inp = training_inp();
    inp.roll_sensor_cd = -9500;
    inp.pitch_sensor_cd = 3000;
    let out = training_mode_nav_tick(&inp);
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, -4500);
    assert_eq!(out.nav_pitch_cd, -2500);
    assert!(!out.training_manual_roll);
    assert!(!out.training_manual_pitch);
}

#[test]
fn training_mode_nav_skips_other_modes() {
    let mut inp = training_inp();
    inp.control_mode = ModeNumber::FlyByWireA.as_number();
    inp.roll_sensor_cd = 5000;
    let out = training_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert_eq!(out.nav_roll_cd, 0);
    assert!(!out.training_manual_roll);
}

#[test]
fn main_loop_applies_training_mode_nav_holds() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Training.as_number();
    vehicle.stabilize_demands.roll_limit_cd = 4000;
    vehicle.stabilize_demands.pitch_limit_min_cd = -1500;
    vehicle.stabilize_demands.pitch_limit_max_cd = 2000;
    vehicle.attitude.roll_sensor_cd = 4500;
    vehicle.attitude.pitch_sensor_cd = -1800;
    vehicle.update_control_mode();

    assert!(vehicle.training_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 4000);
    assert_eq!(
        vehicle.navigation_scheduler_inputs.commanded_pitch_cd,
        -1500
    );
    assert!(!vehicle.training_manual_roll);
    assert!(!vehicle.training_manual_pitch);
    assert!(!vehicle.manual_mode_nav_applied);
    assert!(!vehicle.fbwa_mode_nav_applied);
    assert!(!vehicle.stabilize_mode_nav_applied);
    assert!(!vehicle.acro_mode_nav_applied);
}

#[test]
fn main_loop_applies_training_manual_inside_envelope() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Training.as_number();
    vehicle.stabilize_demands.roll_limit_cd = 4000;
    vehicle.stabilize_demands.pitch_limit_min_cd = -1500;
    vehicle.stabilize_demands.pitch_limit_max_cd = 2000;
    vehicle.attitude.roll_sensor_cd = 500;
    vehicle.attitude.pitch_sensor_cd = -200;
    vehicle.update_control_mode();

    assert!(vehicle.training_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 0);
    assert_eq!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, 0);
    assert!(vehicle.training_manual_roll);
    assert!(vehicle.training_manual_pitch);
}
