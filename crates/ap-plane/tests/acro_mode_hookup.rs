//! Acro mode hookup for locked/unlocked nav demands.

use ap_plane::acro_mode_hookup::{acro_mode_nav_tick, AcroModeNavInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

#[test]
fn acro_mode_nav_unlocked_mirrors_attitude_sensors() {
    let out = acro_mode_nav_tick(&AcroModeNavInputs {
        control_mode: ModeNumber::Acro.as_number(),
        features: BuildFeatures::default(),
        locked_roll: false,
        locked_pitch: false,
        locked_roll_err: 12.7,
        locked_pitch_cd: 900,
        roll_sensor_cd: 1400,
        pitch_sensor_cd: -600,
    });
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 1400);
    assert_eq!(out.nav_pitch_cd, -600);
}

#[test]
fn acro_mode_nav_locked_uses_lock_state() {
    let out = acro_mode_nav_tick(&AcroModeNavInputs {
        control_mode: ModeNumber::Acro.as_number(),
        features: BuildFeatures::default(),
        locked_roll: true,
        locked_pitch: true,
        locked_roll_err: 12.7,
        locked_pitch_cd: 900,
        roll_sensor_cd: 1400,
        pitch_sensor_cd: -600,
    });
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 12);
    assert_eq!(out.nav_pitch_cd, 900);
}

#[test]
fn acro_mode_nav_skips_other_modes() {
    let out = acro_mode_nav_tick(&AcroModeNavInputs {
        control_mode: ModeNumber::Stabilize.as_number(),
        features: BuildFeatures::default(),
        locked_roll: true,
        locked_pitch: true,
        locked_roll_err: 12.7,
        locked_pitch_cd: 900,
        roll_sensor_cd: 1400,
        pitch_sensor_cd: -600,
    });
    assert!(!out.applied);
}

#[test]
fn main_loop_applies_acro_mode_nav_from_sensors() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Acro.as_number();
    vehicle.attitude.roll_sensor_cd = 700;
    vehicle.attitude.pitch_sensor_cd = -250;
    vehicle.update_control_mode();

    assert!(vehicle.acro_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 700);
    assert_eq!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, -250);
    assert!(!vehicle.manual_mode_nav_applied);
    assert!(!vehicle.fbwa_mode_nav_applied);
    assert!(!vehicle.stabilize_mode_nav_applied);
}

#[test]
fn main_loop_applies_acro_locked_nav_demands() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Acro.as_number();
    vehicle.acro_locked_roll = true;
    vehicle.acro_locked_pitch = true;
    vehicle.acro_locked_roll_err = -3.9;
    vehicle.acro_locked_pitch_cd = 1100;
    vehicle.attitude.roll_sensor_cd = 700;
    vehicle.attitude.pitch_sensor_cd = -250;
    vehicle.update_control_mode();

    assert!(vehicle.acro_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, -3);
    assert_eq!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, 1100);
}
