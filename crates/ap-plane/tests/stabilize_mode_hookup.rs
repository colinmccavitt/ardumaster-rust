//! Stabilize mode hookup for wings-level nav demands.

use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::stabilize_mode_hookup::{stabilize_mode_nav_tick, StabilizeModeNavInputs};

#[test]
fn stabilize_mode_nav_zeros_demands() {
    let out = stabilize_mode_nav_tick(&StabilizeModeNavInputs {
        control_mode: ModeNumber::Stabilize.as_number(),
        features: BuildFeatures::default(),
    });
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 0);
    assert_eq!(out.nav_pitch_cd, 0);
}

#[test]
fn stabilize_mode_nav_skips_other_modes() {
    let out = stabilize_mode_nav_tick(&StabilizeModeNavInputs {
        control_mode: ModeNumber::FlyByWireA.as_number(),
        features: BuildFeatures::default(),
    });
    assert!(!out.applied);
}

#[test]
fn main_loop_applies_stabilize_mode_nav_zeros() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Stabilize.as_number();
    vehicle.nav_tecs.nav_roll_cd = 1800;
    vehicle.navigation_scheduler_inputs.commanded_pitch_cd = -900;
    vehicle.update_control_mode();

    assert!(vehicle.stabilize_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 0);
    assert_eq!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, 0);
    assert!(!vehicle.manual_mode_nav_applied);
    assert!(!vehicle.fbwa_mode_nav_applied);
}
