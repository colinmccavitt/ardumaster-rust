//! CRUISE mode hookup for heading-lock nav roll mapping.

use ap_plane::cruise_mode_hookup::{cruise_mode_nav_tick, CruiseModeNavInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn cruise_inp() -> CruiseModeNavInputs {
    CruiseModeNavInputs {
        control_mode: ModeNumber::Cruise.as_number(),
        features: BuildFeatures::default(),
        roll_norm: 0.5,
        rudder_norm: 0.0,
        locked_heading: false,
        nav_scripting_active: false,
        roll_limit_cd: 4500,
        commanded_roll_cd: 1800,
    }
}

#[test]
fn cruise_mode_nav_unlocked_maps_roll_stick_to_nav_roll() {
    let out = cruise_mode_nav_tick(&cruise_inp());
    assert!(out.applied);
    assert!(!out.locked_heading);
    assert_eq!(out.nav_roll_cd, 2250);
}

#[test]
fn cruise_mode_nav_locked_uses_commanded_roll() {
    let mut inp = cruise_inp();
    inp.roll_norm = 0.0;
    inp.locked_heading = true;
    let out = cruise_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(out.locked_heading);
    assert_eq!(out.nav_roll_cd, 1800);
}

#[test]
fn cruise_mode_nav_roll_stick_unlocks_heading() {
    let mut inp = cruise_inp();
    inp.locked_heading = true;
    let out = cruise_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.locked_heading);
    assert_eq!(out.nav_roll_cd, 2250);
}

#[test]
fn cruise_mode_nav_rudder_unlocks_heading() {
    let mut inp = cruise_inp();
    inp.roll_norm = 0.0;
    inp.rudder_norm = 0.2;
    inp.locked_heading = true;
    let out = cruise_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.locked_heading);
    assert_eq!(out.nav_roll_cd, 0);
}

#[test]
fn cruise_mode_nav_scripting_unlocks_heading() {
    let mut inp = cruise_inp();
    inp.roll_norm = 0.0;
    inp.locked_heading = true;
    inp.nav_scripting_active = true;
    let out = cruise_mode_nav_tick(&inp);
    assert!(out.applied);
    assert!(!out.locked_heading);
    assert_eq!(out.nav_roll_cd, 0);
}

#[test]
fn cruise_mode_nav_skips_other_modes() {
    let mut inp = cruise_inp();
    inp.control_mode = ModeNumber::FlyByWireB.as_number();
    let out = cruise_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert_eq!(out.nav_roll_cd, 0);
}

#[test]
fn main_loop_applies_cruise_mode_nav_from_roll_stick() {
    use ap_plane::rc_failsafe_scheduler_hookup::{
        RcChannelConfig, RcFailsafeConfig, RcFailsafeSchedulerInputs,
    };

    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Cruise.as_number();
    vehicle.stabilize_demands.roll_limit_cd = 4000;
    vehicle.stabilize_demands.pitch_limit_min_cd = -1500;
    vehicle.stabilize_demands.pitch_limit_max_cd = 2000;
    vehicle.rc_failsafe_inputs = RcFailsafeSchedulerInputs {
        has_valid_input: true,
        roll_pwm: Some(1600),
        pitch_pwm: Some(1300),
        yaw_pwm: None,
        throttle_pwm: None,
        roll_cfg: RcChannelConfig::default(),
        pitch_cfg: RcChannelConfig::default(),
        yaw_cfg: RcChannelConfig::default(),
        throttle_cfg: RcChannelConfig::default(),
        failsafe_cfg: RcFailsafeConfig::default(),
        flap_pwm: None,
        flap_cfg: RcChannelConfig::default(),
    };
    vehicle.update_control_mode();

    assert!(vehicle.cruise_mode_nav_applied);
    assert!(!vehicle.fbwb_mode_nav_applied);
    assert!(!vehicle.fbwa_mode_nav_applied);
    assert!(!vehicle.manual_mode_nav_applied);
    assert!(!vehicle.stabilize_mode_nav_applied);
    assert!(!vehicle.acro_mode_nav_applied);
    assert!(!vehicle.training_mode_nav_applied);
    assert!(!vehicle.cruise_locked_heading);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 1000);
    // Cruise-assisted: elevator stick must not overwrite TECS pitch the way FBWA does
    // (FBWA maps pitch PWM 1300 / limits -1500..2000 to -750).
    assert_ne!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, -750);
}

#[test]
fn main_loop_cruise_locked_heading_keeps_nav_controller_roll() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Cruise.as_number();
    vehicle.cruise_locked_heading = true;
    vehicle.navigation_scheduler_inputs.commanded_roll_cd = 1800;
    vehicle.stabilize_demands.roll_limit_cd = 4000;
    vehicle.update_control_mode();

    assert!(vehicle.cruise_mode_nav_applied);
    assert!(vehicle.cruise_locked_heading);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 1800);
}
