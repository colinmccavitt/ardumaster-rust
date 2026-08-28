//! AUTOTUNE mode hookup for FBWA-delegated nav stick mapping.

use ap_plane::autotune_mode_hookup::{autotune_mode_nav_tick, AutotuneModeNavInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn autotune_inp() -> AutotuneModeNavInputs {
    AutotuneModeNavInputs {
        control_mode: ModeNumber::Autotune.as_number(),
        features: BuildFeatures::default(),
        roll_norm: 0.5,
        pitch_norm: 0.0,
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
        roll_sensor_cd: 0,
    }
}

#[test]
fn autotune_mode_nav_maps_roll_stick_to_nav_roll() {
    let out = autotune_mode_nav_tick(&autotune_inp());
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 2250);
    assert_eq!(out.nav_pitch_cd, 0);
}

#[test]
fn autotune_mode_nav_inverts_pitch_when_inverted() {
    let mut inp = autotune_inp();
    inp.roll_norm = 0.0;
    inp.pitch_norm = 0.4;
    inp.roll_sensor_cd = -9500;
    let out = autotune_mode_nav_tick(&inp);
    assert!(out.applied);
    assert_eq!(out.nav_pitch_cd, -1000);
}

#[test]
fn autotune_mode_nav_skips_other_modes() {
    let mut inp = autotune_inp();
    inp.control_mode = ModeNumber::FlyByWireA.as_number();
    let out = autotune_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert_eq!(out.nav_roll_cd, 0);
}

#[test]
fn main_loop_applies_autotune_mode_nav_from_sticks() {
    use ap_plane::rc_failsafe_scheduler_hookup::{
        RcChannelConfig, RcFailsafeConfig, RcFailsafeSchedulerInputs,
    };

    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Autotune.as_number();
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

    assert!(vehicle.autotune_mode_nav_applied);
    assert!(!vehicle.fbwa_mode_nav_applied);
    assert!(!vehicle.circle_mode_nav_applied);
    assert!(!vehicle.manual_mode_nav_applied);
    assert!(!vehicle.stabilize_mode_nav_applied);
    assert!(!vehicle.acro_mode_nav_applied);
    assert!(!vehicle.training_mode_nav_applied);
    assert!(!vehicle.fbwb_mode_nav_applied);
    assert!(!vehicle.cruise_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 1000);
    assert_eq!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, -750);
}
