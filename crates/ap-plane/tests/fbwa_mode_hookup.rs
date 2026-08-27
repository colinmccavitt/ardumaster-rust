//! FBWA mode hookup for nav stick mapping.

use ap_plane::fbwa_mode_hookup::{
    fbwa_mode_nav_tick, fbwa_nav_pitch_from_stick, FbwaModeNavInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

#[test]
fn fbwa_nav_pitch_positive_uses_max_limit() {
    assert_eq!(fbwa_nav_pitch_from_stick(0.5, -2000, 3000), 1500);
}

#[test]
fn fbwa_nav_pitch_negative_uses_min_limit() {
    assert_eq!(fbwa_nav_pitch_from_stick(-0.5, -2000, 3000), -1000);
}

#[test]
fn fbwa_mode_nav_maps_roll_stick_to_nav_roll() {
    let out = fbwa_mode_nav_tick(&FbwaModeNavInputs {
        control_mode: ModeNumber::FlyByWireA.as_number(),
        features: BuildFeatures::default(),
        roll_norm: 0.5,
        pitch_norm: 0.0,
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
        roll_sensor_cd: 0,
    });
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 2250);
    assert_eq!(out.nav_pitch_cd, 0);
}

#[test]
fn fbwa_mode_nav_inverts_pitch_when_inverted() {
    let out = fbwa_mode_nav_tick(&FbwaModeNavInputs {
        control_mode: ModeNumber::FlyByWireA.as_number(),
        features: BuildFeatures::default(),
        roll_norm: 0.0,
        pitch_norm: 0.4,
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
        roll_sensor_cd: -9500,
    });
    assert!(out.applied);
    assert_eq!(out.nav_pitch_cd, -1000);
}

#[test]
fn fbwa_mode_nav_skips_other_modes() {
    let out = fbwa_mode_nav_tick(&FbwaModeNavInputs {
        control_mode: ModeNumber::Stabilize.as_number(),
        features: BuildFeatures::default(),
        roll_norm: 1.0,
        pitch_norm: 1.0,
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
        roll_sensor_cd: 0,
    });
    assert!(!out.applied);
}

#[test]
fn main_loop_applies_fbwa_mode_nav_from_sticks() {
    use ap_plane::rc_failsafe_scheduler_hookup::{
        RcChannelConfig, RcFailsafeConfig, RcFailsafeSchedulerInputs,
    };

    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::FlyByWireA.as_number();
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

    assert!(vehicle.fbwa_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 1000);
    assert_eq!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, -750);
}
