//! CIRCLE mode hookup for loiter-assisted nav roll.

use ap_plane::circle_mode_hookup::{circle_mode_nav_tick, CircleModeNavInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn circle_inp() -> CircleModeNavInputs {
    CircleModeNavInputs {
        control_mode: ModeNumber::Circle.as_number(),
        features: BuildFeatures::default(),
        roll_limit_cd: 4500,
    }
}

#[test]
fn circle_mode_nav_banks_one_third_roll_limit() {
    let out = circle_mode_nav_tick(&circle_inp());
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 1500);
}

#[test]
fn circle_mode_nav_skips_other_modes() {
    let mut inp = circle_inp();
    inp.control_mode = ModeNumber::Loiter.as_number();
    let out = circle_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert_eq!(out.nav_roll_cd, 0);
}

#[test]
fn main_loop_applies_circle_mode_nav_third_bank() {
    use ap_plane::rc_failsafe_scheduler_hookup::{
        RcChannelConfig, RcFailsafeConfig, RcFailsafeSchedulerInputs,
    };

    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Circle.as_number();
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

    assert!(vehicle.circle_mode_nav_applied);
    assert!(!vehicle.autotune_mode_nav_applied);
    assert!(!vehicle.fbwa_mode_nav_applied);
    assert!(!vehicle.fbwb_mode_nav_applied);
    assert!(!vehicle.cruise_mode_nav_applied);
    assert!(!vehicle.manual_mode_nav_applied);
    assert!(!vehicle.stabilize_mode_nav_applied);
    assert!(!vehicle.acro_mode_nav_applied);
    assert!(!vehicle.training_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 4000 / 3);
    // Loiter-assisted: elevator stick must not overwrite TECS pitch the way FBWA does
    // (FBWA maps pitch PWM 1300 / limits -1500..2000 to -750).
    assert_ne!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, -750);
}
