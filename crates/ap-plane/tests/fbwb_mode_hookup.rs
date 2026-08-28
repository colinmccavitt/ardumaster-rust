//! FBWB mode hookup for cruise-assisted nav stick mapping.

use ap_plane::fbwb_mode_hookup::{fbwb_mode_nav_tick, FbwbModeNavInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::mode_table::{BuildFeatures, ModeNumber};

fn fbwb_inp() -> FbwbModeNavInputs {
    FbwbModeNavInputs {
        control_mode: ModeNumber::FlyByWireB.as_number(),
        features: BuildFeatures::default(),
        roll_norm: 0.5,
        roll_limit_cd: 4500,
    }
}

#[test]
fn fbwb_mode_nav_maps_roll_stick_to_nav_roll() {
    let out = fbwb_mode_nav_tick(&fbwb_inp());
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 2250);
}

#[test]
fn fbwb_mode_nav_skips_other_modes() {
    let mut inp = fbwb_inp();
    inp.control_mode = ModeNumber::FlyByWireA.as_number();
    let out = fbwb_mode_nav_tick(&inp);
    assert!(!out.applied);
    assert_eq!(out.nav_roll_cd, 0);
}

#[test]
fn main_loop_applies_fbwb_mode_nav_from_roll_stick() {
    use ap_plane::rc_failsafe_scheduler_hookup::{
        RcChannelConfig, RcFailsafeConfig, RcFailsafeSchedulerInputs,
    };

    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::FlyByWireB.as_number();
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

    assert!(vehicle.fbwb_mode_nav_applied);
    assert!(!vehicle.fbwa_mode_nav_applied);
    assert!(!vehicle.manual_mode_nav_applied);
    assert!(!vehicle.stabilize_mode_nav_applied);
    assert!(!vehicle.acro_mode_nav_applied);
    assert!(!vehicle.training_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 1000);
    // Cruise-assisted: elevator stick must not overwrite TECS pitch the way FBWA does
    // (FBWA maps pitch PWM 1300 / limits -1500..2000 to -750).
    assert_ne!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, -750);
}
