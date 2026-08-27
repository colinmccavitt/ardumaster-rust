//! Manual mode hookup for nav mirror and RC passthrough.

use ap_plane::landing_hookup::ServoOutputState;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::manual_mode_hookup::{
    manual_mode_nav_tick, manual_mode_servos_tick, stick_to_scaled, ManualModeNavInputs,
    ManualModeServosInputs, SERVO_MAX,
};
use ap_plane::mode_table::{BuildFeatures, ModeNumber};
use ap_plane::stabilize_hookup::RcStickInputs;

#[test]
fn manual_mode_nav_mirrors_attitude_sensors() {
    let out = manual_mode_nav_tick(&ManualModeNavInputs {
        control_mode: ModeNumber::Manual.as_number(),
        features: BuildFeatures::default(),
        roll_sensor_cd: 1200,
        pitch_sensor_cd: -800,
    });
    assert!(out.applied);
    assert_eq!(out.nav_roll_cd, 1200);
    assert_eq!(out.nav_pitch_cd, -800);
}

#[test]
fn manual_mode_nav_skips_other_modes() {
    let out = manual_mode_nav_tick(&ManualModeNavInputs {
        control_mode: ModeNumber::Stabilize.as_number(),
        features: BuildFeatures::default(),
        roll_sensor_cd: 1200,
        pitch_sensor_cd: -800,
    });
    assert!(!out.applied);
}

#[test]
fn manual_mode_servos_maps_sticks_to_scaled_outputs() {
    let out = manual_mode_servos_tick(
        ServoOutputState::default(),
        &ManualModeServosInputs {
            control_mode: ModeNumber::Manual.as_number(),
            features: BuildFeatures::default(),
            rc_sticks: RcStickInputs {
                roll_norm_dz: 0.5,
                pitch_norm_dz: -0.25,
                yaw_norm_dz: 1.0,
            },
        },
    );
    assert!(out.applied);
    assert!((out.servos.aileron_scaled - stick_to_scaled(0.5)).abs() < 1e-6);
    assert!((out.servos.rudder_scaled - SERVO_MAX).abs() < 1e-6);
    assert!(out.servos.elevator_pwm < 1500);
}

#[test]
fn manual_mode_servos_skips_fbwa() {
    let base = ServoOutputState {
        aileron_scaled: 100.0,
        ..ServoOutputState::default()
    };
    let out = manual_mode_servos_tick(
        base,
        &ManualModeServosInputs {
            control_mode: ModeNumber::FlyByWireA.as_number(),
            features: BuildFeatures::default(),
            rc_sticks: RcStickInputs {
                roll_norm_dz: 1.0,
                ..RcStickInputs::default()
            },
        },
    );
    assert!(!out.applied);
    assert_eq!(out.servos.aileron_scaled, 100.0);
}

#[test]
fn main_loop_applies_manual_mode_nav_and_servos() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ModeNumber::Manual.as_number();
    vehicle.attitude.roll_sensor_cd = 500;
    vehicle.attitude.pitch_sensor_cd = -300;
    vehicle.update_control_mode();

    assert!(vehicle.manual_mode_nav_applied);
    assert_eq!(vehicle.nav_tecs.nav_roll_cd, 500);
    assert_eq!(vehicle.navigation_scheduler_inputs.commanded_pitch_cd, -300);

    vehicle.rc_sticks = RcStickInputs {
        roll_norm_dz: 0.2,
        pitch_norm_dz: 0.0,
        yaw_norm_dz: -0.4,
    };
    vehicle.set_servos();

    assert!(vehicle.manual_mode_servos_applied);
    assert!((vehicle.servos.aileron_scaled - stick_to_scaled(0.2)).abs() < 1e-6);
    assert!((vehicle.servos.rudder_scaled - stick_to_scaled(-0.4)).abs() < 1e-6);
}
