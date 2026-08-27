//! Stabilize controller hookup in the main loop.

use ap_control::{RateGains, RollController};
use ap_ins::ImuInstance;
use ap_math::scalar::cd_to_rad;
use ap_math::vector3::Vector3f;
use ap_pid::PidGains;
use ap_plane::ahrs_hookup::AhrsAttitude;
use ap_plane::main_loop::{PlaneMainLoop, StabilizeDispatch};
use ap_plane::mode_run::StickMixing;
use ap_plane::stabilize_hookup::{
    apply_stabilize_to_servos, calc_nav_demands, calc_speed_scaler, nonlinear_stick_input,
    prepare_stabilize_path, scaled_to_pwm_trim, stabilize_controllers, stabilize_stick_mixing_fbw,
    NavCommandInputs, RcStickInputs, SpeedScalerInputs, StabilizeContext, StabilizeControllers,
    StabilizeDemands, StabilizeServoDemands,
};

#[test]
fn dispatch_flags_select_which_controllers_run() {
    let mut controllers = StabilizeControllers::default();
    let attitude = AhrsAttitude::default();
    let imu = ImuInstance::default();
    let demands = StabilizeDemands::default();
    let ctx = StabilizeContext::default();

    let out = stabilize_controllers(
        &mut controllers,
        &attitude,
        &imu,
        StabilizeDispatch {
            roll: true,
            pitch: false,
            yaw: true,
            fbw_stick_mixing: false,
        },
        &demands,
        &ctx,
        1.0 / 400.0,
    );

    assert_eq!(
        out.run,
        ap_plane::main_loop::StabilizeRun {
            roll: true,
            pitch: false,
            yaw: true,
        }
    );
}

#[test]
fn roll_servo_out_uses_attitude_sensor_and_nav_demand() {
    let pid = PidGains {
        p: 0.08,
        i: 0.0,
        d: 0.0,
        ff: 0.0,
        dff: 0.0,
        imax: 0.0,
        pdmax: 0.0,
        filt_t_hz: 20.0,
        filt_e_hz: 0.0,
        filt_d_hz: 0.0,
        srmax: 0.0,
        srtau: 1.0,
    };
    let mut controllers = StabilizeControllers {
        roll: RollController::new(pid, RateGains::default()),
        ..StabilizeControllers::default()
    };
    let attitude = AhrsAttitude {
        roll_sensor_cd: 0,
        pitch_sensor_cd: 0,
        yaw_sensor_cd: 0,
    };
    let mut imu = ImuInstance::default();
    imu.notify_gyro_raw_sample(Vector3f::new(cd_to_rad(100.0), 0.0, 0.0), 0, 400, 2500);
    imu.update_gyro();

    let out = stabilize_controllers(
        &mut controllers,
        &attitude,
        &imu,
        StabilizeDispatch {
            roll: true,
            pitch: false,
            yaw: false,
            fbw_stick_mixing: false,
        },
        &StabilizeDemands {
            nav_roll_cd: 3000,
            roll_limit_cd: 4500,
            ..StabilizeDemands::default()
        },
        &StabilizeContext {
            scaler: 1.0,
            airspeed_eas: Some(15.0),
            airspeed_min: 10,
            now_ms: 100,
            ..StabilizeContext::default()
        },
        1.0 / 400.0,
    );

    assert!(out.run.roll);
    assert_eq!(out.servos.elevator_scaled, 0.0);
    assert_eq!(out.servos.rudder_scaled, 0.0);
    assert!(out.servos.aileron_scaled.is_finite());
}

#[test]
fn calc_nav_demands_limits_roll_and_pitch() {
    let mut demands = StabilizeDemands {
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
        ..StabilizeDemands::default()
    };
    calc_nav_demands(
        &mut demands,
        &NavCommandInputs {
            commanded_roll_cd: 9000,
            commanded_pitch_cd: 5000,
        },
    );
    assert_eq!(demands.nav_roll_cd, 4500);
    assert_eq!(demands.nav_pitch_cd, 2500);
}

#[test]
fn calc_speed_scaler_matches_cruise_airspeed() {
    let scaler = calc_speed_scaler(&SpeedScalerInputs {
        airspeed_eas: Some(15.0),
        scaling_speed: 15.0,
        airspeed_min: 10.0,
        airspeed_max: 30.0,
        armed: true,
        throttle_scaled: 50.0,
    });
    assert!((scaler - 1.0).abs() < 0.01);
}

#[test]
fn calc_speed_scaler_without_airspeed_uses_throttle_when_armed() {
    let scaler = calc_speed_scaler(&SpeedScalerInputs {
        airspeed_eas: None,
        scaling_speed: 15.0,
        airspeed_min: 10.0,
        airspeed_max: 30.0,
        armed: true,
        throttle_scaled: 45.0,
    });
    assert!((scaler - 1.0).abs() < 0.01);
}

#[test]
fn nonlinear_stick_input_doubles_at_full_deflection() {
    assert!((nonlinear_stick_input(1.0) - 2.0).abs() < 0.001);
    assert!((nonlinear_stick_input(-1.0) - (-2.0)).abs() < 0.001);
    assert!((nonlinear_stick_input(0.25) - 0.25).abs() < 0.001);
}

#[test]
fn stick_mixing_shifts_nav_roll() {
    let mut demands = StabilizeDemands {
        nav_roll_cd: 0,
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
        ..StabilizeDemands::default()
    };
    stabilize_stick_mixing_fbw(
        &mut demands,
        &RcStickInputs {
            roll_norm_dz: 1.0,
            pitch_norm_dz: 0.0,
        },
        true,
        false,
    );
    assert_eq!(demands.nav_roll_cd, 4500);
}

#[test]
fn prepare_stabilize_path_applies_stick_mixing_when_enabled() {
    let mut demands = StabilizeDemands {
        roll_limit_cd: 4500,
        pitch_limit_min_cd: -2000,
        pitch_limit_max_cd: 2500,
        ..StabilizeDemands::default()
    };
    let mut ctx = StabilizeContext::default();
    prepare_stabilize_path(
        &mut demands,
        &mut ctx,
        &NavCommandInputs {
            commanded_roll_cd: 0,
            commanded_pitch_cd: 0,
        },
        &SpeedScalerInputs {
            airspeed_eas: Some(15.0),
            scaling_speed: 15.0,
            airspeed_min: 10.0,
            airspeed_max: 30.0,
            armed: true,
            throttle_scaled: 50.0,
        },
        StabilizeDispatch {
            roll: true,
            pitch: true,
            yaw: true,
            fbw_stick_mixing: true,
        },
        &RcStickInputs {
            roll_norm_dz: 0.5,
            pitch_norm_dz: 0.0,
        },
        Some(StickMixing::Fbw),
        0,
    );
    assert_eq!(demands.nav_roll_cd, 2250);
    assert!((ctx.scaler - 1.0).abs() < 0.01);
}

#[test]
fn set_servos_publishes_stabilize_demands() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.last_stabilize = StabilizeDispatch {
        roll: true,
        pitch: true,
        yaw: false,
        fbw_stick_mixing: false,
    };
    vehicle.stabilize_servos = StabilizeServoDemands {
        aileron_scaled: 500.0,
        elevator_scaled: -250.0,
        rudder_scaled: 0.0,
    };

    vehicle.set_servos();

    assert_eq!(vehicle.ticks.set_servos, 1);
    assert_eq!(vehicle.servos.aileron_scaled, 500.0);
    assert_eq!(vehicle.servos.elevator_pwm, scaled_to_pwm_trim(-250.0));
}

#[test]
fn main_loop_stabilize_and_set_servos_wire_controllers() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.mode.control_mode = ap_plane::mode_table::ModeNumber::Stabilize.as_number();
    vehicle.update_control_mode();
    vehicle.nav_tecs.nav_roll_cd = 2000;
    vehicle.stabilize_demands.roll_limit_cd = 4500;
    vehicle.speed_scaler_inputs.airspeed_eas = Some(15.0);
    vehicle.speed_scaler_inputs.scaling_speed = 15.0;
    vehicle.speed_scaler_inputs.airspeed_min = 10.0;
    vehicle.speed_scaler_inputs.airspeed_max = 30.0;
    vehicle.attitude.roll_sensor_cd = 0;

    vehicle.stabilize();
    vehicle.set_servos();

    assert!(vehicle.last_stabilize_run.roll);
    assert!(vehicle.last_stabilize_run.pitch);
    assert!(vehicle.last_stabilize_run.yaw);
    assert_eq!(vehicle.stabilize_demands.nav_roll_cd, 2000);
}

#[test]
fn apply_stabilize_to_servos_copies_all_axes() {
    let stabilize = StabilizeServoDemands {
        aileron_scaled: 100.0,
        elevator_scaled: 200.0,
        rudder_scaled: -50.0,
    };
    let mut servos = ap_plane::landing_hookup::ServoOutputState::default();

    apply_stabilize_to_servos(&stabilize, &mut servos);

    assert_eq!(servos.aileron_scaled, 100.0);
    assert_eq!(servos.rudder_scaled, -50.0);
    assert_eq!(servos.elevator_pwm, scaled_to_pwm_trim(200.0));
}
