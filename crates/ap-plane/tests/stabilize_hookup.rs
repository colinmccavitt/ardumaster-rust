//! Stabilize controller hookup in the main loop.

use ap_control::{RateGains, RollController};
use ap_ins::ImuInstance;
use ap_math::scalar::cd_to_rad;
use ap_math::vector3::Vector3f;
use ap_pid::PidGains;
use ap_plane::ahrs_hookup::AhrsAttitude;
use ap_plane::main_loop::{PlaneMainLoop, StabilizeDispatch};
use ap_plane::stabilize_hookup::{
    apply_stabilize_to_servos, scaled_to_pwm_trim, stabilize_controllers, StabilizeContext,
    StabilizeControllers, StabilizeDemands, StabilizeServoDemands,
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
    vehicle.stabilize_demands.nav_roll_cd = 2000;
    vehicle.stabilize_demands.roll_limit_cd = 4500;
    vehicle.attitude.roll_sensor_cd = 0;

    vehicle.stabilize();
    vehicle.set_servos();

    assert!(vehicle.last_stabilize_run.roll);
    assert!(vehicle.last_stabilize_run.pitch);
    assert!(vehicle.last_stabilize_run.yaw);
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
