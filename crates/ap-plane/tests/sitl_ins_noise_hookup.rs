//! SITL INS noise hookup in the plane scheduler tick.

use ap_ins::sitl::{SitlBodyState, SitlImuBackend, SitlInsCluster};
use ap_ins::{SitlInsFilePlaybackParams, SitlInsMotorRuntime, SitlInsNoiseParams};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_ins_noise_hookup::SitlInsNoiseHookup;
use ap_math::vector3::Vector3f;
use ap_sim::{AttitudeSim, PlaneMotorFrame};

fn motor_runtime_from_frame(frame: PlaneMotorFrame) -> SitlInsMotorRuntime {
    SitlInsMotorRuntime {
        motors_on: frame.motors_on,
        throttle: (frame.throttle_pct / 100.0) as f32,
        motor_rpm: frame.motor_rpm.map(|rpm| rpm as f32),
        ..SitlInsMotorRuntime::default()
    }
}

fn body_from_sim(sim: &mut AttitudeSim) -> SitlBodyState {
    let sample = sim.step(ap_sim::level(0.0), 1.0 / 400.0);
    SitlBodyState {
        roll_rate_dps: (sample.gyro.x * 180.0 / core::f64::consts::PI) as f32,
        pitch_rate_dps: (sample.gyro.y * 180.0 / core::f64::consts::PI) as f32,
        yaw_rate_dps: (sample.gyro.z * 180.0 / core::f64::consts::PI) as f32,
        x_accel: sample.accel.x as f32,
        y_accel: sample.accel.y as f32,
        z_accel: sample.accel.z as f32,
        ..SitlBodyState::default()
    }
}

#[test]
fn scheduler_tick_feeds_noisy_ins_into_ahrs_update() {
    let mut vehicle = PlaneMainLoop::default();
    let mut hookup = SitlInsNoiseHookup {
        cluster: SitlInsCluster::new(),
        noise_params: SitlInsNoiseParams::default(),
        file_playback_params: SitlInsFilePlaybackParams::default(),
    };
    hookup.cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
    vehicle.sitl_ins_noise = Some(hookup);
    vehicle.sitl_ins_motor = motor_runtime_from_frame(PlaneMotorFrame::from_throttle_pct(80.0));

    let mut sim = AttitudeSim::new();
    vehicle.sitl_body = body_from_sim(&mut sim);
    vehicle.sitl_now_us = 0;

    vehicle.ahrs_update();

    assert_eq!(vehicle.ticks.ahrs_update, 1);
    assert!(vehicle.ins.primary_imu().is_some());
}

#[test]
fn plane_motor_frame_converts_to_runtime() {
    let frame = PlaneMotorFrame::from_throttle_pct(50.0);
    let runtime = motor_runtime_from_frame(frame);
    assert!(runtime.motors_on);
    assert!((runtime.throttle - 0.5).abs() < 1e-5);
}

#[test]
fn hookup_tick_runs_before_ins_publish_in_main_loop() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.sitl_ins_noise = Some(SitlInsNoiseHookup::with_default_backend());
    vehicle.sitl_ins_motor = SitlInsMotorRuntime {
        motors_on: true,
        throttle: 1.0,
        ..SitlInsMotorRuntime::default()
    };
    vehicle.sitl_body = SitlBodyState {
        z_accel: -9.80665,
        ..SitlBodyState::default()
    };
    vehicle.sitl_now_us = 0;

    vehicle.ahrs_update();

    let hookup = vehicle.sitl_ins_noise.as_ref().unwrap();
    assert!(hookup
        .cluster
        .backend(0)
        .unwrap()
        .noise_config
        .is_some());
}


#[test]
fn file_playback_runtime_feeds_recorded_accel_into_ahrs_update() {
    fn encode(v: Vector3f) -> [u8; 12] {
        let mut out = [0_u8; 12];
        for (i, c) in [v.x, v.y, v.z].into_iter().enumerate() {
            out[i * 4..i * 4 + 4].copy_from_slice(&c.to_le_bytes());
        }
        out
    }

    let mut vehicle = PlaneMainLoop::default();
    let mut hookup = SitlInsNoiseHookup {
        cluster: SitlInsCluster::new(),
        noise_params: SitlInsNoiseParams::default(),
        file_playback_params: SitlInsFilePlaybackParams::from_sim_file_rw(1, 0),
    };
    hookup.cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
    vehicle.sitl_ins_noise = Some(hookup);
    let frame = encode(Vector3f::new(0.0, 0.0, -4.0));
    vehicle.sitl_ins_host_files[0].accel[..12].copy_from_slice(&frame);
    vehicle.sitl_ins_host_files[0].accel_len = 12;
    vehicle.sitl_body = SitlBodyState { z_accel: -9.80665, ..SitlBodyState::default() };
    vehicle.sitl_now_us = 0;
    vehicle.ahrs_update();
    let imu = vehicle.ins.primary_imu().expect("primary IMU");
    assert!((imu.accel().z + 4.0).abs() < 1e-3, "got {}", imu.accel().z);
}
