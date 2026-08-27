//! INS harmonic notch scheduler hookup in the plane main loop.

use ap_ins::sitl::{SitlBodyState, SitlImuBackend, SitlInsCluster};
use ap_ins::{InsHntchParams, SitlInsMotorRuntime, SitlInsNoiseParams};
use ap_plane::ins_hntch_scheduler_hookup::{InsHntchHookup, InsHntchSchedulerInputs};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_ins_noise_hookup::SitlInsNoiseHookup;

#[test]
fn main_loop_configures_hntch_on_vehicle_ins() {
    let mut vehicle = PlaneMainLoop::default();
    let mut hntch = InsHntchHookup::default();
    hntch.params = InsHntchParams {
        enable: true,
        freq_hz: 80.0,
        bandwidth_hz: 40.0,
        attenuation_db: 40.0,
        harmonics: 1,
        mode: 0,
        ..InsHntchParams::default()
    };
    vehicle.ins_hntch = Some(hntch);
    vehicle.ins.register_sitl_backend(8000, 1000).unwrap();

    vehicle.ahrs_update();

    assert!(vehicle.ins.imu(0).unwrap().gyro_notch_is_initialised());
}

#[test]
fn main_loop_runs_hntch_before_sitl_cluster_samples() {
    let mut vehicle = PlaneMainLoop::default();
    let mut noise = SitlInsNoiseHookup {
        cluster: SitlInsCluster::new(),
        noise_params: SitlInsNoiseParams::default(),
        file_playback_params: Default::default(),
    };
    noise.cluster.register(SitlImuBackend::new(1000, 1000)).unwrap();
    vehicle.sitl_ins_noise = Some(noise);
    let mut hntch = InsHntchHookup::default();
    hntch.params = InsHntchParams {
        enable: true,
        freq_hz: 80.0,
        bandwidth_hz: 40.0,
        attenuation_db: 40.0,
        harmonics: 1,
        mode: 0,
        ..InsHntchParams::default()
    };
    vehicle.ins_hntch = Some(hntch);
    vehicle.sitl_ins_motor = SitlInsMotorRuntime::default();
    vehicle.sitl_body = SitlBodyState {
        z_accel: -9.80665,
        ..SitlBodyState::default()
    };
    vehicle.sitl_now_us = 0;

    vehicle.ahrs_update();

    let hookup = vehicle.sitl_ins_noise.as_ref().unwrap();
    assert!(hookup.cluster.backend(0).unwrap().imu.gyro_notch_is_initialised());
    assert!(vehicle.ins.imu(0).unwrap().gyro_notch_is_initialised());
}

#[test]
fn throttle_tracking_retunes_notch_in_main_loop() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.ins.register_sitl_backend(1000, 1000).unwrap();
    vehicle.ins_hntch = Some({
        let mut hntch = InsHntchHookup::default();
        hntch.params = InsHntchParams {
            enable: true,
            freq_hz: 100.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
            harmonics: 1,
            reference: 1.0,
            mode: 1,
            freq_min_ratio: 0.5,
            ..InsHntchParams::default()
        };
        hntch
    });
    vehicle.sitl_ins_motor = SitlInsMotorRuntime {
        motors_on: true,
        throttle: 1.0,
        ..SitlInsMotorRuntime::default()
    };

    vehicle.ahrs_update();
    vehicle.sitl_ins_motor.throttle = 0.25;
    for _ in 0..32 {
        vehicle.ahrs_update();
    }

    let center = vehicle
        .ins
        .imu(0)
        .unwrap()
        .gyro_notch_center(0)
        .expect("notch");
    assert!((center - 50.0).abs() < 1.0, "quarter throttle -> 50 Hz, got {center}");
}
