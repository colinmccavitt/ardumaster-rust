//! Integration tests for baro arm calibration hookup.

use ap_plane::baro_arm_calibration_hookup::BaroArmCalibrationInputs;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_baro_hookup::{SitlBaroHookup, SitlBaroTruth};

#[test]
fn arm_calibration_zeros_relative_altitude_at_arm_point() {
    let mut hookup = SitlBaroHookup::default();
    hookup.truth = SitlBaroTruth {
        sim_altitude_m: 200.0,
        now_ms: 1000,
        ..SitlBaroTruth::default()
    };
    let _ = hookup.publish();
    let out = hookup.arm_calibration_tick(BaroArmCalibrationInputs {
        soft_armed: true,
        was_soft_armed: false,
    });
    assert!(out.latched);
    hookup.truth.now_ms = 2000;
    let published = hookup.publish();
    let rel = published.relative_altitude_m.expect("relative alt");
    assert!(rel.abs() < 2.0, "expected ~0 m at arm point, got {rel}");
}

#[test]
fn dual_instance_arm_calibration_latches_both_instances() {
    let mut hookup = SitlBaroHookup::with_dual_backends();
    hookup.truth = SitlBaroTruth {
        sim_altitude_m: 150.0,
        now_ms: 1000,
        ..SitlBaroTruth::default()
    };
    let _ = hookup.publish();
    let out = hookup.arm_calibration_tick(BaroArmCalibrationInputs {
        soft_armed: true,
        was_soft_armed: false,
    });
    assert!(out.latched);
    assert!(hookup.frontend().is_calibrated(0));
    assert!(hookup.frontend().is_calibrated(1));
}

#[test]
fn main_loop_arm_calibration_latches_on_soft_armed_rising_edge() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    if let Some(baro) = vehicle.sitl_baro.as_mut() {
        baro.truth.sim_altitude_m = 100.0;
        baro.truth.now_ms = 1000;
    }
    vehicle.ahrs_update();
    assert!(!vehicle.baro_arm_calibration_latched);
    vehicle.soft_armed = true;
    vehicle.ahrs_update();
    assert!(vehicle.baro_arm_calibration_latched);
    vehicle.soft_armed = false;
    vehicle.ahrs_update();
    assert!(!vehicle.baro_arm_calibration_latched);
}
