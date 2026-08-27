//! Compass health scheduler: mag publish, failover, and main-loop wiring.

use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_plane::compass_health_scheduler_hookup::{
    compass_health_scheduler_tick, CompassHealthSchedulerInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{
    hookup_with_disabled_primary, SitlCompassHookup, SitlCompassTruth,
};

#[test]
fn main_loop_health_scheduler_publishes_dual_instance_flags() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_compass = Some(SitlCompassHookup::with_dual_backends());
    vehicle.sitl_compass.as_mut().unwrap().truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };

    vehicle.ahrs_update();

    assert_eq!(vehicle.compass_health.instance_count, 2);
    assert!(vehicle.compass_healthy);
    assert!(vehicle.compass_health.primary_healthy());
    assert!(vehicle.mag_sample.is_some());
    assert!(vehicle.compass.is_some());
    let mag = vehicle.mag_sample.unwrap();
    assert!(mag.have_sample);
    assert!(mag.mag_body.length() > 0.1);
}

#[test]
fn main_loop_health_scheduler_failover_keeps_pre_arm_ok() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_compass = Some(hookup_with_disabled_primary());
    vehicle.sitl_compass.as_mut().unwrap().truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert_eq!(vehicle.compass_health.primary, 1);
    assert!(vehicle.compass_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn scheduler_tick_reports_primary_switch_on_failover() {
    let mut hookup = hookup_with_disabled_primary();
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let out = compass_health_scheduler_tick(
        &mut hookup,
        &CompassHealthSchedulerInputs {
            attitude: Matrix3f::identity(),
            loop_dt: 0.0025,
            gps: None,
        },
    );
    assert!(out.primary_switched);
    assert_eq!(out.health.primary, 1);
    assert!(out.sample.have_sample);
    assert!(out.yaw_compass.is_some());
}
