//! Airspeed health scheduler: pitot publish, failover, and main-loop wiring.

use ap_math::vector3::Vector3f;
use ap_plane::airspeed_health_scheduler_hookup::{
    airspeed_health_scheduler_tick, AirspeedHealthSchedulerInputs,
};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{
    hookup_with_disabled_primary, SitlAirspeedHookup, SitlAirspeedTruth,
};

#[test]
fn main_loop_health_scheduler_publishes_dual_instance_flags() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(22.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();

    assert_eq!(vehicle.airspeed_health.instance_count, 2);
    assert!(vehicle.airspeed_healthy);
    assert!(vehicle.airspeed_health.primary_healthy());
    assert!((vehicle.airspeed_tas - 22.0).abs() < 1e-6);
}

#[test]
fn main_loop_health_scheduler_failover_keeps_pre_arm_ok() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_airspeed = Some(hookup_with_disabled_primary());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(18.0, 0.0, 0.0),
        now_ms: 10,
    };
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert_eq!(vehicle.airspeed_health.primary, 1);
    assert!(vehicle.airspeed_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn scheduler_tick_reports_primary_switch_on_failover() {
    let mut hookup = hookup_with_disabled_primary();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(16.0, 0.0, 0.0),
        now_ms: 10,
    };
    let out = airspeed_health_scheduler_tick(
        &mut hookup,
        &AirspeedHealthSchedulerInputs { eas2tas: 1.0 },
    );
    assert!(out.primary_switched);
    assert_eq!(out.health.primary, 1);
    assert!((out.sample.tas_mps - 16.0).abs() < 1e-6);
}
