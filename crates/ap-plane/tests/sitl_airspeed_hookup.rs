//! SITL airspeed hookup: pitot TAS/EAS, health flags, and TECS use_airspeed path.

use ap_baro::eas2tas_for_alt_amsl;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_airspeed_hookup::{
    hookup_with_disabled_primary, SitlAirspeedHookup, SitlAirspeedTruth,
};
use ap_plane::sitl_baro_hookup::{SitlBaroHookup, SitlBaroTruth};

#[test]
fn sitl_airspeed_publish_emits_pitot_tas_and_health() {
    let mut hookup = SitlAirspeedHookup::default();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert!(published.sample.have_sample);
    assert!((published.sample.tas_mps - 20.0).abs() < 1e-6);
    assert!((published.sample.eas_mps - 20.0).abs() < 1e-6);
    assert!(published.healthy);
    assert!(published.health.primary_healthy());
}

#[test]
fn sitl_airspeed_eas_scales_with_eas2tas() {
    let mut hookup = SitlAirspeedHookup::default();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(25.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.25);
    assert!((published.sample.tas_mps - 25.0).abs() < 1e-6);
    assert!((published.sample.eas_mps - 20.0).abs() < 1e-6);
}

#[test]
fn dual_airspeed_failover_publishes_secondary_when_primary_disabled() {
    let mut hookup = hookup_with_disabled_primary();
    hookup.truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(18.0, 0.0, 0.0),
        now_ms: 10,
    };
    let published = hookup.publish(1.0);
    assert_eq!(published.health.instance_count, 2);
    assert_eq!(published.health.primary, 1);
    assert!(published.health.primary_healthy());
    assert!(published.healthy);
    assert!((published.sample.tas_mps - 18.0).abs() < 1e-6);
}

#[test]
fn ahrs_update_wires_sitl_airspeed_tas_and_health_flags() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(22.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();

    assert!(vehicle.airspeed_sample.is_some());
    assert!(vehicle.airspeed_healthy);
    assert_eq!(vehicle.airspeed_health.instance_count, 2);
    assert!((vehicle.airspeed_tas - 22.0).abs() < 1e-6);
    assert_eq!(vehicle.ticks.ahrs_update, 1);

    use ap_plane::ahrs_hookup::drift_motion_inputs;
    let mut last_fix = 0_u32;
    let motion = drift_motion_inputs(
        vehicle.yaw_ctx,
        vehicle.gps_yaw,
        vehicle.gps_velocity,
        vehicle.airspeed_tas,
        vehicle.eas2tas,
        &mut last_fix,
    );
    assert!((motion.airspeed_tas - 22.0).abs() < 1e-6);
}

#[test]
fn ahrs_update_tecs_use_airspeed_when_pitot_healthy() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::default());
    let eas2tas = eas2tas_for_alt_amsl(0.0);
    vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
        sim_altitude_m: 0.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(15.0, 0.0, 0.0),
        now_ms: 10,
    };

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert!(vehicle.airspeed_healthy);
    assert!(vehicle.airspeed_tas > 1.0);
    assert!(vehicle.last_altitude_tecs_ran);
    assert!(vehicle.eas2tas > 0.99);
    let _ = eas2tas;
}

#[test]
fn main_loop_pre_arm_passes_with_healthy_sitl_airspeed() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::with_dual_backends());
    vehicle.sitl_airspeed.as_mut().unwrap().truth = SitlAirspeedTruth {
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
    };
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert!(vehicle.airspeed_health.primary_healthy());
    assert!(vehicle.airspeed_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn main_loop_pre_arm_refuses_when_airspeed_unhealthy() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.sitl_airspeed = Some(SitlAirspeedHookup::default());
    vehicle.ahrs_pre_arm_ok = true;
    vehicle.airspeed_health = ap_airspeed::sitl::AirspeedHealthFlags::default();

    vehicle.update_control_mode();

    assert!(!vehicle.airspeed_pre_arm_ok);
    assert!(!vehicle.pre_arm_ok);
}

#[test]
fn main_loop_pre_arm_passes_after_airspeed_failover() {
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
    assert!(vehicle.airspeed_health.primary_healthy());
    assert!(vehicle.airspeed_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

