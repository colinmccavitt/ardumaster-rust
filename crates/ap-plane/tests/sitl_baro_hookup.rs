//! SITL baro hookup: EAS2TAS, health flags, and pressure into ahrs_update.

use ap_baro::sitl::BaroHealthFlags;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_baro_hookup::{SitlBaroPublish, 
    hookup_with_disabled_primary, hookup_with_disabled_secondary, SitlBaroHookup, SitlBaroTruth,
};

#[test]
fn sitl_baro_publish_emits_sea_level_pressure_at_zero_alt() {
    let mut hookup = SitlBaroHookup::default();
    hookup.truth = SitlBaroTruth {
        sim_altitude_m: 0.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    let published = hookup.publish();
    assert!(published.sample.have_sample);
    assert!((published.sample.pressure_pa - 101_325.0).abs() < 500.0);
    assert!((published.eas2tas - 1.0).abs() < 0.01);
    assert!(published.healthy);
    assert!(published.health.primary_healthy());
}

#[test]
fn sitl_baro_eas2tas_grows_with_altitude() {
    let mut low = SitlBaroHookup::default();
    low.truth = SitlBaroTruth {
        sim_altitude_m: 0.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    let mut high = SitlBaroHookup::default();
    high.truth = SitlBaroTruth {
        sim_altitude_m: 5000.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    let low_pub = low.publish();
    let high_pub = high.publish();
    assert!(high_pub.eas2tas > low_pub.eas2tas);
}

#[test]
fn dual_baro_failover_publishes_secondary_when_primary_disabled() {
    let mut hookup = hookup_with_disabled_primary();
    hookup.truth = SitlBaroTruth {
        sim_altitude_m: 200.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    let published = hookup.publish();
    assert_eq!(published.health.instance_count, 2);
    assert_eq!(published.health.primary, 1);
    assert!(published.health.primary_healthy());
    assert!(published.healthy);
    assert!((published.sample.altitude_m - 200.0).abs() < 1.0);
}

#[test]
fn dual_baro_health_flags_expose_secondary_unhealthy() {
    let mut hookup = hookup_with_disabled_secondary();
    hookup.truth = SitlBaroTruth {
            sim_altitude_m: 50.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    let published = hookup.publish();
    assert_eq!(published.health.instance_count, 2);
    assert!(published.health.healthy[0]);
    assert!(!published.health.healthy[1]);
    assert!(published.health.any_healthy());
    assert!(published.healthy);
}

#[test]
fn ahrs_update_wires_sitl_baro_eas2tas_and_health_flags() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_baro = Some(SitlBaroHookup::with_dual_backends());
    vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
        sim_altitude_m: 1000.0,
        airspeed_bf: Vector3f::new(20.0, 0.0, 0.0),
        now_ms: 10,
        ..SitlBaroTruth::default()
    };

    vehicle.ahrs_update();

    assert!(vehicle.baro_sample.is_some());
    assert!(vehicle.baro_healthy);
    assert_eq!(vehicle.baro_health.instance_count, 2);
    assert!(vehicle.baro_health.primary_healthy());
    assert!(vehicle.eas2tas > 1.0);
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
    assert!((motion.eas2tas - vehicle.eas2tas).abs() < 1e-6);
}

#[test]
fn main_loop_pre_arm_passes_with_healthy_sitl_baro() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
        sim_altitude_m: 10.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert!(vehicle.baro_health.any_healthy());
    assert!(vehicle.baro_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn main_loop_pre_arm_refuses_when_baro_unhealthy() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    vehicle.ahrs_pre_arm_ok = true;
    vehicle.baro_health = BaroHealthFlags::default();

    vehicle.update_control_mode();

    assert!(!vehicle.baro_pre_arm_ok);
    assert!(!vehicle.pre_arm_ok);
}

#[test]
fn main_loop_pre_arm_passes_after_baro_failover() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_baro = Some(hookup_with_disabled_primary());
    vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
        sim_altitude_m: 150.0,
        now_ms: 10,
        ..SitlBaroTruth::default()
    };
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.ahrs_update();
    vehicle.update_control_mode();

    assert_eq!(vehicle.baro_health.primary, 1);
    assert!(vehicle.baro_health.primary_healthy());
    assert!(vehicle.baro_pre_arm_ok);
    assert!(vehicle.pre_arm_ok);
}

#[test]
fn sitl_baro_publish_includes_climb_rate() {
    let mut hookup = SitlBaroHookup::default();
    let rate_mps = 3.0;
    let dt_ms = 10;
    let step_m = rate_mps * dt_ms as f32 * 0.001;
    let mut alt = 50.0_f32;
    let mut t = 0_u32;
    let mut last = SitlBaroPublish::default();
    for _ in 0..25 {
        t += dt_ms;
        alt += step_m;
        hookup.truth = SitlBaroTruth {
            sim_altitude_m: alt,
            now_ms: t,
            ..SitlBaroTruth::default()
        };
        last = hookup.publish();
    }
    assert!(last.climb_rate_mps > 0.5);
    assert!(
        (last.climb_rate_mps - rate_mps).abs() < 0.5,
        "got {}",
        last.climb_rate_mps
    );
}

#[test]
fn ahrs_update_publishes_baro_climb_rate() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    let rate_mps = 2.5;
    let dt_ms = 10;
    let step_m = rate_mps * dt_ms as f32 * 0.001;
    let mut alt = 120.0_f32;
    let mut t = 0_u32;
    for _ in 0..25 {
        t += dt_ms;
        alt += step_m;
        vehicle.sitl_baro.as_mut().unwrap().truth = SitlBaroTruth {
            sim_altitude_m: alt,
            now_ms: t,
            ..SitlBaroTruth::default()
        };
        vehicle.ahrs_update();
    }
    assert!(vehicle.baro_climb_rate_mps > 0.5);
    assert!((vehicle.baro_climb_rate_mps - rate_mps).abs() < 0.5);
}

#[test]
fn main_loop_pre_arm_refuses_before_first_baro_tick() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.sitl_baro = Some(SitlBaroHookup::default());
    vehicle.ahrs_pre_arm_ok = true;

    vehicle.update_control_mode();

    assert!(!vehicle.baro_health.primary_healthy());
    assert!(!vehicle.baro_pre_arm_ok);
    assert!(!vehicle.pre_arm_ok);
}
