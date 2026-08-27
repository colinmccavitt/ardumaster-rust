//! SITL baro hookup: EAS2TAS, health flags, and pressure into ahrs_update.

use ap_baro::sitl::BaroHealthFlags;
use ap_ins::LoopTiming;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_baro_hookup::{hookup_with_disabled_secondary, SitlBaroHookup, SitlBaroTruth};

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
