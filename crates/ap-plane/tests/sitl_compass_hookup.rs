//! SITL compass hookup: mag sample, health flags, and yaw drift into ahrs_update.

use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{
    hookup_with_disabled_primary, SitlCompassHookup, SitlCompassTruth,
};

#[test]
fn sitl_compass_publish_emits_body_field_and_health() {
    let mut hookup = SitlCompassHookup::default();
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let published = hookup.publish(Matrix3f::identity(), 0.0025);
    assert!(published.sample.have_sample);
    assert!(published.sample.mag_body.length() > 0.1);
    assert!(published.healthy);
    assert!(published.health.primary_healthy());
    assert!(published.yaw_compass.is_some());
}

#[test]
fn dual_compass_failover_publishes_secondary_when_primary_disabled() {
    let mut hookup = hookup_with_disabled_primary();
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let published = hookup.publish(Matrix3f::identity(), 0.0025);
    assert_eq!(published.health.instance_count, 2);
    assert_eq!(published.health.primary, 1);
    assert!(published.health.primary_healthy());
    assert!(published.healthy);
    assert!(published.yaw_compass.is_some());
}

#[test]
fn ahrs_update_wires_sitl_compass_sample_and_health_flags() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    vehicle.sitl_compass = Some(SitlCompassHookup::with_dual_backends());
    vehicle.sitl_compass.as_mut().unwrap().truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };

    vehicle.ahrs_update();

    assert!(vehicle.mag_sample.is_some());
    assert!(vehicle.compass_healthy);
    assert_eq!(vehicle.compass_health.instance_count, 2);
    assert!(vehicle.compass.is_some());
    let mag = vehicle.mag_sample.unwrap();
    assert!(mag.have_sample);
    assert!(mag.mag_body.length() > 0.1);
}
