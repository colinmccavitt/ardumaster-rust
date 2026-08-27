//! SITL GPS fix producer hookup into yaw publish.

use ap_ahrs::GPS_SPEED_MIN;
use ap_gps::SitlGpsBackend;
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_gps_hookup::{SitlGpsHookup, SitlGpsTruth};

#[test]
fn gps_backend_producer_fills_yaw_publish_at_200ms() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth = SitlGpsTruth {
        velocity_ned: Vector3f::new(0.0, GPS_SPEED_MIN + 3.0, 0.0),
        latitude_deg: 47.0,
        longitude_deg: -122.0,
        altitude_m: 50.0,
        now_ms: 200,
    };
    hookup.compass_use_for_yaw = false;
    let samples = hookup.publish_yaw_samples(Matrix3f::identity(), 0.0025);
    let gps = samples.gps_yaw.expect("gps fix produced");
    assert!((gps.ground_speed - (GPS_SPEED_MIN + 3.0)).abs() < 1e-3);
    assert!((gps.ground_course_deg - 90.0).abs() < 1e-2);
    assert_eq!(gps.last_fix_time_ms, 200);
    assert!(samples.yaw_ctx.have_gps);
}

#[test]
fn main_loop_uses_sitl_gps_producer_before_dcm() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    let gps = vehicle.gps_yaw.expect("gps from producer");
    assert!((gps.ground_speed - 10.0).abs() < 1e-3);
    assert_eq!(vehicle.ticks.ahrs_update, 1);
}

#[test]
fn gps_lag_sec_exposed_for_drift_consumers() {
    let hookup = SitlGpsHookup::default();
    assert!((hookup.gps_lag_sec() - SitlGpsBackend::default().lag_sec()).abs() < 1e-6);
}

#[test]
fn lag_buffer_feeds_yaw_publish_with_delayed_velocity() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    let _ = hookup.publish_yaw_samples(Matrix3f::identity(), 0.0025);

    hookup.truth.velocity_ned = Vector3f::new(25.0, 0.0, 0.0);
    hookup.truth.now_ms = 450;
    let samples = hookup.publish_yaw_samples(Matrix3f::identity(), 0.0025);
    let gps = samples.gps_yaw.expect("delayed gps fix");
    assert!((gps.ground_speed - 10.0).abs() < 1e-3, "yaw uses lag-buffered speed");
    assert!((hookup.current_fix().ground_speed - 25.0).abs() < 1e-3);
    assert!((hookup.delayed_fix().ground_speed - 10.0).abs() < 1e-3);
}

#[test]
fn gps_status_publish_exposes_lag_buffered_velocity() {
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(10.0, 0.0, -2.0);
    hookup.truth.now_ms = 200;
    let status = hookup.gps_status_publish();
    assert!(status.have_fix);
    assert!(status.has_3d_fix());
    assert!((status.velocity_ned.x - 10.0).abs() < 1e-3);
    assert!((status.velocity_ned.z - (-2.0)).abs() < 1e-3);
    assert_eq!(status.num_sats, 15);
}

#[test]
fn main_loop_publishes_gps_status_from_producer() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing.delta_time = 1.0 / 400.0;
    let mut hookup = SitlGpsHookup::default();
    hookup.truth.velocity_ned = Vector3f::new(8.0, 6.0, 0.0);
    hookup.truth.now_ms = 200;
    hookup.compass_use_for_yaw = false;
    vehicle.sitl_gps = Some(hookup);

    vehicle.ahrs_update();

    let status = vehicle.gps_status.expect("gps status published");
    assert!(status.have_fix);
    assert!((status.ground_speed - 10.0).abs() < 1e-2);
    assert_eq!(vehicle.ticks.ahrs_update, 1);
}

