//! Compass filter-range: `COMPASS_FLTR_RNG` / `AP_Compass_Backend::field_ok`.

use ap_compass::filter_range::{filter_enabled, COMPASS_FLTR_RNG_DEFAULT};
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_plane::compass_filter_range_hookup::{apply_filter_range, compass_filter_range_tick};
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::SitlCompassHookup;

#[test]
fn hookup_default_range_publishes_offset_spike() {
    let mut hookup = SitlCompassHookup::default();
    let out = compass_filter_range_tick(&hookup);
    assert_eq!(out.filter_range, COMPASS_FLTR_RNG_DEFAULT);
    assert!(!filter_enabled(out.filter_range));

    hookup.truth.now_ms = 10;
    let first = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!(first.sample.have_sample);
    let baseline = first.sample.mag_body;

    let mut params = *hookup.compass_params();
    params.compass1.offset = Vector3f::new(10.0, 0.0, 0.0);
    hookup.apply_compass_params(params);
    hookup.truth.now_ms = 20;
    let second = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!((second.sample.mag_body.x - (baseline.x + 10.0)).abs() < 1e-4);
    assert_eq!(compass_filter_range_tick(&hookup).error_count, 0);
}

#[test]
fn hookup_enabled_range_rejects_offset_spike() {
    let mut hookup = SitlCompassHookup::default();
    apply_filter_range(&mut hookup, 10);
    hookup.truth.now_ms = 10;
    let first = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!(first.sample.have_sample);
    let baseline = first.sample.mag_body;

    let mut params = *hookup.compass_params();
    params.filter_range = 10;
    params.compass1.offset = Vector3f::new(10.0, 0.0, 0.0);
    hookup.apply_compass_params(params);
    hookup.truth.now_ms = 20;
    let second = hookup.publish(Matrix3f::identity(), 0.0025, None);
    assert!((second.sample.mag_body.x - baseline.x).abs() < 1e-5);
    assert!((second.sample.mag_body.y - baseline.y).abs() < 1e-5);
    assert!((second.sample.mag_body.z - baseline.z).abs() < 1e-5);
    assert_eq!(compass_filter_range_tick(&hookup).error_count, 1);
}

#[test]
fn main_loop_fltr_rng_rejects_spike() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    apply_filter_range(&mut hookup, 10);
    hookup.truth.now_ms = 10;
    let first = hookup.publish(Matrix3f::identity(), 0.0025, None);
    let baseline = first.sample.mag_body;

    let mut params = *hookup.compass_params();
    params.filter_range = 10;
    params.compass1.offset = Vector3f::new(10.0, 0.0, 0.0);
    hookup.apply_compass_params(params);
    hookup.truth.now_ms = 20;
    vehicle.sitl_compass = Some(hookup);

    vehicle.ahrs_update();
    let sample = vehicle.mag_sample.expect("mag sample");
    assert!((sample.mag_body.x - baseline.x).abs() < 1e-5);
    let hookup = vehicle.sitl_compass.as_ref().expect("sitl compass");
    let out = compass_filter_range_tick(hookup);
    assert!(out.filter_enabled);
    assert_eq!(out.error_count, 1);
}
