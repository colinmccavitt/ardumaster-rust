//! Compass field-strength / expected-field check stub.

use ap_compass::field::{
    expected_earth_field_ga, COMPASS_MAGFIELD_ERROR_THRESHOLD, COMPASS_MAGFIELD_MAX,
    COMPASS_MAGFIELD_MIN,
};
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_plane::compass_field_hookup::compass_field_tick;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

#[test]
fn hookup_published_wmm_field_matches_expected() {
    let mut hookup = SitlCompassHookup::default();
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let attitude = Matrix3f::identity();
    let published = hookup.publish(attitude, 0.0025, None);
    let out = compass_field_tick(&hookup, published.sample.mag_body, attitude);
    assert!(out.length_ok);
    assert!(out.expected_ok);
    assert!(out.field_ok);
    assert!(out.length_mgauss >= COMPASS_MAGFIELD_MIN);
    assert!(out.length_mgauss <= COMPASS_MAGFIELD_MAX);
    let earth = expected_earth_field_ga(51.875, -0.154);
    assert!((published.sample.mag_body.x - earth.x).abs() < 1e-5);
    assert!((published.sample.mag_body.y - earth.y).abs() < 1e-5);
    assert!((published.sample.mag_body.z - earth.z).abs() < 1e-5);
    let _ = COMPASS_MAGFIELD_ERROR_THRESHOLD;
}

#[test]
fn main_loop_offset_fails_expected_field() {
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    let mut params = *hookup.compass_params();
    // 0.2 gauss = 200 mG on X, above the 100 mG XY threshold.
    params.compass1.offset = Vector3f::new(0.2, 0.0, 0.0);
    hookup.apply_compass_params(params);
    vehicle.sitl_compass = Some(hookup);

    vehicle.ahrs_update();
    let sample = vehicle.mag_sample.expect("mag sample");
    let hookup = vehicle.sitl_compass.as_ref().expect("sitl compass");
    let out = compass_field_tick(hookup, sample.mag_body, Matrix3f::identity());
    assert!(!out.expected_ok);
    assert!(!out.field_ok);
}
