//! Compass soft-iron stub: COMPASS_DIA / COMPASS_ODI.

use ap_compass::params::CompassParams;
use ap_compass::sitl::mag_field_body_ned;
use ap_compass::soft_iron::apply_soft_iron;
use ap_ins::LoopTiming;
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;
use ap_plane::compass_soft_iron_hookup::compass_soft_iron_tick;
use ap_plane::main_loop::PlaneMainLoop;
use ap_plane::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

#[test]
fn hookup_soft_iron_scales_published_field() {
    let dia = Vector3f::new(1.1, 0.9, 1.0);
    let odi = Vector3f::zero();
    let mut hookup = SitlCompassHookup::with_dual_backends();
    let mut params = CompassParams::default();
    params.compass1.diagonals = dia;
    params.compass1.offdiagonals = odi;
    params.compass2.diagonals = dia;
    params.compass2.offdiagonals = odi;
    hookup.apply_compass_params(params);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };

    let out = compass_soft_iron_tick(&hookup);
    assert!((out.diagonals.x - 1.1).abs() < 1e-6);
    assert!(out.applied);

    let attitude = Matrix3f::identity();
    let published = hookup.publish(attitude, 0.0025, None);
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, attitude);
    let expected = apply_soft_iron(wmm, dia, odi);
    assert!((published.sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((published.sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((published.sample.mag_body.z - expected.z).abs() < 1e-5);
}

#[test]
fn main_loop_soft_iron_applies_to_mag_sample() {
    let dia = Vector3f::new(1.1, 0.9, 1.0);
    let mut vehicle = PlaneMainLoop::default();
    vehicle.loop_timing = LoopTiming::new(1.0 / 400.0);
    let mut hookup = SitlCompassHookup::default();
    let mut params = CompassParams::default();
    params.compass1.diagonals = dia;
    hookup.apply_compass_params(params);
    hookup.truth = SitlCompassTruth {
        latitude_deg: 51.875,
        longitude_deg: -0.154,
        now_ms: 10,
    };
    vehicle.sitl_compass = Some(hookup);

    vehicle.ahrs_update();
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = vehicle.mag_sample.expect("mag sample");
    let expected = apply_soft_iron(wmm, dia, Vector3f::zero());
    assert!((sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((sample.mag_body.z - expected.z).abs() < 1e-5);
}
