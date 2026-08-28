//! Compass scale factor stub: COMPASS_SCALE.

use ap_compass::params::CompassParams;
use ap_compass::scale::{
    apply_scale, have_scale_factor, COMPASS_MAX_SCALE_FACTOR, COMPASS_MIN_SCALE_FACTOR,
    COMPASS_SCALE_DEFAULT,
};
use ap_compass::sitl::{mag_field_body_ned, SitlCompassBackend, SitlCompassConfig};
use ap_math::matrix3::Matrix3f;

#[test]
fn compass_params_scale_default_is_no_scaling() {
    let params = CompassParams::default();
    assert!((params.compass1.scale - COMPASS_SCALE_DEFAULT).abs() < 1e-6);
    assert!(!have_scale_factor(params.compass1.scale));
}

#[test]
fn scale_multiplies_published_field() {
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        scale: 1.1,
        ..SitlCompassConfig::default()
    });
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = compass.update().expect("pending sample");
    let expected = apply_scale(wmm, 1.1);
    assert!((sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((sample.mag_body.z - expected.z).abs() < 1e-5);
}

#[test]
fn default_scale_leaves_published_field() {
    let mut compass = SitlCompassBackend::default();
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = compass.update().expect("pending sample");
    assert!((sample.mag_body.x - wmm.x).abs() < 1e-5);
    assert!((sample.mag_body.y - wmm.y).abs() < 1e-5);
    assert!((sample.mag_body.z - wmm.z).abs() < 1e-5);
}

#[test]
fn params_apply_scale_to_backend() {
    let mut params = CompassParams::default();
    params.compass1.scale = 1.1;
    let mut backend = SitlCompassBackend::default();
    params.apply_instance(0, &mut backend);
    assert!((backend.config().scale - 1.1).abs() < 1e-6);
    assert!(have_scale_factor(backend.config().scale));
    assert!(COMPASS_MIN_SCALE_FACTOR < 1.0);
    assert!(COMPASS_MAX_SCALE_FACTOR > 1.0);
}
