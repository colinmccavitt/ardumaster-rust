//! Compass soft-iron stub: COMPASS_DIA / COMPASS_ODI.

use ap_compass::params::CompassParams;
use ap_compass::sitl::{mag_field_body_ned, SitlCompassBackend, SitlCompassConfig};
use ap_compass::soft_iron::{
    apply_soft_iron, have_diagonals, COMPASS_DIA_DEFAULT, COMPASS_ODI_DEFAULT,
};
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;

#[test]
fn compass_params_soft_iron_default_is_identity() {
    let params = CompassParams::default();
    assert!((params.compass1.diagonals.x - COMPASS_DIA_DEFAULT.x).abs() < 1e-6);
    assert!((params.compass1.diagonals.y - COMPASS_DIA_DEFAULT.y).abs() < 1e-6);
    assert!((params.compass1.diagonals.z - COMPASS_DIA_DEFAULT.z).abs() < 1e-6);
    assert!((params.compass1.offdiagonals.x - COMPASS_ODI_DEFAULT.x).abs() < 1e-6);
    assert!(have_diagonals(params.compass1.diagonals));
}

#[test]
fn soft_iron_scales_published_field() {
    let dia = Vector3f::new(1.1, 0.9, 1.0);
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        diagonals: dia,
        ..SitlCompassConfig::default()
    });
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = compass.update().expect("pending sample");
    let expected = apply_soft_iron(wmm, dia, COMPASS_ODI_DEFAULT);
    assert!((sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((sample.mag_body.z - expected.z).abs() < 1e-5);
}

#[test]
fn default_soft_iron_leaves_published_field() {
    let mut compass = SitlCompassBackend::default();
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = compass.update().expect("pending sample");
    assert!((sample.mag_body.x - wmm.x).abs() < 1e-5);
    assert!((sample.mag_body.y - wmm.y).abs() < 1e-5);
    assert!((sample.mag_body.z - wmm.z).abs() < 1e-5);
}

#[test]
fn params_apply_soft_iron_to_backend() {
    let mut params = CompassParams::default();
    params.compass1.diagonals = Vector3f::new(1.1, 0.9, 1.0);
    params.compass1.offdiagonals = Vector3f::new(0.05, 0.0, 0.0);
    let mut backend = SitlCompassBackend::default();
    params.apply_instance(0, &mut backend);
    assert!((backend.config().diagonals.x - 1.1).abs() < 1e-6);
    assert!((backend.config().offdiagonals.x - 0.05).abs() < 1e-6);
    assert!(have_diagonals(backend.config().diagonals));
}
