//! Compass field-strength / expected-field check stub.

use ap_compass::field::{
    expected_earth_field_mgauss, expected_field_ok, field_length_ok, field_ok, field_strength_ok,
    gauss_to_mgauss, COMPASS_MAGFIELD_ERROR_THRESHOLD, COMPASS_MAGFIELD_EXPECTED,
    COMPASS_MAGFIELD_MAX, COMPASS_MAGFIELD_MIN,
};
use ap_compass::params::CompassParams;
use ap_math::vector3::Vector3f;

#[test]
fn default_params_have_no_offset_so_wmm_field_is_ok() {
    let params = CompassParams::default();
    assert!(params.compass1.offset.is_zero());
    let earth = expected_earth_field_mgauss(51.875, -0.154);
    assert!(field_length_ok(earth.length()));
    assert!(field_ok(
        earth,
        earth,
        earth,
        COMPASS_MAGFIELD_ERROR_THRESHOLD
    ));
}

#[test]
fn weak_and_strong_fields_fail_length() {
    assert!(!field_strength_ok(Vector3f::new(
        COMPASS_MAGFIELD_MIN - 10.0,
        0.0,
        0.0
    )));
    assert!(!field_strength_ok(Vector3f::new(
        COMPASS_MAGFIELD_MAX + 10.0,
        0.0,
        0.0
    )));
    assert!(field_strength_ok(Vector3f::new(
        COMPASS_MAGFIELD_EXPECTED,
        0.0,
        0.0
    )));
}

#[test]
fn expected_field_rejects_xy_beyond_threshold() {
    let earth = expected_earth_field_mgauss(51.875, -0.154);
    let shifted = earth + Vector3f::new(COMPASS_MAGFIELD_ERROR_THRESHOLD + 1.0, 0.0, 0.0);
    assert!(!expected_field_ok(
        shifted,
        earth,
        COMPASS_MAGFIELD_ERROR_THRESHOLD
    ));
    assert!((gauss_to_mgauss(0.53) - 530.0).abs() < 1e-4);
}
