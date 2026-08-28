//! Compass external / orientation stub: COMPASS_ORIENT / COMPASS_EXTERNAL.

use ap_compass::orientation::{
    apply_orientation, is_external, rotate_field, COMPASS_EXTERNAL_DEFAULT, COMPASS_ORIENT_DEFAULT,
    COMPASS_ORIENT_YAW_90,
};
use ap_compass::params::CompassParams;
use ap_compass::sitl::{mag_field_body_ned, SitlCompassBackend, SitlCompassConfig};
use ap_math::matrix3::Matrix3f;

#[test]
fn compass_params_orient_defaults_match_upstream() {
    let params = CompassParams::default();
    assert_eq!(params.compass1.orientation, COMPASS_ORIENT_DEFAULT);
    assert_eq!(params.compass1.external, COMPASS_EXTERNAL_DEFAULT);
    assert_eq!(params.board_orientation, COMPASS_ORIENT_DEFAULT);
    assert!(!is_external(params.compass1.external));
}

#[test]
fn yaw90_orient_rotates_published_field() {
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        orientation: COMPASS_ORIENT_YAW_90,
        ..SitlCompassConfig::default()
    });
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = compass.update().expect("pending sample");
    let expected = apply_orientation(wmm, COMPASS_ORIENT_YAW_90);
    assert!((sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((sample.mag_body.z - expected.z).abs() < 1e-5);
}

#[test]
fn external_skips_board_orientation_on_publish() {
    let attitude = Matrix3f::identity();
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, attitude);

    let mut internal = SitlCompassBackend::with_config(SitlCompassConfig {
        external: false,
        board_orientation: COMPASS_ORIENT_YAW_90,
        ..SitlCompassConfig::default()
    });
    assert!(internal.timer_tick(51.875, -0.154, attitude, 10));
    let got = internal.update().expect("internal").mag_body;
    let expect_internal = rotate_field(wmm, COMPASS_ORIENT_DEFAULT, false, COMPASS_ORIENT_YAW_90);
    assert!((got.x - expect_internal.x).abs() < 1e-5);
    assert!((got.y - expect_internal.y).abs() < 1e-5);

    let mut external = SitlCompassBackend::with_config(SitlCompassConfig {
        external: true,
        board_orientation: COMPASS_ORIENT_YAW_90,
        ..SitlCompassConfig::default()
    });
    assert!(external.timer_tick(51.875, -0.154, attitude, 10));
    let got = external.update().expect("external").mag_body;
    assert!((got.x - wmm.x).abs() < 1e-5);
    assert!((got.y - wmm.y).abs() < 1e-5);
    assert!((got.z - wmm.z).abs() < 1e-5);
}

#[test]
fn params_apply_orient_and_external_to_backend() {
    let mut params = CompassParams::default();
    params.compass1.orientation = COMPASS_ORIENT_YAW_90;
    params.compass1.external = true;
    params.board_orientation = COMPASS_ORIENT_YAW_90;
    let mut backend = SitlCompassBackend::default();
    params.apply_instance(0, &mut backend);
    assert_eq!(backend.config().orientation, COMPASS_ORIENT_YAW_90);
    assert!(backend.config().external);
    assert_eq!(backend.config().board_orientation, COMPASS_ORIENT_YAW_90);
}
