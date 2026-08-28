//! Compass motor compensation stub: COMPASS_MOT / COMPASS_MOTCT current-based hard-iron.

use ap_compass::motor_comp::{
    apply_motor_compensation, learn_motor_compensation, motor_comp_enabled, motor_offset,
    COMPASS_MOTCT_DEFAULT, COMPASS_MOT_COMP_CURRENT, COMPASS_MOT_COMP_DISABLED,
    COMPASS_MOT_COMP_THROTTLE,
};
use ap_compass::params::CompassParams;
use ap_compass::sitl::{mag_field_body_ned, SitlCompassBackend, SitlCompassConfig};
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;

#[test]
fn compass_params_motct_defaults_disabled() {
    let params = CompassParams::default();
    assert_eq!(params.motor_comp_type, COMPASS_MOTCT_DEFAULT);
    assert_eq!(params.motor_comp_type, COMPASS_MOT_COMP_DISABLED);
    assert_eq!(params.compass1.motor_compensation, Vector3f::zero());
    assert!(!motor_comp_enabled(params.motor_comp_type));
}

#[test]
fn apply_compass_mot_shifts_published_field() {
    let mot = Vector3f::new(0.01, -0.02, 0.005);
    let current = 10.0;
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        motor_compensation: mot,
        motor_comp_type: COMPASS_MOT_COMP_CURRENT,
        ..SitlCompassConfig::default()
    });
    compass.set_thr_or_curr(current);
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = compass.update().expect("pending sample");
    let expected = apply_motor_compensation(wmm, mot, COMPASS_MOT_COMP_CURRENT, current);
    assert!((sample.mag_body.x - expected.x).abs() < 1e-5);
    assert!((sample.mag_body.y - expected.y).abs() < 1e-5);
    assert!((sample.mag_body.z - expected.z).abs() < 1e-5);
    let ofs = motor_offset(mot, COMPASS_MOT_COMP_CURRENT, current);
    assert!((sample.mag_body.x - (wmm.x + ofs.x)).abs() < 1e-5);
}

#[test]
fn disabled_motct_leaves_wmm_untouched() {
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        motor_compensation: Vector3f::new(0.05, 0.0, 0.0),
        motor_comp_type: COMPASS_MOT_COMP_DISABLED,
        ..SitlCompassConfig::default()
    });
    compass.set_thr_or_curr(12.0);
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let sample = compass.update().expect("pending sample");
    assert!((sample.mag_body.x - wmm.x).abs() < 1e-5);
    assert!((sample.mag_body.y - wmm.y).abs() < 1e-5);
    assert!((sample.mag_body.z - wmm.z).abs() < 1e-5);
}

#[test]
fn learn_motor_comp_cancels_current_bias() {
    let current = 8.0;
    let bias_per_amp = Vector3f::new(0.02, -0.01, 0.0);
    let (wmm, _) = mag_field_body_ned(51.875, -0.154, Matrix3f::identity());
    let raw = wmm + bias_per_amp * current;
    let mot = learn_motor_compensation(raw, wmm, current).expect("current");
    let mut compass = SitlCompassBackend::with_config(SitlCompassConfig {
        hardiron_bias: bias_per_amp * current,
        motor_compensation: mot,
        motor_comp_type: COMPASS_MOT_COMP_CURRENT,
        ..SitlCompassConfig::default()
    });
    compass.set_thr_or_curr(current);
    assert!(compass.timer_tick(51.875, -0.154, Matrix3f::identity(), 10));
    let after = compass.update().expect("compensated sample");
    assert!((after.mag_body.x - wmm.x).abs() < 1e-5);
    assert!((after.mag_body.y - wmm.y).abs() < 1e-5);
    assert!((after.mag_body.z - wmm.z).abs() < 1e-5);
}

#[test]
fn throttle_mode_uses_same_multiply() {
    let mot = Vector3f::new(0.1, 0.0, 0.0);
    let field = Vector3f::new(1.0, 2.0, 3.0);
    let out = apply_motor_compensation(field, mot, COMPASS_MOT_COMP_THROTTLE, 0.5);
    assert!((out.x - 1.05).abs() < 1e-6);
}
