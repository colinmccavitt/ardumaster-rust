//! Does compass yaw drift correction actually pull heading back?
//!
//! A stationary level vehicle with a north-pointing magnetometer should
//! produce zero proportional correction. A deliberate yaw misalignment should
//! produce a proportional term that opposes the error.

use ap_ahrs::{YawCompassSample, YawDriftCorrector, YawDriftGains, YawDriftInputs, YawDriftOutcome};
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::radians;
use ap_math::vector3::Vector3f;

fn level_dcm(yaw_rad: f32) -> Matrix3f {
    Matrix3f::from_euler(0.0, 0.0, yaw_rad)
}

/// Body-frame magnetometer consistent with `dcm` and zero declination.
fn mag_body_for_dcm(dcm: Matrix3f) -> Vector3f {
    dcm.transposed() * Vector3f::new(1.0, 0.0, 0.0)
}

#[test]
fn aligned_compass_produces_zero_yaw_correction() {
    let yaw = 0.3_f32;
    let dcm = level_dcm(yaw);
    let mut yaw_corr = YawDriftCorrector::new();
    let sample = YawCompassSample {
        mag_body: mag_body_for_dcm(dcm),
        declination_rad: 0.0,
        update_interval_s: Some(0.05),
        calibrating: false,
    };
    let inputs = YawDriftInputs {
        dcm_matrix: dcm,
        omega: Vector3f::zero(),
        accel_ef_xy_mag: 0.0,
        compass: sample,
    };
    let (outcome, omega_i_z) = yaw_corr.correct(&inputs, &YawDriftGains::default());
    assert_eq!(outcome, YawDriftOutcome::Corrected);
    assert_eq!(omega_i_z, 0.0);
    assert!(
        yaw_corr.omega_yaw_p.z.abs() < 1e-5,
        "aligned compass should not drive yaw P, got {}",
        yaw_corr.omega_yaw_p.z
    );
}

#[test]
fn yaw_misalignment_produces_corrective_omega() {
    let truth_yaw = 0.0_f32;
    let est_yaw = radians(5.0);
    let dcm = level_dcm(est_yaw);
    let mut yaw_corr = YawDriftCorrector::new();
    let sample = YawCompassSample {
        mag_body: mag_body_for_dcm(level_dcm(truth_yaw)),
        declination_rad: 0.0,
        update_interval_s: Some(0.05),
        calibrating: false,
    };
    let inputs = YawDriftInputs {
        dcm_matrix: dcm,
        omega: Vector3f::zero(),
        accel_ef_xy_mag: 0.0,
        compass: sample,
    };
    let (outcome, _) = yaw_corr.correct(&inputs, &YawDriftGains::default());
    assert_eq!(outcome, YawDriftOutcome::Corrected);
    assert!(
        yaw_corr.omega_yaw_p.z * est_yaw < 0.0,
        "correction should oppose positive yaw error, got omega_yaw_p.z={}",
        yaw_corr.omega_yaw_p.z
    );
}

#[test]
fn stale_compass_decays_proportional_yaw() {
    let mut yaw_corr = YawDriftCorrector::new();
    yaw_corr.omega_yaw_p = Vector3f::new(0.0, 0.0, 1.0);
    let sample = YawCompassSample {
        mag_body: Vector3f::new(1.0, 0.0, 0.0),
        declination_rad: 0.0,
        update_interval_s: None,
        calibrating: false,
    };
    let inputs = YawDriftInputs {
        dcm_matrix: Matrix3f::identity(),
        omega: Vector3f::zero(),
        accel_ef_xy_mag: 0.0,
        compass: sample,
    };
    let (outcome, omega_i_z) = yaw_corr.correct(&inputs, &YawDriftGains::default());
    assert_eq!(outcome, YawDriftOutcome::Decayed);
    assert_eq!(omega_i_z, 0.0);
    assert!(
        yaw_corr.omega_yaw_p.z < 1.0 && yaw_corr.omega_yaw_p.z > 0.9,
        "expected decay toward zero, got {}",
        yaw_corr.omega_yaw_p.z
    );
}

#[test]
fn high_horizontal_accel_reduces_yaw_gain() {
    let low = YawDriftCorrector::yaw_gain(0.0);
    let high = YawDriftCorrector::yaw_gain(10.0);
    assert!(low > high, "turning should reduce compass yaw gain");
    assert!((low - 0.9).abs() < 1e-5);
    assert!((high - 0.1).abs() < 1e-5);
}
