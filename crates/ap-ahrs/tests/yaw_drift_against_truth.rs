//! Does compass yaw drift correction actually pull heading back?
//!
//! A stationary level vehicle with a north-pointing magnetometer should
//! produce zero proportional correction. A deliberate yaw misalignment should
//! produce a proportional term that opposes the error.

use ap_ahrs::{
    YawCompassSample, YawDriftContext, YawDriftCorrector, YawDriftGains, YawDriftInputs,
    YawDriftOutcome, YawGpsSample, YawMatrixAction, GPS_SPEED_MIN,
};
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::{degrees, radians};
use ap_math::vector3::Vector3f;

fn level_dcm(yaw_rad: f32) -> Matrix3f {
    Matrix3f::from_euler(0.0, 0.0, yaw_rad)
}

/// Body-frame magnetometer consistent with `dcm` and zero declination.
fn mag_body_for_dcm(dcm: Matrix3f) -> Vector3f {
    dcm.transposed() * Vector3f::new(1.0, 0.0, 0.0)
}

fn compass_only_inputs(dcm: Matrix3f, sample: YawCompassSample) -> YawDriftInputs {
    let (_, _, yaw) = dcm.to_euler();
    YawDriftInputs {
        dcm_matrix: dcm,
        omega: Vector3f::zero(),
        accel_ef_xy_mag: 0.0,
        compass: Some(sample),
        gps: None,
        roll_rad: 0.0,
        pitch_rad: 0.0,
        ctx: YawDriftContext {
            fly_forward: false,
            have_gps: false,
            compass_use_for_yaw: true,
            estimated_yaw_rad: yaw,
            wind_speed_xy: 0.0,
            now_ms: 0,
        },
    }
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
    let inputs = compass_only_inputs(dcm, sample);
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
    let inputs = compass_only_inputs(dcm, sample);
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
    let inputs = compass_only_inputs(Matrix3f::identity(), sample);
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

#[test]
fn gps_yaw_error_matches_course_delta() {
    let (_, yaw_error_rad) = YawDriftCorrector::yaw_error_gps(90.0, radians(0.0));
    assert!((yaw_error_rad - radians(90.0)).abs() < 1e-5);
}

#[test]
fn compass_gps_disagreement_switches_to_gps() {
    let mut yaw_corr = YawDriftCorrector::new();
    let gps = YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: 10.0,
        last_fix_time_ms: 1000,
    };
    let ctx = YawDriftContext {
        fly_forward: true,
        have_gps: true,
        compass_use_for_yaw: true,
        estimated_yaw_rad: radians(90.0),
        wind_speed_xy: 1.0,
        now_ms: 5000,
    };
    assert!(
        !yaw_corr.use_compass(&ctx, Some(&gps)),
        "large compass/GPS disagreement should prefer GPS"
    );
}

#[test]
fn gps_first_fix_resets_attitude_yaw() {
    let est_yaw = radians(30.0);
    let dcm = level_dcm(est_yaw);
    let mut yaw_corr = YawDriftCorrector::new();
    let gps = YawGpsSample {
        ground_course_deg: 0.0,
        ground_speed: GPS_SPEED_MIN,
        last_fix_time_ms: 200,
    };
    let inputs = YawDriftInputs {
        dcm_matrix: dcm,
        omega: Vector3f::zero(),
        accel_ef_xy_mag: 0.0,
        compass: None,
        gps: Some(gps),
        roll_rad: 0.0,
        pitch_rad: 0.0,
        ctx: YawDriftContext {
            fly_forward: true,
            have_gps: true,
            compass_use_for_yaw: false,
            estimated_yaw_rad: est_yaw,
            wind_speed_xy: 0.0,
            now_ms: 200,
        },
    };
    let result = yaw_corr.drift_correction_yaw(&inputs, &YawDriftGains::default());
    assert_eq!(result.outcome, YawDriftOutcome::Corrected);
    assert_eq!(
        result.matrix_action,
        YawMatrixAction::ResetAttitude {
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    );
    assert!(yaw_corr.have_initial_yaw);
}

#[test]
fn gps_course_correction_opposes_yaw_error() {
    let est_yaw = radians(10.0);
    let dcm = level_dcm(est_yaw);
    let mut yaw_corr = YawDriftCorrector::new();
    yaw_corr.have_initial_yaw = true;
    let gps = YawGpsSample {
        ground_course_deg: degrees(0.0),
        ground_speed: 5.0,
        last_fix_time_ms: 200,
    };
    let inputs = YawDriftInputs {
        dcm_matrix: dcm,
        omega: Vector3f::zero(),
        accel_ef_xy_mag: 0.0,
        compass: None,
        gps: Some(gps),
        roll_rad: 0.0,
        pitch_rad: 0.0,
        ctx: YawDriftContext {
            fly_forward: true,
            have_gps: true,
            compass_use_for_yaw: false,
            estimated_yaw_rad: est_yaw,
            wind_speed_xy: 0.0,
            now_ms: 200,
        },
    };
    let result = yaw_corr.drift_correction_yaw(&inputs, &YawDriftGains::default());
    assert_eq!(result.outcome, YawDriftOutcome::Corrected);
    assert!(
        yaw_corr.omega_yaw_p.z * est_yaw < 0.0,
        "GPS correction should oppose positive yaw error"
    );
}
