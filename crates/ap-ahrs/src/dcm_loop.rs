//! Wire INS delta-angle samples into the DCM matrix update, upstream
//! `AP_AHRS_DCM::matrix_update` and the `normalize` that follows in
//! `update()`.
//!
//! This is the seam where FW-011 meets FW-008: the flight loop publishes
//! accumulated gyro data from [`ImuInstance::update_gyro`], and the AHRS
//! reads [`ImuInstance::get_delta_angle`] and [`ImuInstance::gyro`] to
//! advance the direction cosine matrix.

use ap_ins::{ImuInstance, LoopTiming};
use ap_math::vector3::Vector3f;

use crate::{Dcm, MatrixHealth};

/// Proportional and integral drift terms fed into the matrix rotation.
///
/// Upstream keeps these as `_omega_I`, `_omega_P`, and `_omega_yaw_P`
/// on `AP_AHRS_DCM`; they are passed through rather than computed here.
#[derive(Debug, Clone, Copy, Default)]
pub struct DcmDriftOmega {
    /// Integral drift estimate, upstream `_omega_I`.
    pub omega_i: Vector3f,
    /// Proportional roll/pitch correction, upstream `_omega_P`.
    pub omega_p: Vector3f,
    /// Proportional yaw correction, upstream `_omega_yaw_P`.
    pub omega_yaw_p: Vector3f,
}

/// Advance the DCM matrix from the primary IMU, upstream `matrix_update`.
///
/// The caller must have called [`ImuInstance::update_gyro`] since the last
/// loop tick so accumulated delta angles are published.
pub fn dcm_matrix_update_from_ins(
    dcm: &mut Dcm,
    imu: &ImuInstance,
    timing: &LoopTiming,
    drift: DcmDriftOmega,
) {
    let delta_angle = imu
        .get_delta_angle(timing)
        .filter(|(_, dt)| *dt > 0.0);
    dcm.matrix_update(
        delta_angle,
        imu.gyro(),
        drift.omega_i,
        drift.omega_p,
        drift.omega_yaw_p,
    );
}

/// Matrix update followed by renormalisation, upstream `matrix_update` then
/// `normalize` within `AP_AHRS_DCM::update`.
pub fn dcm_matrix_step_from_ins(
    dcm: &mut Dcm,
    imu: &ImuInstance,
    timing: &LoopTiming,
    drift: DcmDriftOmega,
) -> MatrixHealth {
    dcm_matrix_update_from_ins(dcm, imu, timing, drift);
    dcm.normalize()
}
