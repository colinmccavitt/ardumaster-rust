//! `ARMING_ACCTHRESH` / accelerometer-error-threshold named-check. FW-026.
//!
//! Upstream `AP_Arming::accel_error_threshold` / `ARMING_ACCTHRESH`:
//! the INS named check fails when the accelerometer error magnitude is
//! outside this threshold (`AP_InertialSensor::accels_consistent` compares
//! `vec_diff.length()` to the stored value). Default is 0.75 m/s/s.
//!
//! This slice is the threshold gate, not the multi-IMU 10 s dwell. A
//! lone IMU cannot disagree with another and therefore passes; two
//! samples fail when their Z-weighted difference magnitude is outside
//! `ARMING_ACCTHRESH`.

use crate::{Check, NamedCheck};

/// Default `ARMING_ACCTHRESH`, upstream `AP_ARMING_ACCEL_ERROR_THRESHOLD`.
pub const ARMING_ACCTHRESH_DEFAULT: f32 = 0.75;

/// Registry name for the INS / accel named check this threshold gates.
pub const ACCEL_CHECK_NAME: &str = "INS";

/// Whether an accel-error magnitude is inside `ARMING_ACCTHRESH`.
///
/// Upstream refuses when `vec_diff.length() > threshold`. Equal passes.
#[must_use]
pub fn accel_magnitude_within_threshold(error_magnitude: f32, threshold: f32) -> bool {
    error_magnitude <= threshold
}

/// Squared Z-weighted vector-difference magnitude, upstream `accels_consistent`.
///
/// EKF is less sensitive to Z-axis error, so Z is halved before the length.
/// Squared so this `no_std` crate does not need `sqrt`.
#[must_use]
pub fn accel_error_magnitude_sq(primary: [f32; 3], other: [f32; 3]) -> f32 {
    let dx = other[0] - primary[0];
    let dy = other[1] - primary[1];
    let dz = (other[2] - primary[2]) * 0.5;
    dx * dx + dy * dy + dz * dz
}

/// Whether two accel samples agree within `ARMING_ACCTHRESH`.
///
/// IMU3's 3x allowance is a later slice.
#[must_use]
pub fn accels_within_threshold(primary: [f32; 3], other: [f32; 3], threshold: f32) -> bool {
    accel_error_magnitude_sq(primary, other) <= threshold * threshold
}

/// Fill `Check::Ins` from an accel-error magnitude vs `ARMING_ACCTHRESH`.
#[must_use]
pub fn accel_threshold_named_check(error_magnitude: f32, threshold: f32) -> NamedCheck {
    NamedCheck {
        check: Check::Ins,
        name: ACCEL_CHECK_NAME,
        ok: accel_magnitude_within_threshold(error_magnitude, threshold),
    }
}

/// Fill `Check::Ins` from two accel samples vs `ARMING_ACCTHRESH`.
#[must_use]
pub fn accel_threshold_named_check_from_samples(
    primary: [f32; 3],
    other: [f32; 3],
    threshold: f32,
) -> NamedCheck {
    NamedCheck {
        check: Check::Ins,
        name: ACCEL_CHECK_NAME,
        ok: accels_within_threshold(primary, other, threshold),
    }
}
