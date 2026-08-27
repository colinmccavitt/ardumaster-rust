//! Yaw drift correction from the compass, upstream `drift_correction_yaw` /
//! `yaw_error_compass` without GPS-heading fallback yet.
//!
//! Roll and pitch drift uses gravity; yaw needs a horizontal reference. On
//! fixed-wing aircraft that is usually the magnetometer, with gain reduced when
//! horizontal acceleration makes GPS velocity a more reliable heading source.

use ap_math::matrix3::Matrix3f;
use ap_math::scalar::{radians, Real};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

use crate::drift::{DriftCorrector, SPIN_RATE_LIMIT_DEG};

/// Minimum proportional yaw gain, upstream `AP_AHRS_YAW_P_MIN`.
pub const YAW_P_MIN: f32 = 0.05;

/// Integral gain on yaw error, upstream `_ki_yaw`.
pub const KI_YAW: f32 = 0.01;

/// Proportional decay when the yaw reference goes stale, upstream's 0.97 factor.
pub const YAW_P_DECAY: f32 = 0.97;

/// Gains and flags for yaw correction.
#[derive(Debug, Clone, Copy)]
pub struct YawDriftGains {
    /// Proportional gain, upstream `AHRS_YAW_P`. Clamped up to [`YAW_P_MIN`].
    pub kp_yaw: f32,
    /// Whether to multiply the proportional term by eight, upstream
    /// `use_fast_gains()`.
    pub fast_gains: bool,
}

impl Default for YawDriftGains {
    fn default() -> Self {
        Self {
            kp_yaw: 0.2,
            fast_gains: false,
        }
    }
}

/// One compass sample for yaw correction.
#[derive(Debug, Clone, Copy)]
pub struct YawCompassSample {
    /// Body-frame magnetic field, upstream `compass.get_field()`.
    pub mag_body: Vector3f,
    /// Local declination in radians, upstream `compass.get_declination()`.
    pub declination_rad: f32,
    /// Seconds since the previous compass update. `None` means no new sample
    /// this cycle — upstream decays `_omega_yaw_P` instead.
    pub update_interval_s: Option<f32>,
    /// When true, upstream skips yaw correction during calibration.
    pub calibrating: bool,
}

/// Vehicle motion facts yaw correction reads each cycle.
#[derive(Debug, Clone, Copy)]
pub struct YawDriftInputs {
    /// Current attitude estimate.
    pub dcm_matrix: Matrix3f,
    /// Corrected body rates, upstream `_omega`.
    pub omega: Vector3f,
    /// Horizontal earth-frame acceleration magnitude, upstream
    /// `_accel_ef.xy().length()` feeding `_yaw_gain()`.
    pub accel_ef_xy_mag: f32,
    /// Compass sample for this cycle.
    pub compass: YawCompassSample,
}

/// Why a yaw correction cycle produced nothing useful.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YawDriftOutcome {
    /// Proportional yaw correction ran (integral may also have accumulated).
    Corrected,
    /// No new compass sample; proportional term decayed.
    Decayed,
    /// Compass calibrating or no usable field.
    Skipped,
}

/// Compass-based yaw drift correction, upstream `_omega_yaw_P` and the z
/// contribution to `_omega_I_sum`.
#[derive(Debug, Clone, Copy, Default)]
pub struct YawDriftCorrector {
    /// Proportional yaw correction, upstream `_omega_yaw_P`.
    pub omega_yaw_p: Vector3f,
    /// Filtered yaw error magnitude, upstream `_error_yaw`.
    pub error_yaw: f32,
}

impl YawDriftCorrector {
    /// A corrector with no proportional yaw term applied yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Yaw error from the compass, upstream `yaw_error_compass`.
    #[must_use]
    pub fn yaw_error_compass(dcm: Matrix3f, mag_body: Vector3f, declination_rad: f32) -> f32 {
        let mut rb = dcm.mul_xy(mag_body);
        if rb.length_squared() < f32::EPSILON {
            return 0.0;
        }
        if !rb.normalize() || rb.is_inf() {
            return 0.0;
        }

        let mag_earth = Vector2f::new(declination_rad.cos(), declination_rad.sin());
        rb.cross(mag_earth)
    }

    /// Observability-based gain scale, upstream `_yaw_gain()`.
    #[must_use]
    pub fn yaw_gain(accel_ef_xy_mag: f32) -> f32 {
        if accel_ef_xy_mag <= 4.0 {
            0.2 * (4.5 - accel_ef_xy_mag)
        } else {
            0.1
        }
    }

    /// Run one yaw correction cycle. Returns the z integral increment to add
    /// to [`DriftCorrector`]'s pending sum when [`YawDriftOutcome::Corrected`].
    pub fn correct(
        &mut self,
        inputs: &YawDriftInputs,
        gains: &YawDriftGains,
    ) -> (YawDriftOutcome, f32) {
        if inputs.compass.calibrating {
            return (YawDriftOutcome::Skipped, 0.0);
        }

        let Some(yaw_deltat) = inputs.compass.update_interval_s else {
            self.omega_yaw_p *= YAW_P_DECAY;
            return (YawDriftOutcome::Decayed, 0.0);
        };

        let yaw_error = Self::yaw_error_compass(
            inputs.dcm_matrix,
            inputs.compass.mag_body,
            inputs.compass.declination_rad,
        );

        let error_z = inputs.dcm_matrix.c.z * yaw_error;
        let spin_rate = inputs.omega.length();

        let kp_yaw = if gains.kp_yaw < YAW_P_MIN {
            YAW_P_MIN
        } else {
            gains.kp_yaw
        };

        let mut omega_yaw_p_z =
            error_z * DriftCorrector::p_gain(spin_rate) * kp_yaw * Self::yaw_gain(inputs.accel_ef_xy_mag);
        if gains.fast_gains {
            omega_yaw_p_z *= 8.0;
        }
        self.omega_yaw_p = Vector3f::new(0.0, 0.0, omega_yaw_p_z);
        self.error_yaw = 0.8 * self.error_yaw + 0.2 * yaw_error.abs();

        let mut omega_i_z = 0.0;
        if yaw_deltat < 2.0 && spin_rate < radians(SPIN_RATE_LIMIT_DEG) {
            omega_i_z = error_z * KI_YAW * yaw_deltat;
        }

        (YawDriftOutcome::Corrected, omega_i_z)
    }
}
