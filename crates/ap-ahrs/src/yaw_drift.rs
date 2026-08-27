//! Yaw drift correction from the compass or GPS, upstream `drift_correction_yaw`
//! / `yaw_error_compass` with GPS-heading fallback when the compass disagrees.
//!
//! Roll and pitch drift uses gravity; yaw needs a horizontal reference. On
//! fixed-wing aircraft that is usually the magnetometer, with gain reduced when
//! horizontal acceleration makes GPS velocity a more reliable heading source.

use ap_math::matrix3::Matrix3f;
use ap_math::scalar::{degrees, radians, wrap_180, wrap_pi, Real};
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

use crate::drift::{DriftCorrector, SPIN_RATE_LIMIT_DEG};

/// Minimum ground speed before GPS course is usable, upstream `GPS_SPEED_MIN`.
pub const GPS_SPEED_MIN: f32 = 3.0;

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

/// One GPS sample for yaw correction when the compass is unavailable or
/// untrusted.
#[derive(Debug, Clone, Copy)]
pub struct YawGpsSample {
    /// Ground course in degrees, upstream `AP_GPS::ground_course()`.
    pub ground_course_deg: f32,
    /// Ground speed in m/s, upstream `AP_GPS::ground_speed()`.
    pub ground_speed: f32,
    /// Fix timestamp in milliseconds, upstream `AP_GPS::last_fix_time_ms()`.
    pub last_fix_time_ms: u32,
}

/// Vehicle configuration and motion facts for compass vs GPS selection.
#[derive(Debug, Clone, Copy, Default)]
pub struct YawDriftContext {
    /// Upstream `AP::ahrs().get_fly_forward()`.
    pub fly_forward: bool,
    /// Upstream `have_gps()`.
    pub have_gps: bool,
    /// Upstream `compass.use_for_yaw()`.
    pub compass_use_for_yaw: bool,
    /// Current yaw estimate in radians, upstream `yaw`.
    pub estimated_yaw_rad: f32,
    /// Estimated wind speed in the horizontal plane, upstream `_wind.xy().length()`.
    pub wind_speed_xy: f32,
    /// Monotonic time in milliseconds, upstream `AP_HAL::millis()`.
    pub now_ms: u32,
    pub gps_lat_e7: Option<i32>,
    pub gps_lng_e7: Option<i32>,
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
    /// Compass sample for this cycle, if any.
    pub compass: Option<YawCompassSample>,
    /// GPS sample for this cycle, if any.
    pub gps: Option<YawGpsSample>,
    /// Roll and pitch for GPS hard-reset, upstream `roll` / `pitch`.
    pub roll_rad: f32,
    pub pitch_rad: f32,
    pub ctx: YawDriftContext,
}

/// Why a yaw correction cycle produced nothing useful.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YawDriftOutcome {
    /// Proportional yaw correction ran (integral may also have accumulated).
    Corrected,
    /// No new yaw reference; proportional term decayed.
    Decayed,
    /// Compass calibrating or no usable reference.
    Skipped,
}

/// Hard attitude reset requested by the GPS yaw path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum YawMatrixAction {
    None,
    /// Reset DCM to `(roll, pitch, yaw)`, upstream `from_euler` in the GPS
    /// reset block.
    ResetAttitude {
        roll: f32,
        pitch: f32,
        yaw: f32,
    },
}

/// Result of one `drift_correction_yaw` cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct YawDriftResult {
    pub outcome: YawDriftOutcome,
    pub omega_i_z: f32,
    pub matrix_action: YawMatrixAction,
}

/// Compass- or GPS-based yaw drift correction, upstream `_omega_yaw_P` and the z
/// contribution to `_omega_I_sum`.
#[derive(Debug, Clone, Copy)]
pub struct YawDriftCorrector {
    /// Proportional yaw correction, upstream `_omega_yaw_P`.
    pub omega_yaw_p: Vector3f,
    /// Filtered yaw error magnitude, upstream `_error_yaw`.
    pub error_yaw: f32,
    gps_last_update_ms: u32,
    last_consistent_heading_ms: u32,
    /// Whether a yaw reference has ever been applied, upstream
    /// `have_initial_yaw`.
    pub have_initial_yaw: bool,
}

impl Default for YawDriftCorrector {
    fn default() -> Self {
        Self::new()
    }
}

impl YawDriftCorrector {
    /// A corrector with no proportional yaw term applied yet.
    #[must_use]
    pub fn new() -> Self {
        Self {
            omega_yaw_p: Vector3f::zero(),
            error_yaw: 0.0,
            gps_last_update_ms: 0,
            last_consistent_heading_ms: 0,
            have_initial_yaw: false,
        }
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

    /// Yaw error from GPS course, upstream the GPS branch of
    /// `drift_correction_yaw`.
    #[must_use]
    pub fn yaw_error_gps(gps_course_deg: f32, yaw_rad: f32) -> (f32, f32) {
        let gps_course_rad = radians(gps_course_deg);
        let yaw_error_rad = wrap_pi(gps_course_rad - yaw_rad);
        (yaw_error_rad.sin(), yaw_error_rad)
    }

    /// Whether to use the compass this cycle, upstream `use_compass()`.
    pub fn use_compass(&mut self, ctx: &YawDriftContext, gps: Option<&YawGpsSample>) -> bool {
        if !ctx.compass_use_for_yaw {
            return false;
        }
        if !ctx.fly_forward || !ctx.have_gps {
            return true;
        }
        let Some(gps) = gps else {
            return true;
        };
        if gps.ground_speed < GPS_SPEED_MIN {
            return true;
        }

        let error = wrap_180(degrees(ctx.estimated_yaw_rad) - gps.ground_course_deg).abs();
        if error > 45.0 && ctx.wind_speed_xy < gps.ground_speed * 0.8 {
            if ctx.now_ms.saturating_sub(self.last_consistent_heading_ms) > 2000 {
                return false;
            }
        } else {
            self.last_consistent_heading_ms = ctx.now_ms;
        }
        true
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

    /// Run one yaw correction cycle, upstream `drift_correction_yaw`.
    pub fn drift_correction_yaw(
        &mut self,
        inputs: &YawDriftInputs,
        gains: &YawDriftGains,
    ) -> YawDriftResult {
        if inputs
            .compass
            .is_some_and(|sample| sample.calibrating)
        {
            return YawDriftResult {
                outcome: YawDriftOutcome::Skipped,
                omega_i_z: 0.0,
                matrix_action: YawMatrixAction::None,
            };
        }

        let mut new_value = false;
        let mut yaw_error = 0.0_f32;
        let mut yaw_deltat = 0.0_f32;
        let mut matrix_action = YawMatrixAction::None;

        if self.use_compass(&inputs.ctx, inputs.gps.as_ref()) {
            if let Some(compass) = inputs.compass {
                if let Some(interval) = compass.update_interval_s {
                    yaw_deltat = interval;
                    new_value = true;
                    yaw_error = Self::yaw_error_compass(
                        inputs.dcm_matrix,
                        compass.mag_body,
                        compass.declination_rad,
                    );
                    if let Some(gps) = inputs.gps {
                        self.gps_last_update_ms = gps.last_fix_time_ms;
                    }
                }
            }
        } else if inputs.ctx.fly_forward && inputs.ctx.have_gps {
            if let Some(gps) = inputs.gps {
                if gps.last_fix_time_ms != self.gps_last_update_ms
                    && gps.ground_speed >= GPS_SPEED_MIN
                {
                    yaw_deltat =
                        (gps.last_fix_time_ms.saturating_sub(self.gps_last_update_ms)) as f32
                            * 1.0e-3;
                    self.gps_last_update_ms = gps.last_fix_time_ms;
                    new_value = true;
                    let gps_course_rad = radians(gps.ground_course_deg);
                    let (error, yaw_error_rad) =
                        Self::yaw_error_gps(gps.ground_course_deg, inputs.ctx.estimated_yaw_rad);

                    if !self.have_initial_yaw
                        || yaw_deltat > 20.0
                        || (gps.ground_speed >= 3.0 * GPS_SPEED_MIN
                            && Real::abs(yaw_error_rad) >= 1.047)
                    {
                        matrix_action = YawMatrixAction::ResetAttitude {
                            roll: inputs.roll_rad,
                            pitch: inputs.pitch_rad,
                            yaw: gps_course_rad,
                        };
                        self.omega_yaw_p = Vector3f::zero();
                        self.have_initial_yaw = true;
                        yaw_error = 0.0;
                    } else {
                        yaw_error = error;
                    }
                }
            }
        }

        if !new_value {
            self.omega_yaw_p *= YAW_P_DECAY;
            return YawDriftResult {
                outcome: YawDriftOutcome::Decayed,
                omega_i_z: 0.0,
                matrix_action: YawMatrixAction::None,
            };
        }

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

        YawDriftResult {
            outcome: YawDriftOutcome::Corrected,
            omega_i_z,
            matrix_action,
        }
    }

    /// Compass-only convenience wrapper retained for unit tests.
    pub fn correct(
        &mut self,
        inputs: &YawDriftInputs,
        gains: &YawDriftGains,
    ) -> (YawDriftOutcome, f32) {
        let result = self.drift_correction_yaw(inputs, gains);
        (result.outcome, result.omega_i_z)
    }
}
