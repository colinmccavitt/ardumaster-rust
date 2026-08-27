//! DCM drift correction: the part that stops the attitude wandering. FW-008.
//!
//! Integrating gyros alone drifts — [`crate::Dcm`] on its own loses about ten
//! degrees in ten seconds against a one-degree-per-second bias, which the
//! simulator measures exactly. Drift correction is what pulls it back, using
//! gravity as an absolute reference.
//!
//! # How it uses gravity when the vehicle is accelerating
//!
//! An accelerometer cannot separate gravity from acceleration. The trick is to
//! form two versions of the same vector and compare them: `ga_e`, gravity as
//! it *should* look in earth frame, and `ga_b`, gravity as the accelerometers
//! *say* it looks, rotated by the current attitude estimate. Any angle between
//! them is attitude error, and the cross product of the two is that error as a
//! rotation.
//!
//! When the vehicle is turning, the centrifugal term makes the measured vector
//! lean. Upstream corrects for that by adding the change in velocity — from
//! GPS, or from airspeed when there is no GPS — to the reference before
//! comparing. Without that correction a steady turn would be read as a
//! persistent roll error, which is exactly the failure DCM is famous for on
//! vehicles without GPS.
//!
//! # Two terms, doing different jobs
//!
//! The **proportional** term drags the attitude toward the accelerometer
//! reading quickly and is discarded each cycle. The **integral** term
//! accumulates and becomes the gyro bias estimate — it is what actually
//! cancels the drift, and it is deliberately slow.
//!
//! # What this slice does not include
//!
//! Yaw correction (`drift_correction_yaw`), the multi-accelerometer selection
//! that picks whichever sensor gives the smallest error, wind estimation
//! and dead-reckoning position. GPS lag buffering lives in [`crate::GpsLagBuffer`].

use ap_math::matrix3::Matrix3f;
use crate::GpsLagBuffer;
use ap_math::scalar::{constrain_value, radians};
use ap_math::vector3::Vector3f;

/// Standard gravity, m/s2, upstream `GRAVITY_MSS`.
const GRAVITY_MSS: f32 = 9.806_65;

/// Above this spin rate the integral term stops accumulating, degrees/s.
/// Upstream `SPIN_RATE_LIMIT`.
///
/// A fast spin makes the gravity reference meaningless, and integrating
/// nonsense would poison the bias estimate long after the spin stopped.
pub const SPIN_RATE_LIMIT_DEG: f32 = 20.0;

/// Floor on the proportional gain, upstream `AP_AHRS_RP_P_MIN`.
pub const RP_P_MIN: f32 = 0.05;

/// Integral gain, upstream's `static constexpr float _ki`.
///
/// Not a parameter: upstream fixes it, so the bias estimate converges at the
/// same rate on every airframe.
pub const KI: f32 = 0.0087;

/// Gains and vehicle facts the correction needs.
#[derive(Debug, Clone, Copy)]
pub struct DriftGains {
    /// Proportional gain, upstream `AHRS_RP_P`. Clamped up to [`RP_P_MIN`].
    pub kp: f32,
    /// Multiplier on the velocity-based centrifugal correction, upstream
    /// `AHRS_GPS_GAIN`.
    pub gps_gain: f32,
    /// Maximum gyro drift the hardware is specified to have, radians/s.
    /// Upstream reads `AP_InertialSensor::get_gyro_drift_rate()`, which is a
    /// fixed `radians(0.5/60)` — half a degree per minute.
    pub gyro_drift_rate: f32,
    /// Whether to multiply the proportional term by eight, upstream
    /// `use_fast_gains()`. True shortly after startup, to converge quickly.
    pub fast_gains: bool,
}

impl Default for DriftGains {
    /// Upstream's parameter defaults.
    fn default() -> Self {
        Self {
            kp: 0.2,
            gps_gain: 1.0,
            gyro_drift_rate: radians(0.5 / 60.0),
            fast_gains: false,
        }
    }
}

/// What the correction reads about the vehicle's motion this cycle.
#[derive(Debug, Clone, Copy)]
pub struct DriftInputs {
    /// Earth-frame acceleration accumulated since the last correction,
    /// multiplied by the interval — upstream's `_ra_sum`. Built by
    /// [`DriftCorrector::accumulate`].
    pub ra_sum: Vector3f,
    /// Seconds the accumulation covers, upstream `_ra_deltat`.
    pub ra_deltat: f32,
    /// Change in earth-frame velocity since the last correction, m/s. `None`
    /// when there is neither a GPS fix nor an airspeed-derived estimate, in
    /// which case the centrifugal correction is skipped and a turn will be
    /// misread as roll error.
    pub velocity_delta: Option<Vector3f>,
    /// Current attitude estimate, used to bring the error into body frame.
    pub dcm_matrix: Matrix3f,
    /// Current corrected body rates, upstream `_omega`. Its length is the spin
    /// rate that scales the proportional gain.
    pub omega: Vector3f,
    /// Whether the inertial sensors are healthy. When they are not, upstream
    /// zeroes the error rather than trusting it.
    pub ins_healthy: bool,
    /// Apply GPS lag delay to the measured gravity vector, upstream
    /// `using_gps_corrections`.
    pub using_gps_corrections: bool,
}

/// Why a correction cycle produced nothing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriftOutcome {
    /// The correction ran.
    Corrected,
    /// Not enough time has accumulated, or the interval was not positive.
    NotEnoughData,
    /// The measured or reference vector was zero or infinite — upstream waits
    /// for acceleration information rather than correcting toward nothing.
    NoUsableAcceleration,
    /// The computed error was not finite. Upstream treats this as a matrix
    /// problem and asks for a health check.
    BadError,
}

/// The roll and pitch drift correction, upstream's part of `AP_AHRS_DCM`.
#[derive(Debug, Clone, Copy, Default)]
pub struct DriftCorrector {
    /// Proportional correction, upstream `_omega_P`. Recomputed each cycle and
    /// fed straight into the matrix update.
    pub omega_p: Vector3f,
    /// Integral correction, upstream `_omega_I`. This is the gyro bias
    /// estimate, and the thing that actually removes drift.
    pub omega_i: Vector3f,
    /// Filtered error magnitude, upstream `_error_rp`. Reported, not acted on.
    pub error_rp: f32,

    omega_i_sum: Vector3f,
    omega_i_sum_time: f32,
}

impl DriftCorrector {
    /// A corrector with no accumulated bias estimate.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add one accelerometer sample to the running earth-frame sum, upstream's
    /// `_ra_sum[i] += accel_ef * deltat`.
    ///
    /// `accel_ef` is the body-frame delta velocity divided by its own
    /// interval, rotated to earth frame. Upstream uses delta velocity rather
    /// than the instantaneous accelerometer reading specifically to avoid
    /// aliasing: each sensor is then sampled over exactly its own interval,
    /// so a vibration near the sampling rate does not fold down into an
    /// apparent steady acceleration.
    pub fn accumulate(ra_sum: &mut Vector3f, ra_deltat: &mut f32, accel_ef: Vector3f, dt: f32) {
        *ra_sum += accel_ef * dt;
        *ra_deltat += dt;
    }

    /// Proportional gain multiplier for a given spin rate, upstream `_P_gain`.
    ///
    /// Unity below 50 degrees per second, rising linearly to ten at 500 and
    /// held there. A spinning vehicle needs a much harder pull toward the
    /// accelerometer reading, because the first-order integration is losing
    /// accuracy fast.
    #[must_use]
    pub fn p_gain(spin_rate: f32) -> f32 {
        if spin_rate < radians(50.0) {
            return 1.0;
        }
        if spin_rate > radians(500.0) {
            return 10.0;
        }
        spin_rate / radians(50.0)
    }

    /// Run one correction cycle.
    ///
    /// Returns what happened; on anything but [`DriftOutcome::Corrected`] the
    /// caller should leave the accumulator alone and try again next cycle,
    /// exactly as upstream's early returns do.
    pub fn correct(
        &mut self,
        inp: &DriftInputs,
        gains: &DriftGains,
        gps_lag: &mut GpsLagBuffer,
    ) -> DriftOutcome {
        if inp.ra_deltat <= 0.0 {
            return DriftOutcome::NotEnoughData;
        }

        // Gravity as it should look in earth frame: straight down, one g. The
        // accumulated acceleration is scaled into the same units so the two
        // can be compared as directions.
        let ra_scale = 1.0 / (inp.ra_deltat * GRAVITY_MSS);
        let mut ga_e = Vector3f::new(0.0, 0.0, -1.0);

        if let Some(velocity_delta) = inp.velocity_delta {
            // The centrifugal correction. Without it a steady turn reads as a
            // persistent roll error.
            ga_e += velocity_delta * (gains.gps_gain * ra_scale);
            if !ga_e.normalize() || ga_e.is_inf() {
                return DriftOutcome::NoUsableAcceleration;
            }
        }

        let mut ga_b = inp.ra_sum * ra_scale;
        if inp.using_gps_corrections {
            ga_b = gps_lag.ra_delayed(ga_b);
        }
        if ga_b.is_zero() || !ga_b.normalize() || ga_b.is_inf() {
            return DriftOutcome::NoUsableAcceleration;
        }

        // The angle between measured and expected gravity, as a rotation.
        let mut error = ga_b.cross(ga_e);
        let error_dirn = ga_b.dot(ga_e);
        let mut best_error = error.length();
        if error_dirn < 0.0 {
            // Opposite and parallel: the cross product is near zero even
            // though the attitude is 180 degrees out, so the magnitude has to
            // be forced high or the estimator would think it was perfect.
            best_error = 1.0;
        }

        if inp.ins_healthy {
            error = inp.dcm_matrix.mul_transpose(error);
        } else {
            // Unhealthy sensors: stop correcting rather than correct toward
            // a bad reading, and let the gyros carry it for a while.
            error = Vector3f::zero();
        }

        if error.is_nan() || error.is_inf() {
            return DriftOutcome::BadError;
        }

        self.error_rp = 0.8 * self.error_rp + 0.2 * best_error;

        let spin_rate = inp.omega.length();
        let kp = if gains.kp < RP_P_MIN {
            RP_P_MIN
        } else {
            gains.kp
        };

        self.omega_p = error * (Self::p_gain(spin_rate) * kp);
        if gains.fast_gains {
            self.omega_p *= 8.0;
        }

        // The integral term only accumulates when the vehicle is not spinning
        // hard: above the limit the gravity reference is meaningless and
        // integrating it would poison the bias estimate.
        if spin_rate < radians(SPIN_RATE_LIMIT_DEG) {
            self.omega_i_sum += error * (KI * inp.ra_deltat);
            self.omega_i_sum_time += inp.ra_deltat;
        }

        // The bias estimate updates in five-second batches, and each batch is
        // clamped to the drift the hardware is actually specified to have.
        // That is what stops a burst of short-term error from walking the
        // estimate somewhere the gyro could never have drifted to.
        if self.omega_i_sum_time >= 5.0 {
            let limit = gains.gyro_drift_rate * self.omega_i_sum_time;
            self.omega_i_sum.x = constrain_value(self.omega_i_sum.x, -limit, limit);
            self.omega_i_sum.y = constrain_value(self.omega_i_sum.y, -limit, limit);
            self.omega_i_sum.z = constrain_value(self.omega_i_sum.z, -limit, limit);
            self.omega_i += self.omega_i_sum;
            self.omega_i_sum = Vector3f::zero();
            self.omega_i_sum_time = 0.0;
        }

        DriftOutcome::Corrected
    }

    /// The pending integral accumulation and how long it has been building,
    /// upstream `_omega_I_sum` and `_omega_I_sum_time`.
    ///
    /// Exposed because the five-second batching is otherwise invisible: the
    /// bias estimate sits unchanged for five seconds and then steps.
    #[must_use]
    pub const fn pending_integral(&self) -> (Vector3f, f32) {
        (self.omega_i_sum, self.omega_i_sum_time)
    }

    /// Add yaw drift integral from [`crate::YawDriftCorrector`], upstream
    /// `_omega_I_sum.z += error_z * _ki_yaw * yaw_deltat`.
    pub fn add_yaw_integral_z(&mut self, delta: f32) {
        self.omega_i_sum.z += delta;
    }
}
