//! Gyro-bias state update, upstream `AP_NavEKF3_GyroBias.cpp`.
//!
//! Body-axis gyro bias lives in states 10..12 as a delta-angle (rad).
//! This slice is the reset / constrain / inactive-IMU learn path:
//!
//! - [`GyroBias::reset`] zeros the three states and reseeds `P[10..12]`
//!   to `sq(radians(0.5 * dtIMUavg))` — the 0.5 deg/s static-calibration
//!   accuracy named in `resetGyroBias`.
//! - [`GyroBias::constrain`] is the gyro-bias half of `ConstrainStates`:
//!   each axis is clamped to `±getGyroBiasLimit() * dtEkfAvg`. The INS
//!   fallback limit is 0.5 rad/s when no backend has claimed the IMU.
//! - [`GyroBias::learn_inactive`] is the gyro half of
//!   `learnInactiveBiases`: copy the active filter estimate, or pull an
//!   unused IMU's bias toward the bias-corrected rate difference.
//!
//! [`GyroBias::get_gyro_bias`] converts the stored delta-angle back to
//! rad/s the way `NavEKF3_core::getGyroBias` does (`bias / dtEkfAvg`).
//! Covariance prediction and the Kalman update that learns the *active*
//! bias from attitude residuals are not here.

use ap_math::scalar::{constrain_value, radians, sq};
use ap_math::vector3::Vector3;
use ap_math::Ftype;

use crate::measurements::EKF_TARGET_DT;
use crate::{StateIndex, StateVector};

/// Typical INS loop delta used to seed `dtIMUavg` (400 Hz).
const DEFAULT_DT_IMU_AVG: Ftype = 0.0025;

/// Legacy INS fallback for `get_gyro_bias_limit`, rad/s.
pub const GYRO_BIAS_LIMIT_RAD_S: Ftype = 0.5;

/// Sensor-agnostic `get_gyro_bias_init_dps` fallback, deg/s.
pub const GYRO_BIAS_INIT_DPS: Ftype = 2.5;

/// Calibration accuracy assumed by `resetGyroBias`, deg/s.
const RESET_CAL_DPS: Ftype = 0.5;

/// Single-sample error clamp in `learnInactiveBiases`, deg/s.
const LEARN_ERROR_LIMIT_DPS: Ftype = 5.0;

/// Inactive-IMU pull gain, upstream `1.0e-4f * dtEkfAvg`.
const LEARN_GAIN: Ftype = 1.0e-4;

/// `getGyroBias` refuses to divide below this `dtEkfAvg`.
const DT_EKF_MIN: Ftype = 1.0e-6;

/// Body-axis gyro bias (delta-angle, rad) plus the matching `P` diagonals.
///
/// Upstream overlays this on `statesArray[10..12]`. The port keeps a
/// [`Vector3`] so reset / constrain / learn can run without a covariance
/// matrix; [`GyroBias::write_into_states`] copies back onto the 24-vector.
#[derive(Debug, Clone, Copy)]
pub struct GyroBias {
    /// Delta-angle bias (rad), upstream `stateStruct.gyro_bias`.
    bias: Vector3<Ftype>,
    /// `P[10][10]`, `P[11][11]`, `P[12][12]` after `resetGyroBias`.
    variance: Vector3<Ftype>,
    /// Expected IMU sample interval (s), upstream `dtIMUavg`.
    dt_imu_avg: Ftype,
    /// Expected EKF update interval (s), upstream `dtEkfAvg`.
    dt_ekf_avg: Ftype,
    /// Clamp in rad/s, upstream `getGyroBiasLimit`.
    gyro_bias_limit: Ftype,
    /// Seed 1-sigma in deg/s, upstream `InitialGyroBiasUncertainty`.
    initial_uncertainty_dps: Ftype,
}

impl Default for GyroBias {
    fn default() -> Self {
        Self::new()
    }
}

impl GyroBias {
    /// Zero bias, zero variance, INS / EKF default intervals and limits.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            bias: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            variance: Vector3 {
                x: 0.0 as Ftype,
                y: 0.0 as Ftype,
                z: 0.0 as Ftype,
            },
            dt_imu_avg: DEFAULT_DT_IMU_AVG,
            dt_ekf_avg: EKF_TARGET_DT,
            gyro_bias_limit: GYRO_BIAS_LIMIT_RAD_S,
            initial_uncertainty_dps: GYRO_BIAS_INIT_DPS,
        }
    }

    /// Construct with explicit `dtIMUavg` / `dtEkfAvg`.
    #[must_use]
    pub const fn with_dt(dt_imu_avg: Ftype, dt_ekf_avg: Ftype) -> Self {
        let mut bias = Self::new();
        bias.dt_imu_avg = dt_imu_avg;
        bias.dt_ekf_avg = dt_ekf_avg;
        bias
    }

    /// Delta-angle bias (rad), upstream `stateStruct.gyro_bias`.
    #[must_use]
    pub const fn bias(&self) -> Vector3<Ftype> {
        self.bias
    }

    /// Covariance diagonals reseeded by [`reset`](Self::reset).
    #[must_use]
    pub const fn variance(&self) -> Vector3<Ftype> {
        self.variance
    }

    /// Expected IMU sample interval (s).
    #[must_use]
    pub const fn dt_imu_avg(&self) -> Ftype {
        self.dt_imu_avg
    }

    /// Expected EKF update interval (s).
    #[must_use]
    pub const fn dt_ekf_avg(&self) -> Ftype {
        self.dt_ekf_avg
    }

    /// Gyro bias clamp in rad/s, upstream `getGyroBiasLimit`.
    #[must_use]
    pub const fn gyro_bias_limit(&self) -> Ftype {
        self.gyro_bias_limit
    }

    /// Seed 1-sigma in deg/s, upstream `InitialGyroBiasUncertainty`.
    #[must_use]
    pub const fn initial_gyro_bias_uncertainty(&self) -> Ftype {
        self.initial_uncertainty_dps
    }

    /// Override the INS clamp, for tests and later DAL wiring.
    pub fn set_gyro_bias_limit(&mut self, limit_rad_s: Ftype) {
        self.gyro_bias_limit = limit_rad_s;
    }

    /// Override `InitialGyroBiasUncertainty`, for tests and later DAL wiring.
    pub fn set_initial_uncertainty_dps(&mut self, dps: Ftype) {
        self.initial_uncertainty_dps = dps;
    }

    /// Poke the stored delta-angle bias (rad). Tests use this to reach
    /// the constrain / learn paths without a Kalman update.
    pub fn set_bias(&mut self, bias: Vector3<Ftype>) {
        self.bias = bias;
    }

    /// Upstream `NavEKF3_core::resetGyroBias`.
    ///
    /// Zeros the three states, clears the matching variance/covariance
    /// block (`zeroStatesVarCov(10, 12)` is a diagonal-only stub here),
    /// then seeds `P[10..12]` from a 0.5 deg/s calibration.
    pub fn reset(&mut self) {
        self.bias.set_zero();
        let sigma = radians(RESET_CAL_DPS * self.dt_imu_avg);
        let var = sq(sigma);
        self.variance = Vector3::new(var, var, var);
    }

    /// Gyro-bias half of upstream `ConstrainStates`.
    ///
    /// `statesArray[10..12] = constrain(…, ±gyro_bias_limit * dtEkfAvg)`.
    pub fn constrain(&mut self) {
        let bound = self.gyro_bias_limit * self.dt_ekf_avg;
        self.bias.x = constrain_value(self.bias.x, -bound, bound);
        self.bias.y = constrain_value(self.bias.y, -bound, bound);
        self.bias.z = constrain_value(self.bias.z, -bound, bound);
    }

    /// Convert stored delta-angle bias to rad/s, upstream `getGyroBias`.
    ///
    /// Returns zero when `dtEkfAvg` is below `1e-6`, matching the C++
    /// divide-by-near-zero guard.
    #[must_use]
    pub fn get_gyro_bias(&self) -> Vector3<Ftype> {
        if self.dt_ekf_avg < DT_EKF_MIN {
            return Vector3::zero();
        }
        self.bias / self.dt_ekf_avg
    }

    /// Gyro half of upstream `learnInactiveBiases`.
    ///
    /// When `copy_active` is true this slot *is* the active IMU and just
    /// copies the filter estimate. Otherwise the unused IMU's bias is
    /// pulled toward the bias-corrected rate difference, with the
    /// single-sample error clamped to ±5 deg/s and a `1e-4 * dtEkfAvg`
    /// step (a 5 deg/s error takes about a minute to wipe).
    pub fn learn_inactive(
        &mut self,
        active: &Self,
        gyro_active: Vector3<Ftype>,
        gyro_inactive: Vector3<Ftype>,
        copy_active: bool,
    ) {
        if copy_active {
            self.bias = active.bias;
            return;
        }
        if self.dt_ekf_avg < DT_EKF_MIN || active.dt_ekf_avg < DT_EKF_MIN {
            return;
        }
        let filtered_active = gyro_active - (active.bias / active.dt_ekf_avg);
        let filtered_inactive = gyro_inactive - (self.bias / self.dt_ekf_avg);
        let mut error = filtered_active - filtered_inactive;
        let limit = radians(LEARN_ERROR_LIMIT_DPS);
        error.x = constrain_value(error.x, -limit, limit);
        error.y = constrain_value(error.y, -limit, limit);
        error.z = constrain_value(error.z, -limit, limit);
        self.bias -= error * (LEARN_GAIN * self.dt_ekf_avg);
    }

    /// Copy this bias onto `statesArray[10..12]`.
    pub fn write_into_states(&self, states: &mut StateVector) {
        write_axis(states, StateIndex::GyroBiasX, self.bias.x);
        write_axis(states, StateIndex::GyroBiasY, self.bias.y);
        write_axis(states, StateIndex::GyroBiasZ, self.bias.z);
    }

    /// Read `statesArray[10..12]` into the stored bias.
    pub fn read_from_states(&mut self, states: &StateVector) {
        self.bias.x = read_axis(states, StateIndex::GyroBiasX);
        self.bias.y = read_axis(states, StateIndex::GyroBiasY);
        self.bias.z = read_axis(states, StateIndex::GyroBiasZ);
    }
}

fn write_axis(states: &mut StateVector, index: StateIndex, value: Ftype) {
    if let Some(slot) = states.get_mut(index.as_usize()) {
        *slot = value;
    }
}

fn read_axis(states: &StateVector, index: StateIndex) -> Ftype {
    match states.get(index.as_usize()) {
        Some(&value) => value,
        None => 0.0 as Ftype,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: Ftype, b: Ftype) {
        let err = if a > b { a - b } else { b - a };
        assert!(err < 1.0e-6 as Ftype, "{a} !~= {b}");
    }

    #[test]
    fn reset_zeros_bias_and_seeds_covariance() {
        let mut gyro = GyroBias::with_dt(0.0025 as Ftype, EKF_TARGET_DT);
        gyro.set_bias(Vector3::new(0.1 as Ftype, -0.2 as Ftype, 0.3 as Ftype));
        gyro.reset();

        let bias = gyro.bias();
        near(bias.x, 0.0 as Ftype);
        near(bias.y, 0.0 as Ftype);
        near(bias.z, 0.0 as Ftype);

        let expected = sq(radians(RESET_CAL_DPS * gyro.dt_imu_avg()));
        let var = gyro.variance();
        near(var.x, expected);
        near(var.y, expected);
        near(var.z, expected);
        near(gyro.initial_gyro_bias_uncertainty(), GYRO_BIAS_INIT_DPS);
        near(gyro.gyro_bias_limit(), GYRO_BIAS_LIMIT_RAD_S);
    }

    #[test]
    fn constrain_clamps_each_axis_to_limit_times_dt() {
        let mut gyro = GyroBias::with_dt(0.0025 as Ftype, EKF_TARGET_DT);
        let bound = GYRO_BIAS_LIMIT_RAD_S * EKF_TARGET_DT;
        gyro.set_bias(Vector3::new(
            bound * 3.0 as Ftype,
            -bound * 4.0 as Ftype,
            bound * 0.25 as Ftype,
        ));
        gyro.constrain();

        let bias = gyro.bias();
        near(bias.x, bound);
        near(bias.y, -bound);
        near(bias.z, bound * 0.25 as Ftype);
    }

    #[test]
    fn learn_inactive_copies_active_slot() {
        let mut active = GyroBias::new();
        active.set_bias(Vector3::new(0.01 as Ftype, -0.02 as Ftype, 0.03 as Ftype));
        let mut inactive = GyroBias::new();
        inactive.learn_inactive(
            &active,
            Vector3::zero(),
            Vector3::zero(),
            true,
        );
        let bias = inactive.bias();
        near(bias.x, 0.01 as Ftype);
        near(bias.y, -0.02 as Ftype);
        near(bias.z, 0.03 as Ftype);
    }

    #[test]
    fn learn_inactive_pulls_toward_rate_error() {
        let active = GyroBias::new();
        let mut inactive = GyroBias::new();
        // Unused gyro reads 0.05 rad/s high (under the 5 deg/s clamp);
        // the step should raise its bias so later `gyro - bias/dt` moves
        // toward the active IMU.
        let rate_err = 0.05 as Ftype;
        inactive.learn_inactive(
            &active,
            Vector3::zero(),
            Vector3::new(rate_err, 0.0 as Ftype, 0.0 as Ftype),
            false,
        );
        let step = rate_err * (LEARN_GAIN * EKF_TARGET_DT);
        near(inactive.bias().x, step);
        near(inactive.bias().y, 0.0 as Ftype);
        near(inactive.bias().z, 0.0 as Ftype);
    }

    #[test]
    fn get_gyro_bias_is_rate_when_dt_valid() {
        let mut gyro = GyroBias::with_dt(0.0025 as Ftype, EKF_TARGET_DT);
        gyro.set_bias(Vector3::new(EKF_TARGET_DT, 0.0 as Ftype, 0.0 as Ftype));
        let rate = gyro.get_gyro_bias();
        near(rate.x, 1.0 as Ftype);
        near(rate.y, 0.0 as Ftype);
        near(rate.z, 0.0 as Ftype);
    }

    #[test]
    fn get_gyro_bias_is_zero_when_dt_tiny() {
        let mut gyro = GyroBias::with_dt(0.0025 as Ftype, 1.0e-9 as Ftype);
        gyro.set_bias(Vector3::new(1.0 as Ftype, 1.0 as Ftype, 1.0 as Ftype));
        let rate = gyro.get_gyro_bias();
        near(rate.x, 0.0 as Ftype);
        near(rate.y, 0.0 as Ftype);
        near(rate.z, 0.0 as Ftype);
    }

    #[test]
    fn write_and_read_round_trip_states_10_to_12() {
        let mut gyro = GyroBias::new();
        gyro.set_bias(Vector3::new(0.004 as Ftype, -0.005 as Ftype, 0.006 as Ftype));
        let mut states: StateVector = [0.0 as Ftype; crate::STATE_VECTOR_LEN];
        gyro.write_into_states(&mut states);
        near(
            match states.get(StateIndex::GyroBiasX.as_usize()) {
                Some(&v) => v,
                None => 0.0 as Ftype,
            },
            0.004 as Ftype,
        );
        near(
            match states.get(StateIndex::GyroBiasZ.as_usize()) {
                Some(&v) => v,
                None => 0.0 as Ftype,
            },
            0.006 as Ftype,
        );

        let mut copy = GyroBias::new();
        copy.read_from_states(&states);
        near(copy.bias().x, 0.004 as Ftype);
        near(copy.bias().y, -0.005 as Ftype);
        near(copy.bias().z, 0.006 as Ftype);
    }
}
