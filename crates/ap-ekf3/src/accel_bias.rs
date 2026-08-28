//! Accel-bias state update, upstream `AP_NavEKF3` AccelBias.
//!
//! Body-axis accel bias lives in states 13..15 as a delta-velocity (m/s).
//! This slice is the reset / constrain / inactive-IMU learn path, parallel
//! to [`crate::gyro_bias`]:
//!
//! - [`AccelBias::reset`] zeros the three states and reseeds `P[13..15]`
//!   to `sq(ACCEL_BIAS_LIM_SCALER * accBiasLim * dtEkfAvg)` — the
//!   CovarianceInit / variance-collapse path in `AP_NavEKF3_core.cpp`
//!   (there is no public `resetAccelBias`; this is that internal reset).
//! - [`AccelBias::constrain`] is the accel-bias half of `ConstrainStates`:
//!   each axis is clamped to `±_accBiasLim * dtEkfAvg`. The parameter
//!   default is 1.0 m/s/s (`EK3_ACC_BIAS_LIM`).
//! - [`AccelBias::learn_inactive`] is the accel half of
//!   `learnInactiveBiases`: copy the active filter estimate, or pull an
//!   unused IMU's bias toward the bias-corrected accel difference.
//!
//! [`AccelBias::get_accel_bias`] converts the stored delta-velocity back
//! to m/s/s the way `NavEKF3_core::getAccelBias` does (`bias / dtEkfAvg`).
//! Covariance prediction and the Kalman update that learns the *active*
//! bias from velocity residuals are not here.

use ap_math::scalar::{constrain_value, sq};
use ap_math::vector3::Vector3;
use ap_math::Ftype;

use crate::measurements::EKF_TARGET_DT;
use crate::{StateIndex, StateVector};

/// Typical INS loop delta used to seed `dtIMUavg` (400 Hz).
const DEFAULT_DT_IMU_AVG: Ftype = 0.0025;

/// Initial accel-bias uncertainty as a fraction of the state limit,
/// upstream `ACCEL_BIAS_LIM_SCALER`.
pub const ACCEL_BIAS_LIM_SCALER: Ftype = 0.2;

/// Default `EK3_ACC_BIAS_LIM`, m/s/s.
pub const ACCEL_BIAS_LIMIT_MPS2: Ftype = 1.0;

/// Single-sample error clamp in `learnInactiveBiases` accel half, m/s/s.
const LEARN_ERROR_LIMIT_MPS2: Ftype = 1.0;

/// Inactive-IMU pull gain, upstream `1.0e-4f * dtEkfAvg`.
const LEARN_GAIN: Ftype = 1.0e-4;

/// `getAccelBias` refuses to divide below this `dtEkfAvg`.
const DT_EKF_MIN: Ftype = 1.0e-6;

/// Body-axis accel bias (delta-velocity, m/s) plus the matching `P` diagonals.
///
/// Upstream overlays this on `statesArray[13..15]`. The port keeps a
/// [`Vector3`] so reset / constrain / learn can run without a covariance
/// matrix; [`AccelBias::write_into_states`] copies back onto the 24-vector.
#[derive(Debug, Clone, Copy)]
pub struct AccelBias {
    /// Delta-velocity bias (m/s), upstream `stateStruct.accel_bias`.
    bias: Vector3<Ftype>,
    /// `P[13][13]`, `P[14][14]`, `P[15][15]` after the variance reset.
    variance: Vector3<Ftype>,
    /// Expected IMU sample interval (s), upstream `dtIMUavg`.
    dt_imu_avg: Ftype,
    /// Expected EKF update interval (s), upstream `dtEkfAvg`.
    dt_ekf_avg: Ftype,
    /// Clamp in m/s/s, upstream `frontend->_accBiasLim`.
    acc_bias_limit: Ftype,
}

impl Default for AccelBias {
    fn default() -> Self {
        Self::new()
    }
}

impl AccelBias {
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
            acc_bias_limit: ACCEL_BIAS_LIMIT_MPS2,
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

    /// Delta-velocity bias (m/s), upstream `stateStruct.accel_bias`.
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

    /// Accel bias clamp in m/s/s, upstream `_accBiasLim`.
    #[must_use]
    pub const fn acc_bias_limit(&self) -> Ftype {
        self.acc_bias_limit
    }

    /// Override the `EK3_ACC_BIAS_LIM` clamp, for tests and later DAL wiring.
    pub fn set_acc_bias_limit(&mut self, limit_mps2: Ftype) {
        self.acc_bias_limit = limit_mps2;
    }

    /// Poke the stored delta-velocity bias (m/s). Tests use this to reach
    /// the constrain / learn paths without a Kalman update.
    pub fn set_bias(&mut self, bias: Vector3<Ftype>) {
        self.bias = bias;
    }

    /// Internal accel-bias reset, matching CovarianceInit / the
    /// variance-collapse path in `ForceSymmetry` / `ConstrainVariances`.
    ///
    /// Zeros the three states, clears the matching variance/covariance
    /// block (`zeroStatesVarCov(13, 15)` is a diagonal-only stub here),
    /// then seeds `P[13..15]` from `ACCEL_BIAS_LIM_SCALER * _accBiasLim`.
    pub fn reset(&mut self) {
        self.bias.set_zero();
        let sigma = ACCEL_BIAS_LIM_SCALER * self.acc_bias_limit * self.dt_ekf_avg;
        let var = sq(sigma);
        self.variance = Vector3::new(var, var, var);
    }

    /// Accel-bias half of upstream `ConstrainStates`.
    ///
    /// `statesArray[13..15] = constrain(…, ±_accBiasLim * dtEkfAvg)`.
    pub fn constrain(&mut self) {
        let bound = self.acc_bias_limit * self.dt_ekf_avg;
        self.bias.x = constrain_value(self.bias.x, -bound, bound);
        self.bias.y = constrain_value(self.bias.y, -bound, bound);
        self.bias.z = constrain_value(self.bias.z, -bound, bound);
    }

    /// Convert stored delta-velocity bias to m/s/s, upstream `getAccelBias`.
    ///
    /// Returns zero when `dtEkfAvg` is below `1e-6`, matching the gyro
    /// divide-by-near-zero guard (C++ only guards `statesInitialised`).
    #[must_use]
    pub fn get_accel_bias(&self) -> Vector3<Ftype> {
        if self.dt_ekf_avg < DT_EKF_MIN {
            return Vector3::zero();
        }
        self.bias / self.dt_ekf_avg
    }

    /// Accel half of upstream `learnInactiveBiases`.
    ///
    /// When `copy_active` is true this slot *is* the active IMU and just
    /// copies the filter estimate. Otherwise the unused IMU's bias is
    /// pulled toward the bias-corrected accel difference, with the
    /// single-sample error clamped to ±1.0 m/s/s and a `1e-4 * dtEkfAvg`
    /// step (a 0.5 m/s/s error takes about a minute to wipe).
    pub fn learn_inactive(
        &mut self,
        active: &Self,
        accel_active: Vector3<Ftype>,
        accel_inactive: Vector3<Ftype>,
        copy_active: bool,
    ) {
        if copy_active {
            self.bias = active.bias;
            return;
        }
        if self.dt_ekf_avg < DT_EKF_MIN || active.dt_ekf_avg < DT_EKF_MIN {
            return;
        }
        let filtered_active = accel_active - (active.bias / active.dt_ekf_avg);
        let filtered_inactive = accel_inactive - (self.bias / self.dt_ekf_avg);
        let mut error = filtered_active - filtered_inactive;
        let limit = LEARN_ERROR_LIMIT_MPS2;
        error.x = constrain_value(error.x, -limit, limit);
        error.y = constrain_value(error.y, -limit, limit);
        error.z = constrain_value(error.z, -limit, limit);
        self.bias -= error * (LEARN_GAIN * self.dt_ekf_avg);
    }

    /// Copy this bias onto `statesArray[13..15]`.
    pub fn write_into_states(&self, states: &mut StateVector) {
        write_axis(states, StateIndex::AccelBiasX, self.bias.x);
        write_axis(states, StateIndex::AccelBiasY, self.bias.y);
        write_axis(states, StateIndex::AccelBiasZ, self.bias.z);
    }

    /// Read `statesArray[13..15]` into the stored bias.
    pub fn read_from_states(&mut self, states: &StateVector) {
        self.bias.x = read_axis(states, StateIndex::AccelBiasX);
        self.bias.y = read_axis(states, StateIndex::AccelBiasY);
        self.bias.z = read_axis(states, StateIndex::AccelBiasZ);
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
        let mut accel = AccelBias::with_dt(0.0025 as Ftype, EKF_TARGET_DT);
        accel.set_bias(Vector3::new(0.1 as Ftype, -0.2 as Ftype, 0.3 as Ftype));
        accel.reset();

        let bias = accel.bias();
        near(bias.x, 0.0 as Ftype);
        near(bias.y, 0.0 as Ftype);
        near(bias.z, 0.0 as Ftype);

        let expected = sq(ACCEL_BIAS_LIM_SCALER * ACCEL_BIAS_LIMIT_MPS2 * accel.dt_ekf_avg());
        let var = accel.variance();
        near(var.x, expected);
        near(var.y, expected);
        near(var.z, expected);
        near(accel.acc_bias_limit(), ACCEL_BIAS_LIMIT_MPS2);
    }

    #[test]
    fn constrain_clamps_each_axis_to_limit_times_dt() {
        let mut accel = AccelBias::with_dt(0.0025 as Ftype, EKF_TARGET_DT);
        let bound = ACCEL_BIAS_LIMIT_MPS2 * EKF_TARGET_DT;
        accel.set_bias(Vector3::new(
            bound * 3.0 as Ftype,
            -bound * 4.0 as Ftype,
            bound * 0.25 as Ftype,
        ));
        accel.constrain();

        let bias = accel.bias();
        near(bias.x, bound);
        near(bias.y, -bound);
        near(bias.z, bound * 0.25 as Ftype);
    }

    #[test]
    fn learn_inactive_copies_active_slot() {
        let mut active = AccelBias::new();
        active.set_bias(Vector3::new(0.01 as Ftype, -0.02 as Ftype, 0.03 as Ftype));
        let mut inactive = AccelBias::new();
        inactive.learn_inactive(&active, Vector3::zero(), Vector3::zero(), true);
        let bias = inactive.bias();
        near(bias.x, 0.01 as Ftype);
        near(bias.y, -0.02 as Ftype);
        near(bias.z, 0.03 as Ftype);
    }

    #[test]
    fn learn_inactive_pulls_toward_accel_error() {
        let active = AccelBias::new();
        let mut inactive = AccelBias::new();
        // Unused accel reads 0.05 m/s/s high (under the 1.0 m/s/s clamp);
        // the step should raise its bias so later `accel - bias/dt` moves
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
    fn get_accel_bias_is_rate_when_dt_valid() {
        let mut accel = AccelBias::with_dt(0.0025 as Ftype, EKF_TARGET_DT);
        accel.set_bias(Vector3::new(EKF_TARGET_DT, 0.0 as Ftype, 0.0 as Ftype));
        let rate = accel.get_accel_bias();
        near(rate.x, 1.0 as Ftype);
        near(rate.y, 0.0 as Ftype);
        near(rate.z, 0.0 as Ftype);
    }

    #[test]
    fn get_accel_bias_is_zero_when_dt_tiny() {
        let mut accel = AccelBias::with_dt(0.0025 as Ftype, 1.0e-9 as Ftype);
        accel.set_bias(Vector3::new(1.0 as Ftype, 1.0 as Ftype, 1.0 as Ftype));
        let rate = accel.get_accel_bias();
        near(rate.x, 0.0 as Ftype);
        near(rate.y, 0.0 as Ftype);
        near(rate.z, 0.0 as Ftype);
    }

    #[test]
    fn write_and_read_round_trip_states_13_to_15() {
        let mut accel = AccelBias::new();
        accel.set_bias(Vector3::new(0.004 as Ftype, -0.005 as Ftype, 0.006 as Ftype));
        let mut states: StateVector = [0.0 as Ftype; crate::STATE_VECTOR_LEN];
        accel.write_into_states(&mut states);
        near(
            match states.get(StateIndex::AccelBiasX.as_usize()) {
                Some(&v) => v,
                None => 0.0 as Ftype,
            },
            0.004 as Ftype,
        );
        near(
            match states.get(StateIndex::AccelBiasZ.as_usize()) {
                Some(&v) => v,
                None => 0.0 as Ftype,
            },
            0.006 as Ftype,
        );

        let mut copy = AccelBias::new();
        copy.read_from_states(&states);
        near(copy.bias().x, 0.004 as Ftype);
        near(copy.bias().y, -0.005 as Ftype);
        near(copy.bias().z, 0.006 as Ftype);
    }
}
