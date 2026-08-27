//! Wire INS delta-velocity into drift correction and feed omega back into
//! the DCM matrix update, upstream `AP_AHRS_DCM::drift_correction` without
//! GPS, multi-accel, or yaw correction yet.
//!
//! This is the seam where the roll/pitch drift core in [`crate::DriftCorrector`]
//! meets the INS hookup in [`crate::dcm_matrix_step_from_ins`]: each loop
//! publishes delta velocity, the corrector accumulates earth-frame acceleration,
//! and once 0.2s has built up (upstream's no-GPS fallback interval) it
//! produces the `_omega_P` and `_omega_I` terms for the next matrix step.

use ap_ins::{ImuInstance, LoopTiming};
use ap_math::vector3::Vector3f;

use crate::dcm_loop::{dcm_matrix_step_from_ins, DcmDriftOmega};
use crate::{Dcm, DriftCorrector, DriftGains, DriftInputs, DriftOutcome, MatrixHealth};

/// Minimum interval before running drift correction without GPS, upstream
/// fallback when `_ra_deltat < 0.2f`.
pub const DRIFT_CORRECTION_INTERVAL_S: f32 = 0.2;

/// Running drift-correction state between AHRS updates.
#[derive(Debug, Clone)]
pub struct DcmDriftLoop {
    /// The roll/pitch corrector, upstream `_omega_I`, `_omega_P`, `_error_rp`.
    pub corrector: DriftCorrector,
    /// Proportional gain and drift-rate clamp, upstream AHRS parameters.
    pub gains: DriftGains,
    ra_sum: Vector3f,
    ra_deltat: f32,
}

impl Default for DcmDriftLoop {
    fn default() -> Self {
        Self::new(DriftGains::default())
    }
}

impl DcmDriftLoop {
    #[must_use]
    pub fn new(gains: DriftGains) -> Self {
        Self {
            corrector: DriftCorrector::new(),
            gains,
            ra_sum: Vector3f::zero(),
            ra_deltat: 0.0,
        }
    }

    #[must_use]
    pub fn drift_omega(&self) -> DcmDriftOmega {
        DcmDriftOmega {
            omega_i: self.corrector.omega_i,
            omega_p: self.corrector.omega_p,
            omega_yaw_p: Vector3f::zero(),
        }
    }

    /// Accumulate earth-frame acceleration from the IMU delta-velocity path,
    /// upstream `drift_correction`'s per-accel body.
    pub fn accumulate_from_ins(
        &mut self,
        dcm: &Dcm,
        imu: &ImuInstance,
        timing: &LoopTiming,
        loop_dt: f32,
    ) {
        let Some((delta_velocity, delta_velocity_dt)) = imu.get_delta_velocity(timing) else {
            return;
        };
        if delta_velocity_dt <= 0.0 {
            return;
        }
        let accel_ef = dcm.matrix * (delta_velocity / delta_velocity_dt);
        DriftCorrector::accumulate(&mut self.ra_sum, &mut self.ra_deltat, accel_ef, loop_dt);
    }

    /// Run correction once enough has accumulated. Resets the accumulator on
    /// success, upstream's post-correction memset of `_ra_sum`.
    pub fn try_correct(&mut self, dcm: &Dcm, imu: &ImuInstance) -> DriftOutcome {
        if self.ra_deltat < DRIFT_CORRECTION_INTERVAL_S {
            return DriftOutcome::NotEnoughData;
        }

        let inputs = DriftInputs {
            ra_sum: self.ra_sum,
            ra_deltat: self.ra_deltat,
            velocity_delta: None,
            dcm_matrix: dcm.matrix,
            omega: dcm.omega,
            ins_healthy: imu.gyro_healthy() && imu.accel_healthy(),
        };

        let outcome = self.corrector.correct(&inputs, &self.gains);
        if outcome == DriftOutcome::Corrected {
            self.ra_sum = Vector3f::zero();
            self.ra_deltat = 0.0;
        }
        outcome
    }
}

/// One AHRS update: matrix step from INS, then drift accumulation and
/// correction. Order matches upstream `update()` for the matrix and drift
/// paths.
pub fn dcm_step_with_drift_from_ins(
    dcm: &mut Dcm,
    drift: &mut DcmDriftLoop,
    imu: &ImuInstance,
    timing: &LoopTiming,
) -> MatrixHealth {
    let health = dcm_matrix_step_from_ins(dcm, imu, timing, drift.drift_omega());
    drift.accumulate_from_ins(dcm, imu, timing, timing.delta_time());
    let _ = drift.try_correct(dcm, imu);
    health
}
