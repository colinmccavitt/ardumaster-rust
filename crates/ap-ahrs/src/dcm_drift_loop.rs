//! Wire INS delta-velocity into drift correction and feed omega back into
//! the DCM matrix update, upstream `AP_AHRS_DCM::drift_correction` with
//! compass yaw correction; GPS and multi-accel paths not yet.

use ap_ins::{ImuInstance, LoopTiming};
use ap_math::scalar::Real;
use ap_math::vector3::Vector3f;

use crate::dcm_loop::{dcm_matrix_step_from_ins, DcmDriftOmega};
use crate::yaw_drift::{YawCompassSample, YawDriftCorrector, YawDriftGains, YawDriftInputs};
use crate::{Dcm, DriftCorrector, DriftGains, DriftInputs, DriftOutcome, MatrixHealth};

/// Minimum interval before running drift correction without GPS, upstream
/// fallback when `_ra_deltat < 0.2f`.
pub const DRIFT_CORRECTION_INTERVAL_S: f32 = 0.2;

/// Running drift-correction state between AHRS updates.
#[derive(Debug, Clone)]
pub struct DcmDriftLoop {
    /// The roll/pitch corrector, upstream `_omega_I`, `_omega_P`, `_error_rp`.
    pub corrector: DriftCorrector,
    /// Compass yaw corrector, upstream `_omega_yaw_P` and yaw `_error_yaw`.
    pub yaw: YawDriftCorrector,
    /// Proportional gain and drift-rate clamp, upstream AHRS parameters.
    pub gains: DriftGains,
    /// Yaw proportional gain, upstream `AHRS_YAW_P`.
    pub yaw_gains: YawDriftGains,
    ra_sum: Vector3f,
    ra_deltat: f32,
}

impl Default for DcmDriftLoop {
    fn default() -> Self {
        Self::new(DriftGains::default())
    }
}

impl DcmDriftLoop {
    /// Drift loop with roll/pitch gains and default yaw gains.
    #[must_use]
    pub fn new(gains: DriftGains) -> Self {
        Self {
            corrector: DriftCorrector::new(),
            yaw: YawDriftCorrector::new(),
            gains,
            yaw_gains: YawDriftGains::default(),
            ra_sum: Vector3f::zero(),
            ra_deltat: 0.0,
        }
    }

    /// Proportional and integral terms for the next matrix step.
    #[must_use]
    pub fn drift_omega(&self) -> DcmDriftOmega {
        DcmDriftOmega {
            omega_i: self.corrector.omega_i,
            omega_p: self.corrector.omega_p,
            omega_yaw_p: self.yaw.omega_yaw_p,
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

    /// Run roll/pitch correction once enough has accumulated. Resets the
    /// accumulator on success, upstream's post-correction memset of `_ra_sum`.
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

    /// Run compass yaw correction, upstream `drift_correction_yaw`.
    pub fn correct_yaw(&mut self, dcm: &Dcm, compass: YawCompassSample, accel_ef_xy_mag: f32) {
        let inputs = YawDriftInputs {
            dcm_matrix: dcm.matrix,
            omega: dcm.omega,
            accel_ef_xy_mag,
            compass,
        };
        let (_, omega_i_z) = self.yaw.correct(&inputs, &self.yaw_gains);
        if omega_i_z != 0.0 {
            self.corrector.add_yaw_integral_z(omega_i_z);
        }
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
    compass: Option<YawCompassSample>,
) -> MatrixHealth {
    let health = dcm_matrix_step_from_ins(dcm, imu, timing, drift.drift_omega());
    drift.accumulate_from_ins(dcm, imu, timing, timing.delta_time());
    let _ = drift.try_correct(dcm, imu);
    if let Some(sample) = compass {
        let accel_ef_xy_mag = {
            let ef = dcm.matrix * imu.accel();
            (ef.x * ef.x + ef.y * ef.y).sqrt()
        };
        drift.correct_yaw(dcm, sample, accel_ef_xy_mag);
    }
    health
}
