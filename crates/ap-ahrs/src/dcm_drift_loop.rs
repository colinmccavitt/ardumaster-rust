//! Wire INS delta-velocity into drift correction and feed omega back into
//! the DCM matrix update, upstream `AP_AHRS_DCM::drift_correction` with
//! compass yaw correction and GPS-heading fallback.

use ap_ins::{InertialSensorFrontend, LoopTiming};
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::Real;
use ap_math::vector3::Vector3f;

use crate::dcm_loop::{dcm_matrix_step_from_ins, DcmDriftOmega};
use crate::yaw_drift::{
    YawCompassSample, YawDriftContext, YawDriftCorrector, YawDriftGains, YawDriftInputs,
    YawGpsSample, YawMatrixAction,
};
use crate::{Dcm, DriftCorrector, DriftGains, DriftInputs, DriftOutcome, MatrixHealth};

/// Minimum interval before running drift correction without GPS, upstream
/// fallback when `_ra_deltat < 0.2f`.
pub const DRIFT_CORRECTION_INTERVAL_S: f32 = 0.2;

/// Compass and GPS samples plus vehicle context for yaw correction.
#[derive(Debug, Clone, Copy, Default)]
pub struct YawUpdateInputs {
    pub compass: Option<YawCompassSample>,
    pub gps: Option<YawGpsSample>,
    pub ctx: YawDriftContext,
}

/// Running drift-correction state between AHRS updates.
#[derive(Debug, Clone)]
pub struct DcmDriftLoop {
    /// The roll/pitch corrector, upstream `_omega_I`, `_omega_P`, `_error_rp`.
    pub corrector: DriftCorrector,
    /// Compass/GPS yaw corrector, upstream `_omega_yaw_P` and yaw `_error_yaw`.
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
        ins: &InertialSensorFrontend,
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
    pub fn try_correct(&mut self, dcm: &Dcm, ins: &InertialSensorFrontend) -> DriftOutcome {
        if self.ra_deltat < DRIFT_CORRECTION_INTERVAL_S {
            return DriftOutcome::NotEnoughData;
        }

        let inputs = DriftInputs {
            ra_sum: self.ra_sum,
            ra_deltat: self.ra_deltat,
            velocity_delta: None,
            dcm_matrix: dcm.matrix,
            omega: dcm.omega,
            ins_healthy: ins.get_gyro_health() && ins.get_accel_health(),
        };

        let outcome = self.corrector.correct(&inputs, &self.gains);
        if outcome == DriftOutcome::Corrected {
            self.ra_sum = Vector3f::zero();
            self.ra_deltat = 0.0;
        }
        outcome
    }

    /// Run compass or GPS yaw correction, upstream `drift_correction_yaw`.
    pub fn correct_yaw(&mut self, dcm: &mut Dcm, yaw: YawUpdateInputs, accel_ef_xy_mag: f32) {
        let (roll_rad, pitch_rad, estimated_yaw_rad) = dcm.matrix.to_euler();
        let mut ctx = yaw.ctx;
        ctx.estimated_yaw_rad = estimated_yaw_rad;

        let inputs = YawDriftInputs {
            dcm_matrix: dcm.matrix,
            omega: dcm.omega,
            accel_ef_xy_mag,
            compass: yaw.compass,
            gps: yaw.gps,
            roll_rad,
            pitch_rad,
            ctx,
        };

        let result = self.yaw.drift_correction_yaw(&inputs, &self.yaw_gains);
        if let YawMatrixAction::ResetAttitude { roll, pitch, yaw: yaw_rad } = result.matrix_action {
            dcm.matrix = Matrix3f::from_euler(roll, pitch, yaw_rad);
        }
        if result.omega_i_z != 0.0 {
            self.corrector.add_yaw_integral_z(result.omega_i_z);
        }
    }
}

/// One AHRS update: matrix step from INS, then drift accumulation and
/// correction. Order matches upstream `update()` for the matrix and drift
/// paths.
pub fn dcm_step_with_drift_from_ins(
    dcm: &mut Dcm,
    drift: &mut DcmDriftLoop,
    ins: &InertialSensorFrontend,
    timing: &LoopTiming,
    compass: Option<YawCompassSample>,
) -> MatrixHealth {
    let yaw = compass.map(|sample| YawUpdateInputs {
        compass: Some(sample),
        gps: None,
        ctx: YawDriftContext {
            compass_use_for_yaw: true,
            ..YawDriftContext::default()
        },
    });
    dcm_step_with_drift_from_ins_yaw(dcm, drift, ins, timing, yaw)
}

/// Same as [`dcm_step_with_drift_from_ins`] but accepts full yaw inputs
/// including GPS fallback context.
pub fn dcm_step_with_drift_from_ins_yaw(
    dcm: &mut Dcm,
    drift: &mut DcmDriftLoop,
    ins: &InertialSensorFrontend,
    timing: &LoopTiming,
    yaw: Option<YawUpdateInputs>,
) -> MatrixHealth {
    let health = dcm_matrix_step_from_ins(dcm, ins, timing, drift.drift_omega());
    drift.accumulate_from_ins(dcm, ins, timing, timing.delta_time());
    let _ = drift.try_correct(dcm, ins);
    if let Some(yaw_inputs) = yaw {
        let accel_ef_xy_mag = {
            let ef = dcm.matrix * ins.get_accel();
            ap_math::scalar::safe_sqrt(ef.x * ef.x + ef.y * ef.y)
        };
        drift.correct_yaw(dcm, yaw_inputs, accel_ef_xy_mag);
    }
    health
}
