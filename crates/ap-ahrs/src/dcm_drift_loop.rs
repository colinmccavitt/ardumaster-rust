//! Wire INS delta-velocity into drift correction and feed omega back into
//! the DCM matrix update, upstream `AP_AHRS_DCM::drift_correction` with
//! compass yaw correction and GPS-heading fallback.

use ap_ins::{InertialSensorFrontend, LoopTiming};
use ap_math::matrix3::Matrix3f;
use ap_math::scalar::Real;
use ap_math::vector3::Vector3f;

use crate::dcm_loop::{dcm_matrix_step_from_ins, DcmDriftOmega};
use crate::wind_estimation::{WindEstimateInputs, WindEstimator};
use crate::yaw_drift::{
    YawCompassSample, YawDriftContext, YawDriftCorrector, YawDriftGains, YawDriftInputs,
    YawGpsSample, YawMatrixAction,
};
use crate::dead_reckoning::DeadReckoningPosition;
use crate::multi_accel::{MultiAccelAccumulator, MultiAccelSelection, INS_MAX_INSTANCES};
use crate::{Dcm, DriftCorrector, DriftGains, DriftInputs, DriftOutcome, GpsLagBuffer, MatrixHealth};

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

/// GPS, airspeed, and wind-estimation context for roll/pitch drift correction.
#[derive(Debug, Clone, Copy)]
pub struct DriftMotionInputs {
    pub now_ms: u32,
    /// Earth-frame velocity when GPS is locked.
    pub gps_velocity: Option<Vector3f>,
    /// True when a new GPS fix arrived this cycle.
    pub new_gps_fix: bool,
    pub have_gps: bool,
    pub fly_forward: bool,
    pub airspeed_tas: f32,
    pub eas2tas: f32,
    pub wind_estimation_enabled: bool,
    pub correct_centrifugal: bool,
    /// GPS latitude in deg*1e7 when a fix is available.
    pub gps_lat_e7: Option<i32>,
    /// GPS longitude in deg*1e7 when a fix is available.
    pub gps_lng_e7: Option<i32>,
}

impl Default for DriftMotionInputs {
    fn default() -> Self {
        Self {
            now_ms: 0,
            gps_velocity: None,
            new_gps_fix: false,
            have_gps: false,
            fly_forward: false,
            airspeed_tas: 0.0,
            eas2tas: 1.0,
            wind_estimation_enabled: true,
            correct_centrifugal: true,
            gps_lat_e7: None,
            gps_lng_e7: None,
        }
    }
}

/// Running drift-correction state between AHRS updates.
#[derive(Debug, Clone)]
pub struct DcmDriftLoop {
    /// The roll/pitch corrector, upstream `_omega_I`, `_omega_P`, `_error_rp`.
    pub corrector: DriftCorrector,
    /// Compass/GPS yaw corrector, upstream `_omega_yaw_P` and yaw `_error_yaw`.
    pub yaw: YawDriftCorrector,
    /// Wind estimate for no-GPS drift and yaw consistency.
    pub wind: WindEstimator,
    /// Proportional gain and drift-rate clamp, upstream AHRS parameters.
    pub gains: DriftGains,
    /// Yaw proportional gain, upstream `AHRS_YAW_P`.
    pub yaw_gains: YawDriftGains,
    ra_sums: MultiAccelAccumulator,
    ra_deltat: f32,
    active_accel_instance: i8,
    pub position: DeadReckoningPosition,
    last_velocity: Vector3f,
    have_gps_lock: bool,
    drift_velocity_initialized: bool,
    /// GPS lag delay line for drift correction, upstream `_ra_delay_buffer`.
    gps_lag: GpsLagBuffer,
}

impl Default for DcmDriftLoop {
    fn default() -> Self {
        Self::new(DriftGains::default())
    }
}

impl DcmDriftLoop {
    /// Whether GPS velocity is fused for drift correction, upstream `using_gps()`.
    #[must_use]
    pub fn using_gps(&self) -> bool {
        self.have_gps_lock
    }

    /// Drift loop with roll/pitch gains and default yaw gains.
    #[must_use]
    pub fn new(gains: DriftGains) -> Self {
        Self {
            corrector: DriftCorrector::new(),
            yaw: YawDriftCorrector::new(),
            wind: WindEstimator::new(),
            gains,
            yaw_gains: YawDriftGains::default(),
            ra_sums: MultiAccelAccumulator::default(),
            ra_deltat: 0.0,
            active_accel_instance: -1,
            position: DeadReckoningPosition::default(),
            last_velocity: Vector3f::zero(),
            have_gps_lock: false,
            drift_velocity_initialized: false,
            gps_lag: GpsLagBuffer::default(),
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
        let accel_count = ins.accel_count().min(INS_MAX_INSTANCES as u8);
        for i in 0..accel_count {
            if !ins.accel_usable(i) {
                continue;
            }
            let Some(imu) = ins.imu(i) else {
                continue;
            };
            let Some((delta_velocity, delta_velocity_dt)) = imu.get_delta_velocity(timing) else {
                continue;
            };
            if delta_velocity_dt <= 0.0 {
                continue;
            }
            let accel_ef = dcm.matrix * (delta_velocity / delta_velocity_dt);
            DriftCorrector::accumulate(
                &mut self.ra_sums.ra_sum[i as usize],
                &mut self.ra_deltat,
                accel_ef,
                loop_dt,
            );
        }
    }

    fn update_wind(&mut self, dcm: &Dcm, motion: DriftMotionInputs) {
        if !motion.new_gps_fix {
            return;
        }
        let Some(velocity) = motion.gps_velocity else {
            return;
        };
        let airspeed_eas = if motion.airspeed_tas > 0.0 && motion.eas2tas > 0.0 {
            Some(motion.airspeed_tas / motion.eas2tas)
        } else {
            None
        };
        self.wind.estimate(WindEstimateInputs {
            now_ms: motion.now_ms,
            velocity,
            fuselage_direction: dcm.matrix.colx(),
            airspeed_eas,
            eas2tas: motion.eas2tas,
            enabled: motion.wind_estimation_enabled,
        });
    }

    fn resolve_velocity(&mut self, dcm: &Dcm, motion: DriftMotionInputs) -> Option<Vector3f> {
        if motion.have_gps {
            if !motion.new_gps_fix {
                return None;
            }
            let velocity = motion.gps_velocity?;
            if !self.have_gps_lock {
                self.last_velocity = velocity;
            }
            self.have_gps_lock = true;
            Some(velocity)
        } else {
            self.have_gps_lock = false;
            if !motion.fly_forward || motion.airspeed_tas <= 0.0 {
                return None;
            }
            Some(self.wind.ground_velocity_no_gps(dcm.matrix.colx(), motion.airspeed_tas))
        }
    }


    fn update_position(&mut self, motion: DriftMotionInputs, velocity: Option<Vector3f>) {
        if motion.have_gps && motion.new_gps_fix {
            if let (Some(lat), Some(lng)) = (motion.gps_lat_e7, motion.gps_lng_e7) {
                self.position.on_gps_fix(lat, lng, motion.now_ms);
            } else if motion.gps_velocity.is_some() {
                self.position.on_gps_fix(0, 0, motion.now_ms);
            }
        } else if let Some(v) = velocity {
            self.position.integrate(v, self.ra_deltat, motion.have_gps);
        }
    }

    /// Run roll/pitch correction once enough has accumulated. Resets the
    /// accumulator on success, upstream's post-correction memset of `_ra_sum`.
    pub fn try_correct(
        &mut self,
        dcm: &Dcm,
        ins: &InertialSensorFrontend,
        motion: DriftMotionInputs,
    ) -> DriftOutcome {
        if self.ra_deltat < DRIFT_CORRECTION_INTERVAL_S {
            return DriftOutcome::NotEnoughData;
        }

        self.update_wind(dcm, motion);
        let velocity = self.resolve_velocity(dcm, motion);

        if !self.drift_velocity_initialized {
            if let Some(v) = velocity {
                self.last_velocity = v;
                self.drift_velocity_initialized = true;
            }
            self.ra_sums.reset();
            self.ra_deltat = 0.0;
            return DriftOutcome::NotEnoughData;
        }

        let velocity_delta = if motion.correct_centrifugal
            && (self.have_gps_lock || motion.fly_forward)
        {
            velocity.map(|v| v - self.last_velocity)
        } else {
            None
        };

let ra_scale = if self.ra_deltat > 0.0 {
            1.0 / (self.ra_deltat * 9.806_65)
        } else {
            0.0
        };
        let mut ga_e = Vector3f::new(0.0, 0.0, -1.0);
        if let Some(velocity_delta) = velocity_delta {
            ga_e += velocity_delta * (self.gains.gps_gain * ra_scale);
            let _ = ga_e.normalize();
        }

        let preselected = MultiAccelSelection::select(
            &self.ra_sums.ra_sum,
            ins.accel_count().min(INS_MAX_INSTANCES as u8),
            |i| ins.accel_usable(i),
            ga_e,
            ra_scale,
            self.have_gps_lock && motion.have_gps,
            &mut self.gps_lag,
        );

        let primary_ra_sum = preselected
            .map(|sel| {
                self.active_accel_instance = sel.active_instance;
                self.ra_sums.ra_sum[sel.active_instance as usize]
            })
            .unwrap_or(Vector3f::zero());

        let inputs = DriftInputs {
            ra_sum: primary_ra_sum,
            ra_deltat: self.ra_deltat,
            velocity_delta,
            dcm_matrix: dcm.matrix,
            omega: dcm.omega,
            ins_healthy: ins.get_gyro_health() && ins.get_accel_health(),
            using_gps_corrections: self.have_gps_lock && motion.have_gps,
            preselected_error: preselected,
        };

        let outcome = self.corrector.correct(&inputs, &self.gains, &mut self.gps_lag);
        if outcome == DriftOutcome::Corrected {
            if let Some(v) = velocity {
                self.last_velocity = v;
            }
            self.ra_sums.reset();
            self.ra_deltat = 0.0;
        }
        outcome
    }

    /// Run compass or GPS yaw correction, upstream `drift_correction_yaw`.
    pub fn correct_yaw(&mut self, dcm: &mut Dcm, yaw: YawUpdateInputs, accel_ef_xy_mag: f32) {
        let (roll_rad, pitch_rad, estimated_yaw_rad) = dcm.matrix.to_euler();
        let mut ctx = yaw.ctx;
        ctx.estimated_yaw_rad = estimated_yaw_rad;
        ctx.wind_speed_xy = self.wind.wind_speed_xy();

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
    dcm_step_with_drift_from_ins_yaw(dcm, drift, ins, timing, yaw, DriftMotionInputs::default())
}

/// Same as [`dcm_step_with_drift_from_ins`] but accepts full yaw inputs
/// including GPS fallback context.
pub fn dcm_step_with_drift_from_ins_yaw(
    dcm: &mut Dcm,
    drift: &mut DcmDriftLoop,
    ins: &InertialSensorFrontend,
    timing: &LoopTiming,
    yaw: Option<YawUpdateInputs>,
    motion: DriftMotionInputs,
) -> MatrixHealth {
    let health = dcm_matrix_step_from_ins(dcm, ins, timing, drift.drift_omega());
    drift.accumulate_from_ins(dcm, ins, timing, timing.delta_time());
    drift.update_wind(dcm, motion);
    let velocity = drift.resolve_velocity(dcm, motion);
    drift.update_position(motion, velocity);
    let _ = drift.try_correct(dcm, ins, motion);
    if let Some(yaw_inputs) = yaw {
        let accel_ef_xy_mag = {
            let ef = dcm.matrix * ins.get_accel();
            ap_math::scalar::safe_sqrt(ef.x * ef.x + ef.y * ef.y)
        };
        drift.correct_yaw(dcm, yaw_inputs, accel_ef_xy_mag);
    }
    health
}
