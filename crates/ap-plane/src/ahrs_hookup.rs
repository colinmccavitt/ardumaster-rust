//! AHRS attitude feed for the main vehicle loop, upstream `Plane::ahrs_update`.
//!
//! The scheduler calls [`PlaneMainLoop::ahrs_update`] every fast tick; this
//! module owns the DCM state and publishes roll/pitch/yaw sensors the stabilize
//! path reads on the next tasks.

use ap_ahrs::{
    active_backend_kind, backend_for_kind, dcm_step_with_drift_from_ins_yaw, ekf3_full_update_from_ins,
    head_wind_from_yaw, wind_alignment, AhrsBackendKind, Dcm, DcmDriftLoop, DriftMotionInputs, Ekf3Loop,
    MatrixHealth, WindVaneSample, YawCompassSample, YawDriftContext, YawGpsSample, YawUpdateInputs,
};
use ap_ins::{InertialSensorFrontend, LoopTiming};
use ap_math::scalar::{cd_to_rad, rad_to_cd, radians, wrap_180_cd, wrap_360_cd, Real};
use ap_math::vector3::Vector3f;

/// Attitude sensors published each loop, upstream `AP_AHRS` roll/pitch/yaw_sensor.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AhrsAttitude {
    /// Roll, centidegrees. Upstream `ahrs.roll_sensor`.
    pub roll_sensor_cd: i32,
    /// Pitch, centidegrees. Upstream `ahrs.pitch_sensor`.
    pub pitch_sensor_cd: i32,
    /// Yaw, centidegrees, `0..35999`. Upstream `ahrs.yaw_sensor`.
    pub yaw_sensor_cd: i32,
}

impl AhrsAttitude {
    /// Roll attitude in radians, upstream `AP_AHRS::get_roll_rad()`.
    #[must_use]
    pub fn roll_rad(self) -> f32 {
        cd_to_rad(self.roll_sensor_cd as f32)
    }

    /// Pitch attitude in radians, upstream `AP_AHRS::get_pitch_rad()`.
    #[must_use]
    pub fn pitch_rad(self) -> f32 {
        cd_to_rad(self.pitch_sensor_cd as f32)
    }

    /// Yaw attitude in radians, upstream `AP_AHRS::get_yaw_rad()`.
    #[must_use]
    pub fn yaw_rad(self) -> f32 {
        cd_to_rad(self.yaw_sensor_cd as f32)
    }
}

/// Running AHRS state the vehicle loop owns, upstream `AP::ahrs()`.
#[derive(Debug, Clone)]
pub struct AhrsFeed {
    /// Parameter-selected backend, upstream `configured_ekf_type`.
    pub configured_backend: AhrsBackendKind,
    /// Backend driving attitude this cycle, upstream `active_EKF_type`.
    pub active_backend: AhrsBackendKind,
    /// EKF health for fallback decisions; stub until NavEKF3 is ported.
    pub ekf_healthy: bool,
    /// NavEKF3 filter stub, upstream `NavEKF3`.
    pub ekf3: Ekf3Loop,
    /// Direction-cosine attitude estimate.
    pub dcm: Dcm,
    /// Roll/pitch drift and compass yaw correction.
    pub drift: DcmDriftLoop,
    pub(crate) last_gps_fix_ms: u32,
    /// Latest matrix health from the last update.
    pub matrix_health: MatrixHealth,
}

impl Default for AhrsFeed {
    fn default() -> Self {
        Self {
            configured_backend: AhrsBackendKind::default(),
            active_backend: AhrsBackendKind::default(),
            ekf_healthy: false,
            ekf3: Ekf3Loop::default(),
            dcm: Dcm::new(),
            drift: DcmDriftLoop::default(),
            last_gps_fix_ms: 0,
            matrix_health: MatrixHealth::Ok,
        }
    }
}

/// Extract attitude sensors from the DCM matrix, upstream the sensor getters.
#[must_use]
pub fn attitude_from_dcm(dcm: &Dcm) -> AhrsAttitude {
    let (roll, pitch, yaw) = dcm.matrix.to_euler();
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream assigns float euler to int32_t sensors with truncation"
    )]
    AhrsAttitude {
        roll_sensor_cd: wrap_180_cd(rad_to_cd(roll) as i32),
        pitch_sensor_cd: wrap_180_cd(rad_to_cd(pitch) as i32),
        yaw_sensor_cd: wrap_360_cd(rad_to_cd(yaw) as i32),
    }
}

impl AhrsFeed {
    /// One AHRS update from INS samples, upstream `AP_AHRS_DCM::update`.
    /// Refresh active backend from configured type and EKF health.
    pub fn update_active_backend(&mut self) {
        self.active_backend = active_backend_kind(self.configured_backend, self.ekf_healthy);
    }

    /// Set configured backend from `AHRS_EKF_TYPE`, upstream `update_configured_ekf_type`.
    pub fn set_configured_backend(&mut self, kind: AhrsBackendKind) {
        self.configured_backend = kind;
        self.active_backend = backend_for_kind(kind);
    }

    /// One AHRS update from INS samples, upstream `AP_AHRS::update`.
    pub fn update_from_ins(
        &mut self,
        ins: &InertialSensorFrontend,
        timing: &LoopTiming,
        yaw: Option<YawUpdateInputs>,
        motion: DriftMotionInputs,
    ) -> (MatrixHealth, AhrsAttitude) {
        let result = match backend_for_kind(self.configured_backend) {
            AhrsBackendKind::Ekf3 => self.update_ekf3_from_ins(ins, timing, yaw, motion),
            AhrsBackendKind::Dcm => self.update_dcm_from_ins(ins, timing, yaw, motion),
        };
        self.ekf_healthy = self.ekf3.healthy;
        self.active_backend = active_backend_kind(self.configured_backend, self.ekf_healthy);
        self.matrix_health = result.0;
        result
    }

    fn update_dcm_from_ins(
        &mut self,
        ins: &InertialSensorFrontend,
        timing: &LoopTiming,
        yaw: Option<YawUpdateInputs>,
        motion: DriftMotionInputs,
    ) -> (MatrixHealth, AhrsAttitude) {
        let health = dcm_step_with_drift_from_ins_yaw(
            &mut self.dcm,
            &mut self.drift,
            ins,
            timing,
            yaw,
            motion,
        );
        (health, attitude_from_dcm(&self.dcm))
    }

    fn update_ekf3_from_ins(
        &mut self,
        ins: &InertialSensorFrontend,
        timing: &LoopTiming,
        yaw: Option<YawUpdateInputs>,
        motion: DriftMotionInputs,
    ) -> (MatrixHealth, AhrsAttitude) {
        let outcome = ekf3_full_update_from_ins(
            &mut self.ekf3,
            &mut self.dcm,
            &mut self.drift,
            ins,
            timing,
            yaw,
            motion,
        );
        (outcome.health, attitude_from_dcm(&self.dcm))
    }

    /// Estimated wind velocity in NED, upstream `AP_AHRS::wind_estimate`.
    #[must_use]
    pub fn wind_estimate(&self) -> Vector3f {
        self.drift.wind.wind
    }

    /// Head-wind along fuselage, upstream `AP_AHRS::head_wind`.
    #[must_use]
    pub fn head_wind(&self) -> f32 {
        let (_, _, yaw) = self.dcm.matrix.to_euler();
        head_wind_from_yaw(yaw, self.wind_estimate())
    }

    /// Wind alignment for a heading in degrees, upstream `AP_AHRS::wind_alignment`.
    #[must_use]
    pub fn wind_alignment(&self, heading_deg: f32) -> f32 {
        wind_alignment(heading_deg, self.wind_estimate())
    }

    /// NE offset from last GPS fix while dead-reckoning, upstream position offsets.
    #[must_use]
    /// Whether AHRS is healthy for arming and navigation, upstream `AP_AHRS::healthy()`.
    #[must_use]
    pub fn healthy(&self) -> bool {
        ahrs_healthy(self.matrix_health, self.ekf_healthy, self.active_backend)
    }

    /// NE offset from last GPS fix while dead-reckoning, upstream position offsets.
    #[must_use]
    pub fn dead_reckoning_offset(&self) -> (f32, f32, bool) {
        let p = &self.drift.position;
        (p.offset_north_m, p.offset_east_m, p.have_position)
    }

    /// Seed wind from a wind vane when sensor data is available.
    pub fn apply_wind_vane(&mut self, vane: WindVaneSample) {
        if vane.speed_true_mps > 0.0 {
            self.drift.wind.wind = vane.to_wind_ned();
        }
    }
}

/// Combined AHRS health for arming checks, upstream `AP_AHRS::healthy()`.
#[must_use]
pub fn ahrs_healthy(
    matrix_health: MatrixHealth,
    ekf_healthy: bool,
    active_backend: AhrsBackendKind,
) -> bool {
    if matrix_health != MatrixHealth::Ok {
        return false;
    }
    match active_backend {
        AhrsBackendKind::Ekf3 => ekf_healthy,
        AhrsBackendKind::Dcm => true,
    }
}

/// Build yaw correction inputs when compass, GPS, or context is available.
#[must_use]
pub fn yaw_update_inputs(
    compass: Option<YawCompassSample>,
    gps: Option<YawGpsSample>,
    ctx: YawDriftContext,
) -> Option<YawUpdateInputs> {
    if compass.is_none() && gps.is_none() && !ctx.have_gps && !ctx.compass_use_for_yaw {
        None
    } else {
        Some(YawUpdateInputs {
            compass,
            gps,
            ctx,
        })
    }
}

/// Build drift motion inputs from GPS course/speed and vehicle context.
#[must_use]
pub fn drift_motion_inputs(
    ctx: YawDriftContext,
    gps: Option<YawGpsSample>,
    airspeed_tas: f32,
    eas2tas: f32,
    last_gps_fix_ms: &mut u32,
) -> DriftMotionInputs {
    let new_gps_fix = gps
        .map(|sample| sample.last_fix_time_ms != *last_gps_fix_ms)
        .unwrap_or(false);
    if let Some(sample) = gps {
        if new_gps_fix {
            *last_gps_fix_ms = sample.last_fix_time_ms;
        }
    }

    let gps_velocity = gps.and_then(|sample| {
        if !ctx.have_gps {
            return None;
        }
        let course = radians(sample.ground_course_deg);
        Some(Vector3f::new(
            sample.ground_speed * Real::cos(course),
            sample.ground_speed * Real::sin(course),
            0.0,
        ))
    });

    DriftMotionInputs {
        now_ms: ctx.now_ms,
        gps_velocity,
        new_gps_fix,
        have_gps: ctx.have_gps,
        fly_forward: ctx.fly_forward,
        airspeed_tas,
        eas2tas,
        gps_lat_e7: ctx.gps_lat_e7,
        gps_lng_e7: ctx.gps_lng_e7,
        ..DriftMotionInputs::default()
    }
}

