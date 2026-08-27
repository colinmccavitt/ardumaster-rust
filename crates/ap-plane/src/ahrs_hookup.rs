//! AHRS attitude feed for the main vehicle loop, upstream `Plane::ahrs_update`.
//!
//! The scheduler calls [`PlaneMainLoop::ahrs_update`] every fast tick; this
//! module owns the DCM state and publishes roll/pitch/yaw sensors the stabilize
//! path reads on the next tasks.

use ap_ahrs::{
    dcm_step_with_drift_from_ins_yaw, Dcm, DcmDriftLoop, MatrixHealth, YawCompassSample,
    YawDriftContext, YawGpsSample, YawUpdateInputs,
};
use ap_ins::{InertialSensorFrontend, LoopTiming};
use ap_math::scalar::{rad_to_cd, wrap_180_cd, wrap_360_cd};

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

/// Running DCM estimator state the vehicle loop owns.
#[derive(Debug, Clone)]
pub struct AhrsFeed {
    /// Direction-cosine attitude estimate.
    pub dcm: Dcm,
    /// Roll/pitch drift and compass yaw correction.
    pub drift: DcmDriftLoop,
}

impl Default for AhrsFeed {
    fn default() -> Self {
        Self {
            dcm: Dcm::new(),
            drift: DcmDriftLoop::default(),
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
    pub fn update_from_ins(
        &mut self,
        ins: &InertialSensorFrontend,
        timing: &LoopTiming,
        yaw: Option<YawUpdateInputs>,
    ) -> (MatrixHealth, AhrsAttitude) {
        let health = dcm_step_with_drift_from_ins_yaw(
            &mut self.dcm,
            &mut self.drift,
            ins,
            timing,
            yaw,
        );
        (health, attitude_from_dcm(&self.dcm))
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
