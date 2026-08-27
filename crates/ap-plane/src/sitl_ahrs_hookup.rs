//! SITL AHRS publish extension: airspeed TAS and EAS2TAS into drift motion inputs.
//!
//! Builds on [`sitl_yaw_hookup`] so one SITL source feeds compass/GPS yaw
//! samples, true airspeed, and baro EAS2TAS before [`PlaneMainLoop::ahrs_update`]
//! builds [`DriftMotionInputs`](ap_ahrs::DriftMotionInputs).

use ap_math::matrix3::Matrix3f;

use crate::sitl_yaw_hookup::{publish_sitl_yaw_samples, SitlYawPublish, SitlYawSamples};

/// SITL vehicle context for one AHRS publish cycle.
#[derive(Debug, Clone, Copy)]
pub struct SitlAhrsPublish {
    /// Compass/GPS yaw publish source.
    pub yaw: SitlYawPublish,
    /// True airspeed, m/s. Upstream `AP_AHRS::airspeed` / pitot in SITL.
    pub airspeed_tas_mps: f32,
    /// EAS to TAS scale, upstream `AP_Baro::get_EAS2TAS()`.
    pub eas2tas: f32,
}

impl Default for SitlAhrsPublish {
    fn default() -> Self {
        Self {
            yaw: SitlYawPublish::default(),
            airspeed_tas_mps: 0.0,
            eas2tas: 1.0,
        }
    }
}

impl From<SitlYawPublish> for SitlAhrsPublish {
    fn from(yaw: SitlYawPublish) -> Self {
        Self {
            yaw,
            airspeed_tas_mps: 0.0,
            eas2tas: 1.0,
        }
    }
}

/// Yaw samples plus airspeed and EAS2TAS published before `ahrs_update`.
#[derive(Debug, Clone, Copy)]
pub struct SitlAhrsSamples {
    pub yaw: SitlYawSamples,
    pub airspeed_tas: f32,
    pub eas2tas: f32,
}

/// Publish SITL yaw, airspeed, and EAS2TAS samples for one AHRS cycle.
#[must_use]
pub fn publish_sitl_ahrs_samples(
    source: &SitlAhrsPublish,
    attitude: Matrix3f,
    loop_dt: f32,
) -> SitlAhrsSamples {
    SitlAhrsSamples {
        yaw: publish_sitl_yaw_samples(&source.yaw, attitude, loop_dt),
        airspeed_tas: source.airspeed_tas_mps,
        eas2tas: source.eas2tas,
    }
}
