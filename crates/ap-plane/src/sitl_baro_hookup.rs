//! SITL barometer producer wired into AHRS drift motion, upstream `AP_Baro_SITL` → `get_EAS2TAS()`.
//!
//! [`SitlBaroHookup`] runs the [`SitlBaroBackend`] timer/update path and publishes
//! pressure, altitude, and EAS2TAS before [`PlaneMainLoop::ahrs_update`] builds
//! [`DriftMotionInputs`](ap_ahrs::DriftMotionInputs).

use ap_baro::eas2tas_for_alt_amsl;
use ap_baro::sitl::{BaroSampleState, SitlBaroBackend};
use ap_math::vector3::Vector3f;

/// Sim truth fed into the SITL baro backend each tick.
#[derive(Debug, Clone, Copy)]
pub struct SitlBaroTruth {
    pub sim_altitude_m: f32,
    pub airspeed_bf: Vector3f,
    pub now_ms: u32,
    pub noise_sample: f32,
}

impl Default for SitlBaroTruth {
    fn default() -> Self {
        Self {
            sim_altitude_m: 0.0,
            airspeed_bf: Vector3f::zero(),
            now_ms: 0,
            noise_sample: 0.0,
        }
    }
}

/// SITL baro backend hookup for the vehicle main loop.
#[derive(Debug, Clone)]
pub struct SitlBaroHookup {
    backend: SitlBaroBackend,
    pub truth: SitlBaroTruth,
}

impl Default for SitlBaroHookup {
    fn default() -> Self {
        Self {
            backend: SitlBaroBackend::default(),
            truth: SitlBaroTruth::default(),
        }
    }
}

/// Baro sample and EAS2TAS published before `ahrs_update`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SitlBaroPublish {
    pub sample: BaroSampleState,
    pub eas2tas: f32,
    pub healthy: bool,
}

impl SitlBaroHookup {
    #[must_use]
    pub const fn backend(&self) -> &SitlBaroBackend {
        &self.backend
    }

    /// Run timer tick and frontend update, upstream `AP_Baro_SITL::_timer` + `update`.
    #[must_use]
    pub fn publish(&mut self) -> SitlBaroPublish {
        let _ = self.backend.timer_tick(
            self.truth.sim_altitude_m,
            self.truth.airspeed_bf,
            self.truth.now_ms,
            self.truth.noise_sample,
        );
        let sample = self.backend.update().unwrap_or(*self.backend.state());
        let eas2tas = if sample.have_sample {
            eas2tas_for_alt_amsl(sample.altitude_m)
        } else {
            1.0
        };
        SitlBaroPublish {
            sample,
            eas2tas,
            healthy: self.backend.healthy(),
        }
    }
}
