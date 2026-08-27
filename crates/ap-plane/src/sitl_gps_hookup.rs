//! SITL GPS fix producer wired into yaw publish, upstream `AP_GPS_SITL` → yaw drift.
//!
//! [`SitlGpsHookup`] runs the [`SitlGpsBackend`] read path and fills
//! [`SitlYawPublish`] GPS fields before compass/GPS samples reach the DCM.

use ap_gps::SitlGpsBackend;
use ap_math::vector3::Vector3f;

use crate::sitl_yaw_hookup::{publish_sitl_yaw_samples, SitlYawPublish, SitlYawSamples};

/// SITL ground truth for one GPS update cycle.
#[derive(Debug, Clone, Copy)]
pub struct SitlGpsTruth {
    /// NED velocity, upstream `sitl->state.speedN/E/D`.
    pub velocity_ned: Vector3f,
    /// Latitude, degrees.
    pub latitude_deg: f32,
    /// Longitude, degrees.
    pub longitude_deg: f32,
    /// Altitude AMSL, metres.
    pub altitude_m: f32,
    /// Monotonic time, ms. Upstream `AP_HAL::millis()`.
    pub now_ms: u32,
}

impl Default for SitlGpsTruth {
    fn default() -> Self {
        Self {
            velocity_ned: Vector3f::zero(),
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            altitude_m: 0.0,
            now_ms: 0,
        }
    }
}

/// GPS backend plus yaw/compass context for one SITL vehicle.
#[derive(Debug, Clone, Copy)]
pub struct SitlGpsHookup {
    backend: SitlGpsBackend,
    pub truth: SitlGpsTruth,
    pub fly_forward: bool,
    pub compass_use_for_yaw: bool,
    pub wind_speed_xy: f32,
}

impl Default for SitlGpsHookup {
    fn default() -> Self {
        Self {
            backend: SitlGpsBackend::default(),
            truth: SitlGpsTruth::default(),
            fly_forward: true,
            compass_use_for_yaw: true,
            wind_speed_xy: 0.0,
        }
    }
}

impl SitlGpsHookup {
    /// GPS lag in seconds for drift consumers, upstream `AP_GPS::get_lag()`.
    #[must_use]
    pub const fn gps_lag_sec(&self) -> f32 {
        self.backend.lag_sec()
    }

    /// Run the GPS backend read and build yaw publish fields.
    #[must_use]
    pub fn yaw_publish(&mut self) -> SitlYawPublish {
        self.backend.read(
            self.truth.velocity_ned,
            self.truth.latitude_deg,
            self.truth.longitude_deg,
            self.truth.altitude_m,
            self.truth.now_ms,
        );
        let fix = self.backend.state();
        SitlYawPublish {
            latitude_deg: fix.latitude_deg,
            longitude_deg: fix.longitude_deg,
            ground_speed_mps: fix.ground_speed,
            ground_course_deg: fix.ground_course_deg,
            last_fix_time_ms: fix.last_fix_time_ms,
            now_ms: self.truth.now_ms,
            fly_forward: self.fly_forward,
            compass_use_for_yaw: self.compass_use_for_yaw,
            wind_speed_xy: self.wind_speed_xy,
            have_gps: fix.have_fix,
        }
    }

    /// Publish compass/GPS yaw samples after running the fix producer.
    #[must_use]
    pub fn publish_yaw_samples(
        &mut self,
        attitude: ap_math::matrix3::Matrix3f,
        loop_dt: f32,
    ) -> SitlYawSamples {
        let yaw = self.yaw_publish();
        publish_sitl_yaw_samples(&yaw, attitude, loop_dt)
    }
}
