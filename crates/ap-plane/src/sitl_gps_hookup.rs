//! SITL GPS fix producer wired into yaw publish, upstream `AP_GPS_SITL` → yaw drift.
//!
//! [`SitlGpsHookup`] runs the [`SitlGpsBackend`] read path and fills
//! [`SitlYawPublish`] GPS fields before compass/GPS samples reach the DCM.

use ap_gps::{
    GpsAutoSwitch, GpsDualStub, GpsHealthFlags, GpsInstanceTruth, GpsStatus,
    GpsVelocityProducer, GpsVelocitySample, SitlGpsBackend,
};
use ap_math::vector3::Vector3f;

use crate::sitl_yaw_hookup::{publish_sitl_yaw_samples, SitlYawPublish, SitlYawSamples};

#[derive(Debug, Clone, Copy)]
pub struct SitlGpsTruth {
    pub velocity_ned: Vector3f,
    pub latitude_deg: f32,
    pub longitude_deg: f32,
    pub altitude_m: f32,
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

#[derive(Debug, Clone, Copy)]
pub struct SitlGpsHookup {
    backend: SitlGpsBackend,
    pub dual: Option<GpsDualStub>,
    pub truth: SitlGpsTruth,
    pub fly_forward: bool,
    pub compass_use_for_yaw: bool,
    pub wind_speed_xy: f32,
}

impl Default for SitlGpsHookup {
    fn default() -> Self {
        Self {
            backend: SitlGpsBackend::default(),
            dual: None,
            truth: SitlGpsTruth::default(),
            fly_forward: true,
            compass_use_for_yaw: true,
            wind_speed_xy: 0.0,
        }
    }
}

impl SitlGpsHookup {
    #[must_use]
    pub const fn gps_lag_sec(&self) -> f32 {
        self.backend.lag_sec()
    }

    #[must_use]
    pub fn current_fix(&self) -> ap_gps::GpsFixState {
        *self.backend.state()
    }

    #[must_use]
    pub fn delayed_fix(&self) -> ap_gps::GpsFixState {
        self.backend.delayed_state(self.truth.now_ms)
    }

    fn sync_dual_truth(&mut self) {
        if let Some(dual) = self.dual.as_mut() {
            dual.primary_truth = GpsInstanceTruth {
                velocity_ned: self.truth.velocity_ned,
                latitude_deg: self.truth.latitude_deg,
                longitude_deg: self.truth.longitude_deg,
                altitude_m: self.truth.altitude_m,
                now_ms: self.truth.now_ms,
            };
            dual.select_primary_healthy();
        }
    }

    /// Enable dual-GPS stub with blending, upstream `GPS2_TYPE` + `GPS_AUTO_SWITCH`.
    pub fn enable_dual_gps(&mut self, auto_switch: GpsAutoSwitch) {
        let mut dual = GpsDualStub::default();
        dual.dual_enabled = true;
        dual.auto_switch = auto_switch;
        self.dual = Some(dual);
        self.sync_dual_truth();
    }

    /// Lag-buffered GPS status for vehicle consumers, upstream `AP_GPS::status()`.
    #[must_use]
    pub fn gps_status_publish(&mut self) -> GpsStatus {
        self.sync_dual_truth();
        if let Some(dual) = self.dual.as_mut() {
            return dual.output_status();
        }
        self.backend.read(
            self.truth.velocity_ned,
            self.truth.latitude_deg,
            self.truth.longitude_deg,
            self.truth.altitude_m,
            self.truth.now_ms,
        );
        let fix = self.backend.delayed_state(self.truth.now_ms);
        GpsStatus::from_fix(&fix, self.gps_lag_sec())
    }

    /// Lag-buffered NED velocity for AHRS drift, upstream `state.velocity`.
    #[must_use]
    pub fn gps_velocity_publish(&mut self) -> GpsVelocitySample {
        self.sync_dual_truth();
        if let Some(dual) = self.dual.as_mut() {
            return dual.output_velocity();
        }
        let status = self.gps_status_publish();
        GpsVelocityProducer::publish_status(&status)
    }

    /// GPS health flags for arming and drift gating, upstream `isHealthy()`.
    #[must_use]
    pub fn gps_health_publish(&mut self) -> GpsHealthFlags {
        self.sync_dual_truth();
        if let Some(dual) = self.dual.as_mut() {
            return dual.output_health();
        }
        let status = self.gps_status_publish();
        GpsHealthFlags::from_status(&status)
    }

    /// Whether the active GPS output is the blended virtual instance.
    #[must_use]
    pub fn gps_output_is_blended(&self) -> bool {
        self.dual.is_some_and(|dual| dual.output_is_blended())
    }

    /// Active GPS instance index (0/1/blended=2), upstream `primary_instance()`.
    #[must_use]
    pub fn gps_active_instance(&mut self) -> u8 {
        self.sync_dual_truth();
        if let Some(dual) = self.dual.as_mut() {
            return dual.output_active_instance();
        }
        0
    }

    /// Dual-GPS pre-arm gate, upstream `AP_GPS::pre_arm_checks`.
    ///
    /// Blend requires both instances healthy; UsePrimary/UseBest follow the active
    /// failover output so arming succeeds when the selected receiver is healthy.
    #[must_use]
    pub fn gps_dual_pre_arm_ok(&mut self) -> bool {
        self.sync_dual_truth();
        if let Some(dual) = self.dual.as_mut() {
            if !dual.dual_enabled {
                return dual.output_health().is_healthy();
            }
            match dual.auto_switch {
                GpsAutoSwitch::Blend => {
                    let primary = dual.instance_status(0);
                    let secondary = dual.instance_status(1);
                    GpsHealthFlags::from_status(&primary).is_healthy()
                        && GpsHealthFlags::from_status(&secondary).is_healthy()
                }
                GpsAutoSwitch::UsePrimary | GpsAutoSwitch::UseBest => {
                    dual.output_health().is_healthy()
                }
            }
        } else {
            self.gps_health_publish().is_healthy()
        }
    }

    #[must_use]
    pub fn yaw_publish(&mut self) -> SitlYawPublish {
        self.sync_dual_truth();
        if let Some(dual) = self.dual.as_mut() {
            let status = dual.output_status();
            return SitlYawPublish {
                latitude_deg: status.latitude_deg,
                longitude_deg: status.longitude_deg,
                ground_speed_mps: status.ground_speed,
                ground_course_deg: status.ground_course_deg,
                last_fix_time_ms: status.last_fix_time_ms,
                now_ms: self.truth.now_ms,
                fly_forward: self.fly_forward,
                compass_use_for_yaw: self.compass_use_for_yaw,
                wind_speed_xy: self.wind_speed_xy,
                have_gps: status.have_fix,
            };
        }
        self.backend.read(
            self.truth.velocity_ned,
            self.truth.latitude_deg,
            self.truth.longitude_deg,
            self.truth.altitude_m,
            self.truth.now_ms,
        );
        let fix = self.backend.delayed_state(self.truth.now_ms);
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

/// Hookup with disabled primary GPS for UsePrimary failover tests.
#[must_use]
pub fn hookup_with_disabled_primary() -> SitlGpsHookup {
    SitlGpsHookup {
        backend: SitlGpsBackend::default(),
        dual: Some(GpsDualStub::with_disabled_primary()),
        truth: SitlGpsTruth::default(),
        fly_forward: true,
        compass_use_for_yaw: true,
        wind_speed_xy: 0.0,
    }
}
