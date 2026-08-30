//! SITL GPS fix producer wired into yaw publish, upstream `AP_GPS_SITL` → yaw drift.
//!
//! [`SitlGpsHookup`] runs the [`SitlGpsBackend`] read path and fills
//! [`SitlYawPublish`] GPS fields before compass/GPS samples reach the DCM.

use ap_gps::{
    GpsAutoSwitch, GpsDualHealthFlags, GpsDualStub, GpsHealthFlags, GpsInstanceTruth,
    GpsParams, GpsStatus, GpsVelocityProducer, GpsVelocitySample, SitlGpsBackend, GPS_MIN_NSATS,
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
    min_nsats: u8,
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
            min_nsats: GPS_MIN_NSATS,
        }
    }
}

impl SitlGpsHookup {
    #[must_use]
    pub const fn gps_lag_sec(&self) -> f32 {
        self.backend.lag_sec()
    }

    /// C++ SitlHarness GPS is 200 ms rate-limited with no lag buffer.
    /// Upstream `AP_GPS_SITL` default lag is 0.1 s; sitl_run zeros it so
    /// DCM `new_gps_fix` lines up with each 5 Hz read the way C++ does.
    pub fn set_lag_sec(&mut self, lag_sec: f32) {
        self.backend.set_lag_sec(lag_sec);
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

    /// Alias for param-table init, upstream var_info-driven frontend setup.
    pub fn configure_from_params(&mut self, params: GpsParams) {
        self.apply_gps_params(params);
    }

    pub fn apply_gps_params(&mut self, params: GpsParams) {
        self.min_nsats = params.min_nsats;
        params.apply_instance(0, &mut self.backend);
        if params.dual_enabled() {
            let mut dual = params
                .configure_dual_stub()
                .unwrap_or_else(GpsDualStub::default);
            dual.primary_truth = GpsInstanceTruth {
                velocity_ned: self.truth.velocity_ned,
                latitude_deg: self.truth.latitude_deg,
                longitude_deg: self.truth.longitude_deg,
                altitude_m: self.truth.altitude_m,
                now_ms: self.truth.now_ms,
            };
            self.dual = Some(dual);
        } else {
            self.dual = None;
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

    #[must_use]
    pub fn gps_dual_health_publish(&mut self) -> GpsDualHealthFlags {
        self.sync_dual_truth();
        if let Some(dual) = self.dual.as_mut() {
            return dual.dual_health_flags_at(self.truth.now_ms);
        }
        let health = self.gps_health_publish();
        GpsDualHealthFlags {
            per_instance: [health, GpsHealthFlags::default()],
            instance_count: 1,
            primary: 0,
            have_gps_yaw: [false, false],
            rtk_yaw_fresh: true,
        }
    }

    /// GPS health flags for arming and drift gating, upstream `isHealthy()`.
    #[must_use]
    pub fn gps_health_publish(&mut self) -> GpsHealthFlags {
        self.sync_dual_truth();
        let now_ms = self.truth.now_ms;
        if let Some(dual) = self.dual.as_mut() {
            return dual.output_health_at(now_ms);
        }
        let fix = self.backend.delayed_state(now_ms);
        if !fix.have_fix {
            let _ = self.backend.read(
                self.truth.velocity_ned,
                self.truth.latitude_deg,
                self.truth.longitude_deg,
                self.truth.altitude_m,
                now_ms,
            );
        }
        let fix = self.backend.delayed_state(now_ms);
        let status = GpsStatus::from_fix(&fix, self.gps_lag_sec());
        GpsHealthFlags::from_status_at_min(&status, now_ms, self.min_nsats)
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
        let now_ms = self.truth.now_ms;
        if let Some(dual) = self.dual.as_mut() {
            let health_ok = if !dual.dual_enabled {
                dual.output_health_at(now_ms).is_healthy()
            } else {
                match dual.auto_switch {
                    GpsAutoSwitch::Blend => dual.output_health_at(now_ms).is_healthy(),
                    GpsAutoSwitch::UsePrimary | GpsAutoSwitch::UseBest => {
                        dual.output_health_at(now_ms).is_healthy()
                    }
                }
            };
            health_ok && dual.dual_health_flags_at(now_ms).rtk_yaw_fresh
        } else {
            self.gps_health_publish().is_healthy()
        }
    }

    fn gps_yaw_from_dual(dual: &mut GpsDualStub) -> (Option<f32>, Option<f32>, Option<u32>) {
        let mb = dual.moving_baseline();
        let inst = dual.output_active_instance();
        let now_ms = if inst == 0 {
            dual.primary_truth.now_ms
        } else {
            dual.secondary_truth.now_ms
        };
        mb.gps_yaw_deg(inst, now_ms)
            .map(|(yaw, acc, t)| (Some(yaw), Some(acc), Some(t)))
            .unwrap_or((None, None, None))
    }

    #[must_use]
    pub fn yaw_publish(&mut self) -> SitlYawPublish {
        self.sync_dual_truth();
        if let Some(dual) = self.dual.as_mut() {
            let status = dual.output_status();
            let (gps_yaw_deg, gps_yaw_accuracy_deg, gps_yaw_time_ms) =
                Self::gps_yaw_from_dual(dual);
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
                gps_yaw_deg,
                gps_yaw_accuracy_deg,
                gps_yaw_time_ms,
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
            gps_yaw_deg: None,
            gps_yaw_accuracy_deg: None,
            gps_yaw_time_ms: None,
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
        min_nsats: GPS_MIN_NSATS,
    }
}
