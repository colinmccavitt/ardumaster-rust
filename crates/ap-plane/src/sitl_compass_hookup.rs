//! SITL compass producer wired into AHRS yaw drift, upstream `AP_Compass_SITL`.
//!
//! [`SitlCompassHookup`] runs the [`SitlCompassCluster`] timer/update path and
//! publishes body-frame mag samples and health before
//! [`PlaneMainLoop::ahrs_update`] builds [`YawUpdateInputs`](ap_ahrs::YawUpdateInputs).

use ap_ahrs::YawCompassSample;
use ap_compass::offset::learn_offsets_enabled;
use ap_compass::{CompassDeclinationState, CompassParams, GpsDeclinationFix};
use ap_math::vector3::Vector3f;
use ap_compass::sitl::{
    CompassHealthFlags, MagSampleState, SitlCompassBackend, SitlCompassCluster,
};
use ap_math::matrix3::Matrix3f;

/// Sim truth fed into the SITL compass backend each tick.
#[derive(Debug, Clone, Copy)]
pub struct SitlCompassTruth {
    pub latitude_deg: f32,
    pub longitude_deg: f32,
    pub now_ms: u32,
}

impl Default for SitlCompassTruth {
    fn default() -> Self {
        Self {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 0,
        }
    }
}

/// SITL compass cluster hookup for the vehicle main loop.
#[derive(Debug, Clone)]
pub struct SitlCompassHookup {
    cluster: SitlCompassCluster,
    params: CompassParams,
    declination: CompassDeclinationState,
    pub truth: SitlCompassTruth,
    pub compass_use_for_yaw: bool,
    /// Battery current in amps (or throttle 0..1), upstream motor compensation.
    pub battery_current_amps: f32,
}

impl Default for SitlCompassHookup {
    fn default() -> Self {
        Self {
            cluster: SitlCompassCluster::default(),
            params: CompassParams::default(),
            declination: CompassDeclinationState::default(),
            truth: SitlCompassTruth::default(),
            compass_use_for_yaw: true,
            battery_current_amps: 0.0,
        }
    }
}

/// Mag sample and health published before `ahrs_update`.
#[derive(Debug, Clone, Copy, Default)]
pub struct SitlCompassPublish {
    pub sample: MagSampleState,
    pub healthy: bool,
    pub health: CompassHealthFlags,
    pub yaw_compass: Option<YawCompassSample>,
}

impl SitlCompassHookup {
    /// One primary plus one secondary SITL compass backend.
    #[must_use]
    pub fn with_dual_backends() -> Self {
        let mut cluster = SitlCompassCluster::default();
        let _ = cluster.register(SitlCompassBackend::default());
        Self {
            cluster,
            params: CompassParams::default(),
            declination: CompassDeclinationState::default(),
            truth: SitlCompassTruth::default(),
            compass_use_for_yaw: true,
            battery_current_amps: 0.0,
        }
    }

    #[must_use]
    pub const fn compass_params(&self) -> &CompassParams {
        &self.params
    }

    pub fn apply_compass_params(&mut self, params: CompassParams) {
        self.params = params;
        params.apply_to_cluster(&mut self.cluster);
        self.compass_use_for_yaw = params.primary_use_for_yaw();
    }

    /// Inject the same SITL hard-iron bias on every registered instance.
    pub fn set_hardiron_bias(&mut self, bias: Vector3f) {
        for i in 0..self.cluster.instance_count() {
            if let Some(backend) = self.cluster.backend_mut(i) {
                let mut cfg = *backend.config();
                cfg.hardiron_bias = bias;
                backend.set_config(cfg);
            }
        }
    }

    /// Latch throttle/current for `COMPASS_MOT` on every instance.
    pub fn set_thr_or_curr(&mut self, value: f32) {
        self.battery_current_amps = value;
        self.cluster.set_thr_or_curr(value);
    }

    /// Latch `COMPASS_OFS` on every enabled instance when learn is enabled.
    #[must_use]
    pub fn learn_offsets(&mut self) -> bool {
        if !learn_offsets_enabled(self.params.learn) {
            return false;
        }
        self.cluster.learn_offsets(self.params.offsets_max)
    }

    /// Persist backend `COMPASS_OFS` into params, upstream `Compass::save_offsets`.
    #[must_use]
    pub fn save_offsets(&mut self) -> bool {
        ap_compass::persist::save_offsets(&mut self.params, &self.cluster)
    }

    #[must_use]
    pub const fn cluster(&self) -> &SitlCompassCluster {
        &self.cluster
    }

    #[must_use]
    pub fn backend(&self) -> Option<&SitlCompassBackend> {
        self.cluster.backend(self.cluster.primary())
    }

    /// Run timer tick and publish mag sample + yaw drift input.
    #[must_use]
    pub fn publish(&mut self, attitude: Matrix3f, loop_dt: f32, gps: Option<GpsDeclinationFix>) -> SitlCompassPublish {
        self.declination.try_set_initial_location(&self.params, gps, true);
        self.cluster.set_thr_or_curr(self.battery_current_amps);
        self.cluster.timer_tick_all(
            self.truth.latitude_deg,
            self.truth.longitude_deg,
            attitude,
            self.truth.now_ms,
        );
        self.cluster.select_primary_healthy();
        let health = self.cluster.health_flags();
        let sample = self
            .cluster
            .primary_sample()
            .unwrap_or(*self.cluster.backend(self.cluster.primary()).unwrap().state());
        let healthy = health.primary_healthy();
        let yaw_compass = (healthy && self.compass_use_for_yaw).then_some(YawCompassSample {
            mag_body: sample.mag_body,
            declination_rad: self.declination.effective_declination_rad(&self.params),
            update_interval_s: Some(loop_dt),
            calibrating: false,
        });
        SitlCompassPublish {
            sample,
            healthy,
            health,
            yaw_compass,
        }
    }
}

/// Mark primary instance unhealthy when disabled, for dual-compass tests.
#[must_use]
pub fn hookup_with_disabled_primary() -> SitlCompassHookup {
    SitlCompassHookup {
        cluster: SitlCompassCluster::cluster_with_disabled_primary(),
        params: CompassParams::default(),
        declination: CompassDeclinationState::default(),
        truth: SitlCompassTruth::default(),
        compass_use_for_yaw: true,
        battery_current_amps: 0.0,
    }
}
