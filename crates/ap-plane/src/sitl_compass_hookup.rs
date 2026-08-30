//! SITL compass producer wired into AHRS yaw drift, upstream `AP_Compass_SITL`.
//!
//! [`SitlCompassHookup`] runs the [`SitlCompassCluster`] timer/update path and
//! publishes body-frame mag samples and health before
//! [`PlaneMainLoop::ahrs_update`] builds [`YawUpdateInputs`](ap_ahrs::YawUpdateInputs).

use ap_ahrs::YawCompassSample;
use ap_compass::calibrate::CompassCalibrator;
use ap_compass::offset::learn_offsets_enabled;
use ap_compass::sitl::{
    CompassHealthFlags, MagSampleState, SitlCompassBackend, SitlCompassCluster,
    SITL_COMPASS_MAX_INSTANCES,
};
use ap_compass::{CompassDeclinationState, CompassParams, GpsDeclinationFix};
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;

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
    /// When set, rotate the earth mag field by this DCM (sim truth) instead of
    /// the AHRS estimate. C++ SitlHarness uses `sim_plane.dcm`.
    pub body_attitude_override: Option<Matrix3f>,
    calibrators: [CompassCalibrator; SITL_COMPASS_MAX_INSTANCES],
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
            body_attitude_override: None,
            calibrators: [CompassCalibrator::default(); SITL_COMPASS_MAX_INSTANCES],
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
            body_attitude_override: None,
            calibrators: [CompassCalibrator::default(); SITL_COMPASS_MAX_INSTANCES],
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

    /// Select `_first_usable` from `COMPASS_USE` / `USE2`, upstream `Compass::read`.
    pub fn select_first_usable(&mut self) -> u8 {
        let idx = self.params.first_usable();
        self.cluster.set_primary(idx);
        idx
    }

    /// Start MAG_CAL on every healthy `COMPASS_USE` instance.
    #[must_use]
    pub fn start_calibration_all(&mut self) -> bool {
        let n = self.cluster.instance_count() as usize;
        let mut healthy = [false; SITL_COMPASS_MAX_INSTANCES];
        let mut use_for_yaw = [false; SITL_COMPASS_MAX_INSTANCES];
        for i in 0..n {
            healthy[i] = self
                .cluster
                .backend(i as u8)
                .is_some_and(SitlCompassBackend::healthy);
            use_for_yaw[i] = if i == 0 {
                self.params.compass1.use_for_yaw
            } else {
                self.params.compass2.use_for_yaw
            };
        }
        ap_compass::calibrate::start_calibration_all(&mut self.calibrators[..n], &healthy[..n], &use_for_yaw[..n])
    }

    /// Cancel MAG_CAL on every instance, upstream `cancel_calibration_all`.
    pub fn cancel_calibration_all(&mut self) {
        ap_compass::calibrate::cancel_calibration_all(&mut self.calibrators);
    }

    /// Upstream `Compass::is_calibrating`.
    #[must_use]
    pub fn is_calibrating(&self) -> bool {
        ap_compass::calibrate::is_calibrating(&self.calibrators[..self.cluster.instance_count() as usize])
    }

    /// Run timer tick and publish mag sample + yaw drift input.
    #[must_use]
    pub fn publish(&mut self, attitude: Matrix3f, loop_dt: f32, gps: Option<GpsDeclinationFix>) -> SitlCompassPublish {
        let attitude = self.body_attitude_override.unwrap_or(attitude);
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
            calibrating: self.is_calibrating(),
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
        body_attitude_override: None,
        calibrators: [CompassCalibrator::default(); SITL_COMPASS_MAX_INSTANCES],
    }
}
