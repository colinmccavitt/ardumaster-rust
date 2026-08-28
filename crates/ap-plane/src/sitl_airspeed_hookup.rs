//! SITL airspeed producer wired into AHRS drift motion and TECS, upstream `AP_Airspeed_SITL`.
//!
//! [`SitlAirspeedHookup`] runs the [`SitlAirspeedCluster`] timer/update path and
//! publishes pitot TAS/EAS samples and health before
//! [`PlaneMainLoop::ahrs_update`] builds [`DriftMotionInputs`](ap_ahrs::DriftMotionInputs).

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::sitl::{
    AirspeedHealthFlags, AirspeedSampleState, SitlAirspeedBackend, SitlAirspeedCluster,
    ARSPD_AUTOCAL_DEFAULT, ARSPD_RATIO_DEFAULT, ARSPD_SKIP_CAL_DEFAULT, ARSPD_TEMP_REF_C,
};
use ap_math::vector3::Vector3f;

/// Sim truth fed into the SITL airspeed backend each tick.
#[derive(Debug, Clone, Copy)]
pub struct SitlAirspeedTruth {
    pub airspeed_bf: Vector3f,
    pub now_ms: u32,
}

impl Default for SitlAirspeedTruth {
    fn default() -> Self {
        Self {
            airspeed_bf: Vector3f::zero(),
            now_ms: 0,
        }
    }
}

/// SITL airspeed cluster hookup for the vehicle main loop.
#[derive(Debug, Clone)]
pub struct SitlAirspeedHookup {
    cluster: SitlAirspeedCluster,
    params: AirspeedParams,
    pub truth: SitlAirspeedTruth,
    /// Horizontal GPS groundspeed (m/s) for `ARSPD_AUTOCAL`.
    pub gps_groundspeed_mps: f32,
}

impl Default for SitlAirspeedHookup {
    fn default() -> Self {
        Self {
            cluster: SitlAirspeedCluster::default(),
            params: AirspeedParams::default(),
            truth: SitlAirspeedTruth::default(),
            gps_groundspeed_mps: 0.0,
        }
    }
}

/// Pitot sample and health published before `ahrs_update`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SitlAirspeedPublish {
    pub sample: AirspeedSampleState,
    pub healthy: bool,
    pub health: AirspeedHealthFlags,
    /// Primary instance pitot ratio, upstream `ARSPD_RATIO`.
    pub ratio: f32,
    /// Whether TAS is used for TECS/nav, upstream `ARSPD_USE`.
    pub use_airspeed: bool,
    /// Primary instance temperature (deg C), upstream `get_temperature()`.
    pub temperature_c: f32,
    /// Automatic pitot-ratio calibration, upstream `ARSPD_AUTOCAL`.
    pub autocal: u8,
    /// Skip startup / requested offset calibration, upstream ARSPD_SKIP_CAL.
    pub skip_cal: bool,
    /// Pitot connector order, upstream `ARSPD_TUBE_ORDER`.
    pub tube_order: u8,
    /// I2C bus, upstream `ARSPD_BUS`.
    pub bus: u8,
    /// Sensor device ID, upstream `ARSPD_DEVID`.
    pub devid: i32,
}

impl SitlAirspeedHookup {
    /// One primary plus one secondary SITL airspeed backend.
    #[must_use]
    pub fn with_dual_backends() -> Self {
        let mut cluster = SitlAirspeedCluster::default();
        let _ = cluster.register(SitlAirspeedBackend::default());
        Self {
            cluster,
            params: AirspeedParams::default(),
            truth: SitlAirspeedTruth::default(),
            gps_groundspeed_mps: 0.0,
        }
    }

    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
        params.apply_to_cluster(&mut self.cluster);
    }

    /// Set `ARSPD_RATIO` on every enabled instance.
    pub fn set_pitot_ratio(&mut self, ratio: f32) {
        let mut params = self.params;
        params.airspeed1.ratio = ratio;
        params.airspeed2.ratio = ratio;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_USE` on every enabled instance.
    pub fn set_use_airspeed(&mut self, use_airspeed: u8) {
        let mut params = self.params;
        params.airspeed1.use_airspeed = use_airspeed;
        params.airspeed2.use_airspeed = use_airspeed;
        self.apply_airspeed_params(params);
    }

    /// Set temperature-compensation coefficient on every enabled instance.
    pub fn set_temp_coeff(&mut self, temp_coeff: f32) {
        let mut params = self.params;
        params.airspeed1.temp_coeff = temp_coeff;
        params.airspeed2.temp_coeff = temp_coeff;
        self.apply_airspeed_params(params);
    }

    /// Set sensor / ISA temperature (deg C) on every enabled instance.
    pub fn set_temperature_c(&mut self, temperature_c: f32) {
        let mut params = self.params;
        params.airspeed1.temperature_c = temperature_c;
        params.airspeed2.temperature_c = temperature_c;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_AUTOCAL` on every enabled instance.
    pub fn set_autocal(&mut self, autocal: u8) {
        let mut params = self.params;
        params.airspeed1.autocal = autocal;
        params.airspeed2.autocal = autocal;
        self.apply_airspeed_params(params);
    }

    /// Set ARSPD_SKIP_CAL on every enabled instance.
    pub fn set_skip_cal(&mut self, skip_cal: bool) {
        let mut params = self.params;
        params.airspeed1.skip_cal = skip_cal;
        params.airspeed2.skip_cal = skip_cal;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_TYPE` on every enabled instance.
    pub fn set_sensor_type(&mut self, sensor_type: u8) {
        let mut params = self.params;
        params.airspeed1.sensor_type = sensor_type;
        params.airspeed2.sensor_type = sensor_type;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_TUBE_ORDER` on every enabled instance.
    pub fn set_tube_order(&mut self, tube_order: u8) {
        let mut params = self.params;
        params.airspeed1.tube_order = tube_order;
        params.airspeed2.tube_order = tube_order;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_BUS` on every enabled instance.
    pub fn set_bus(&mut self, bus: u8) {
        let mut params = self.params;
        params.airspeed1.bus = bus;
        params.airspeed2.bus = bus;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_DEVID` on every enabled instance.
    pub fn set_devid(&mut self, devid: i32) {
        let mut params = self.params;
        params.airspeed1.devid = devid;
        params.airspeed2.devid = devid;
        self.apply_airspeed_params(params);
    }

    /// Latch or clear DEVID after a backend probe, upstream `set_bus_id`.
    pub fn assign_devid_from_probe(&mut self, found: bool) {
        let devid = ap_airspeed::devid::devid_after_probe(
            found,
            self.params.primary_sensor_type(),
            self.params.primary_bus(),
            self.params.primary,
        );
        self.set_devid(devid);
    }

    #[must_use]
    pub const fn cluster(&self) -> &SitlAirspeedCluster {
        &self.cluster
    }

    #[must_use]
    pub fn backend(&self) -> Option<&SitlAirspeedBackend> {
        self.cluster.backend(self.cluster.primary())
    }

    /// Latch pitot offsets on every enabled instance, upstream `calibrate()`.
    #[must_use]
    pub fn calibrate_offsets(&mut self) -> bool {
        self.cluster.calibrate_offsets()
    }

    /// Run timer tick and publish pitot TAS/EAS + health.
    #[must_use]
    pub fn publish(&mut self, eas2tas: f32) -> SitlAirspeedPublish {
        self.cluster.timer_tick_all(self.truth.airspeed_bf, eas2tas, self.truth.now_ms);
        self.cluster.select_primary_healthy();
        self.cluster.update_autocal_all(self.gps_groundspeed_mps);
        let health = self.cluster.health_flags();
        let sample = self
            .cluster
            .primary_sample()
            .unwrap_or(*self.cluster.backend(self.cluster.primary()).unwrap().state());
        let healthy = health.primary_healthy();
        let ratio = self
            .cluster
            .backend(self.cluster.primary())
            .map(|backend| backend.config().ratio)
            .unwrap_or(ARSPD_RATIO_DEFAULT);
        SitlAirspeedPublish {
            sample,
            healthy,
            health,
            ratio,
            use_airspeed: self.cluster.primary_use_for_control(),
            temperature_c: self
                .cluster
                .backend(self.cluster.primary())
                .map(|backend| backend.config().temperature_c)
                .unwrap_or(ARSPD_TEMP_REF_C),
            autocal: self
                .cluster
                .backend(self.cluster.primary())
                .map(|backend| backend.config().autocal)
                .unwrap_or(ARSPD_AUTOCAL_DEFAULT),
            skip_cal: self
                .cluster
                .backend(self.cluster.primary())
                .map(|backend| backend.config().skip_cal)
                .unwrap_or(ARSPD_SKIP_CAL_DEFAULT),
            tube_order: self.params.primary_tube_order(),
            bus: self.params.primary_bus(),
            devid: self.params.primary_devid(),
        }
    }
}

/// Mark primary instance unhealthy when disabled, for dual-airspeed tests.
#[must_use]
pub fn hookup_with_disabled_primary() -> SitlAirspeedHookup {
    let mut params = AirspeedParams::default();
    params.airspeed1.disabled = true;
    SitlAirspeedHookup {
        cluster: SitlAirspeedCluster::cluster_with_disabled_primary(),
        params,
        truth: SitlAirspeedTruth::default(),
        gps_groundspeed_mps: 0.0,
    }
}
