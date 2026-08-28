//! Airspeed parameter table stub, upstream AP_Airspeed var_info. FW-010.

use crate::sitl::{
    SitlAirspeedBackend, SitlAirspeedCluster, SitlAirspeedConfig, ARSPD_RATIO_DEFAULT,
    SITL_AIRSPEED_MAX_INSTANCES,
};

/// Upstream `ARSPD_RATIO` / `ARSPD2_RATIO` default.
pub const ARSPD_RATIO_PARAM_DEFAULT: f32 = ARSPD_RATIO_DEFAULT;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedInstanceParams {
    pub disabled: bool,
    pub offset_mps: f32,
    /// Skip startup / requested calibration, upstream `ARSPD_SKIP_CAL`.
    pub skip_cal: bool,
    /// Pitot tube ratio, upstream `ARSPD_RATIO`.
    pub ratio: f32,
    /// Use TAS for TECS/nav, upstream `ARSPD_USE`.
    pub use_airspeed: u8,
}

impl Default for AirspeedInstanceParams {
    fn default() -> Self {
        Self {
            disabled: false,
            offset_mps: 0.0,
            skip_cal: false,
            ratio: ARSPD_RATIO_DEFAULT,
            use_airspeed: crate::sitl::ARSPD_USE_DEFAULT,
        }
    }
}

impl AirspeedInstanceParams {
    #[must_use]
    pub fn apply_to_config(self) -> SitlAirspeedConfig {
        SitlAirspeedConfig {
            disabled: self.disabled,
            offset_mps: self.offset_mps,
            skip_cal: self.skip_cal,
            ratio: self.ratio,
            use_airspeed: self.use_airspeed,
        }
    }
}

/// Dual-instance airspeed params, upstream `AP_Airspeed` / `ARSPD_*` / `ARSPD2_*`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedParams {
    pub airspeed1: AirspeedInstanceParams,
    pub airspeed2: AirspeedInstanceParams,
    pub primary: u8,
}

impl Default for AirspeedParams {
    fn default() -> Self {
        Self {
            airspeed1: AirspeedInstanceParams::default(),
            airspeed2: AirspeedInstanceParams::default(),
            primary: 0,
        }
    }
}

impl AirspeedParams {
    pub fn apply_instance(&self, instance: u8, backend: &mut SitlAirspeedBackend) {
        let inst = if instance == 0 {
            self.airspeed1
        } else {
            self.airspeed2
        };
        backend.set_config(inst.apply_to_config());
    }

    pub fn apply_to_cluster(&self, cluster: &mut SitlAirspeedCluster) {
        cluster.set_primary(self.primary.min((SITL_AIRSPEED_MAX_INSTANCES - 1) as u8));
        for i in 0..cluster.instance_count() {
            if let Some(backend) = cluster.backend_mut(i) {
                self.apply_instance(i, backend);
            }
        }
    }

    /// Primary instance pitot ratio, upstream `ARSPD_RATIO` / `ARSPD2_RATIO`.
    #[must_use]
    pub fn primary_ratio(&self) -> f32 {
        if self.primary == 0 {
            self.airspeed1.ratio
        } else {
            self.airspeed2.ratio
        }
    }

    /// Primary instance `ARSPD_USE` / `ARSPD2_USE`.
    #[must_use]
    pub fn primary_use_airspeed(&self) -> u8 {
        if self.primary == 0 {
            self.airspeed1.use_airspeed
        } else {
            self.airspeed2.use_airspeed
        }
    }
}
