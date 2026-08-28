//! Airspeed parameter table stub, upstream AP_Airspeed var_info. FW-010.

use crate::analog::{AnalogAirspeedConfig, ARSPD_PIN_DEFAULT, ARSPD_PSI_RANGE_DEFAULT};
use crate::backend::ARSPD_TYPE_DEFAULT;
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
    /// Sensor / ISA temperature (deg C), upstream SITL `get_temperature`.
    pub temperature_c: f32,
    /// Linear TAS temperature-compensation coefficient (1/deg C).
    pub temp_coeff: f32,
    /// Automatic pitot-ratio calibration, upstream `ARSPD_AUTOCAL`.
    pub autocal: u8,
    /// Analog input pin, upstream `ARSPD_PIN` (`-1` disables).
    pub pin: i8,
    /// Sensor PSI range, upstream `ARSPD_PSI_RANGE`.
    pub psi_range: f32,
    /// Sensor backend type, upstream `ARSPD_TYPE`.
    pub sensor_type: u8,
}

impl Default for AirspeedInstanceParams {
    fn default() -> Self {
        Self {
            disabled: false,
            offset_mps: 0.0,
            skip_cal: crate::sitl::ARSPD_SKIP_CAL_DEFAULT,
            ratio: ARSPD_RATIO_DEFAULT,
            use_airspeed: crate::sitl::ARSPD_USE_DEFAULT,
            temperature_c: crate::sitl::ARSPD_TEMP_REF_C,
            temp_coeff: crate::sitl::ARSPD_TEMP_COEFF_DEFAULT,
            autocal: crate::sitl::ARSPD_AUTOCAL_DEFAULT,
            pin: ARSPD_PIN_DEFAULT,
            psi_range: ARSPD_PSI_RANGE_DEFAULT,
            sensor_type: ARSPD_TYPE_DEFAULT,
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
            temperature_c: self.temperature_c,
            temp_coeff: self.temp_coeff,
            autocal: self.autocal,
        }
    }

    /// Analog pin / PSI-range config, upstream `ARSPD_PIN` / `ARSPD_PSI_RANGE`.
    #[must_use]
    pub fn analog_config(self) -> AnalogAirspeedConfig {
        AnalogAirspeedConfig {
            pin: self.pin,
            psi_range: self.psi_range,
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

    /// Primary instance temperature (deg C).
    #[must_use]
    pub fn primary_temperature_c(&self) -> f32 {
        if self.primary == 0 {
            self.airspeed1.temperature_c
        } else {
            self.airspeed2.temperature_c
        }
    }

    /// Primary instance temperature-compensation coefficient.
    #[must_use]
    pub fn primary_temp_coeff(&self) -> f32 {
        if self.primary == 0 {
            self.airspeed1.temp_coeff
        } else {
            self.airspeed2.temp_coeff
        }
    }

    /// Primary instance `ARSPD_AUTOCAL` / `ARSPD2_AUTOCAL`.
    #[must_use]
    pub fn primary_autocal(&self) -> u8 {
        if self.primary == 0 {
            self.airspeed1.autocal
        } else {
            self.airspeed2.autocal
        }
    }

    /// Primary instance `ARSPD_SKIP_CAL` / `ARSPD2_SKIP_CAL`.
    #[must_use]
    pub fn primary_skip_cal(&self) -> bool {
        if self.primary == 0 {
            self.airspeed1.skip_cal
        } else {
            self.airspeed2.skip_cal
        }
    }

    /// Primary instance `ARSPD_PIN` / `ARSPD2_PIN`.
    #[must_use]
    pub fn primary_pin(&self) -> i8 {
        if self.primary == 0 {
            self.airspeed1.pin
        } else {
            self.airspeed2.pin
        }
    }

    /// Primary instance `ARSPD_PSI_RANGE` / `ARSPD2_PSI_RANGE`.
    #[must_use]
    pub fn primary_psi_range(&self) -> f32 {
        if self.primary == 0 {
            self.airspeed1.psi_range
        } else {
            self.airspeed2.psi_range
        }
    }

    /// Primary instance `ARSPD_TYPE` / `ARSPD2_TYPE`.
    #[must_use]
    pub fn primary_sensor_type(&self) -> u8 {
        if self.primary == 0 {
            self.airspeed1.sensor_type
        } else {
            self.airspeed2.sensor_type
        }
    }
}
