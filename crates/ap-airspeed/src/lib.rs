//! Airspeed drivers, upstream `libraries/AP_Airspeed`. FW-010.
//!
//! The SITL backend produces pitot true/equivalent airspeed from body-frame
//! air-relative velocity, matching `SIM_Aircraft` before `AP_Airspeed_SITL`
//! reads the pitot tube.

#![no_std]

pub mod params;
pub mod sitl;

pub use sitl::{
    apply_pitot_ratio, apply_temp_compensation, eas_from_tas, pitot_tas_from_body,
    sitl_airspeed_temperature_c, tas_for_nav, use_airspeed_for_control, AirspeedHealthFlags,
    AirspeedSampleState, SitlAirspeedBackend, SitlAirspeedCluster, SitlAirspeedConfig,
    ARSPD_RATIO_DEFAULT, ARSPD_TEMP_COEFF_DEFAULT, ARSPD_TEMP_REF_C, ARSPD_USE_DEFAULT,
    ISA_LAPSE_K_PER_M, SITL_AIRSPEED_MAX_INSTANCES, SITL_AIRSPEED_UPDATE_MS,
};

pub use params::{
    AirspeedInstanceParams, AirspeedParams, ARSPD_RATIO_PARAM_DEFAULT,
};
