//! Airspeed drivers, upstream `libraries/AP_Airspeed`. FW-010.
//!
//! The SITL backend produces pitot true/equivalent airspeed from body-frame
//! air-relative velocity, matching `SIM_Aircraft` before `AP_Airspeed_SITL`
//! reads the pitot tube.

#![no_std]

pub mod params;
pub mod sitl;

pub use sitl::{
    apply_pitot_ratio, eas_from_tas, pitot_tas_from_body, AirspeedHealthFlags,
    AirspeedSampleState, SitlAirspeedBackend, SitlAirspeedCluster, SitlAirspeedConfig,
    ARSPD_RATIO_DEFAULT, SITL_AIRSPEED_MAX_INSTANCES, SITL_AIRSPEED_UPDATE_MS,
};

pub use params::{
    AirspeedInstanceParams, AirspeedParams, ARSPD_RATIO_PARAM_DEFAULT,
};
