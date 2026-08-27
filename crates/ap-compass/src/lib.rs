//! Compass drivers, upstream `libraries/AP_Compass`. FW-014.
//!
//! The SITL backend produces body-frame magnetic samples from the world
//! magnetic model and true attitude, matching `SIM_Aircraft::update_mag_field_bf`
//! before `AP_Compass_SITL` reads `_sitl->state.bodyMagField`.

#![no_std]

pub mod sitl;

pub use sitl::{
    mag_field_body_ned, CompassHealthFlags, MagSampleState, SitlCompassBackend,
    SitlCompassCluster, SitlCompassConfig, SITL_COMPASS_MAX_INSTANCES,
    SITL_COMPASS_UPDATE_MS,
};
