//! Compass drivers, upstream `libraries/AP_Compass`. FW-014.
//!
//! The SITL backend produces body-frame magnetic samples from the world
//! magnetic model and true attitude, matching `SIM_Aircraft::update_mag_field_bf`
//! before `AP_Compass_SITL` reads `_sitl->state.bodyMagField`.

#![no_std]

pub mod declination;
pub mod params;
pub mod sitl;

pub use declination::{CompassDeclinationState, GpsDeclinationFix};

pub use sitl::{
    mag_field_body_ned, CompassHealthFlags, MagSampleState, SitlCompassBackend,
    SitlCompassCluster, SitlCompassConfig, SITL_COMPASS_MAX_INSTANCES,
    SITL_COMPASS_UPDATE_MS,
};

pub use params::{CompassInstanceParams, CompassParams, COMPASS_AUTODEC_DEFAULT, COMPASS_USE_DEFAULT};
