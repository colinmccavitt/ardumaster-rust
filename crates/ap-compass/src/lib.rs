//! Compass drivers, upstream `libraries/AP_Compass`. FW-014.
//!
//! The SITL backend produces body-frame magnetic samples from the world
//! magnetic model and true attitude, matching `SIM_Aircraft::update_mag_field_bf`
//! before `AP_Compass_SITL` reads `_sitl->state.bodyMagField`.

#![no_std]

pub mod consistent;
pub mod declination;
pub mod learn;
pub mod motor_comp;
pub mod offset;
pub mod orientation;
pub mod params;
pub mod persist;
pub mod scale;
pub mod sitl;
pub mod soft_iron;

pub use declination::{CompassDeclinationState, GpsDeclinationFix};

pub use sitl::{
    mag_field_body_ned, CompassHealthFlags, MagSampleState, SitlCompassBackend, SitlCompassCluster,
    SitlCompassConfig, SITL_COMPASS_MAX_INSTANCES, SITL_COMPASS_UPDATE_MS,
};

pub use consistent::{consistent, use_for_yaw_if_consistent, CompassInstanceField};
pub use learn::LearnType;
pub use motor_comp::{
    apply_motor_compensation, learn_motor_compensation, motor_comp_enabled, motor_offset,
    COMPASS_MOTCT_DEFAULT, COMPASS_MOT_COMP_CURRENT, COMPASS_MOT_COMP_DISABLED,
    COMPASS_MOT_COMP_THROTTLE,
};
pub use offset::{
    apply_offsets, learn_offsets, learn_offsets_enabled, offsets_within_max, COMPASS_LEARN_DEFAULT,
    COMPASS_LEARN_EKF, COMPASS_LEARN_INFLIGHT, COMPASS_LEARN_NONE, COMPASS_OFFSETS_MAX_DEFAULT,
};
pub use orientation::{
    apply_orientation, is_external, rotate_field, COMPASS_EXTERNAL_DEFAULT, COMPASS_ORIENT_DEFAULT,
    COMPASS_ORIENT_YAW_90,
};
pub use params::{
    CompassInstanceParams, CompassParams, COMPASS_AUTODEC_DEFAULT, COMPASS_USE_DEFAULT,
};
pub use persist::{offsets_already_saved, save_instance_offset, save_offsets};
pub use scale::{
    apply_scale, have_scale_factor, COMPASS_MAX_SCALE_FACTOR, COMPASS_MIN_SCALE_FACTOR,
    COMPASS_SCALE_DEFAULT,
};
pub use soft_iron::{apply_soft_iron, have_diagonals, COMPASS_DIA_DEFAULT, COMPASS_ODI_DEFAULT};
