//! Compass drivers, upstream `libraries/AP_Compass`. FW-014.
//!
//! The SITL backend produces body-frame magnetic samples from the world
//! magnetic model and true attitude, matching `SIM_Aircraft::update_mag_field_bf`
//! before `AP_Compass_SITL` reads `_sitl->state.bodyMagField`.

#![no_std]

pub mod auto_rot;
pub mod calibrate;
pub mod consistent;
pub mod declination;
pub mod disable_mask;
pub mod field;
pub mod filter_range;
pub mod learn;
pub mod motor_comp;
pub mod offset;
pub mod orientation;
pub mod params;
pub mod persist;
pub mod primary;
pub mod scale;
pub mod sitl;
pub mod soft_iron;

pub use declination::{CompassDeclinationState, GpsDeclinationFix};
pub use disable_mask::{
    driver_enabled, instance_disabled, sitl_enabled, DriverType, COMPASS_DISBLMSK_DEFAULT,
};
pub use field::{
    expected_earth_field_ga, expected_earth_field_mgauss, expected_field_ok, field_length_ok,
    field_ok, field_strength_ok, gauss_to_mgauss, COMPASS_MAGFIELD_ERROR_THRESHOLD,
    COMPASS_MAGFIELD_EXPECTED, COMPASS_MAGFIELD_MAX, COMPASS_MAGFIELD_MIN,
};
pub use filter_range::{
    filter_enabled, FilterRangeState, COMPASS_FLTR_RNG_DEFAULT, FILTER_KOEF,
};

pub use sitl::{
    mag_field_body_ned, CompassHealthFlags, MagSampleState, SitlCompassBackend, SitlCompassCluster,
    SitlCompassConfig, SITL_COMPASS_MAX_INSTANCES, SITL_COMPASS_UPDATE_MS,
};

pub use auto_rot::{
    accept_detected_orientation, always_45_deg, check_enabled, fix_orientation, settings_for_start,
    AutoRot, AutoRotSettings, COMPASS_AUTO_ROT_CHECK_AND_FIX, COMPASS_AUTO_ROT_CHECK_ONLY,
    COMPASS_AUTO_ROT_DEFAULT, COMPASS_AUTO_ROT_DISABLED, COMPASS_AUTO_ROT_FIX_45,
};
pub use calibrate::{
    cancel_calibration_all, is_calibrating, start_calibration, start_calibration_all,
    CompassCalStatus, CompassCalibrator,
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
pub use primary::first_usable;
pub use scale::{
    apply_scale, have_scale_factor, COMPASS_MAX_SCALE_FACTOR, COMPASS_MIN_SCALE_FACTOR,
    COMPASS_SCALE_DEFAULT,
};
pub use soft_iron::{apply_soft_iron, have_diagonals, COMPASS_DIA_DEFAULT, COMPASS_ODI_DEFAULT};
