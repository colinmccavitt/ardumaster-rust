//! Precision landing leftovers, upstream `libraries/AC_PrecLand`.
//!
//! Tracked as **COP-028**.
//!
//! This crate owns the first contiguous `AC_PrecLand` leftovers:
//! [`PrecLand::init`], [`PrecLand::update`], the thin
//! [`PrecLand::handle_msg`] dispatch, the estimator frontend
//! ([`PrecLand::run_estimator`], [`PrecLand::check_ekf_init_timeout`],
//! [`PrecLand::construct_pos_meas_using_rangefinder`],
//! [`PrecLand::retrieve_los_meas`]), Kalman [`PosVelEKF`],
//! [`PrecLand::run_output_prediction`], and the getters /
//! `check_target_status` leftover. `init` is the constructor's
//! follow-on: constrain `PLND_LAG`, size the inertial history ring,
//! pick a sensor backend from `PLND_TYPE`, run that backend's `init()`,
//! and rotate the body-frame approach vector by `PLND_ORIENT`.
//! `update` is the 400 Hz frontend: early-return when there is no
//! backend or no inertial ring, convert the rangefinder argument
//! from centimetres to metres, then record leftovers for the AHRS
//! history push, `_backend->update()`, `run_estimator`,
//! `check_target_status`, and 25 Hz `Write_Precland`.
//! The estimator leftover is the RAW / Kalman switch, the EKF init
//! timeout, rangefinder NED construction, and LOS retrieve. Kalman
//! `PosVelEKF` predict / init / fuse / NIS run here.
//! `run_output_prediction` lag-compensates the estimate and the getters
//! / `check_target_status` leftover consume that output.
//!
//! Copter Land's last 6% (`land_run_normal_or_precland`, `precland_run`,
//! `precland_retry_position`) is blocked on this crate. This slice does
//! not touch COP-013.
//!
//! # `NONE` is a real type, not "not initialised"
//!
//! `AC_PrecLand::init` early-returns only when a backend pointer already
//! exists. `PLND_TYPE=0` (`NONE`) never creates one, so a later `init`
//! with a different type still runs. A port that treated "already called
//! init" as the guard would disagree the first time a vehicle changed
//! type after boot.
//!
//! # Backend `init` is not one function
//!
//! MAVLink's `init` only sets healthy. IRLock and SITL-Gazebo leave
//! healthy false and hand the I2C bus to `irlock.init`. SITL stores
//! `AP::sitl()`. ADR-0004 forbids those drivers and the SITL singleton;
//! [`InitLeftover`] records the leftover.
//!
//! # `update` argument units
//!
//! The header names the first argument `rangefinder_alt_m`. The `.cpp`
//! parameter is `rangefinder_alt_cm` and multiplies by `0.01f`. Copter's
//! caller (`precision_landing.cpp`) passes
//! `rangefinder_state.alt_glitch_protected_m * 100`. This port takes
//! centimetres and converts, matching the body, not the header name.
//!
//! # What this crate does not own yet
//!
//! [`leftover::REMAINING`] is the catalog: logging, the inertial ring,
//! the four sensor `update` paths, and `AC_PrecLand_StateMachine`.

#![no_std]

pub mod estimator;
pub mod leftover;
pub mod pos_vel_ekf;
pub mod precland;
pub mod prediction;

pub use estimator::{
    EkfInitTimeoutLeftover, EstimatorInput, EstimatorWorld, InertialSample, LosSample,
    RunEstimatorLeftover, ACCEL_NOISE_DEFAULT, EKF_INIT_SENSOR_MIN_UPDATE_MS, EKF_INIT_TIME_MS,
    EKF_INIT_VEL_VAR_NAV_INVALID, EKF_INIT_VEL_VAR_NAV_VALID, EKF_NIS_REJECT_THRESHOLD,
    EKF_OUTLIER_REJECT_LIMIT, LANDING_TARGET_TIMEOUT_MS,
};
pub use leftover::REMAINING;
pub use pos_vel_ekf::PosVelEKF;
pub use precland::{
    EstimatorType, HandleMsgLeftover, InitLeftover, LandingTargetMsg, PrecLand, PrecLandParams,
    TargetState, Type, UpdateLeftover, VectorFrame, LAG_S_DEFAULT, LAG_S_MAX, LAG_S_MIN,
    LOG_INTERVAL_MS, OPTION_DISABLED, OPTION_FAST_DESCEND, OPTION_MOVING_TARGET,
    OPTION_PRECLAND_AFTER_REPOSITION, ORIENT_DEFAULT_COPTER, XY_MAX_DIST_DESC_M_DEFAULT,
};
pub use prediction::{
    OutputPredictionLeftover, OutputPredictionWorld, LANDING_TARGET_LOST_DIST_THRESH_M,
    LANDING_TARGET_LOST_TIMEOUT_MS, SENSOR_MAX_ALT_M_DEFAULT, SENSOR_MIN_ALT_M_DEFAULT,
};
