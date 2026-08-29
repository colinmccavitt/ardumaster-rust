//! Precision landing leftovers, upstream `libraries/AC_PrecLand`.
//! Tracked as **COP-028**.
//!
//! This crate owns the first real `AC_PrecLand` leftover: [`PrecLand::init`].
//! That is the constructor's follow-on: constrain `PLND_LAG`, size the
//! inertial history ring, pick a sensor backend from `PLND_TYPE`, run that
//! backend's `init()`, and rotate the body-frame approach vector by
//! `PLND_ORIENT`.
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
//! # What this crate does not own yet
//!
//! [`leftover::REMAINING`] is the catalog: `update`, the estimator /
//! EKF, LOS construction, output prediction, logging, the four sensor
//! `update` paths, `PosVelEKF`, and `AC_PrecLand_StateMachine`.

#![no_std]

pub mod leftover;
pub mod precland;

pub use leftover::REMAINING;
pub use precland::{
    EstimatorType, InitLeftover, PrecLand, PrecLandParams, TargetState, Type, VectorFrame,
    LAG_S_DEFAULT, LAG_S_MAX, LAG_S_MIN, OPTION_DISABLED, OPTION_FAST_DESCEND,
    OPTION_MOVING_TARGET, OPTION_PRECLAND_AFTER_REPOSITION, ORIENT_DEFAULT_COPTER,
    XY_MAX_DIST_DESC_M_DEFAULT,
};
