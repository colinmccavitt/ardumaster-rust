//! SITL harness: reusable (non-test) glue that closes PlaneMainLoop around
//! [`ap_sim::sim_plane::SimPlane`].
//!
//! Role counterpart of C++ `fwcpp::hal_sitl::SitlHarness` (CPP-084) and
//! upstream `AP_HAL_SITL` / `SITL_State` — the same *job* (synthesize every
//! sensor Plane actually reads from sim truth, tick the vehicle, feed servos
//! back into the plant), not a line-for-line port of `HAL_SITL_Class.cpp`.
//!
//! Wind estimate is left at PlaneMainLoop's zero default on purpose. C++
//! SitlHarness does the same: `wind_estimate` is a caller-supplied AHRS
//! input that a real vehicle gets from an estimator, not from simulator
//! ground truth. Feeding `SimPlane::wind_ef` would be oracle knowledge.

#![allow(missing_docs)]

pub mod copter_harness;
pub mod harness;

pub use harness::{set_sticks, SitlHarness, SERVO_MAX, SITL_LOOP_HZ};

pub use copter_harness::{
    leftover_apply_collective, leftover_copter_sitl_step, leftover_hold_command,
    leftover_mission_advance, leftover_mission_begin_takeoff, LeftoverCopter, LeftoverMission,
    MissionPhase, SitlCopterHarness,
};
