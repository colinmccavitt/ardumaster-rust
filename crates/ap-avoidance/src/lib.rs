//! Obstacle avoidance leftovers, upstream `libraries/AC_Avoidance`.
//! Tracked as **COP-026**.
//!
//! This is the first real `AC_Avoid` leftover. Copter PosHold / Loiter
//! already record the compiled-out climb-rate identity — with avoidance
//! off the rate passes through unchanged. This crate owns that path when
//! it is compiled in: [`Avoid::adjust_velocity_z`] and the Copter wrapper
//! [`get_avoidance_adjusted_climbrate_ms`].
//!
//! ADR-0004 forbids the fence / AHRS singletons. [`AdjustVelocityZContext`]
//! is the leftover of `AP::fence()`, `get_alt_in_alt_*_frame_m`, and
//! `ahrs.get_hgt_ctrl_limit` / `get_relative_position_D_origin_float`.
//! Proximity upward distance, horizontal `adjust_velocity`, and the
//! OA path planner stay later leftovers.
//!
//! # What this crate does not own
//!
//! Proximity / beacon fences, `adjust_velocity` (NEU), BendyRuler /
//! Dijkstra, the OA database, and lean-angle avoidance in non-GPS modes.

#![no_std]

pub mod avoid;

pub use avoid::{
    get_avoidance_adjusted_climbrate_ms, AdjustVelocityZContext, AdjustVelocityZLeftover, Avoid,
    ACCEL_CMSS_MAX, AVOID_DEFAULT, BACKUP_SPEED_MAX_U_MS_DEFAULT, DISABLED, STOP_AT_BEACON_FENCE,
    STOP_AT_FENCE, USE_PROXIMITY_SENSOR,
};
