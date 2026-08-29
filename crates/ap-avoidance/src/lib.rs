//! Obstacle avoidance leftovers, upstream `libraries/AC_Avoidance`.
//! Tracked as **COP-026**.
//!
//! This crate owns the first real `AC_Avoid` leftovers. Copter PosHold /
//! Loiter already record the compiled-out climb-rate identity — with
//! avoidance off the rate passes through unchanged. When compiled in:
//! [`Avoid::adjust_velocity_z`] and the Copter wrapper
//! [`get_avoidance_adjusted_climbrate_ms`].
//!
//! The second leftover is horizontal: [`Avoid::limit_velocity_ne`] (and
//! [`Avoid::limit_velocity_neu`]) plus the proximity-backed STOP arm
//! [`Avoid::adjust_velocity_proximity`]. [`Avoid::adjust_velocity_ne`] is
//! the NE slice of `AC_Avoid::adjust_velocity` with only that proximity
//! arm compiled in.
//!
//! The third leftover is the full path: [`Avoid::adjust_velocity`] and
//! [`Avoid::adjust_velocity_ned_m`]. Those run proximity (earth → body →
//! earth), the vertical fence tail, and the NE / U backup mix.
//! [`AdjustVelocityContext`] injects the fence / proximity / AHRS reads.
//!
//! The fourth leftover is fence NE: [`Avoid::adjust_velocity_fence`]
//! (classic circle, inclusion / exclusion polygons, polyfence circles,
//! beacon). [`FenceNeContext`] is the leftover of `AP::fence()`,
//! `ahrs.get_relative_position_NE_*`, and `AP::beacon()->get_boundary_points`.
//!
//! ADR-0004 forbids the fence / AHRS / proximity / beacon singletons.
//! [`AdjustVelocityZContext`] is the leftover of `AP::fence()`,
//! `get_alt_in_alt_*_frame_m`, and `ahrs.get_hgt_ctrl_limit` /
//! `get_relative_position_D_origin_float`. [`ProximityStopContext`] is
//! the leftover of `AP::proximity()->get_obstacle` /
//! `closest_point_from_segment_to_obstacle` and the AHRS 2-D yaw
//! rotation. Accel-jerk limiting and the OA path planner stay later
//! leftovers.
//!
//! # What this crate does not own
//!
//! `limit_accel_NEU_cm`, BendyRuler / Dijkstra, the OA database, and
//! lean-angle avoidance in non-GPS modes.

#![no_std]

pub mod avoid;
pub mod fence_ne;

pub use avoid::{
    get_avoidance_adjusted_climbrate_ms, AdjustVelocityContext, AdjustVelocityLeftover,
    AdjustVelocityNeLeftover, AdjustVelocityZContext, AdjustVelocityZLeftover, Avoid,
    ProximityStopContext, ProximityStopLeftover, ACCEL_CMSS_MAX, AVOID_DEFAULT,
    BACKUP_DEADZONE_M_DEFAULT, BACKUP_SPEED_MAX_NE_MS_DEFAULT, BACKUP_SPEED_MAX_U_MS_DEFAULT,
    BEHAVIOR_SLIDE, BEHAVIOR_STOP, DISABLED, MARGIN_M_DEFAULT, STOP_AT_BEACON_FENCE, STOP_AT_FENCE,
    USE_PROXIMITY_SENSOR,
};
pub use fence_ne::{
    AdjustVelocityFenceLeftover, FenceCircle, FenceNeContext, FencePolygon, FENCE_NE_VERTICES_MAX,
};
