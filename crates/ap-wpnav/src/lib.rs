//! Waypoint navigator, upstream `libraries/AC_WPNav/AC_WPNav`.
//! Tracked as **COP-010**.
//!
//! Copter-4.7 has no separate `enable()` or `init()`. Those older names
//! collapsed into [`WpNav::wp_and_spline_init_m`]: clamp the speed and
//! radius floors, compute S-curve jerk/snap from the attitude controller,
//! clear the path legs, and seat origin and destination on the stopping
//! point. After that the next call is
//! [`set_wp_destination_ned_m`](WpNav::set_wp_destination_ned_m) (or the
//! centimetre NEU wrapper), which seats a real destination and clears
//! `reached_destination`. After that the 100 Hz tick is
//! [`update_wpnav`](WpNav::update_wpnav): watch `WP_SPD` / `WP_SPD_UP` /
//! `WP_SPD_DN` for in-flight changes, apply [`set_speed_ne_ms`](WpNav::set_speed_ne_ms)
//! (and the climb / descent setters), then record the leftover of
//! `advance_wp_target_along_track` and `NE_update_controller`. Horizontal
//! distance to the dest is [`get_wp_distance_to_destination_m`](WpNav::get_wp_distance_to_destination_m).
//! [`advance_wp_target_along_track`](WpNav::advance_wp_target_along_track)
//! owns track-time / offset-velocity shaping and the reached-destination
//! flag; S-curve / spline target advance stays in `ap-math`. Bearing is
//! [`get_wp_bearing_to_destination_rad`](WpNav::get_wp_bearing_to_destination_rad).
//! A spline dest is [`set_spline_destination_ned_m`](WpNav::set_spline_destination_ned_m):
//! same re-init and terrain-frame rules as the straight dest setter, then
//! leftover origin / destination velocity vectors for `SplineCurve`.
//! The next-leg preload is
//! [`set_spline_destination_next_ned_m`](WpNav::set_spline_destination_next_ned_m):
//! skip when the next dest terrain frame mismatches, then leftover
//! `_spline_next_leg` origin / dest velocities and this-leg
//! `set_destination_speed_max`.
//! The next straight dest is
//! [`set_wp_destination_next_ned_m`](WpNav::set_wp_destination_next_ned_m):
//! same terrain-frame skip, then leftover next-leg `SCurve::calculate_track`
//! and a fast-waypoint flag. Location wrappers convert through
//! [`get_vector_ned_m`](WpNav::get_vector_ned_m). Terrain offset is
//! [`get_terrain_u_m`](WpNav::get_terrain_u_m) / [`get_terrain_d_m`](WpNav::get_terrain_d_m).
//! Stopping-point centimetre wrappers convert a PosControl leftover.
//! [`force_stop_at_next_wp`](WpNav::force_stop_at_next_wp) clears the fast
//! flag and records the scurve dest-speed / next-leg init leftover.
//!
//! Horizontal loiter is [`loiter`] (**COP-011**):
//! [`Loiter::init_target_m`] / [`Loiter::init_target`] then
//! [`Loiter::update`], plus
//! [`Loiter::set_pilot_desired_acceleration_rad`].
//! Horizontal circle is [`circle`] (**COP-011**):
//! [`Circle::init`] / [`Circle::init_ned_m`] then [`Circle::update_ms`],
//! plus [`Circle::set_center`] and
//! [`Circle::get_closest_point_on_circle_ned_m`].
//!
//! # What this crate does not own
//!
//! The S-curve and spline objects live in `ap-math` (COP-002 / COP-003).
//! The position controller lives in `ap-control` (COP-009). This slice
//! does not rewrite either. ADR-0004 forbids the AHRS / PosControl /
//! millis singletons, so the caller supplies the stopping point, the
//! attitude limits used for jerk, and `now_ms`. The speed and
//! acceleration [`wp_and_spline_init_m`](WpNav::wp_and_spline_init_m)
//! would write into `AC_PosControl` are recorded on the navigator for a
//! later slice to apply.

#![no_std]

pub mod circle;
pub mod loiter;
pub mod wpnav;

pub use circle::{
    Circle, CircleOption, ClosestPointOnCircle, InitCircleContext, InitCircleLeftover,
    SetCenterLeftover, UpdateCircleContext, UpdateCircleLeftover, CIRCLE_ACTIVE_TIMEOUT_MS,
    CIRCLE_ANGULAR_ACCEL_MIN, CIRCLE_DEFAULT_OPTIONS, CIRCLE_RADIUS_MAX_M, CIRCLE_RADIUS_M_DEFAULT,
    CIRCLE_RATE_DEFAULT,
};
pub use loiter::{
    AngleGains, InitTargetContext, InitTargetLeftover, Loiter, LoiterOption, PilotAccelContext,
    ShapingConfig, UpdateLoiterContext, UpdateLoiterLeftover, LOITER_ACCEL_MAX_DEFAULT_MSS,
    LOITER_ACTIVE_TIMEOUT_MS, LOITER_BRAKE_ACCEL_DEFAULT_MSS, LOITER_BRAKE_JERK_DEFAULT_MSSS,
    LOITER_BRAKE_START_DELAY_DEFAULT_S, LOITER_DEFAULT_OPTIONS, LOITER_POS_CORRECTION_MAX_M,
    LOITER_SPEED_DEFAULT_MS, LOITER_SPEED_MIN_MS, LOITER_VEL_CORRECTION_MAX_MS,
};
pub use wpnav::{
    AdvanceWpTargetContext, AdvanceWpTargetLeftover, AttitudeJerkLimits, GetTerrainContext,
    GetVectorNedContext, PosControlSpeedAccel, SetWpDestinationContext, TerrainSource,
    UpdateTrackLimitsLeftover, UpdateWpNavContext, UpdateWpNavLeftover, WpNav, WpNavFlags,
    WPNAV_ACCELERATION_MS, WPNAV_ACTIVE_TIMEOUT_MS, WP_ACC_Z_DEFAULT, WP_JERK_DEFAULT,
    WP_RADIUS_M_DEFAULT, WP_RADIUS_M_MIN, WP_SPD_DEFAULT, WP_SPD_DOWN_DEFAULT, WP_SPD_MIN,
    WP_SPD_UP_DEFAULT,
};
