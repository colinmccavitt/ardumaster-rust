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

pub mod wpnav;

pub use wpnav::{
    AdvanceWpTargetContext, AdvanceWpTargetLeftover, AttitudeJerkLimits, PosControlSpeedAccel,
    SetWpDestinationContext, UpdateWpNavContext, UpdateWpNavLeftover, WpNav, WpNavFlags,
    WPNAV_ACCELERATION_MS, WPNAV_ACTIVE_TIMEOUT_MS, WP_ACC_Z_DEFAULT, WP_JERK_DEFAULT,
    WP_RADIUS_M_DEFAULT, WP_RADIUS_M_MIN, WP_SPD_DEFAULT, WP_SPD_DOWN_DEFAULT, WP_SPD_MIN,
    WP_SPD_UP_DEFAULT,
};
