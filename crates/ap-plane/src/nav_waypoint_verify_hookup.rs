//! Vehicle hookup for NAV_WAYPOINT verify-distance / reached-wp.
//!
//! AUTO already advances a stored waypoint index. This is the per-command
//! complete check that index will call: `Plane::verify_nav_wp` against
//! `WP_RADIUS` (`get_wp_radius`) and the finish-line fly-past test.

use ap_math::location::Location;
use ap_mission::{verify_nav_wp, MissionCommand, VerifyNavWpInputs};

/// HAL inputs for one NAV_WAYPOINT verify tick.
#[derive(Debug, Clone, Copy)]
pub struct NavWaypointVerifyInputs {
    /// Vehicle location this tick, upstream `current_loc`.
    pub current_loc: Location,
    /// Previous waypoint, upstream `prev_WP_loc`.
    pub prev_wp: Location,
    /// Active `MAV_CMD_NAV_WAYPOINT` item.
    pub cmd: MissionCommand,
    /// Upstream `get_wp_radius()` / `WP_RADIUS`, metres.
    pub wp_radius_m: f32,
}

impl Default for NavWaypointVerifyInputs {
    fn default() -> Self {
        Self {
            current_loc: Location::new(0, 0),
            prev_wp: Location::new(0, 0),
            cmd: MissionCommand::none(),
            wp_radius_m: ap_mission::WP_RADIUS_DEFAULT_M,
        }
    }
}

/// Result of one NAV_WAYPOINT verify tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NavWaypointVerifyOutput {
    /// Upstream `verify_nav_wp` returned true: command complete.
    pub reached: bool,
    /// The stored item was `MAV_CMD_NAV_WAYPOINT`.
    pub applied: bool,
}

/// Complete a NAV_WAYPOINT when inside `WP_RADIUS` or past the finish line.
#[must_use]
pub fn nav_waypoint_verify_tick(inp: &NavWaypointVerifyInputs) -> NavWaypointVerifyOutput {
    if !inp.cmd.is_nav_waypoint() {
        return NavWaypointVerifyOutput {
            reached: false,
            applied: false,
        };
    }
    let reached = verify_nav_wp(&VerifyNavWpInputs {
        current_loc: inp.current_loc,
        next_wp: inp.cmd.location,
        prev_wp: inp.prev_wp,
        wp_radius_m: inp.wp_radius_m,
    });
    NavWaypointVerifyOutput {
        reached,
        applied: true,
    }
}
