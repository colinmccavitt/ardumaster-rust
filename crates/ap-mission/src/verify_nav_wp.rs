//! NAV_WAYPOINT verify-distance / reached-wp.
//!
//! Upstream `Plane::verify_nav_wp` (`ArduPlane/commands_logic.cpp`) returns
//! true when the current nav waypoint is complete so AUTO can advance. This
//! stub is the two completion tests that do not need pass-by, `WP_MAX_RADIUS`,
//! or L1 turn-distance yet: distance vs `WP_RADIUS` (`get_wp_radius`) and the
//! finish-line fly-past check (`Location::past_interval_finish_line`).

use ap_math::location::Location;
use ap_math::Ftype;

/// Default `WP_RADIUS` / `g.waypoint_radius` (metres) for fixed-wing.
pub const WP_RADIUS_DEFAULT_M: f32 = 90.0;

/// Inputs for one NAV_WAYPOINT verify tick, upstream `verify_nav_wp`.
#[derive(Debug, Clone, Copy)]
pub struct VerifyNavWpInputs {
    /// Vehicle location this tick, upstream `current_loc`.
    pub current_loc: Location,
    /// Active waypoint, upstream `next_WP_loc`.
    pub next_wp: Location,
    /// Previous waypoint, upstream `prev_WP_loc`.
    pub prev_wp: Location,
    /// Acceptance radius in metres, upstream `get_wp_radius()` / `WP_RADIUS`.
    pub wp_radius_m: f32,
}

impl Default for VerifyNavWpInputs {
    fn default() -> Self {
        Self {
            current_loc: Location::new(0, 0),
            next_wp: Location::new(0, 0),
            prev_wp: Location::new(0, 0),
            wp_radius_m: WP_RADIUS_DEFAULT_M,
        }
    }
}

/// True when the vehicle has reached or flown past the nav waypoint.
///
/// Upstream `Plane::verify_nav_wp`: `wp_dist <= acceptance_distance_m` (here
/// `wp_radius_m` stands in for `turn_distance(get_wp_radius(), ...)`) or
/// `current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)`.
#[must_use]
pub fn verify_nav_wp(inp: &VerifyNavWpInputs) -> bool {
    let acceptance = Ftype::from(inp.wp_radius_m);
    let wp_dist = inp.current_loc.get_distance(inp.next_wp);
    if wp_dist <= acceptance {
        return true;
    }
    inp.current_loc
        .past_interval_finish_line(inp.prev_wp, inp.next_wp)
}
