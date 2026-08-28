//! TAKEOFF mode glue for the main vehicle loop.
//!
//! Upstream ModeTakeoff::_enter clears `takeoff_mode_setup`. ModeTakeoff::update
//! refuses waypoint setup until `current_loc.initialised()` and
//! `AP::ahrs().home_is_set()`. ModeTakeoff::navigate calls `update_loiter(0)`,
//! which uses WP_LOITER_RAD. Stabilization stays on the default arm via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

/// Upstream ModeTakeoff `TKOFF_ALT` default, metres.
pub const TKOFF_ALT_DEFAULT_M: u16 = 50;
/// Upstream ModeTakeoff `TKOFF_DIST` default, metres.
pub const TKOFF_DIST_DEFAULT_M: u16 = 200;

fn is_takeoff_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Takeoff)
}

/// Inputs for TAKEOFF enter plus navigate (ModeTakeoff::_enter and navigate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TakeoffModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// True when Mode::enter just ran this tick.
    pub mode_just_entered: bool,
    /// Upstream AP::ahrs().home_is_set().
    pub home_is_set: bool,
    /// Upstream `plane.current_loc.initialised()`.
    pub current_loc_initialised: bool,
    /// Upstream TKOFF_ALT, metres.
    pub target_alt_m: u16,
    /// Upstream TKOFF_DIST, metres.
    pub target_dist_m: u16,
    /// Upstream WP_LOITER_RAD (aparm.loiter_radius), metres. Negative is CCW.
    pub wp_loiter_rad_m: i16,
}

/// Result of the TAKEOFF enter / navigate tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TakeoffModeNavOutput {
    /// `_enter` cleared takeoff_mode_setup this tick.
    pub started: bool,
    /// `update()` may place the climb/loiter waypoint this tick.
    pub allow_setup: bool,
    /// navigate will call `update_loiter(0)` this tick.
    pub allow_loiter: bool,
    /// Always 0: navigate passes zero so `update_loiter` uses WP_LOITER_RAD.
    pub loiter_radius_m: u16,
    /// WP_LOITER_RAD < 0 selects counterclockwise loiter.
    pub loiter_ccw: bool,
    /// True when WP_LOITER_RAD is non-zero and direction should be applied.
    pub direction_set: bool,
    /// TKOFF_ALT applied this tick.
    pub target_alt_m: u16,
    /// TKOFF_DIST applied this tick.
    pub target_dist_m: u16,
    pub applied: bool,
}

/// Clear takeoff_mode_setup on TAKEOFF entry, gate waypoint setup on home
/// plus a valid loc, and allow `update_loiter(0)`, matching ModeTakeoff
/// enter and navigate.
#[must_use]
pub fn takeoff_mode_nav_tick(inp: &TakeoffModeNavInputs) -> TakeoffModeNavOutput {
    if !is_takeoff_mode(inp.control_mode, &inp.features) {
        return TakeoffModeNavOutput {
            started: false,
            allow_setup: false,
            allow_loiter: false,
            loiter_radius_m: 0,
            loiter_ccw: false,
            direction_set: false,
            target_alt_m: 0,
            target_dist_m: 0,
            applied: false,
        };
    }

    let wp_abs = inp.wp_loiter_rad_m.unsigned_abs();
    let direction_set = wp_abs > 0;
    let loiter_ccw = direction_set && inp.wp_loiter_rad_m < 0;

    TakeoffModeNavOutput {
        started: inp.mode_just_entered,
        allow_setup: inp.home_is_set && inp.current_loc_initialised,
        allow_loiter: true,
        loiter_radius_m: 0,
        loiter_ccw,
        direction_set,
        target_alt_m: inp.target_alt_m,
        target_dist_m: inp.target_dist_m,
        applied: true,
    }
}
