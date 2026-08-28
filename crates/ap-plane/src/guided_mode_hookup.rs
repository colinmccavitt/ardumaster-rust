//! GUIDED mode glue for the main vehicle loop.
//!
//! Upstream ModeGuided::_enter clears `guided_throttle_passthru`, resets
//! `active_radius_m` to 0 (WP_LOITER_RAD), and calls
//! `set_guided_WP(current_loc)`. ModeGuided::navigate calls
//! `update_loiter(active_radius_m)`. Enter-time loiter direction follows the
//! sign of WP_LOITER_RAD, matching `Plane::set_guided_WP`. Stabilization stays
//! on the default arm via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_guided_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Guided)
}

/// Inputs for GUIDED enter plus navigate (ModeGuided::_enter and navigate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidedModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// True when Mode::enter just ran this tick.
    pub mode_just_entered: bool,
    /// Upstream ModeGuided::active_radius_m. Zero uses WP_LOITER_RAD.
    pub active_radius_m: u16,
    /// Upstream WP_LOITER_RAD (aparm.loiter_radius), metres. Negative is CCW.
    pub wp_loiter_rad_m: i16,
    /// Upstream `set_radius_and_direction` CCW flag after enter.
    pub guided_ccw: bool,
}

/// Result of the GUIDED enter / navigate tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidedModeNavOutput {
    /// set_guided_WP armed the hold this tick.
    pub started: bool,
    /// navigate will call update_loiter this tick.
    pub allow_loiter: bool,
    /// active_radius_m after enter reset (0 on enter).
    pub loiter_radius_m: u16,
    /// CCW from WP_LOITER_RAD on enter, or set_radius_and_direction later.
    pub loiter_ccw: bool,
    /// True when a non-zero radius/direction should be applied.
    pub direction_set: bool,
    /// _enter cleared guided_throttle_passthru this tick.
    pub clear_throttle_passthru: bool,
    pub applied: bool,
}

/// Start the current-location hold on GUIDED entry and allow
/// update_loiter(active_radius_m), matching ModeGuided enter and navigate.
#[must_use]
pub fn guided_mode_nav_tick(inp: &GuidedModeNavInputs) -> GuidedModeNavOutput {
    if !is_guided_mode(inp.control_mode, &inp.features) {
        return GuidedModeNavOutput {
            started: false,
            allow_loiter: false,
            loiter_radius_m: 0,
            loiter_ccw: false,
            direction_set: false,
            clear_throttle_passthru: false,
            applied: false,
        };
    }

    // ModeGuided::_enter sets active_radius_m = 0 (WP_LOITER_RAD default).
    let loiter_radius_m = if inp.mode_just_entered {
        0
    } else {
        inp.active_radius_m
    };

    let (loiter_ccw, direction_set) = if inp.mode_just_entered {
        let radius = inp.wp_loiter_rad_m.unsigned_abs();
        (radius > 0 && inp.wp_loiter_rad_m < 0, radius > 0)
    } else {
        (loiter_radius_m > 0 && inp.guided_ccw, loiter_radius_m > 0)
    };

    GuidedModeNavOutput {
        started: inp.mode_just_entered,
        allow_loiter: true,
        loiter_radius_m,
        loiter_ccw,
        direction_set,
        clear_throttle_passthru: inp.mode_just_entered,
        applied: true,
    }
}
