//! GUIDED mode glue for the main vehicle loop.
//!
//! Upstream ModeGuided::_enter clears `guided_throttle_passthru`, resets
//! `active_radius_m` to 0 (WP_LOITER_RAD), and calls
//! `set_guided_WP(current_loc)`. ModeGuided::navigate calls
//! `update_loiter(active_radius_m)`. Enter-time loiter direction follows the
//! sign of WP_LOITER_RAD, matching `Plane::set_guided_WP`. A later location
//! update (`handle_guided_request`) re-runs `set_guided_WP` so `prev_WP` is
//! current and `setup_alt_slope` starts the remaining leg; an altitude-only
//! change (`handle_change_alt_request`) copies onto `next_WP_loc` and
//! `reset_offset_altitude`. Stabilization stays on the default arm via
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

/// Inputs for GUIDED altitude / location remaining-leg
/// (`ModeGuided::handle_guided_request` and `GCS_MAVLINK_Plane::handle_change_alt_request`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidedModeUpdateInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// GCS / companion sent a new target location this tick.
    pub location_update: bool,
    /// GCS sent `DO_CHANGE_ALTITUDE` / change-alt this tick.
    pub altitude_update: bool,
    /// Incoming request uses terrain altitude (`Location::terrain_alt`).
    pub terrain_alt: bool,
}

/// Result of the GUIDED altitude / location remaining-leg tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuidedModeUpdateOutput {
    /// `handle_guided_request` called `set_guided_WP` this tick.
    pub set_guided_wp: bool,
    /// `prev_WP = current`, `setup_alt_slope`, `setup_turn_angle`, crosstrack off.
    pub setup_remaining_leg: bool,
    /// Non-terrain request converted to `AltFrame::ABSOLUTE` this tick.
    pub convert_abs_alt: bool,
    /// `handle_change_alt_request` copied altitude onto `next_WP_loc`.
    pub copy_next_wp_alt: bool,
    /// `reset_offset_altitude` after an altitude-only change.
    pub reset_offset_altitude: bool,
    pub applied: bool,
}

/// Apply a mid-GUIDED location or altitude update and set up the remaining
/// nav leg, matching `handle_guided_request` and `handle_change_alt_request`.
#[must_use]
pub fn guided_mode_update_tick(inp: &GuidedModeUpdateInputs) -> GuidedModeUpdateOutput {
    if !is_guided_mode(inp.control_mode, &inp.features) {
        return GuidedModeUpdateOutput {
            set_guided_wp: false,
            setup_remaining_leg: false,
            convert_abs_alt: false,
            copy_next_wp_alt: false,
            reset_offset_altitude: false,
            applied: false,
        };
    }

    let set_guided_wp = inp.location_update;
    let setup_remaining_leg = inp.location_update;
    let copy_next_wp_alt = inp.altitude_update;
    let reset_offset_altitude = inp.altitude_update;
    let convert_abs_alt = (inp.location_update || inp.altitude_update) && !inp.terrain_alt;

    GuidedModeUpdateOutput {
        set_guided_wp,
        setup_remaining_leg,
        convert_abs_alt,
        copy_next_wp_alt,
        reset_offset_altitude,
        applied: true,
    }
}
