//! AUTOLAND mode glue for the main vehicle loop.
//!
//! Upstream ModeAutoLand::_enter refuses unless already flying, takeoff
//! direction is initialized, and quadplane is unavailable. Navigate walks
//! CLIMB -> LOITER -> LANDING. Climb applies LEVEL_ROLL_LIMIT. Stabilization
//! stays on the default arm via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

/// Upstream ModeAutoLand `AUTOLAND_WP_ALT` default, metres.
pub const AUTOLAND_WP_ALT_DEFAULT_M: u16 = 55;
/// Upstream ModeAutoLand `AUTOLAND_WP_DIST` default, metres.
pub const AUTOLAND_WP_DIST_DEFAULT_M: u16 = 400;
/// Upstream `fast_climb_extra_alt`, metres. Added to the climb target.
pub const FAST_CLIMB_EXTRA_ALT_M: u16 = 10;

/// Upstream `AutoLandStage::CLIMB`.
pub const STAGE_CLIMB: u8 = 0;
/// Upstream `AutoLandStage::LOITER`.
pub const STAGE_LOITER: u8 = 1;
/// Upstream `AutoLandStage::LANDING`.
pub const STAGE_LANDING: u8 = 2;

fn is_autoland_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Autoland)
}

/// Inputs for AUTOLAND enter plus navigate (ModeAutoLand::_enter and navigate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutolandModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// True when Mode::enter just ran this tick.
    pub mode_just_entered: bool,
    /// Upstream `plane.is_flying()`.
    pub is_flying: bool,
    /// Upstream `takeoff_state.initial_direction.initialized`.
    pub takeoff_direction_initialized: bool,
    /// Upstream `quadplane.available()`.
    pub quadplane_available: bool,
    /// Upstream landing type is deepstall (skips loiter-to-alt).
    pub landing_is_deepstall: bool,
    /// Upstream `AUTOLAND_CLIMB` (`terrain_alt_min`), metres. 0 disables.
    pub terrain_alt_min_m: u16,
    /// True when `terrain_alt_min - relative_ground_altitude` is positive.
    pub need_climb: bool,
    /// Current `AutoLandStage` while already in the mode.
    pub current_stage: u8,
    /// Climb stage finished: reached loiter, lost height-above, or extra-alt.
    pub climb_complete: bool,
    /// Upstream `verify_loiter_to_alt(cmd_loiter)`.
    pub loiter_to_alt_complete: bool,
    /// Upstream `AUTOLAND_WP_ALT`, metres.
    pub wp_alt_m: u16,
    /// Upstream `AUTOLAND_WP_DIST`, metres.
    pub wp_dist_m: u16,
}

/// Result of the AUTOLAND enter / navigate tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutolandModeNavOutput {
    /// `_enter` succeeded this tick.
    pub started: bool,
    /// `_enter` refused this tick (not flying, no takeoff heading, quadplane).
    pub refused: bool,
    /// Stage after enter or navigate advance.
    pub stage: u8,
    /// navigate will call `update_loiter` this tick (CLIMB or LOITER).
    pub allow_loiter: bool,
    /// navigate will verify the NAV_LAND command this tick.
    pub allow_land: bool,
    /// `update()` applies LEVEL_ROLL_LIMIT during CLIMB.
    pub apply_level_roll: bool,
    /// `_enter` / climb-complete set `auto_state.next_wp_crosstrack`.
    pub next_wp_crosstrack: bool,
    /// AUTOLAND_WP_ALT applied this tick.
    pub wp_alt_m: u16,
    /// AUTOLAND_WP_DIST applied this tick.
    pub wp_dist_m: u16,
    pub applied: bool,
}

fn enter_allowed(inp: &AutolandModeNavInputs) -> bool {
    inp.is_flying && inp.takeoff_direction_initialized && !inp.quadplane_available
}

fn initial_stage(inp: &AutolandModeNavInputs) -> u8 {
    if inp.landing_is_deepstall {
        STAGE_LANDING
    } else if inp.terrain_alt_min_m > 0 && inp.need_climb {
        STAGE_CLIMB
    } else {
        STAGE_LOITER
    }
}

fn advance_stage(stage: u8, inp: &AutolandModeNavInputs) -> (u8, bool) {
    match stage {
        STAGE_CLIMB if inp.climb_complete => (STAGE_LOITER, true),
        STAGE_LOITER if inp.loiter_to_alt_complete => (STAGE_LANDING, false),
        other => (other, false),
    }
}

/// Gate AUTOLAND entry, pick the first stage, and allow loiter or land
/// navigate, matching ModeAutoLand enter and navigate.
#[must_use]
pub fn autoland_mode_nav_tick(inp: &AutolandModeNavInputs) -> AutolandModeNavOutput {
    if !is_autoland_mode(inp.control_mode, &inp.features) {
        return AutolandModeNavOutput {
            started: false,
            refused: false,
            stage: 0,
            allow_loiter: false,
            allow_land: false,
            apply_level_roll: false,
            next_wp_crosstrack: false,
            wp_alt_m: 0,
            wp_dist_m: 0,
            applied: false,
        };
    }

    if inp.mode_just_entered && !enter_allowed(inp) {
        return AutolandModeNavOutput {
            started: false,
            refused: true,
            stage: inp.current_stage,
            allow_loiter: false,
            allow_land: false,
            apply_level_roll: false,
            next_wp_crosstrack: false,
            wp_alt_m: 0,
            wp_dist_m: 0,
            applied: true,
        };
    }

    let (stage, climb_handed_off) = if inp.mode_just_entered {
        (initial_stage(inp), false)
    } else {
        advance_stage(inp.current_stage, inp)
    };

    AutolandModeNavOutput {
        started: inp.mode_just_entered,
        refused: false,
        stage,
        allow_loiter: stage == STAGE_CLIMB || stage == STAGE_LOITER,
        allow_land: stage == STAGE_LANDING,
        apply_level_roll: stage == STAGE_CLIMB,
        next_wp_crosstrack: inp.mode_just_entered || climb_handed_off,
        wp_alt_m: inp.wp_alt_m,
        wp_dist_m: inp.wp_dist_m,
        applied: true,
    }
}
