//! RTL mode glue for the main vehicle loop.
//!
//! Upstream ModeRTL::_enter calls do_RTL(get_RTL_altitude_cm()) and clears
//! rtl.done_climb. ModeRTL::navigate calls update_loiter(rtl_radius) once
//! home is set, using the sign of RTL_RADIUS for loiter direction.
//! ModeRTL::update then holds LEVEL_ROLL_LIMIT until the climb-then-home
//! remaining-leg threshold is reached (`CLIMB_BEFORE_TURN` or `RTL_CLIMB_MIN`).
//! Stabilization stays on the default arm via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).

use crate::mode_table::{BuildFeatures, ModeNumber};

fn is_rtl_mode(control_mode: u8, features: &BuildFeatures) -> bool {
    ModeNumber::from_number(control_mode, features) == Some(ModeNumber::Rtl)
}

/// Inputs for RTL enter plus navigate (ModeRTL::_enter and navigate).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlModeNavInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// True when Mode::enter just ran this tick.
    pub mode_just_entered: bool,
    /// Upstream AP::ahrs().home_is_set().
    pub home_is_set: bool,
    /// Upstream RTL_RADIUS, metres. Negative is CCW; zero uses WP_LOITER_RAD.
    pub rtl_radius_m: i16,
}

/// Result of the RTL enter / navigate tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlModeNavOutput {
    /// do_RTL armed the return this tick.
    pub started: bool,
    /// navigate will call update_loiter this tick.
    pub allow_loiter: bool,
    /// abs(RTL_RADIUS); zero means use WP_LOITER_RAD.
    pub loiter_radius_m: u16,
    /// RTL_RADIUS < 0 selects counterclockwise loiter.
    pub loiter_ccw: bool,
    /// True when RTL_RADIUS is non-zero and direction should be applied.
    pub direction_set: bool,
    pub applied: bool,
}

/// Start the home return on RTL entry and gate update_loiter() on home,
/// matching ModeRTL enter and navigate.
#[must_use]
pub fn rtl_mode_nav_tick(inp: &RtlModeNavInputs) -> RtlModeNavOutput {
    if !is_rtl_mode(inp.control_mode, &inp.features) {
        return RtlModeNavOutput {
            started: false,
            allow_loiter: false,
            loiter_radius_m: 0,
            loiter_ccw: false,
            direction_set: false,
            applied: false,
        };
    }

    let loiter_radius_m = inp.rtl_radius_m.unsigned_abs();
    let direction_set = loiter_radius_m > 0;
    let loiter_ccw = direction_set && inp.rtl_radius_m < 0;

    RtlModeNavOutput {
        started: inp.mode_just_entered,
        allow_loiter: inp.home_is_set,
        loiter_radius_m,
        loiter_ccw,
        direction_set,
        applied: true,
    }
}

/// Inputs for RTL climb-then-home remaining-leg (ModeRTL::update).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlModeClimbInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// Upstream `rtl.done_climb` from the last tick (cleared on enter).
    pub done_climb: bool,
    /// Upstream FlightOptions::CLIMB_BEFORE_TURN. Overrides RTL_CLIMB_MIN.
    pub climb_before_turn: bool,
    /// Upstream `g2.rtl_climb_min`, metres. Zero disables the climb-min gate.
    pub rtl_climb_min_m: u16,
    /// Upstream `current_loc.alt`, centimetres.
    pub current_alt_cm: i32,
    /// Upstream `next_WP_loc.alt` (RTL altitude), centimetres.
    pub next_wp_alt_cm: i32,
    /// Upstream `prev_WP_loc.alt` (altitude when RTL entered), centimetres.
    pub prev_wp_alt_cm: i32,
}

/// Result of the RTL climb-then-home remaining-leg tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlModeClimbOutput {
    /// Climb gate is enabled this tick (`CLIMB_BEFORE_TURN` or RTL_CLIMB_MIN).
    pub climb_gated: bool,
    /// Updated `rtl.done_climb`.
    pub done_climb: bool,
    /// Still climbing: constrain roll to LEVEL_ROLL_LIMIT.
    pub constrain_roll: bool,
    /// Climb completed this tick: `prev_WP = current`, `setup_alt_slope`.
    pub setup_remaining_leg: bool,
    pub applied: bool,
}

/// Hold a wings-level climb until the RTL altitude or RTL_CLIMB_MIN threshold,
/// then start the remaining home-bound leg, matching ModeRTL::update.
#[must_use]
pub fn rtl_mode_climb_tick(inp: &RtlModeClimbInputs) -> RtlModeClimbOutput {
    if !is_rtl_mode(inp.control_mode, &inp.features) {
        return RtlModeClimbOutput {
            climb_gated: false,
            done_climb: false,
            constrain_roll: false,
            setup_remaining_leg: false,
            applied: false,
        };
    }

    let alt_threshold_reached = if inp.climb_before_turn {
        inp.current_alt_cm > inp.next_wp_alt_cm
    } else if inp.rtl_climb_min_m > 0 {
        (inp.current_alt_cm - inp.prev_wp_alt_cm) > i32::from(inp.rtl_climb_min_m) * 100
    } else {
        return RtlModeClimbOutput {
            climb_gated: false,
            done_climb: inp.done_climb,
            constrain_roll: false,
            setup_remaining_leg: false,
            applied: true,
        };
    };

    let setup_remaining_leg = !inp.done_climb && alt_threshold_reached;
    let done_climb = inp.done_climb || setup_remaining_leg;

    RtlModeClimbOutput {
        climb_gated: true,
        done_climb,
        constrain_roll: !done_climb,
        setup_remaining_leg,
        applied: true,
    }
}
