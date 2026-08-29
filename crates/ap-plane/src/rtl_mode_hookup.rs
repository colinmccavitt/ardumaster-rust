//! RTL mode glue for the main vehicle loop.
//!
//! Upstream ModeRTL::_enter calls do_RTL(get_RTL_altitude_cm()) and clears
//! rtl.done_climb. ModeRTL::navigate calls update_loiter(rtl_radius) once
//! home is set, using the sign of RTL_RADIUS for loiter direction.
//! ModeRTL::update then holds LEVEL_ROLL_LIMIT until the climb-then-home
//! remaining-leg threshold is reached (`CLIMB_BEFORE_TURN` or `RTL_CLIMB_MIN`).
//! Stabilization stays on the default arm via
//! [dispatch_stabilize_from_mode](crate::mode_table_hookup::dispatch_stabilize_from_mode).
//!
//! `ModeRTL::navigate` also runs a plain `Q_RTL_MODE == SWITCH_QRTL` handoff
//! to QRTL once close enough to home for a VTOL landing
//! ([rtl_mode_switch_qrtl_tick]) behind a 1-second post-mode-change debounce.
//! This is one of three separate, mutually exclusive real upstream paths
//! that can switch RTL into QRTL: `ModeRTL::_enter`'s own three checks
//! (`QRTL_ALWAYS`, `guided_wait_takeoff_on_mode_enter`, and an
//! already-spooled-up `SWITCH_QRTL`/`VTOL_APPROACH_QRTL` within
//! `get_VTOL_return_radius()`) are already covered by [rtl_mode_nav_tick]
//! below; `navigate`'s own separate `VTOL_APPROACH_QRTL` landing-approach
//! state machine (`verify_landing_vtol_approach`) is a materially different,
//! larger, and still-unported function, not this one.

use ap_math::location::Location;
use ap_quadplane::quadplane_completeness::RtlMode;

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

/// The real 1-second post-mode-change debounce before `switch_QRTL()` is even
/// evaluated, upstream `ModeRTL::navigate` real line 92:
/// `AP_HAL::millis() - plane.last_mode_change_ms > 1000`.
const QRTL_SWITCH_DEBOUNCE_MS: u32 = 1000;

/// Inputs for the RTL-to-QRTL VTOL handoff, upstream `ModeRTL::switch_QRTL`
/// (real lines 143-166) plus its real call-site debounce in
/// `ModeRTL::navigate` (real lines 92-93).
///
/// `past_interval_finish_line` is not taken as a precomputed bool: it is
/// evaluated here by calling the already-ported
/// [`Location::past_interval_finish_line`](ap_math::location::Location::past_interval_finish_line)
/// directly against `current_loc` / `prev_wp_loc` / `next_wp_loc`, matching
/// this port's own established precedent in `ap-mission`'s `verify_nav_wp`
/// rather than reimplementing the finish-line math or asking a caller to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RtlModeSwitchQrtlInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,
    /// Upstream `Q_RTL_MODE` / `plane.quadplane.rtl_mode`. Only
    /// `RtlMode::SwitchQrtl` can trigger this handoff: `QrtlAlways` and
    /// `VtolApproachQrtl` are two different, separately-handled real code
    /// paths (see module docs), not this one.
    pub rtl_mode: RtlMode,
    /// Upstream `plane.g.rtl_radius`, metres. A real, independent value from
    /// `RtlModeNavInputs::rtl_radius_m` (same upstream parameter, but this
    /// function's own separate real read of it with its own fallback below).
    pub rtl_radius_m: i16,
    /// Upstream `plane.aparm.loiter_radius` (`WP_LOITER_RAD`), metres. Used
    /// only when `rtl_radius_m` is zero. A real, independent fallback: it is
    /// NOT the same loiter radius as `RtlModeNavOutput::loiter_radius_m`,
    /// which has no such fallback at all.
    pub loiter_radius_m: i16,
    /// Upstream `plane.nav_controller->reached_loiter_target()`, taken as an
    /// explicit bool per this port's own established convention (see
    /// `rtl_autoland_hookup.rs`, `mission_scheduler_hookup.rs`,
    /// `target_altitude.rs`) rather than modelling the nav controller's own
    /// internal state.
    pub reached_loiter_target: bool,
    /// Upstream `plane.current_loc`.
    pub current_loc: Location,
    /// Upstream `plane.prev_WP_loc`.
    pub prev_wp_loc: Location,
    /// Upstream `plane.next_WP_loc`.
    pub next_wp_loc: Location,
    /// Upstream `plane.auto_state.wp_distance`, metres.
    pub wp_distance_m: f32,
    /// Upstream `plane.quadplane.stopping_distance_m()` — itself an
    /// already-disclosed leftover elsewhere in this port
    /// (`ap-quadplane/src/position_controller.rs`, `stopping_distance_m`),
    /// taken as an explicit externally-supplied value here too rather than
    /// computed.
    pub stopping_distance_m: f32,
    /// Upstream `AP_HAL::millis() - plane.last_mode_change_ms`, already
    /// subtracted by the caller (this port's simpler alternative to passing
    /// two raw millis stamps, since the debounce here has no "never
    /// happened" sentinel to represent).
    pub millis_since_last_mode_change: u32,
}

/// Result of the RTL-to-QRTL VTOL handoff tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RtlModeSwitchQrtlOutput {
    /// The real `qrtl_radius` after the zero-fallback to `loiter_radius_m`
    /// (real lines 149-151). Zero when the debounce blocked evaluation or
    /// `rtl_mode` was not `SwitchQrtl` (real `switch_QRTL()` never ran).
    pub qrtl_radius_m: u16,
    /// True once the real 1-second call-site debounce has elapsed.
    pub debounce_elapsed: bool,
    /// `switch_QRTL()`'s own real return value: true means the caller should
    /// switch to QRTL (`ModeReason::RTL_COMPLETE_SWITCHING_TO_VTOL_LAND_RTL`).
    /// Matching this port's own established convention, this function
    /// reports the decision rather than performing the mode switch itself.
    pub switch_to_qrtl: bool,
    pub applied: bool,
}

/// Decide whether RTL should hand off to QRTL, matching `ModeRTL::navigate`'s
/// real debounce gate (real line 92) and `ModeRTL::switch_QRTL` (real lines
/// 143-166).
///
/// Gated on `is_rtl_mode` first, matching [rtl_mode_nav_tick] and
/// [rtl_mode_climb_tick]. The real call site's `plane.quadplane.available()`
/// gate one level up is not re-checked here: this function's caller is
/// already inside that same `available()` block, matching real upstream's
/// own structure (real line 76).
#[must_use]
pub fn rtl_mode_switch_qrtl_tick(inp: &RtlModeSwitchQrtlInputs) -> RtlModeSwitchQrtlOutput {
    if !is_rtl_mode(inp.control_mode, &inp.features) {
        return RtlModeSwitchQrtlOutput {
            qrtl_radius_m: 0,
            debounce_elapsed: false,
            switch_to_qrtl: false,
            applied: false,
        };
    }

    // Real short-circuit `&&`: switch_QRTL() itself is not called at all
    // until the debounce has elapsed (real line 92).
    let debounce_elapsed = inp.millis_since_last_mode_change > QRTL_SWITCH_DEBOUNCE_MS;
    if !debounce_elapsed {
        return RtlModeSwitchQrtlOutput {
            qrtl_radius_m: 0,
            debounce_elapsed: false,
            switch_to_qrtl: false,
            applied: true,
        };
    }

    if inp.rtl_mode != RtlMode::SwitchQrtl {
        return RtlModeSwitchQrtlOutput {
            qrtl_radius_m: 0,
            debounce_elapsed,
            switch_to_qrtl: false,
            applied: true,
        };
    }

    let mut qrtl_radius_m = inp.rtl_radius_m.unsigned_abs();
    if qrtl_radius_m == 0 {
        qrtl_radius_m = inp.loiter_radius_m.unsigned_abs();
    }

    // Real MAX, not MIN (real line 156): the effective gate is whichever of
    // the configured radius and the stopping distance is LARGER.
    let distance_gate = f32::from(qrtl_radius_m).max(inp.stopping_distance_m);

    let past_finish_line = inp
        .current_loc
        .past_interval_finish_line(inp.prev_wp_loc, inp.next_wp_loc);

    let switch_to_qrtl =
        inp.reached_loiter_target || past_finish_line || inp.wp_distance_m < distance_gate;

    RtlModeSwitchQrtlOutput {
        qrtl_radius_m,
        debounce_elapsed,
        switch_to_qrtl,
        applied: true,
    }
}
