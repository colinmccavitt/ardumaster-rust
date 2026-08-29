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
//!
//! [`guided_mode_offboard_tick`] ports the outer branch-selection structure
//! of `ModeGuided::update()` itself (Plane-4.7.0 `ArduPlane/mode_guided.cpp`
//! real lines 30-102): the VTOL early return, the forced-RPY/GUIDED_TIMEOUT
//! mechanism for roll and pitch, and the forced-throttle mechanism, tying
//! all three together with their real fallbacks
//! (`calc_nav_roll`/`calc_nav_pitch`/`calc_throttle`). Two real, larger
//! sub-features of that same function are deliberately deferred to separate
//! future tickets and are NOT built here:
//! - the heading-slew PID computation itself (real lines 48-71, gated
//!   `#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED`, using `g2.guidedHeading`)
//!   - this ticket only decides whether that branch's condition applies
//!   (`heading_slew_active`) and threads its already-computed result
//!   through, via [`GuidedModeOffboardTickInputs::heading_slew_nav_roll_cd`].
//! - `ModeGuided::handle_change_airspeed` (real lines 123-151) and the
//!   offboard incremental-stepping branch of `update_target_altitude` (real
//!   lines 170-202).

use ap_math::scalar::constrain_int32;
use ap_servo::function::Function;

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

/// True when `now_ms - last_ms` is strictly younger than `timeout_s` seconds
/// — upstream's repeated `millis() - guided_state.last_forced_*_ms <
/// g2.guided_timeout*1000.0f` shape (real lines 40, 79, 96).
///
/// Upstream subtracts a `uint32_t millis()` from Vector3l's `int32_t`
/// timestamp components (`last_forced_rpy_ms.x`/`.y`); C++'s usual
/// arithmetic conversions promote the `int32_t` operand to `uint32_t` for
/// that subtraction (same rank, unsigned wins), so the result already has
/// the same wraparound behaviour as an unsigned subtraction. `wrapping_sub`
/// on `u32` reproduces that directly.
fn within_guided_timeout(now_ms: u32, last_ms: u32, timeout_s: f32) -> bool {
    #[allow(
        clippy::cast_precision_loss,
        reason = "upstream promotes the uint32 age to float against g2.guided_timeout*1000.0f"
    )]
    let age_ms = now_ms.wrapping_sub(last_ms) as f32;
    age_ms < timeout_s * 1000.0
}

/// Which real source drove GUIDED's `nav_roll_cd` this tick, upstream
/// `ModeGuided::update`'s roll three-way (real lines 39-76).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidedRollSource {
    /// Forced RPY within `g2.guided_timeout` (real lines 39-44).
    ForcedRpy,
    /// The deferred heading-slew PID branch (real lines 48-71,
    /// `#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED`) — its own applicability
    /// and result are supplied externally, not computed here.
    HeadingSlew,
    /// `calc_nav_roll()` fallback (real line 75).
    CalcNavRoll,
}

/// Which real source drove GUIDED's `nav_pitch_cd` this tick, upstream
/// `ModeGuided::update`'s pitch two-way (real lines 78-83). No heading-slew
/// equivalent exists for pitch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidedPitchSource {
    /// Forced RPY within `g2.guided_timeout` (real lines 78-80).
    ForcedRpy,
    /// `calc_nav_pitch()` fallback (real line 82).
    CalcNavPitch,
}

/// Which real source drove the `k_throttle` servo write this tick, upstream
/// `ModeGuided::update`'s throttle three-way (real lines 85-100).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GuidedThrottleSource {
    /// Manual RC passthrough during a fence breach (real lines 87-89) —
    /// unconditional, independent of `g2.guided_timeout`.
    Passthrough,
    /// Forced throttle within `g2.guided_timeout`, gated on
    /// `aparm.throttle_cruise > 1` (real lines 91-95).
    ForcedThrottle,
    /// `calc_throttle()` (TECS) fallback (real line 99).
    CalcThrottle,
}

/// Inputs for `ModeGuided::update()`'s own outer branch-selection structure
/// (real lines 30-102): the VTOL early return, and the forced-RPY /
/// forced-throttle `GUIDED_TIMEOUT` mechanism for roll, pitch, and throttle.
///
/// `calc_nav_roll_cd`/`calc_nav_pitch_cd`/`calc_throttle`/
/// `heading_slew_nav_roll_cd` are Plane's own separate, larger real
/// computations (`calc_nav_roll`, `calc_nav_pitch`, `calc_throttle`, and the
/// deferred heading-slew PID) — taken as external, already-computed values
/// per this ticket's own scope boundary, not implemented here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedModeOffboardTickInputs {
    pub control_mode: u8,
    pub features: BuildFeatures,

    /// `auto_state.vtol_loiter && quadplane.available()` (real lines 32-37).
    /// When true, the whole tick delegates to `quadplane.guided_update()`
    /// (out of scope) and nothing below applies.
    pub vtol_loiter_active: bool,

    /// `millis()` this tick.
    pub now_ms: u32,

    /// `guided_state.forced_rpy_cd.x`, upstream `Vector3l` (centidegrees).
    pub forced_roll_cd: i32,
    /// `guided_state.last_forced_rpy_ms.x`, upstream `Vector3l` component.
    /// `> 0` means "we have ever heard a forced-roll message" — a real,
    /// separate timestamp from `last_forced_pitch_ms`, not shared.
    pub last_forced_roll_ms: i32,
    /// `guided_state.forced_rpy_cd.y` (centidegrees).
    pub forced_pitch_cd: i32,
    /// `guided_state.last_forced_rpy_ms.y` — real, separate timestamp from
    /// `last_forced_roll_ms`.
    pub last_forced_pitch_ms: i32,

    /// `guided_state.forced_throttle`, upstream `float` (percent).
    pub forced_throttle: f32,
    /// `guided_state.last_forced_throttle_ms`, upstream plain `uint32_t` —
    /// NOT a `Vector3l` component, unlike the roll/pitch timestamps above.
    pub last_forced_throttle_ms: u32,

    /// `g2.guided_timeout`, seconds (`Parameters.h` ~line 571, `AP_Float`).
    /// The same window is reused for roll, pitch, and throttle — only the
    /// timestamp compared against it differs per axis.
    pub guided_timeout_s: f32,

    /// `plane.roll_limit_cd` — roll's clamp is symmetric, `+-roll_limit_cd`.
    pub roll_limit_cd: i32,
    /// `plane.pitch_limit_min*100` — pitch's clamp is independently
    /// configured, NOT a mirror of roll's symmetric range.
    pub pitch_limit_min_cd: i32,
    /// `aparm.pitch_limit_max.get()*100`.
    pub pitch_limit_max_cd: i32,

    /// `#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED`. This port's
    /// `BuildFeatures` has no variant for this compile-time feature yet;
    /// taken as a plain bool since only this ticket currently needs it,
    /// rather than adding a new `BuildFeatures` field for a single caller.
    pub offboard_slew_enabled: bool,
    /// `(control_mode == &mode_guided) && (target_heading_type !=
    /// GUIDED_HEADING_NONE)` — the heading-slew branch's own applicability,
    /// already evaluated by the caller (deferred future ticket).
    pub heading_slew_active: bool,
    /// The deferred heading-slew PID's own already-computed `nav_roll_cd`
    /// result (real line 71's `constrain_int32(desired, -bank_limit,
    /// bank_limit)`), supplied externally rather than computed here.
    pub heading_slew_nav_roll_cd: i32,

    /// `calc_nav_roll()` fallback result — Plane's own general nav-roll
    /// computation, out of this ticket's scope.
    pub calc_nav_roll_cd: i32,
    /// `calc_nav_pitch()` fallback result, out of scope.
    pub calc_nav_pitch_cd: i32,
    /// `calc_throttle()` (TECS) fallback result, out of scope.
    pub calc_throttle: f32,

    /// `plane.guided_throttle_passthru`.
    pub guided_throttle_passthru: bool,
    /// `get_throttle_input(true)` — the manual passthrough value.
    pub throttle_passthru_input: f32,
    /// `aparm.throttle_cruise`. A vehicle with `throttle_cruise <= 1` can
    /// never take the forced-throttle branch, regardless of how recent the
    /// forced-throttle message was — a real, easy-to-miss third gate.
    pub throttle_cruise: f32,
}

/// Result of `ModeGuided::update()`'s outer branch selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedModeOffboardTickOutput {
    /// The VTOL early return fired (real lines 32-37) — when true, none of
    /// the fields below carry a meaningful decision; the caller should have
    /// delegated to `quadplane.guided_update()` and returned already.
    pub vtol_early_return: bool,

    /// Which of the three roll sources won.
    pub roll_source: GuidedRollSource,
    /// The resulting `nav_roll_cd`.
    pub nav_roll_cd: i32,
    /// `update_load_factor()` should run this tick. Real asymmetry: true
    /// for `ForcedRpy` and `HeadingSlew`, false for `CalcNavRoll` — the
    /// plain fallback never calls it (real lines 43 and 71 vs. line 75).
    pub update_load_factor: bool,

    /// Which of the two pitch sources won.
    pub pitch_source: GuidedPitchSource,
    /// The resulting `nav_pitch_cd`.
    pub nav_pitch_cd: i32,

    /// Which of the three throttle sources won.
    pub throttle_source: GuidedThrottleSource,
    /// `(Function::THROTTLE, value)` — the real `SRV_Channels::
    /// set_output_scaled(SRV_Channel::k_throttle, value)` call all three
    /// throttle branches make, with the winning source's value. Reuses
    /// `ap-servo`'s `Function` (FW-042's `output_rudder_and_steering`
    /// convention) rather than a bare `f32`, keeping the channel identity
    /// attached to the value.
    pub throttle: (Function, f32),

    pub applied: bool,
}

/// Resolve `ModeGuided::update()`'s outer branch structure: the VTOL early
/// return, then the forced-RPY/forced-throttle `GUIDED_TIMEOUT` mechanism
/// for roll, pitch, and throttle (Plane-4.7.0 `ArduPlane/mode_guided.cpp`
/// real lines 30-102). Does not perform any I/O — the caller applies
/// `update_load_factor()` and the `k_throttle` servo write itself.
#[must_use]
pub fn guided_mode_offboard_tick(
    inp: &GuidedModeOffboardTickInputs,
) -> GuidedModeOffboardTickOutput {
    let neutral = GuidedModeOffboardTickOutput {
        vtol_early_return: false,
        roll_source: GuidedRollSource::CalcNavRoll,
        nav_roll_cd: 0,
        update_load_factor: false,
        pitch_source: GuidedPitchSource::CalcNavPitch,
        nav_pitch_cd: 0,
        throttle_source: GuidedThrottleSource::CalcThrottle,
        throttle: (Function::THROTTLE, 0.0),
        applied: false,
    };

    if !is_guided_mode(inp.control_mode, &inp.features) {
        return neutral;
    }

    if inp.vtol_loiter_active {
        return GuidedModeOffboardTickOutput {
            vtol_early_return: true,
            applied: true,
            ..neutral
        };
    }

    // Roll: forced RPY, else the deferred heading-slew branch, else
    // calc_nav_roll(). Only the first two call update_load_factor().
    let (roll_source, nav_roll_cd, update_load_factor) = if inp.last_forced_roll_ms > 0
        && within_guided_timeout(
            inp.now_ms,
            inp.last_forced_roll_ms as u32,
            inp.guided_timeout_s,
        ) {
        let nav_roll_cd =
            constrain_int32(inp.forced_roll_cd, -inp.roll_limit_cd, inp.roll_limit_cd);
        (GuidedRollSource::ForcedRpy, nav_roll_cd, true)
    } else if inp.offboard_slew_enabled && inp.heading_slew_active {
        (
            GuidedRollSource::HeadingSlew,
            inp.heading_slew_nav_roll_cd,
            true,
        )
    } else {
        (GuidedRollSource::CalcNavRoll, inp.calc_nav_roll_cd, false)
    };

    // Pitch: forced RPY (independent .y timestamp, asymmetric clamp), else
    // calc_nav_pitch(). update_load_factor() is never associated with pitch.
    let (pitch_source, nav_pitch_cd) = if inp.last_forced_pitch_ms > 0
        && within_guided_timeout(
            inp.now_ms,
            inp.last_forced_pitch_ms as u32,
            inp.guided_timeout_s,
        ) {
        let nav_pitch_cd = constrain_int32(
            inp.forced_pitch_cd,
            inp.pitch_limit_min_cd,
            inp.pitch_limit_max_cd,
        );
        (GuidedPitchSource::ForcedRpy, nav_pitch_cd)
    } else {
        (GuidedPitchSource::CalcNavPitch, inp.calc_nav_pitch_cd)
    };

    // Throttle: manual passthrough (unconditional), else forced throttle
    // (gated on throttle_cruise > 1 AND the timeout window), else
    // calc_throttle() (TECS).
    let (throttle_source, throttle_val) = if inp.guided_throttle_passthru {
        (
            GuidedThrottleSource::Passthrough,
            inp.throttle_passthru_input,
        )
    } else if inp.throttle_cruise > 1.0
        && inp.last_forced_throttle_ms > 0
        && within_guided_timeout(
            inp.now_ms,
            inp.last_forced_throttle_ms,
            inp.guided_timeout_s,
        )
    {
        (GuidedThrottleSource::ForcedThrottle, inp.forced_throttle)
    } else {
        (GuidedThrottleSource::CalcThrottle, inp.calc_throttle)
    };

    GuidedModeOffboardTickOutput {
        vtol_early_return: false,
        roll_source,
        nav_roll_cd,
        update_load_factor,
        pitch_source,
        nav_pitch_cd,
        throttle_source,
        throttle: (Function::THROTTLE, throttle_val),
        applied: true,
    }
}
