//! GUIDED offboard airspeed ramp and altitude stepping.
//!
//! Ports Plane-4.7.0 `ModeGuided::handle_change_airspeed` (real lines 123-151),
//! `Plane::GuidedState::target_location_alt_is_minus_one` (real lines 162-167),
//! and the offboard incremental-stepping branch of
//! `ModeGuided::update_target_altitude` (real lines 170-202), gated
//! `#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED`.
//!
//! The roll/pitch/throttle selector (FW-043) and heading-slew PID (FW-044)
//! stay in their own modules. `Plane::set_target_altitude_location` is not
//! implemented here — the offboard path returns that call as an action.

use ap_math::location::{AltContext, Location};
use ap_math::scalar::{constrain_int32, is_equal, is_zero};

/// Inputs for [`handle_change_airspeed`] (real lines 123-151).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HandleChangeAirspeedInputs {
    /// Demanded airspeed, m/s (`handle_change_airspeed`'s first argument).
    pub airspeed: f32,
    /// Demanded acceleration, m/s². Zero means "as fast as we can" (1000).
    pub acceleration: f32,
    /// `plane.aparm.airspeed_min`.
    pub airspeed_min: f32,
    /// `plane.aparm.airspeed_max`.
    pub airspeed_max: f32,
    /// `guided_state.target_airspeed_cm` before this call.
    pub guided_target_airspeed_cm: f32,
    /// `plane.target_airspeed_cm` — the vehicle's *current* TECS target, not
    /// the new guided demand. Upstream is `int32_t`; C++ promotes it to
    /// `float` for the sign comparison.
    pub plane_target_airspeed_cm: f32,
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
}

/// Guided airspeed-ramp fields written when the demand actually changes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GuidedAirspeedRamp {
    /// `guided_state.target_airspeed_cm` (`airspeed * 100`).
    pub target_airspeed_cm: f32,
    /// `guided_state.target_airspeed_time_ms` (`now_ms`).
    pub target_airspeed_time_ms: u32,
    /// `guided_state.target_airspeed_accel`, signed toward the new target
    /// relative to `plane.target_airspeed_cm`.
    pub target_airspeed_accel: f32,
}

/// Result of [`handle_change_airspeed`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HandleChangeAirspeedOutput {
    /// Function return value. Envelope reject is `false`; same-target and a
    /// newly stored ramp are both `true`.
    pub accepted: bool,
    /// `Some` only when the guided target changed. Same-target early return
    /// leaves the existing accel/time alone (`None`).
    pub ramp: Option<GuidedAirspeedRamp>,
}

/// Upstream `ModeGuided::handle_change_airspeed`.
///
/// Rejects outside `[airspeed_min, airspeed_max]`. A demand whose centimetre
/// target `is_equal`s the stored guided target is a no-op (`true`, no ramp
/// write). Otherwise stores the new cm target and timestamp, takes 1000 for
/// a zero accel or `fabsf` otherwise, and flips the sign when the new guided
/// target is below the *current* vehicle `target_airspeed_cm`.
#[must_use]
pub fn handle_change_airspeed(inp: &HandleChangeAirspeedInputs) -> HandleChangeAirspeedOutput {
    if inp.airspeed > inp.airspeed_max || inp.airspeed < inp.airspeed_min {
        return HandleChangeAirspeedOutput {
            accepted: false,
            ramp: None,
        };
    }

    let new_target_airspeed_cm = inp.airspeed * 100.0;
    if is_equal(new_target_airspeed_cm, inp.guided_target_airspeed_cm) {
        return HandleChangeAirspeedOutput {
            accepted: true,
            ramp: None,
        };
    }

    let mut target_airspeed_accel = if is_zero(inp.acceleration) {
        1000.0
    } else {
        inp.acceleration.abs()
    };

    if new_target_airspeed_cm < inp.plane_target_airspeed_cm {
        target_airspeed_accel *= -1.0;
    }

    HandleChangeAirspeedOutput {
        accepted: true,
        ramp: Some(GuidedAirspeedRamp {
            target_airspeed_cm: new_target_airspeed_cm,
            target_airspeed_time_ms: inp.now_ms,
            target_airspeed_accel,
        }),
    }
}

/// Upstream `Plane::GuidedState::target_location_alt_is_minus_one`.
///
/// Reads `get_alt_cm` in the location's *own* alt frame — not an assumed
/// frame. `true` iff that altitude is `-1`. A failed conversion leaves the
/// C++ out-param at 0, so this returns `false`.
#[must_use]
pub fn target_location_alt_is_minus_one(target_location: &Location, ctx: &AltContext) -> bool {
    match target_location.get_alt_cm(target_location.alt_frame(), ctx) {
        Some(alt_cm) => alt_cm == -1,
        None => false,
    }
}

/// Inputs for [`update_target_altitude`] (real lines 170-202).
#[derive(Debug, Clone, Copy)]
pub struct GuidedUpdateTargetAltitudeInputs {
    /// `AP_HAL::millis()`.
    pub now_ms: u32,
    /// `guided_state.target_alt_time_ms` before this tick.
    pub target_alt_time_ms: u32,
    /// `guided_state.target_alt_rate`, m/s (always stored positive upstream).
    pub target_alt_rate: f32,
    /// `guided_state.target_location`.
    pub target_location: Location,
    /// `plane.current_loc`.
    pub current_loc: Location,
    /// Home / origin / terrain needed to express `current_loc` in the
    /// target's own alt frame.
    pub alt_ctx: AltContext,
}

/// Outcome of [`update_target_altitude`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GuidedUpdateTargetAltitude {
    /// Offboard incremental-stepping branch. Time is consumed even when no
    /// location is produced this tick.
    Offboard {
        /// `set_target_altitude_location(temp_location)` — `None` when
        /// current/target are uninitialised or the frame conversion fails.
        set_target_altitude_location: Option<Location>,
        /// Written back to `guided_state.target_alt_time_ms`.
        target_alt_time_ms: u32,
    },
    /// Fall through to `Mode::update_target_altitude()`.
    UseBaseMode,
}

/// Upstream `ModeGuided::update_target_altitude` offboard branch + fall-through.
///
/// Enters the offboard path when `target_alt_time_ms != 0` **or** the target
/// alt is not `-1` (defaults: alt `-1`, time `0` →
/// [`GuidedUpdateTargetAltitude::UseBaseMode`]). Steps
/// `delta_amt_i = (int32_t)(100.0 * delta * target_alt_rate)` in the target
/// location's own alt frame and constrains `target_location.alt` toward the
/// current altitude in that same frame.
#[must_use]
pub fn update_target_altitude(
    inp: &GuidedUpdateTargetAltitudeInputs,
) -> GuidedUpdateTargetAltitude {
    if inp.target_alt_time_ms == 0
        && target_location_alt_is_minus_one(&inp.target_location, &inp.alt_ctx)
    {
        return GuidedUpdateTargetAltitude::UseBaseMode;
    }

    #[allow(
        clippy::cast_precision_loss,
        reason = "upstream `1e-3f * (now - target_alt_time_ms)` promotes uint32 to float"
    )]
    let delta = 1e-3_f32 * inp.now_ms.wrapping_sub(inp.target_alt_time_ms) as f32;
    let delta_amt_f = delta * inp.target_alt_rate;
    // C++ `(int32_t)(100.0 * delta_amt_f)`: 100.0 is double.
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream truncates the double product toward zero into int32_t"
    )]
    let delta_amt_i = (100.0_f64 * f64::from(delta_amt_f)) as i32;

    let target_frame = inp.target_location.alt_frame();
    let set_target_altitude_location =
        if inp.current_loc.initialised() && inp.target_location.initialised() {
            inp.current_loc
                .get_alt_cm(target_frame, &inp.alt_ctx)
                .map(|target_alt_previous_cm| {
                    let temp_alt_cm = constrain_int32(
                        inp.target_location.alt,
                        target_alt_previous_cm - delta_amt_i,
                        target_alt_previous_cm + delta_amt_i,
                    );
                    let mut temp_location = inp.target_location;
                    temp_location.set_alt_cm(temp_alt_cm, target_frame);
                    temp_location
                })
        } else {
            None
        };

    GuidedUpdateTargetAltitude::Offboard {
        set_target_altitude_location,
        target_alt_time_ms: inp.now_ms,
    }
}
