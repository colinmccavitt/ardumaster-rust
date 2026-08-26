//! Pilot control during a landing, upstream `ArduCopter/mode.cpp:771`,
//! `Mode::land_run_horizontal_control`.
//!
//! A descent is the one manoeuvre where the pilot is most likely to want to
//! intervene and least able to afford a controller fighting them. This is the
//! part of the landing that decides how much of the aircraft they get.

/// Throttle above which a raised stick cancels the landing, upstream
/// `LAND_CANCEL_TRIGGER_THR` (`config.h:331`).
///
/// In `control_in` units, so 700 of a 0..1000 range — comfortably above any
/// throttle a pilot would be holding while watching an automatic descent, and
/// below full travel so it does not need a slammed stick.
pub const LAND_CANCEL_TRIGGER_THR: f32 = 700.0;

/// `THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND`, bit 1 of `THR_BEHAVE`
/// (`defines.h:146`).
pub const THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND: i32 = 1 << 1;

/// Whether raising the throttle should abandon the landing, upstream the
/// first branch of `land_run_horizontal_control`.
///
/// # It is opt-in
///
/// The behaviour is behind a `THR_BEHAVE` bit rather than always on, because
/// a pilot who rests a hand on the throttle during an automatic descent
/// should not thereby abort it. Operators who want the escape hatch ask for
/// it.
///
/// # The throttle is the filtered one
///
/// Upstream reads `rc_throttle_control_in_filter`, not the raw stick. A
/// single noisy sample above the threshold would otherwise abandon a landing,
/// and a landing abandoned by accident puts the aircraft back in the air with
/// a pilot who was not expecting to be flying.
#[must_use]
pub fn land_cancelled_by_throttle(
    throttle_behavior: i32,
    filtered_throttle_control_in: f32,
    has_valid_input: bool,
) -> bool {
    if !has_valid_input {
        return false;
    }
    (throttle_behavior & THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND) != 0
        && filtered_throttle_control_in > LAND_CANCEL_TRIGGER_THR
}

/// Where a cancelled landing goes.
///
/// Upstream tries `LOITER` and falls back to `ALT_HOLD` if that mode change
/// is refused. The order is the useful one: `LOITER` holds position as well
/// as height, so a pilot who has just grabbed the aircraft gets it stopped
/// rather than drifting. `ALT_HOLD` needs no position estimate, which is the
/// likeliest reason `LOITER` would refuse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandCancelDestination {
    /// First choice.
    Loiter,
    /// Taken when Loiter refuses.
    AltHold,
}

/// Upstream's fallback order for a cancelled landing.
#[must_use]
pub fn land_cancel_destination(loiter_accepted: bool) -> LandCancelDestination {
    if loiter_accepted {
        LandCancelDestination::Loiter
    } else {
        LandCancelDestination::AltHold
    }
}

/// The pilot's maximum repositioning speed, upstream
/// `wp_nav->get_wp_acceleration_mss() * 0.5`.
///
/// Upstream's comment gives the reasoning: half the waypoint acceleration as
/// a velocity means the aircraft can stop from full repositioning speed in
/// under a second. A pilot nudging a descending aircraft sideways needs it to
/// stop when they let go, not to coast on over whatever they were avoiding.
#[must_use]
pub fn max_pilot_reposition_speed_ms(wp_acceleration_mss: f32) -> f32 {
    wp_acceleration_mss * 0.5
}

/// What the pilot's repositioning input does to the landing's state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RepositionState {
    /// The pilot is moving the aircraft. Precision landing must not fight
    /// them.
    PilotRepositioning,
    /// The pilot has let go and precision landing is allowed to resume.
    ReleasedToPrecland,
    /// No change: either the pilot is not repositioning, or they have let go
    /// but the operator has not allowed precision landing to resume.
    Unchanged,
}

/// Upstream's `land_repo_active` update.
///
/// # Letting go is not automatically giving back
///
/// Once the pilot has repositioned, `land_repo_active` stays set even after
/// they release the sticks, unless `PLND_OPTION_PRECLAND_AFTER_REPOSITION`
/// says otherwise. The default is that a pilot who has intervened has taken
/// the landing, and the precision-landing target — which they presumably
/// moved away from on purpose — does not get to pull the aircraft back.
/// Operators who want it to resume opt in.
#[must_use]
pub fn reposition_state(
    land_repositioning_enabled: bool,
    has_valid_input: bool,
    pilot_velocity_is_zero: bool,
    allow_precland_after_reposition: bool,
) -> RepositionState {
    if !has_valid_input || !land_repositioning_enabled {
        return RepositionState::Unchanged;
    }
    if !pilot_velocity_is_zero {
        return RepositionState::PilotRepositioning;
    }
    if allow_precland_after_reposition {
        return RepositionState::ReleasedToPrecland;
    }
    RepositionState::Unchanged
}

/// Whether precision landing drives the horizontal controller this iteration,
/// upstream `copter.ap.prec_land_active`.
///
/// Both conditions are required, and the pilot's takes precedence: a
/// repositioning pilot beats an acquired target, because the target does not
/// know why they moved.
#[must_use]
pub fn precision_landing_active(land_repo_active: bool, target_acquired: bool) -> bool {
    !land_repo_active && target_acquired
}

/// What the horizontal landing controller is driven by this iteration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandHorizontalInput {
    /// A position and velocity from the precision-landing target.
    PrecisionTarget,
    /// The pilot's repositioning velocity, or zero if they are not asking for
    /// one.
    VelocityCorrection,
}

/// Which of the two inputs the position controller receives.
///
/// Exactly one of them runs each iteration — upstream's second block is
/// guarded by `if (!copter.ap.prec_land_active)`, so this is a choice rather
/// than two things that might both happen.
#[must_use]
pub fn land_horizontal_input(prec_land_active: bool) -> LandHorizontalInput {
    if prec_land_active {
        LandHorizontalInput::PrecisionTarget
    } else {
        LandHorizontalInput::VelocityCorrection
    }
}
