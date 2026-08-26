//! Where a mode's target altitude comes from, upstream
//! `ArduPlane/mode.cpp:189`, `Mode::update_target_altitude`.
//!
//! Eight branches, six outcomes. The ordering is the content: each branch is
//! a claim that some source of altitude outranks the ones below it, and the
//! aircraft flies whichever wins.

/// What the target altitude is set from.
///
/// # Why six variants for eight branches
///
/// Three of upstream's branches — the landing flare, having reached a loiter
/// target, and the fall-through — all do exactly the same thing: set the
/// target from the next waypoint. They are separate upstream because they are
/// separate *reasons*, and reading them merged would hide that. They are
/// merged here because a port reproduces what the vehicle does, and nothing
/// downstream can tell them apart.
///
/// That is a deliberate choice rather than an oversight, and it is why the
/// parity recording compares actions rather than branch identities: a
/// recording of branch numbers would be pinning a distinction the firmware
/// does not make.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetAltitude {
    /// Set from the next waypoint. Upstream's flare, loiter-reached and
    /// fall-through branches.
    FromNextWaypoint,
    /// Set up a landing glide slope from the previous to the next waypoint,
    /// then adjust it for any rangefinder bump.
    LandingGlideSlope,
    /// Set from the location the landing controller nominates.
    FromLandingTarget,
    /// Set from the current location, and reset the altitude offset. Soaring
    /// only.
    HoldCurrentAndResetOffset,
    /// Terrain-relative, applied inside
    /// `set_target_altitude_proportion_terrain`.
    TerrainProportion,
    /// A proportion of the way between the previous and next waypoints, then
    /// constrained to lie between their altitudes.
    ProportionalToNextWaypoint,
}

/// Everything the ladder reads, except the terrain attempt.
///
/// The terrain branch is not here because it cannot be: its condition has a
/// side effect. See [`target_altitude`].
#[derive(Debug, Clone, Copy)]
pub struct TargetAltitudeInputs {
    /// `landing.is_flaring()`.
    pub landing_is_flaring: bool,
    /// `landing.is_on_approach()`.
    pub landing_is_on_approach: bool,
    /// `landing.get_target_altitude_location(...)` returned a location.
    pub landing_has_target_location: bool,
    /// The soaring controller is active *and* has suppressed the throttle.
    /// Both are required: an active soaring controller that is still using
    /// the motor has not started gliding yet.
    pub soaring_gliding: bool,
    /// `reached_loiter_target()`.
    pub reached_loiter_target: bool,
    /// The next waypoint's altitude is terrain-relative.
    pub next_wp_is_terrain_alt: bool,
    /// `target_altitude.offset_cm`.
    pub offset_cm: i32,
    /// `current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)`.
    pub past_interval_finish_line: bool,
}

/// Which source wins, upstream `Mode::update_target_altitude`.
///
/// # The terrain attempt is a closure, not a flag
///
/// Upstream's terrain branch reads
/// `next_WP_loc.terrain_alt && set_target_altitude_proportion_terrain()`, and
/// the right-hand side is a function that *sets the target altitude* as well
/// as reporting whether it could. C's short-circuit means it only runs when
/// the waypoint is terrain-relative and every branch above has declined.
///
/// A `bool` parameter would lose that. A caller computing it eagerly would
/// have the target altitude written on rows where upstream never touched it,
/// and the bug would be invisible from here. Taking a closure puts the
/// short-circuit in the signature, so it cannot be called at the wrong time.
///
/// # The ordering, from the top
///
/// A flare outranks everything: the aircraft is metres from the ground with
/// its nose up, and no other source is worth consulting. An approach comes
/// next because a glide slope is a plan the landing controller is already
/// flying. Then any other altitude the landing controller nominates.
///
/// Soaring sits below all three and above the rest, and only while it is
/// actually gliding: it holds the target at the current altitude so that a
/// long glide does not accumulate an altitude error the controller would try
/// to fly out of.
///
/// A reached loiter target locks to the final altitude. Below that,
/// terrain-relative waypoints and the proportional climb are two ways of
/// spreading an altitude change along a leg, and the plain waypoint is what
/// is left when none of it applies.
#[must_use]
pub fn target_altitude(
    inputs: &TargetAltitudeInputs,
    try_terrain_proportion: impl FnOnce() -> bool,
) -> TargetAltitude {
    if inputs.landing_is_flaring {
        // TECS_LAND_SINK becomes the target sink rate and the target altitude
        // is ignored, but the location is still set.
        return TargetAltitude::FromNextWaypoint;
    }

    if inputs.landing_is_on_approach {
        return TargetAltitude::LandingGlideSlope;
    }

    if inputs.landing_has_target_location {
        return TargetAltitude::FromLandingTarget;
    }

    if inputs.soaring_gliding {
        return TargetAltitude::HoldCurrentAndResetOffset;
    }

    if inputs.reached_loiter_target {
        return TargetAltitude::FromNextWaypoint;
    }

    // Only now, and only if the waypoint is terrain-relative. The call writes
    // the target altitude when it succeeds, which is why it is not evaluated
    // above this line.
    if inputs.next_wp_is_terrain_alt && try_terrain_proportion() {
        return TargetAltitude::TerrainProportion;
    }

    // Climb or descend across the leg, but only while still short of the
    // finish line. Past it, holding the proportional target would keep
    // commanding a climb the aircraft has already completed.
    if inputs.offset_cm != 0 && !inputs.past_interval_finish_line {
        return TargetAltitude::ProportionalToNextWaypoint;
    }

    TargetAltitude::FromNextWaypoint
}
