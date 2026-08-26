//! Go-around requests and landing-type dispatch, upstream `AP_Landing.cpp`
//! `request_go_around`, `override_servos`, and the slope abort latch.

/// Which landing type is active, upstream `AP_Landing::LandingType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandingType {
    /// Standard glide-slope landing.
    StandardGlideSlope,
    /// Deepstall — not ported in this crate yet.
    Deepstall,
}

/// Landing flags the vehicle carries, upstream `AP_Landing::Flags`.
#[derive(Debug, Clone, Copy, Default)]
pub struct LandingFlags {
    /// A landing sequence is running.
    pub in_progress: bool,
    /// The pilot or logic has commanded a go-around.
    pub commanded_go_around: bool,
}

/// Slope-landing-specific latch state, upstream `type_slope_flags`.
#[derive(Debug, Clone, Copy, Default)]
pub struct SlopeLandingFlags {
    /// Rangefinder slope recalculation already triggered one go-around.
    pub has_aborted_due_to_slope_recalc: bool,
    /// Barometric altitude offset to carry into the next approach.
    pub alt_offset: f32,
}

/// Whether the landing library overrides servos, upstream
/// `AP_Landing::override_servos`.
///
/// Only deepstall overrides today; the slope type never does.
#[must_use]
pub fn override_servos(flags: &LandingFlags, landing_type: LandingType) -> bool {
    if !flags.in_progress {
        return false;
    }
    match landing_type {
        LandingType::Deepstall => false, // AP_Landing_Deepstall not ported here yet
        LandingType::StandardGlideSlope => false,
    }
}

/// Command a go-around, upstream `AP_Landing::type_slope_request_go_around`.
///
/// Always returns `true`; the meaningful effect is setting the flag.
#[must_use]
pub fn request_go_around(flags: &mut LandingFlags) -> bool {
    flags.commanded_go_around = true;
    true
}

/// Apply the go-around latch from a steep slope abort, upstream the tail of
/// `type_slope_adjust_landing_slope_for_rangefinder_bump`.
#[must_use]
pub fn apply_slope_abort_go_around(
    landing: &mut LandingFlags,
    slope: &mut SlopeLandingFlags,
    alt_offset: f32,
) -> bool {
    landing.commanded_go_around = true;
    slope.alt_offset = alt_offset;
    slope.has_aborted_due_to_slope_recalc = true;
    true
}

/// Reset throttle suppression on abort, upstream
/// `type_slope_verify_abort_landing`.
pub fn abort_landing_throttle_suppressed() -> bool {
    false
}
