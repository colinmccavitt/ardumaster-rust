//! Go-around requests and landing-type dispatch, upstream `AP_Landing.cpp`
//! `request_go_around`, `override_servos`, and the slope abort latch.

use crate::deepstall::deepstall_may_go_around;
use crate::deepstall_stage::{DeepstallStage, is_throttle_suppressed};

/// Which landing type is active, upstream `AP_Landing::LandingType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LandingType {
    /// Standard glide-slope landing.
    StandardGlideSlope,
    /// Deepstall landing.
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

/// Whether deepstall overrides servos, upstream `AP_Landing_Deepstall::override_servos`.
#[must_use]
pub fn deepstall_override_servos(stage: DeepstallStage) -> bool {
    is_throttle_suppressed(stage)
}

/// Whether the landing library overrides servos, upstream
/// `AP_Landing::override_servos`.
///
/// Only deepstall overrides today; the slope type never does.
#[must_use]
pub fn override_servos(
    flags: &LandingFlags,
    landing_type: LandingType,
    deepstall_stage: Option<DeepstallStage>,
) -> bool {
    if !flags.in_progress {
        return false;
    }
    match landing_type {
        LandingType::Deepstall => deepstall_stage
            .map(deepstall_override_servos)
            .unwrap_or(false),
        LandingType::StandardGlideSlope => false,
    }
}

/// Command a deepstall go-around when above minimum abort altitude, upstream
/// `AP_Landing_Deepstall::request_go_around`.
#[must_use]
pub fn deepstall_request_go_around(
    flags: &mut LandingFlags,
    min_abort_alt_m: f32,
    relative_alt_m: f32,
) -> bool {
    if deepstall_may_go_around(min_abort_alt_m, relative_alt_m) {
        flags.commanded_go_around = true;
        true
    } else {
        false
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
