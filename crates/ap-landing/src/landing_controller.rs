//! AP_Landing-level dispatch, upstream `AP_Landing.cpp` query methods.
//!
//! Slope and deepstall each own their stage machines; this module wires the
//! vehicle-facing queries that upstream routes through `switch (type)`.

use ap_math::location::Location;

use crate::deepstall_stage::{
    is_flying_forward as deepstall_is_flying_forward,
    is_on_approach as deepstall_is_on_approach,
    is_throttle_suppressed as deepstall_is_throttle_suppressed,
    target_airspeed_cm as deepstall_target_airspeed_cm, DeepstallStage,
};
use crate::go_around::{LandingFlags, LandingType};
use crate::slope_stage::{target_airspeed_cm as slope_target_airspeed_cm, LandingAirspeedParams, SlopeStage};

/// Inputs for [`get_target_airspeed_cm`].
#[derive(Debug, Clone, Copy)]
pub struct TargetAirspeedInputs {
    pub cruise_cm: i32,
    pub pre_flare_cm: i32,
    pub slope_params: LandingAirspeedParams,
    pub head_wind_ms: f32,
}

/// Whether the aircraft is flaring, upstream `AP_Landing::is_flaring`.
#[must_use]
pub fn is_flaring(
    flags: &LandingFlags,
    landing_type: LandingType,
    slope_stage: SlopeStage,
) -> bool {
    if !flags.in_progress {
        return false;
    }
    match landing_type {
        LandingType::StandardGlideSlope => slope_stage.is_flaring(),
        LandingType::Deepstall => false,
    }
}

/// Whether the aircraft is on final, upstream `AP_Landing::is_on_final`.
#[must_use]
pub fn is_on_final(
    flags: &LandingFlags,
    landing_type: LandingType,
    slope_stage: SlopeStage,
) -> bool {
    if !flags.in_progress {
        return false;
    }
    match landing_type {
        LandingType::StandardGlideSlope => slope_stage.is_on_final(),
        LandingType::Deepstall => false,
    }
}

/// Whether the aircraft is on approach, upstream `AP_Landing::is_on_approach`.
#[must_use]
pub fn is_on_approach(
    flags: &LandingFlags,
    landing_type: LandingType,
    slope_stage: SlopeStage,
    deepstall_stage: DeepstallStage,
) -> bool {
    if !flags.in_progress {
        return false;
    }
    match landing_type {
        LandingType::StandardGlideSlope => slope_stage.is_on_approach(),
        LandingType::Deepstall => deepstall_is_on_approach(deepstall_stage),
    }
}

/// Whether ground steering is allowed, upstream
/// `AP_Landing::is_ground_steering_allowed`.
#[must_use]
pub fn is_ground_steering_allowed(
    flags: &LandingFlags,
    landing_type: LandingType,
    slope_stage: SlopeStage,
) -> bool {
    if !flags.in_progress {
        return true;
    }
    match landing_type {
        LandingType::StandardGlideSlope => slope_stage.is_on_approach(),
        LandingType::Deepstall => false,
    }
}

/// Whether ground impact is expected soon, upstream
/// `AP_Landing::is_expecting_impact`.
#[must_use]
pub fn is_expecting_impact(
    flags: &LandingFlags,
    landing_type: LandingType,
    slope_stage: SlopeStage,
) -> bool {
    if !flags.in_progress {
        return false;
    }
    match landing_type {
        LandingType::StandardGlideSlope => slope_stage.is_expecting_impact(),
        LandingType::Deepstall => false,
    }
}

/// Target airspeed in cm/s, upstream `AP_Landing::get_target_airspeed_cm`.
#[must_use]
pub fn get_target_airspeed_cm(
    flags: &LandingFlags,
    landing_type: LandingType,
    slope_stage: SlopeStage,
    deepstall_stage: DeepstallStage,
    inp: &TargetAirspeedInputs,
) -> i32 {
    if !flags.in_progress {
        return inp.cruise_cm;
    }
    match landing_type {
        LandingType::StandardGlideSlope => {
            slope_target_airspeed_cm(slope_stage, &inp.slope_params, inp.head_wind_ms)
        }
        LandingType::Deepstall => deepstall_target_airspeed_cm(
            deepstall_stage,
            inp.cruise_cm,
            inp.pre_flare_cm,
        ),
    }
}

/// Landing target location when the type supplies one, upstream
/// `AP_Landing::get_target_altitude_location`.
#[must_use]
pub fn get_target_altitude_location(
    flags: &LandingFlags,
    landing_type: LandingType,
    landing_point: Location,
) -> Option<Location> {
    if !flags.in_progress {
        return None;
    }
    match landing_type {
        LandingType::Deepstall => Some(landing_point),
        LandingType::StandardGlideSlope => None,
    }
}

/// Whether throttle should be suppressed, upstream
/// `AP_Landing::is_throttle_suppressed`.
#[must_use]
pub fn is_throttle_suppressed(
    flags: &LandingFlags,
    landing_type: LandingType,
    slope_stage: SlopeStage,
    deepstall_stage: DeepstallStage,
) -> bool {
    if !flags.in_progress {
        return false;
    }
    match landing_type {
        LandingType::StandardGlideSlope => slope_stage.is_flaring(),
        LandingType::Deepstall => deepstall_is_throttle_suppressed(deepstall_stage),
    }
}

/// Whether the aircraft is flying forward, upstream
/// `AP_Landing::is_flying_forward`.
#[must_use]
pub fn is_flying_forward(
    flags: &LandingFlags,
    landing_type: LandingType,
    deepstall_stage: DeepstallStage,
) -> bool {
    if !flags.in_progress {
        return true;
    }
    match landing_type {
        LandingType::Deepstall => deepstall_is_flying_forward(deepstall_stage),
        LandingType::StandardGlideSlope => true,
    }
}

/// Whether the landing is complete, upstream `AP_Landing::is_complete`.
#[must_use]
pub fn is_complete(
    _flags: &LandingFlags,
    landing_type: LandingType,
    slope_stage: SlopeStage,
) -> bool {
    match landing_type {
        LandingType::StandardGlideSlope => slope_stage.is_complete(),
        LandingType::Deepstall => false,
    }
}

/// Limit roll during flare, upstream `AP_Landing::constrain_roll`.
#[must_use]
pub fn constrain_roll(
    landing_type: LandingType,
    slope_stage: SlopeStage,
    desired_roll_cd: i32,
    level_roll_limit_cd: i32,
) -> i32 {
    match landing_type {
        LandingType::StandardGlideSlope => {
            slope_stage.constrain_roll(desired_roll_cd, level_roll_limit_cd)
        }
        LandingType::Deepstall => desired_roll_cd,
    }
}
