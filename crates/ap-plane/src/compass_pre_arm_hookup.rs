//! Compass pre-arm gate for the vehicle loop, upstream `AP_Compass::healthy()`.
//!
//! Runs after mode, AHRS, GPS, and baro pre-arm checks. When a SITL compass
//! producer is configured, arming requires at least one healthy instance from
//! [`CompassHealthFlags`].

use ap_compass::sitl::CompassHealthFlags;

use crate::mode_run::PreArmResult;

/// Upstream refusal when compass pre-arm check fails.
pub const COMPASS_REFUSAL: &str = "Compass not healthy";

/// Whether compass satisfies the pre-arm gate, upstream `AP_Compass::healthy()`.
#[must_use]
pub fn compass_pre_arm_check(health: CompassHealthFlags, require_compass: bool) -> bool {
    if !require_compass {
        return true;
    }
    health.primary_healthy()
}

/// Chain compass pre-arm after mode + AHRS + GPS + baro checks.
#[must_use]
pub fn plane_pre_arm_checks_compass(
    prior: PreArmResult<'_>,
    compass_pre_arm_ok: bool,
) -> PreArmResult<'_> {
    if let PreArmResult::Refused(msg) = prior {
        return PreArmResult::Refused(msg);
    }
    if compass_pre_arm_ok {
        PreArmResult::Allowed
    } else {
        PreArmResult::Refused(COMPASS_REFUSAL)
    }
}
