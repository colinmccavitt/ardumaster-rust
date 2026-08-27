//! Airspeed pre-arm gate for the vehicle loop, upstream `AP_Airspeed::healthy()`.
//!
//! Runs after mode, AHRS, GPS, baro, and compass pre-arm checks. When a SITL
//! airspeed producer is configured, arming requires at least one healthy
//! instance from [`AirspeedHealthFlags`].

use ap_airspeed::sitl::AirspeedHealthFlags;

use crate::mode_run::PreArmResult;

/// Upstream refusal when airspeed pre-arm check fails.
pub const AIRSPEED_REFUSAL: &str = "Airspeed not healthy";

/// Whether airspeed satisfies the pre-arm gate, upstream `AP_Airspeed::healthy()`.
#[must_use]
pub fn airspeed_pre_arm_check(health: AirspeedHealthFlags, require_airspeed: bool) -> bool {
    if !require_airspeed {
        return true;
    }
    health.primary_healthy()
}

/// Chain airspeed pre-arm after mode + AHRS + GPS + baro + compass checks.
#[must_use]
pub fn plane_pre_arm_checks_airspeed(
    prior: PreArmResult<'_>,
    airspeed_pre_arm_ok: bool,
) -> PreArmResult<'_> {
    if let PreArmResult::Refused(msg) = prior {
        return PreArmResult::Refused(msg);
    }
    if airspeed_pre_arm_ok {
        PreArmResult::Allowed
    } else {
        PreArmResult::Refused(AIRSPEED_REFUSAL)
    }
}