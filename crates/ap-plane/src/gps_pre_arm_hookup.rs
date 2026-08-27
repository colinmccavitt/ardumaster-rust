//! GPS pre-arm gate for the vehicle loop, upstream `AP_GPS::pre_arm_checks`.
//!
//! Runs after mode and AHRS pre-arm checks. When a GPS producer is configured,
//! arming requires a healthy lag-buffered fix from [`GpsHealthFlags`].

use ap_gps::GpsHealthFlags;

use crate::mode_run::PreArmResult;

/// Upstream refusal when GPS pre-arm check fails.
pub const GPS_REFUSAL: &str = "GPS not healthy";

/// Whether GPS satisfies the pre-arm gate, upstream `AP_GPS::isHealthy()`.
#[must_use]
pub fn gps_pre_arm_check(health: Option<GpsHealthFlags>, require_gps: bool) -> bool {
    if !require_gps {
        return true;
    }
    health.is_some_and(|h| h.is_healthy())
}

/// Chain GPS pre-arm after mode + AHRS checks.
#[must_use]
pub fn plane_pre_arm_checks_gps(prior: PreArmResult<'_>, gps_pre_arm_ok: bool) -> PreArmResult<'_> {
    if let PreArmResult::Refused(msg) = prior {
        return PreArmResult::Refused(msg);
    }
    if gps_pre_arm_ok {
        PreArmResult::Allowed
    } else {
        PreArmResult::Refused(GPS_REFUSAL)
    }
}
