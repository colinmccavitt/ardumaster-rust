//! Baro pre-arm gate for the vehicle loop, upstream `AP_Baro::healthy()`.
//!
//! Runs after mode, AHRS, and GPS pre-arm checks. When a SITL baro producer is
//! configured, arming requires at least one healthy instance from
//! [`BaroHealthFlags`].

use ap_baro::sitl::BaroHealthFlags;

use crate::mode_run::PreArmResult;

/// Upstream refusal when baro pre-arm check fails.
pub const BARO_REFUSAL: &str = "Baro not healthy";

/// Whether baro satisfies the pre-arm gate, upstream `AP_Baro::healthy()`.
#[must_use]
pub fn baro_pre_arm_check(health: BaroHealthFlags, require_baro: bool) -> bool {
    if !require_baro {
        return true;
    }
    health.primary_healthy()
}

/// Chain baro pre-arm after mode + AHRS + GPS checks.
#[must_use]
pub fn plane_pre_arm_checks_baro(prior: PreArmResult<'_>, baro_pre_arm_ok: bool) -> PreArmResult<'_> {
    if let PreArmResult::Refused(msg) = prior {
        return PreArmResult::Refused(msg);
    }
    if baro_pre_arm_ok {
        PreArmResult::Allowed
    } else {
        PreArmResult::Refused(BARO_REFUSAL)
    }
}
