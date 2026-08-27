//! AHRS pre-arm gate for the vehicle loop, upstream `AP_AHRS::pre_arm_check`.
//!
//! Mode pre-arm checks run first; this module refuses arming when the AHRS
//! health gate fails, matching upstream's ordering in `AP_Arming`.

use crate::mode_run::{PreArmResult, GENERIC_REFUSAL};

/// Upstream refusal when AHRS pre-arm check fails.
pub const AHRS_REFUSAL: &str = "AHRS not healthy";

/// Combine mode pre-arm with AHRS `pre_arm_check(false)`.
#[must_use]
pub fn plane_pre_arm_checks(mode: PreArmResult<'_>, ahrs_pre_arm_ok: bool) -> PreArmResult<'_> {
    if let PreArmResult::Refused(msg) = mode {
        return PreArmResult::Refused(msg);
    }
    if ahrs_pre_arm_ok {
        PreArmResult::Allowed
    } else {
        PreArmResult::Refused(AHRS_REFUSAL)
    }
}

/// AHRS-only pre-arm gate for scheduler paths that already validated the mode.
#[must_use]
pub fn ahrs_pre_arm_gate(ahrs_pre_arm_ok: bool, force: bool) -> bool {
    force || ahrs_pre_arm_ok
}
