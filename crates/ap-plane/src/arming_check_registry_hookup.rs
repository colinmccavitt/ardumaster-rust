//! Vehicle hookup for the shared AP_Arming check registry.
//!
//! Plane owns `ARMING_REQUIRE` / `ARMING_SKIPCHK`. The named checks
//! themselves (AHRS, compass, airspeed, ...) already have their own
//! hookups; this module is the gate that can refuse on a named check
//! the operator has not skipped.

use ap_arming::{Arming, NamedCheck, PreArmOutcome};

use crate::mode_run::PreArmResult;

/// Run the shared AP_Arming registry after a prior (mode) result.
///
/// A mode refusal is kept. Otherwise [`Arming::pre_arm_checks`] walks
/// the named checks and this returns that check's name if one fails.
#[must_use]
pub fn plane_pre_arm_checks_registry<'a>(
    prior: PreArmResult<'a>,
    arming: Arming,
    checks: &[NamedCheck],
) -> PreArmResult<'a> {
    if let PreArmResult::Refused(msg) = prior {
        return PreArmResult::Refused(msg);
    }
    match arming.pre_arm_checks(checks) {
        PreArmOutcome::Allowed => PreArmResult::Allowed,
        PreArmOutcome::Refused { name, .. } => PreArmResult::Refused(name),
    }
}
