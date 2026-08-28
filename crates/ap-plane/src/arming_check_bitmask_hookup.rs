//! Vehicle hookup for the pre-4.7 `ARMING_CHECK` enable bitmask.
//!
//! Plane-4.7.0 stores `ARMING_SKIPCHK`. Operators and older params still
//! speak `ARMING_CHECK` (bits set *enable* a named check; bit 0 is ALL).
//! This module converts and runs the shared registry.

use ap_arming::check_bitmask::arming_from_check;
use ap_arming::{Arming, NamedCheck, Required};

use crate::arming_check_registry_hookup::plane_pre_arm_checks_registry;
use crate::mode_run::PreArmResult;

/// Convert `ARMING_REQUIRE` + `ARMING_CHECK` into shared [`Arming`] state.
#[must_use]
pub fn plane_arming_from_check(require: Required, checks_to_perform: u32) -> Arming {
    arming_from_check(require, checks_to_perform)
}

/// Run the shared registry after converting a stored `ARMING_CHECK` mask.
#[must_use]
pub fn plane_pre_arm_checks_arming_check<'a>(
    prior: PreArmResult<'a>,
    require: Required,
    checks_to_perform: u32,
    checks: &[NamedCheck],
) -> PreArmResult<'a> {
    plane_pre_arm_checks_registry(
        prior,
        plane_arming_from_check(require, checks_to_perform),
        checks,
    )
}
