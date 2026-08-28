//! Barometer and AHRS named-check wiring into the AP_Arming registry.
//!
//! Upstream `AP_Arming::barometer_checks` is gated by `Check::BARO`. Plane's
//! AHRS pre-arm (`AP_Arming_Plane::ins_checks`) is gated by `Check::INS` and
//! reports `"AHRS: ..."`. This slice fills those two [`NamedCheck`] entries
//! from the existing baro / AHRS pre-arm hookups; it does not replace them.

use ap_arming::{Arming, Check, NamedCheck};
use ap_baro::sitl::BaroHealthFlags;

use crate::arming_check_registry_hookup::plane_pre_arm_checks_registry;
use crate::baro_pre_arm_hookup::baro_pre_arm_check;
use crate::mode_run::PreArmResult;

/// Registry name for the BARO named check.
pub const BARO_CHECK_NAME: &str = "BARO";

/// Registry name for the INS-gated AHRS named check.
pub const AHRS_CHECK_NAME: &str = "AHRS";

/// Fill `Check::Baro` from the existing baro pre-arm health gate.
#[must_use]
pub fn baro_named_check(health: BaroHealthFlags, require_baro: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Baro,
        name: BARO_CHECK_NAME,
        ok: baro_pre_arm_check(health, require_baro),
    }
}

/// Fill `Check::Ins` from the existing AHRS pre-arm health gate.
///
/// Plane reports AHRS failures under the INS bit, not a dedicated AHRS bit.
#[must_use]
pub fn ahrs_named_check(ahrs_pre_arm_ok: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Ins,
        name: AHRS_CHECK_NAME,
        ok: ahrs_pre_arm_ok,
    }
}

/// The two named checks this slice owns, in upstream walk order:
/// barometer first, then INS-gated AHRS.
#[must_use]
pub fn baro_ahrs_named_checks(
    baro_health: BaroHealthFlags,
    require_baro: bool,
    ahrs_pre_arm_ok: bool,
) -> [NamedCheck; 2] {
    [
        baro_named_check(baro_health, require_baro),
        ahrs_named_check(ahrs_pre_arm_ok),
    ]
}

/// Run the registry with baro + AHRS named checks filled from the
/// existing sensor hookups.
#[must_use]
pub fn plane_pre_arm_checks_baro_ahrs(
    prior: PreArmResult<'_>,
    arming: Arming,
    baro_health: BaroHealthFlags,
    require_baro: bool,
    ahrs_pre_arm_ok: bool,
) -> PreArmResult<'_> {
    let checks = baro_ahrs_named_checks(baro_health, require_baro, ahrs_pre_arm_ok);
    plane_pre_arm_checks_registry(prior, arming, &checks)
}
