//! Compass and airspeed named-check wiring into the AP_Arming registry.
//!
//! Upstream `AP_Arming::compass_checks` is gated by `Check::COMPASS`.
//! Upstream `AP_Arming::airspeed_checks` is gated by `Check::AIRSPEED`.
//! This slice fills those two [`NamedCheck`] entries from the existing
//! compass / airspeed pre-arm hookups; it does not replace them.

use ap_airspeed::sitl::AirspeedHealthFlags;
use ap_arming::{Arming, Check, NamedCheck};
use ap_compass::sitl::CompassHealthFlags;

use crate::airspeed_pre_arm_hookup::airspeed_pre_arm_check;
use crate::arming_check_registry_hookup::plane_pre_arm_checks_registry;
use crate::compass_pre_arm_hookup::compass_pre_arm_check;
use crate::mode_run::PreArmResult;

/// Registry name for the COMPASS named check.
pub const COMPASS_CHECK_NAME: &str = "COMPASS";

/// Registry name for the AIRSPEED named check.
pub const AIRSPEED_CHECK_NAME: &str = "AIRSPEED";

/// Fill `Check::Compass` from the existing compass pre-arm health gate.
#[must_use]
pub fn compass_named_check(health: CompassHealthFlags, require_compass: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Compass,
        name: COMPASS_CHECK_NAME,
        ok: compass_pre_arm_check(health, require_compass),
    }
}

/// Fill `Check::Airspeed` from the existing airspeed pre-arm health gate.
#[must_use]
pub fn airspeed_named_check(health: AirspeedHealthFlags, require_airspeed: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Airspeed,
        name: AIRSPEED_CHECK_NAME,
        ok: airspeed_pre_arm_check(health, require_airspeed),
    }
}

/// The two named checks this slice owns, in upstream walk order:
/// compass first, then airspeed.
#[must_use]
pub fn compass_airspeed_named_checks(
    compass_health: CompassHealthFlags,
    require_compass: bool,
    airspeed_health: AirspeedHealthFlags,
    require_airspeed: bool,
) -> [NamedCheck; 2] {
    [
        compass_named_check(compass_health, require_compass),
        airspeed_named_check(airspeed_health, require_airspeed),
    ]
}

/// Run the registry with compass + airspeed named checks filled from the
/// existing sensor hookups.
#[must_use]
pub fn plane_pre_arm_checks_compass_airspeed(
    prior: PreArmResult<'_>,
    arming: Arming,
    compass_health: CompassHealthFlags,
    require_compass: bool,
    airspeed_health: AirspeedHealthFlags,
    require_airspeed: bool,
) -> PreArmResult<'_> {
    let checks = compass_airspeed_named_checks(
        compass_health,
        require_compass,
        airspeed_health,
        require_airspeed,
    );
    plane_pre_arm_checks_registry(prior, arming, &checks)
}
