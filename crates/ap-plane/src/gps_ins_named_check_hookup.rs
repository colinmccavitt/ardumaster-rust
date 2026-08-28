//! GPS and INS named-check wiring into the AP_Arming registry.
//!
//! Upstream `AP_Arming::gps_checks` is gated by `Check::GPS`. Upstream
//! `AP_Arming::ins_checks` is gated by `Check::INS` and reports gyro/accel
//! health. This slice fills those two [`NamedCheck`] entries from the
//! existing GPS pre-arm hookup and the INS frontend health accessors; it
//! does not replace them.
//!
//! Plane's AHRS pre-arm is a separate named check (`ahrs_named_check`) that
//! shares the INS skip bit. This slice owns the inertial-sensor half.

use ap_arming::{Arming, Check, NamedCheck};
use ap_gps::GpsHealthFlags;
use ap_ins::InertialSensorFrontend;

use crate::arming_check_registry_hookup::plane_pre_arm_checks_registry;
use crate::gps_pre_arm_hookup::gps_pre_arm_check;
use crate::mode_run::PreArmResult;

/// Registry name for the GPS named check.
pub const GPS_CHECK_NAME: &str = "GPS";

/// Registry name for the INS named check.
pub const INS_CHECK_NAME: &str = "INS";

/// Fill `Check::Gps` from the existing GPS pre-arm health gate.
#[must_use]
pub fn gps_named_check(health: Option<GpsHealthFlags>, require_gps: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Gps,
        name: GPS_CHECK_NAME,
        ok: gps_pre_arm_check(health, require_gps),
    }
}

/// Whether gyros and accels pass the INS named-check body, upstream
/// `get_gyro_health()` && `get_accel_health()` in `AP_Arming::ins_checks`.
#[must_use]
pub fn ins_pre_arm_health(gyro_healthy: bool, accel_healthy: bool) -> bool {
    gyro_healthy && accel_healthy
}

/// Fill `Check::Ins` from primary gyro + accel health.
#[must_use]
pub fn ins_named_check(gyro_healthy: bool, accel_healthy: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Ins,
        name: INS_CHECK_NAME,
        ok: ins_pre_arm_health(gyro_healthy, accel_healthy),
    }
}

/// Fill `Check::Ins` from the shared INS frontend.
#[must_use]
pub fn ins_named_check_from_frontend(frontend: &InertialSensorFrontend) -> NamedCheck {
    ins_named_check(frontend.get_gyro_health(), frontend.get_accel_health())
}

/// The two named checks this slice owns, in upstream walk order:
/// GPS first, then INS.
#[must_use]
pub fn gps_ins_named_checks(
    gps_health: Option<GpsHealthFlags>,
    require_gps: bool,
    gyro_healthy: bool,
    accel_healthy: bool,
) -> [NamedCheck; 2] {
    [
        gps_named_check(gps_health, require_gps),
        ins_named_check(gyro_healthy, accel_healthy),
    ]
}

/// Run the registry with GPS + INS named checks filled from the
/// existing sensor hookups.
#[must_use]
pub fn plane_pre_arm_checks_gps_ins(
    prior: PreArmResult<'_>,
    arming: Arming,
    gps_health: Option<GpsHealthFlags>,
    require_gps: bool,
    gyro_healthy: bool,
    accel_healthy: bool,
) -> PreArmResult<'_> {
    let checks = gps_ins_named_checks(gps_health, require_gps, gyro_healthy, accel_healthy);
    plane_pre_arm_checks_registry(prior, arming, &checks)
}
