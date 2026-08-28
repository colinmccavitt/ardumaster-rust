//! `ARMING_NEED_LOC` / require-position-before-arm. FW-026.
//!
//! Upstream `AP_Arming::RequireLocation` / `ARMING_NEED_LOC`: require
//! an absolute position before arm so the vehicle can Return To Launch.
//! Default is 0 (`NO`). Copter and Rover show the parameter; Plane
//! compiles the enum in the shared library even though the groupinfo
//! is frame-gated off.
//!
//! This slice is the location gate, not the GPS named-check body.
//! When `YES`, refuse if AHRS has no home or GPS has no 3D fix
//! (`status < GPS_OK_FIX_3D`). The GPS / AHRS named-check hookups
//! stay where they are.

use crate::{Check, NamedCheck};

/// Default `ARMING_NEED_LOC`, upstream `AP_ARMING_NEED_LOC_DEFAULT`.
pub const ARMING_NEED_LOC_DEFAULT: RequireLocation = RequireLocation::No;

/// Upstream `AP_GPS::GPS_OK_FIX_3D` — the floor this gate treats as a fix.
pub const GPS_OK_FIX_3D: u8 = 3;

/// Registry name used when this gate fills `Check::Gps`.
pub const NEED_LOC_CHECK_NAME: &str = "GPS";

/// Upstream `AP_Arming::RequireLocation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RequireLocation {
    /// 0 — do not require a location before arm.
    No = 0,
    /// 1 — require an absolute position before arm.
    Yes = 1,
}

impl RequireLocation {
    /// Decode a stored `ARMING_NEED_LOC` value.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::No),
            1 => Some(Self::Yes),
            _ => None,
        }
    }

    /// The stored parameter value.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Whether the parameter demands an absolute position.
    #[must_use]
    pub const fn required(self) -> bool {
        matches!(self, Self::Yes)
    }
}

/// Whether GPS status is 3D or better, upstream `>= GPS_OK_FIX_3D`.
#[must_use]
pub const fn gps_has_3d_fix(status: u8) -> bool {
    status >= GPS_OK_FIX_3D
}

/// Whether AHRS/GPS currently has the absolute position this gate wants.
///
/// Upstream `gps_checks` refuses `status < GPS_OK_FIX_3D` and
/// `!ahrs.home_is_set()`. Either missing is not a usable location.
#[must_use]
pub const fn has_absolute_position(home_is_set: bool, gps_status: u8) -> bool {
    home_is_set && gps_has_3d_fix(gps_status)
}

/// Whether `ARMING_NEED_LOC` allows arm given the current AHRS/GPS fix.
///
/// `NO` always allows — location is optional. `YES` refuses when there
/// is no home or no 3D fix.
#[must_use]
pub const fn require_location_allows_arm(
    require: RequireLocation,
    home_is_set: bool,
    gps_status: u8,
) -> bool {
    !require.required() || has_absolute_position(home_is_set, gps_status)
}

/// Fill `Check::Gps` from `ARMING_NEED_LOC` and the current fix.
///
/// When the parameter is `NO` the entry is ok so the registry does not
/// refuse on this gate. When `YES` the entry fails unless home is set
/// and GPS has a 3D fix.
#[must_use]
pub const fn need_loc_named_check(
    require: RequireLocation,
    home_is_set: bool,
    gps_status: u8,
) -> NamedCheck {
    NamedCheck {
        check: Check::Gps,
        name: NEED_LOC_CHECK_NAME,
        ok: require_location_allows_arm(require, home_is_set, gps_status),
    }
}
