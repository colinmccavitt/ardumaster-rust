//! Compass field-strength / expected-field check stub.
//!
//! Reports whether the published body field is inside the arming
//! milligauss window and whether the NED field matches the WMM
//! expected earth field.

use ap_compass::field::{
    expected_earth_field_mgauss, expected_field_ok, field_ok, field_strength_ok, gauss_to_mgauss,
    COMPASS_MAGFIELD_ERROR_THRESHOLD,
};
use ap_math::matrix3::Matrix3f;
use ap_math::vector3::Vector3f;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Snapshot of the field-strength / expected-field check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassFieldOutput {
    /// `|field|` in milligauss.
    pub length_mgauss: f32,
    /// True when length is inside 185–875 mG.
    pub length_ok: bool,
    /// `|get_earth_field_ga| * 1000` at the hookup location.
    pub expected_length_mgauss: f32,
    /// True when NED vs WMM is inside `ARMING_MAGTHRESH`.
    pub expected_ok: bool,
    /// Length and expected-field checks both passed.
    pub field_ok: bool,
}

/// Check a body-frame sample (gauss, as SITL publishes) against expected field.
#[must_use]
pub fn compass_field_tick(
    hookup: &SitlCompassHookup,
    mag_body_ga: Vector3f,
    body_to_ned: Matrix3f,
) -> CompassFieldOutput {
    let field_mgauss = mag_body_ga * 1000.0;
    let length_mgauss = field_mgauss.length();
    let length_ok = field_strength_ok(field_mgauss);
    let earth_mgauss =
        expected_earth_field_mgauss(hookup.truth.latitude_deg, hookup.truth.longitude_deg);
    let measured_ef = body_to_ned * field_mgauss;
    let expected_ok =
        expected_field_ok(measured_ef, earth_mgauss, COMPASS_MAGFIELD_ERROR_THRESHOLD);
    CompassFieldOutput {
        length_mgauss,
        length_ok,
        expected_length_mgauss: earth_mgauss.length(),
        expected_ok,
        field_ok: field_ok(
            field_mgauss,
            measured_ef,
            earth_mgauss,
            COMPASS_MAGFIELD_ERROR_THRESHOLD,
        ),
    }
}

/// Convenience: convert gauss length the same way arming scales WMM intensity.
#[must_use]
pub fn published_length_mgauss(mag_body_ga: Vector3f) -> f32 {
    gauss_to_mgauss(mag_body_ga.length())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::SitlCompassHookup;
    use ap_compass::field::{COMPASS_MAGFIELD_MAX, COMPASS_MAGFIELD_MIN};
    use ap_math::matrix3::Matrix3f;
    use ap_math::vector3::Vector3f;

    #[test]
    fn default_wmm_sample_passes() {
        let hookup = SitlCompassHookup::default();
        let earth_ga = ap_compass::field::expected_earth_field_ga(
            hookup.truth.latitude_deg,
            hookup.truth.longitude_deg,
        );
        let out = compass_field_tick(&hookup, earth_ga, Matrix3f::identity());
        assert!(out.length_ok);
        assert!(out.expected_ok);
        assert!(out.field_ok);
        assert!(out.length_mgauss >= COMPASS_MAGFIELD_MIN);
        assert!(out.length_mgauss <= COMPASS_MAGFIELD_MAX);
    }

    #[test]
    fn weak_field_fails_length() {
        let hookup = SitlCompassHookup::default();
        let out = compass_field_tick(&hookup, Vector3f::new(0.05, 0.0, 0.0), Matrix3f::identity());
        assert!(!out.length_ok);
        assert!(!out.field_ok);
    }
}
