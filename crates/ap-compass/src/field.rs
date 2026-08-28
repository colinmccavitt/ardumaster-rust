//! Compass field-strength / expected-field check stub. FW-014.
//!
//! Upstream `AP_Arming::compass_checks` rejects a field length outside
//! `[AP_ARMING_COMPASS_MAGFIELD_MIN, AP_ARMING_COMPASS_MAGFIELD_MAX]`
//! (185–875 milligauss around expected 530). With a location, it also
//! compares the NED field to `AP_Declination::get_earth_field_ga * 1000`.
//! XY axes use `ARMING_MAGTHRESH` (default 100 mG); Z uses 2× that.

use ap_math::vector3::Vector3f;

use crate::sitl::mag_field_ef_ned;

/// Upstream `AP_ARMING_COMPASS_MAGFIELD_EXPECTED` (milligauss).
pub const COMPASS_MAGFIELD_EXPECTED: f32 = 530.0;
/// Upstream `AP_ARMING_COMPASS_MAGFIELD_MIN` (`0.35 * 530` milligauss).
pub const COMPASS_MAGFIELD_MIN: f32 = 185.0;
/// Upstream `AP_ARMING_COMPASS_MAGFIELD_MAX` (`1.65 * 530` milligauss).
pub const COMPASS_MAGFIELD_MAX: f32 = 875.0;
/// Upstream `AP_ARMING_MAGFIELD_ERROR_THRESHOLD` (milligauss). `0` disables.
pub const COMPASS_MAGFIELD_ERROR_THRESHOLD: f32 = 100.0;

/// Convert WMM / SITL gauss into the milligauss units arming uses.
#[must_use]
pub const fn gauss_to_mgauss(gauss: f32) -> f32 {
    gauss * 1000.0
}

/// True when `|field|` is inside the arming min/max, upstream length check.
#[must_use]
pub fn field_length_ok(length_mgauss: f32) -> bool {
    length_mgauss >= COMPASS_MAGFIELD_MIN && length_mgauss <= COMPASS_MAGFIELD_MAX
}

/// Reject NaN/Inf and out-of-range field length (milligauss).
#[must_use]
pub fn field_strength_ok(field_mgauss: Vector3f) -> bool {
    if field_mgauss.is_nan() || field_mgauss.is_inf() {
        return false;
    }
    field_length_ok(field_mgauss.length())
}

/// Expected earth-frame field in gauss, upstream `get_earth_field_ga`.
#[must_use]
pub fn expected_earth_field_ga(latitude_deg: f32, longitude_deg: f32) -> Vector3f {
    mag_field_ef_ned(latitude_deg, longitude_deg).0
}

/// Expected earth-frame field in milligauss (`get_earth_field_ga * 1000`).
#[must_use]
pub fn expected_earth_field_mgauss(latitude_deg: f32, longitude_deg: f32) -> Vector3f {
    expected_earth_field_ga(latitude_deg, longitude_deg) * 1000.0
}

/// Compare a NED measurement to the WMM earth field.
///
/// `threshold <= 0` disables the check, matching `ARMING_MAGTHRESH`.
#[must_use]
pub fn expected_field_ok(
    measured_ef_mgauss: Vector3f,
    earth_field_mgauss: Vector3f,
    threshold: f32,
) -> bool {
    if threshold <= 0.0 {
        return true;
    }
    let diff = measured_ef_mgauss - earth_field_mgauss;
    if diff.x.abs().max(diff.y.abs()) > threshold {
        return false;
    }
    if diff.z.abs() > threshold * 2.0 {
        return false;
    }
    true
}

/// Length bounds plus optional expected-field vector compare.
#[must_use]
pub fn field_ok(
    field_mgauss: Vector3f,
    measured_ef_mgauss: Vector3f,
    earth_field_mgauss: Vector3f,
    threshold: f32,
) -> bool {
    field_strength_ok(field_mgauss)
        && expected_field_ok(measured_ef_mgauss, earth_field_mgauss, threshold)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expected_bounds_match_upstream() {
        // Upstream comments say 0.35/1.65 * 530; the stored integers are 185/875.
        assert_eq!(COMPASS_MAGFIELD_MIN, 185.0);
        assert_eq!(COMPASS_MAGFIELD_MAX, 875.0);
        assert!(field_length_ok(COMPASS_MAGFIELD_EXPECTED));
        assert!(field_length_ok(COMPASS_MAGFIELD_MIN));
        assert!(field_length_ok(COMPASS_MAGFIELD_MAX));
        assert!(!field_length_ok(COMPASS_MAGFIELD_MIN - 1.0));
        assert!(!field_length_ok(COMPASS_MAGFIELD_MAX + 1.0));
    }

    #[test]
    fn rejects_nan_inf_and_zero() {
        assert!(!field_strength_ok(Vector3f::new(f32::NAN, 0.0, 0.0)));
        assert!(!field_strength_ok(Vector3f::new(f32::INFINITY, 0.0, 0.0)));
        assert!(!field_strength_ok(Vector3f::zero()));
        assert!(field_strength_ok(Vector3f::new(
            COMPASS_MAGFIELD_EXPECTED,
            0.0,
            0.0
        )));
    }

    #[test]
    fn expected_field_xy_and_z_thresholds() {
        let earth = Vector3f::new(400.0, 50.0, 300.0);
        assert!(expected_field_ok(
            earth,
            earth,
            COMPASS_MAGFIELD_ERROR_THRESHOLD
        ));
        assert!(!expected_field_ok(
            earth + Vector3f::new(101.0, 0.0, 0.0),
            earth,
            COMPASS_MAGFIELD_ERROR_THRESHOLD
        ));
        assert!(!expected_field_ok(
            earth + Vector3f::new(0.0, 0.0, 201.0),
            earth,
            COMPASS_MAGFIELD_ERROR_THRESHOLD
        ));
        assert!(expected_field_ok(
            earth + Vector3f::new(500.0, 0.0, 0.0),
            earth,
            0.0
        ));
    }

    #[test]
    fn default_sitl_location_is_inside_bounds() {
        let earth = expected_earth_field_mgauss(51.875, -0.154);
        assert!(field_strength_ok(earth));
        assert!(expected_field_ok(
            earth,
            earth,
            COMPASS_MAGFIELD_ERROR_THRESHOLD
        ));
        assert!(field_ok(
            earth,
            earth,
            earth,
            COMPASS_MAGFIELD_ERROR_THRESHOLD
        ));
    }
}
