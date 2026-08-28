//! Compass consistency stub, upstream `Compass::consistent()`.
//!
//! Each `COMPASS_USE` instance must agree with the primary field on
//! 3-axis angle (`AP_COMPASS_MAX_XYZ_ANG_DIFF`), XY heading angle
//! (`AP_COMPASS_MAX_XY_ANG_DIFF`), and XY length
//! (`AP_COMPASS_MAX_XY_LENGTH_DIFF`). A zero XY field is inconsistent.
//! Instances with `use_for_yaw == false` are skipped.

use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

/// Upstream `AP_COMPASS_MAX_XYZ_ANG_DIFF` (`radians(90)`).
pub const AP_COMPASS_MAX_XYZ_ANG_DIFF: f32 = core::f32::consts::FRAC_PI_2;
/// Upstream `AP_COMPASS_MAX_XY_ANG_DIFF` (`radians(60)`).
pub const AP_COMPASS_MAX_XY_ANG_DIFF: f32 = core::f32::consts::PI / 3.0;
/// Upstream `AP_COMPASS_MAX_XY_LENGTH_DIFF` (milligauss).
pub const AP_COMPASS_MAX_XY_LENGTH_DIFF: f32 = 200.0;

/// One instance field plus its `COMPASS_USE` flag.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassInstanceField {
    /// Body-frame field, upstream `get_field(i)`.
    pub field: Vector3f,
    /// Upstream `use_for_yaw(i)` / `COMPASS_USE`.
    pub use_for_yaw: bool,
}

/// Pair check used inside `Compass::consistent`.
#[must_use]
pub fn instance_pair_consistent(primary: Vector3f, other: Vector3f) -> bool {
    let other_xy: Vector2f = other.xy();
    if other_xy.is_zero() {
        return false;
    }
    let primary_xy = primary.xy();
    if other.angle_to(primary) > AP_COMPASS_MAX_XYZ_ANG_DIFF {
        return false;
    }
    if other_xy.angle_to(primary_xy) > AP_COMPASS_MAX_XY_ANG_DIFF {
        return false;
    }
    (primary_xy - other_xy).length() <= AP_COMPASS_MAX_XY_LENGTH_DIFF
}

/// Runtime `use_for_yaw` after `Compass::consistent()`.
///
/// `COMPASS_USE` stays as configured. AHRS/EKF drop mag-for-yaw when
/// instances fail the consistency check (`!compass.consistent()`).
#[must_use]
pub fn use_for_yaw_if_consistent(configured: bool, instances_consistent: bool) -> bool {
    configured && instances_consistent
}

/// Upstream `Compass::consistent()`.
///
/// `primary` is `get_field()` (first usable). Each `use_for_yaw` instance
/// is compared to that primary field. An empty list is consistent.
#[must_use]
pub fn consistent(primary: Vector3f, instances: &[CompassInstanceField]) -> bool {
    for inst in instances {
        if !inst.use_for_yaw {
            continue;
        }
        if !instance_pair_consistent(primary, inst.field) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_or_unused_is_consistent() {
        let primary = Vector3f::new(400.0, 50.0, 300.0);
        assert!(consistent(primary, &[]));
        assert!(consistent(
            primary,
            &[CompassInstanceField {
                field: Vector3f::new(-400.0, 0.0, 0.0),
                use_for_yaw: false,
            }],
        ));
    }

    #[test]
    fn matching_fields_are_consistent() {
        let field = Vector3f::new(400.0, 50.0, 300.0);
        assert!(instance_pair_consistent(field, field));
        assert!(consistent(
            field,
            &[
                CompassInstanceField {
                    field,
                    use_for_yaw: true,
                },
                CompassInstanceField {
                    field: Vector3f::new(410.0, 40.0, 310.0),
                    use_for_yaw: true,
                },
            ],
        ));
    }

    #[test]
    fn zero_xy_is_inconsistent() {
        let primary = Vector3f::new(400.0, 50.0, 300.0);
        assert!(!instance_pair_consistent(
            primary,
            Vector3f::new(0.0, 0.0, 500.0)
        ));
        assert!(!consistent(
            Vector3f::zero(),
            &[CompassInstanceField {
                field: Vector3f::zero(),
                use_for_yaw: true,
            }],
        ));
    }

    #[test]
    fn opposed_fields_are_inconsistent() {
        // Port Vector3::angle_to returns PI for antiparallel (D-001);
        // that exceeds AP_COMPASS_MAX_XYZ_ANG_DIFF so opposed compasses
        // are inconsistent, unlike upstream's defective angle of 0.
        let primary = Vector3f::new(400.0, 0.0, 0.0);
        let opposed = Vector3f::new(-400.0, 0.0, 0.0);
        assert!(!instance_pair_consistent(primary, opposed));
    }

    #[test]
    fn xy_heading_mismatch_is_inconsistent() {
        let primary = Vector3f::new(400.0, 0.0, 100.0);
        // 90 deg XY yaw exceeds AP_COMPASS_MAX_XY_ANG_DIFF (60 deg).
        let yawed = Vector3f::new(0.0, 400.0, 100.0);
        assert!(!instance_pair_consistent(primary, yawed));
    }

    #[test]
    fn xy_length_mismatch_is_inconsistent() {
        let primary = Vector3f::new(400.0, 0.0, 100.0);
        let stretched = Vector3f::new(700.0, 0.0, 100.0);
        assert!(!instance_pair_consistent(primary, stretched));
    }

    #[test]
    fn inconsistent_disables_use_for_yaw() {
        assert!(use_for_yaw_if_consistent(true, true));
        assert!(!use_for_yaw_if_consistent(true, false));
        assert!(!use_for_yaw_if_consistent(false, true));
        assert!(!use_for_yaw_if_consistent(false, false));
    }
}
