//! Compass consistency stub: `Compass::consistent()`.

use ap_compass::consistent::{
    consistent, instance_pair_consistent, CompassInstanceField, AP_COMPASS_MAX_XYZ_ANG_DIFF,
    AP_COMPASS_MAX_XY_ANG_DIFF,
};
use ap_math::vector3::Vector3f;

#[test]
fn thresholds_match_upstream() {
    assert!((AP_COMPASS_MAX_XYZ_ANG_DIFF - core::f32::consts::FRAC_PI_2).abs() < 1e-6);
    assert!((AP_COMPASS_MAX_XY_ANG_DIFF - core::f32::consts::PI / 3.0).abs() < 1e-6);
}

#[test]
fn primary_agrees_with_itself() {
    let field = Vector3f::new(350.0, 20.0, 280.0);
    assert!(instance_pair_consistent(field, field));
    assert!(consistent(
        field,
        &[CompassInstanceField {
            field,
            use_for_yaw: true,
        }],
    ));
}

#[test]
fn unused_secondary_does_not_fail_check() {
    let primary = Vector3f::new(350.0, 20.0, 280.0);
    assert!(consistent(
        primary,
        &[
            CompassInstanceField {
                field: primary,
                use_for_yaw: true,
            },
            CompassInstanceField {
                field: Vector3f::new(0.0, 350.0, 280.0),
                use_for_yaw: false,
            },
        ],
    ));
}
