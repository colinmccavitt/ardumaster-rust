//! Compass external / orientation stub, upstream `COMPASS_ORIENT` / `COMPASS_EXTERNAL`.
//!
//! `AP_Compass_Backend::rotate_field` applies instance orientation always.
//! Internal compasses also apply AHRS board orientation; external ones skip it
//! (`COMPASS_EXTERNAL=1`). `MAG_BOARD_ORIENTATION` is `ROTATION_NONE` here.

use ap_math::rotations_gen::{rotate, Rotation};
use ap_math::vector3::Vector3f;

/// Upstream `COMPASS_ORIENT` default (`ROTATION_NONE`).
pub const COMPASS_ORIENT_DEFAULT: u8 = Rotation::None as u8;
/// Upstream `ROTATION_YAW_90`, a common `COMPASS_ORIENT` value.
pub const COMPASS_ORIENT_YAW_90: u8 = Rotation::Yaw90 as u8;
/// Upstream `COMPASS_EXTERNAL` default (internal).
pub const COMPASS_EXTERNAL_DEFAULT: bool = false;

/// True when `COMPASS_EXTERNAL` marks an externally mounted compass.
#[must_use]
pub const fn is_external(external: bool) -> bool {
    external
}

/// Apply `COMPASS_ORIENT` (or AHRS board orientation) via `Vector3::rotate`.
///
/// Unknown or bookkeeping rotation values leave the field unchanged, matching
/// upstream `INTERNAL_ERROR(bad_rotation)`.
#[must_use]
pub fn apply_orientation(field: Vector3f, orientation: u8) -> Vector3f {
    let Some(rot) = Rotation::from_u8(orientation) else {
        return field;
    };
    let mut out = field;
    let _ = rotate(&mut out, rot);
    out
}

/// Rotate a raw sample into body frame, upstream `rotate_field`.
///
/// External: `COMPASS_ORIENT` only. Internal: board orientation then
/// `COMPASS_ORIENT`.
#[must_use]
pub fn rotate_field(
    field: Vector3f,
    orientation: u8,
    external: bool,
    board_orientation: u8,
) -> Vector3f {
    let after_board = if is_external(external) {
        field
    } else {
        apply_orientation(field, board_orientation)
    };
    apply_orientation(after_board, orientation)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn none_is_noop() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        let out = apply_orientation(field, COMPASS_ORIENT_DEFAULT);
        assert_eq!(out, field);
        let out = rotate_field(field, COMPASS_ORIENT_DEFAULT, false, COMPASS_ORIENT_DEFAULT);
        assert_eq!(out, field);
    }

    #[test]
    fn yaw90_swaps_horizontal() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        let out = apply_orientation(field, COMPASS_ORIENT_YAW_90);
        // ROTATION_YAW_90: (x, y, z) -> (-y, x, z)
        assert!((out.x + 0.1).abs() < 1e-6);
        assert!((out.y - 0.3).abs() < 1e-6);
        assert!((out.z - 0.4).abs() < 1e-6);
    }

    #[test]
    fn external_skips_board_orientation() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        let internal = rotate_field(field, COMPASS_ORIENT_DEFAULT, false, COMPASS_ORIENT_YAW_90);
        let external = rotate_field(field, COMPASS_ORIENT_DEFAULT, true, COMPASS_ORIENT_YAW_90);
        assert!((internal.x + 0.1).abs() < 1e-6);
        assert_eq!(external, field);
        assert!(is_external(true));
        assert!(!is_external(false));
    }

    #[test]
    fn invalid_orientation_leaves_field() {
        let field = Vector3f::new(1.0, 2.0, 3.0);
        assert_eq!(apply_orientation(field, 99), field);
        assert_eq!(apply_orientation(field, Rotation::Max as u8), field);
    }
}
