//! Compass soft-iron diagonal / off-diagonal stub, upstream `COMPASS_DIA` / `COMPASS_ODI`.
//!
//! Frontend correction is the elliptical matrix
//! `[[DIA_X, ODI_X, ODI_Y], [ODI_X, DIA_Y, ODI_Z], [ODI_Y, ODI_Z, DIA_Z]]`
//! (`AP_Compass_Backend::correct_field`) when `COMPASS_DIA` is non-zero.
//! Default DIA is identity `(1,1,1)` and ODI is zero.

use ap_math::vector3::Vector3f;

/// Upstream `COMPASS_DIA` default (identity diagonal).
pub const COMPASS_DIA_DEFAULT: Vector3f = Vector3f {
    x: 1.0,
    y: 1.0,
    z: 1.0,
};
/// Upstream `COMPASS_ODI` default (zero off-diagonal).
pub const COMPASS_ODI_DEFAULT: Vector3f = Vector3f {
    x: 0.0,
    y: 0.0,
    z: 0.0,
};

/// True when `COMPASS_DIA` is non-zero, upstream `!diagonals.is_zero()`.
#[must_use]
pub fn have_diagonals(diagonals: Vector3f) -> bool {
    !diagonals.is_zero()
}

/// Apply `COMPASS_DIA` / `COMPASS_ODI`, upstream `correct_field` elliptical matrix.
///
/// Zero diagonals leave the field unchanged.
#[must_use]
pub fn apply_soft_iron(field: Vector3f, diagonals: Vector3f, offdiagonals: Vector3f) -> Vector3f {
    if !have_diagonals(diagonals) {
        return field;
    }
    Vector3f::new(
        diagonals.x * field.x + offdiagonals.x * field.y + offdiagonals.y * field.z,
        offdiagonals.x * field.x + diagonals.y * field.y + offdiagonals.z * field.z,
        offdiagonals.y * field.x + offdiagonals.z * field.y + diagonals.z * field.z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_identity_is_noop() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        let out = apply_soft_iron(field, COMPASS_DIA_DEFAULT, COMPASS_ODI_DEFAULT);
        assert!((out.x - field.x).abs() < 1e-6);
        assert!((out.y - field.y).abs() < 1e-6);
        assert!((out.z - field.z).abs() < 1e-6);
        assert!(have_diagonals(COMPASS_DIA_DEFAULT));
    }

    #[test]
    fn zero_diagonals_is_noop() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        assert_eq!(
            apply_soft_iron(field, Vector3f::zero(), COMPASS_ODI_DEFAULT),
            field
        );
        assert!(!have_diagonals(Vector3f::zero()));
    }

    #[test]
    fn diagonal_scales_axes() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        let dia = Vector3f::new(1.1, 0.9, 1.0);
        let out = apply_soft_iron(field, dia, COMPASS_ODI_DEFAULT);
        assert!((out.x - 0.33).abs() < 1e-6);
        assert!((out.y - 0.09).abs() < 1e-6);
        assert!((out.z - 0.4).abs() < 1e-6);
    }

    #[test]
    fn offdiag_mixes_xy() {
        let field = Vector3f::new(1.0, 0.0, 0.0);
        let odi = Vector3f::new(0.1, 0.0, 0.0);
        let out = apply_soft_iron(field, COMPASS_DIA_DEFAULT, odi);
        assert!((out.x - 1.0).abs() < 1e-6);
        assert!((out.y - 0.1).abs() < 1e-6);
        assert!((out.z - 0.0).abs() < 1e-6);
    }
}
