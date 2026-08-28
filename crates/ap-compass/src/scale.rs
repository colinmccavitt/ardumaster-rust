//! Compass scale factor stub, upstream `COMPASS_SCALE`.
//!
//! Frontend correction is `mag *= scale` when the factor is inside
//! `[COMPASS_MIN_SCALE_FACTOR, COMPASS_MAX_SCALE_FACTOR]`
//! (`AP_Compass_Backend::correct_field`). A default of 0 means no scaling.

use ap_math::vector3::Vector3f;

/// Upstream `COMPASS_MAX_SCALE_FACTOR`.
pub const COMPASS_MAX_SCALE_FACTOR: f32 = 1.5;
/// Upstream `COMPASS_MIN_SCALE_FACTOR` (`1.0 / COMPASS_MAX_SCALE_FACTOR`).
pub const COMPASS_MIN_SCALE_FACTOR: f32 = 1.0 / COMPASS_MAX_SCALE_FACTOR;
/// Upstream `COMPASS_SCALE` default (0 = no scaling).
pub const COMPASS_SCALE_DEFAULT: f32 = 0.0;

/// True when `COMPASS_SCALE` is inside the sanity range, upstream `have_scale_factor`.
#[must_use]
pub fn have_scale_factor(scale: f32) -> bool {
    scale >= COMPASS_MIN_SCALE_FACTOR && scale <= COMPASS_MAX_SCALE_FACTOR
}

/// Apply `COMPASS_SCALE`, upstream `correct_field` (`mag *= scale`).
///
/// Out-of-range values (including the 0 default) leave the field unchanged.
#[must_use]
pub fn apply_scale(field: Vector3f, scale: f32) -> Vector3f {
    if have_scale_factor(scale) {
        field * scale
    } else {
        field
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_zero_is_noop() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        assert_eq!(apply_scale(field, COMPASS_SCALE_DEFAULT), field);
        assert!(!have_scale_factor(COMPASS_SCALE_DEFAULT));
    }

    #[test]
    fn in_range_multiplies_field() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        let out = apply_scale(field, 1.1);
        assert!((out.x - 0.33).abs() < 1e-6);
        assert!((out.y - 0.11).abs() < 1e-6);
        assert!((out.z - 0.44).abs() < 1e-6);
        assert!(have_scale_factor(1.1));
        assert!(have_scale_factor(COMPASS_MIN_SCALE_FACTOR));
        assert!(have_scale_factor(COMPASS_MAX_SCALE_FACTOR));
    }

    #[test]
    fn out_of_range_is_noop() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        assert_eq!(apply_scale(field, 0.5), field);
        assert_eq!(apply_scale(field, 1.6), field);
        assert!(!have_scale_factor(0.5));
        assert!(!have_scale_factor(1.6));
    }
}
