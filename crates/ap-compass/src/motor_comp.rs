//! Motor interference compensation, upstream `COMPASS_MOT` / `COMPASS_MOTCT`.
//!
//! Current-based hard-iron: `mag += COMPASS_MOT * current_amps` after offsets
//! (`AP_Compass_Backend::correct_field`). Throttle mode uses the same multiply
//! with throttle in `0..1`. Disabled or zero current is a no-op.

use ap_math::scalar::is_zero;
use ap_math::vector3::Vector3f;

/// Upstream `AP_COMPASS_MOT_COMP_DISABLED`.
pub const COMPASS_MOT_COMP_DISABLED: u8 = 0;
/// Upstream `AP_COMPASS_MOT_COMP_THROTTLE`.
pub const COMPASS_MOT_COMP_THROTTLE: u8 = 1;
/// Upstream `AP_COMPASS_MOT_COMP_CURRENT`.
pub const COMPASS_MOT_COMP_CURRENT: u8 = 2;
/// Upstream `COMPASS_MOTCT` default.
pub const COMPASS_MOTCT_DEFAULT: u8 = COMPASS_MOT_COMP_DISABLED;

/// True when `COMPASS_MOTCT` is throttle or current.
#[must_use]
pub fn motor_comp_enabled(motct: u8) -> bool {
    motct == COMPASS_MOT_COMP_THROTTLE || motct == COMPASS_MOT_COMP_CURRENT
}

/// `COMPASS_MOT * thr_or_curr` when enabled, else zero.
///
/// Upstream `state.motor_offset` in `correct_field`.
#[must_use]
pub fn motor_offset(mot: Vector3f, motct: u8, thr_or_curr: f32) -> Vector3f {
    if !motor_comp_enabled(motct) || is_zero(thr_or_curr) {
        Vector3f::zero()
    } else {
        mot * thr_or_curr
    }
}

/// Apply current-based hard-iron, upstream `mag += motor_offset`.
#[must_use]
pub fn apply_motor_compensation(
    field: Vector3f,
    mot: Vector3f,
    motct: u8,
    thr_or_curr: f32,
) -> Vector3f {
    field + motor_offset(mot, motct, thr_or_curr)
}

/// Learn `COMPASS_MOT` so `raw + mot * current` matches `expected`.
///
/// `mot = (expected - raw) / current`. Returns `None` when current is zero.
#[must_use]
pub fn learn_motor_compensation(
    raw: Vector3f,
    expected: Vector3f,
    current: f32,
) -> Option<Vector3f> {
    if is_zero(current) {
        None
    } else {
        Some((expected - raw) / current)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn disabled_or_zero_current_is_noop() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        let mot = Vector3f::new(0.01, -0.02, 0.0);
        let out = apply_motor_compensation(field, mot, COMPASS_MOT_COMP_DISABLED, 12.0);
        assert_eq!(out, field);
        let out = apply_motor_compensation(field, mot, COMPASS_MOT_COMP_CURRENT, 0.0);
        assert_eq!(out, field);
    }

    #[test]
    fn current_scales_mot_vector() {
        let field = Vector3f::new(0.3, 0.1, 0.4);
        let mot = Vector3f::new(0.01, -0.02, 0.005);
        let out = apply_motor_compensation(field, mot, COMPASS_MOT_COMP_CURRENT, 10.0);
        assert!((out.x - 0.4).abs() < 1e-6);
        assert!((out.y + 0.1).abs() < 1e-6);
        assert!((out.z - 0.45).abs() < 1e-6);
    }

    #[test]
    fn learn_cancels_current_bias() {
        let expected = Vector3f::new(0.3, 0.1, 0.4);
        let current = 8.0;
        let bias_per_amp = Vector3f::new(0.02, -0.01, 0.0);
        let raw = expected + bias_per_amp * current;
        let mot = learn_motor_compensation(raw, expected, current).expect("current");
        let corrected = apply_motor_compensation(raw, mot, COMPASS_MOT_COMP_CURRENT, current);
        assert!((corrected.x - expected.x).abs() < 1e-6);
        assert!((corrected.y - expected.y).abs() < 1e-6);
        assert!((corrected.z - expected.z).abs() < 1e-6);
        assert!((mot.x + bias_per_amp.x).abs() < 1e-6);
    }

    #[test]
    fn learn_none_when_current_zero() {
        assert!(learn_motor_compensation(
            Vector3f::new(1.0, 0.0, 0.0),
            Vector3f::new(0.0, 0.0, 0.0),
            0.0
        )
        .is_none());
    }
}
