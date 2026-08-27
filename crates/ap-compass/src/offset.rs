//! Mag hard-iron offsets, upstream `COMPASS_OFS` / `Compass::learn_offsets`. FW-014.
//!
//! Frontend correction is `mag += offsets` (`AP_Compass_Backend::correct_field`).
//! Learn sets `COMPASS_OFS` so the corrected field matches the expected WMM
//! body-frame field: `offset = expected - raw`.

use ap_math::vector3::Vector3f;

/// Upstream `Compass::LearnType::NONE`.
pub const COMPASS_LEARN_NONE: u8 = 0;
/// Upstream `Compass::LearnType::COPY_FROM_EKF`.
pub const COMPASS_LEARN_EKF: u8 = 2;
/// Upstream `Compass::LearnType::INFLIGHT`.
pub const COMPASS_LEARN_INFLIGHT: u8 = 3;
/// Upstream `COMPASS_LEARN` default (`LearnType::NONE`).
pub const COMPASS_LEARN_DEFAULT: u8 = COMPASS_LEARN_NONE;
/// Upstream `AP_COMPASS_OFFSETS_MAX_DEFAULT` (milligauss).
pub const COMPASS_OFFSETS_MAX_DEFAULT: f32 = 1800.0;

/// Apply `COMPASS_OFS`, upstream `correct_field` (`mag += offsets`).
#[must_use]
pub fn apply_offsets(raw: Vector3f, offset: Vector3f) -> Vector3f {
    raw + offset
}

/// Learn `COMPASS_OFS` so the corrected field matches `expected`.
///
/// `offset = expected - raw`, matching `mag += offsets`.
#[must_use]
pub fn learn_offsets(raw: Vector3f, expected: Vector3f) -> Vector3f {
    expected - raw
}

/// True when `COMPASS_LEARN` is EKF or inflight (not disabled).
#[must_use]
pub fn learn_offsets_enabled(learn: u8) -> bool {
    learn == COMPASS_LEARN_EKF || learn == COMPASS_LEARN_INFLIGHT
}

/// Reject offsets longer than `COMPASS_OFFS_MAX`.
#[must_use]
pub fn offsets_within_max(offset: Vector3f, offsets_max: f32) -> bool {
    offset.length() <= offsets_max
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_offsets_adds_hard_iron() {
        let raw = Vector3f::new(0.3, 0.1, 0.4);
        let ofs = Vector3f::new(-0.05, 0.02, 0.0);
        let out = apply_offsets(raw, ofs);
        assert!((out.x - 0.25).abs() < 1e-6);
        assert!((out.y - 0.12).abs() < 1e-6);
        assert!((out.z - 0.4).abs() < 1e-6);
    }

    #[test]
    fn learn_offsets_cancels_bias() {
        let expected = Vector3f::new(0.3, 0.1, 0.4);
        let bias = Vector3f::new(0.05, -0.02, 0.01);
        let raw = expected + bias;
        let ofs = learn_offsets(raw, expected);
        let corrected = apply_offsets(raw, ofs);
        assert!((corrected.x - expected.x).abs() < 1e-6);
        assert!((corrected.y - expected.y).abs() < 1e-6);
        assert!((corrected.z - expected.z).abs() < 1e-6);
        assert!((ofs.x + bias.x).abs() < 1e-6);
    }

    #[test]
    fn learn_disabled_when_none() {
        assert!(!learn_offsets_enabled(COMPASS_LEARN_NONE));
        assert!(learn_offsets_enabled(COMPASS_LEARN_INFLIGHT));
        assert!(learn_offsets_enabled(COMPASS_LEARN_EKF));
    }

    #[test]
    fn offsets_max_rejects_over_limit() {
        assert!(offsets_within_max(
            Vector3f::new(100.0, 0.0, 0.0),
            COMPASS_OFFSETS_MAX_DEFAULT
        ));
        assert!(!offsets_within_max(
            Vector3f::new(2000.0, 0.0, 0.0),
            COMPASS_OFFSETS_MAX_DEFAULT
        ));
    }
}
