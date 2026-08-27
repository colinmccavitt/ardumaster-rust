//! Mag offset / learn-offsets stub, upstream `COMPASS_OFS` / `Compass::learn_offsets`.
//!
//! When `COMPASS_LEARN` is enabled and a learn is requested, latches
//! `offset = expected_wmm - raw` on every enabled instance so a hard-iron
//! bias cancels on the next sample.

use ap_math::vector3::Vector3f;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Per-tick inputs for mag offset learning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassOffsetCalibrationInputs {
    /// When true, latch learned `COMPASS_OFS` this tick.
    pub request_learn: bool,
}

/// Result of one mag offset learn tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassOffsetCalibrationOutput {
    /// True when at least one instance latched an offset.
    pub learned: bool,
    /// Primary instance `COMPASS_OFS` after this tick.
    pub primary_offset: Vector3f,
}

/// Latch mag offsets when requested, upstream `Compass::learn_offsets`.
#[must_use]
pub fn compass_offset_calibration_tick(
    hookup: &mut SitlCompassHookup,
    inp: CompassOffsetCalibrationInputs,
) -> CompassOffsetCalibrationOutput {
    let learned = if inp.request_learn {
        hookup.learn_offsets()
    } else {
        false
    };
    let primary_offset = hookup
        .backend()
        .map(|backend| backend.config().offset)
        .unwrap_or_else(Vector3f::zero);
    CompassOffsetCalibrationOutput {
        learned,
        primary_offset,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_compass::offset::COMPASS_LEARN_INFLIGHT;
    use ap_compass::params::CompassParams;
    use ap_math::vector3::Vector3f;
    use crate::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

    #[test]
    fn no_request_does_not_learn() {
        let mut hookup = SitlCompassHookup::default();
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let _ = hookup.publish(ap_math::matrix3::Matrix3f::identity(), 0.0025, None);
        let out = compass_offset_calibration_tick(
            &mut hookup,
            CompassOffsetCalibrationInputs {
                request_learn: false,
            },
        );
        assert!(!out.learned);
        assert_eq!(out.primary_offset, Vector3f::zero());
    }

    #[test]
    fn request_learns_primary_offset() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.learn = COMPASS_LEARN_INFLIGHT;
        hookup.apply_compass_params(params);
        hookup.set_hardiron_bias(Vector3f::new(0.05, 0.0, 0.0));
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let _ = hookup.publish(ap_math::matrix3::Matrix3f::identity(), 0.0025, None);
        let out = compass_offset_calibration_tick(
            &mut hookup,
            CompassOffsetCalibrationInputs {
                request_learn: true,
            },
        );
        assert!(out.learned);
        assert!((out.primary_offset.x + 0.05).abs() < 1e-5);
    }
}
