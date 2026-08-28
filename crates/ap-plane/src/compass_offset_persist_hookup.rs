//! Persist learned `COMPASS_OFS` into the param table, upstream `Compass::save_offsets`.
//!
//! Learn latches offsets on the SITL backends. Save copies them into
//! [`CompassParams`] so a later `apply_compass_params` (reboot) restores them.

use ap_compass::persist::offsets_already_saved;
use ap_math::vector3::Vector3f;

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Per-tick inputs for offset persist.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassOffsetPersistInputs {
    /// When true, copy backend `COMPASS_OFS` into the param table this tick.
    pub request_save: bool,
}

/// Result of one offset persist tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CompassOffsetPersistOutput {
    /// True when at least one instance was written to params.
    pub saved: bool,
    /// Primary instance `COMPASS_OFS` now stored in params.
    pub primary_offset: Vector3f,
    /// True when params already matched the backends before or after save.
    pub already_saved: bool,
}

/// Persist learned mag offsets when requested, upstream `Compass::save_offsets`.
#[must_use]
pub fn compass_offset_persist_tick(
    hookup: &mut SitlCompassHookup,
    inp: CompassOffsetPersistInputs,
) -> CompassOffsetPersistOutput {
    let saved = if inp.request_save {
        hookup.save_offsets()
    } else {
        false
    };
    let already_saved = offsets_already_saved(hookup.compass_params(), hookup.cluster());
    CompassOffsetPersistOutput {
        saved,
        primary_offset: hookup.compass_params().compass1.offset,
        already_saved,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_compass::offset::COMPASS_LEARN_INFLIGHT;
    use ap_compass::params::CompassParams;
    use ap_math::vector3::Vector3f;
    use crate::compass_offset_calibration_hookup::{
        compass_offset_calibration_tick, CompassOffsetCalibrationInputs,
    };
    use crate::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};

    #[test]
    fn no_request_does_not_save() {
        let mut hookup = SitlCompassHookup::default();
        let out = compass_offset_persist_tick(
            &mut hookup,
            CompassOffsetPersistInputs { request_save: false },
        );
        assert!(!out.saved);
        assert_eq!(out.primary_offset, Vector3f::zero());
    }

    #[test]
    fn save_after_learn_writes_params() {
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
        let cal = compass_offset_calibration_tick(
            &mut hookup,
            CompassOffsetCalibrationInputs {
                request_learn: true,
            },
        );
        assert!(cal.learned);
        assert_eq!(hookup.compass_params().compass1.offset, Vector3f::zero());

        let out = compass_offset_persist_tick(
            &mut hookup,
            CompassOffsetPersistInputs { request_save: true },
        );
        assert!(out.saved);
        assert!(out.already_saved);
        assert!((out.primary_offset.x + 0.05).abs() < 1e-5);
        assert!((hookup.compass_params().compass1.offset.x + 0.05).abs() < 1e-5);
    }
}
