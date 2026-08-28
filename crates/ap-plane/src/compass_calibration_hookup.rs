//! Compass MAG_CAL start/cancel stub, upstream `Compass::start_calibration_all`.
//!
//! Starts every healthy `COMPASS_USE` instance, then `cancel_calibration_all`
//! returns them to `NOT_STARTED`. The sphere-fit solver is not in this slice.

use crate::sitl_compass_hookup::SitlCompassHookup;

/// Per-tick inputs for MAG_CAL start/cancel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassCalibrationInputs {
    /// GCS `MAG_CAL_START` / `start_calibration_all`.
    pub request_start: bool,
    /// GCS `MAG_CAL_CANCEL` / `cancel_calibration_all`.
    pub request_cancel: bool,
}

/// Result of one MAG_CAL start/cancel tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CompassCalibrationOutput {
    /// True when `start_calibration_all` started at least one instance.
    pub started: bool,
    /// True when this tick ran `cancel_calibration_all`.
    pub cancelled: bool,
    /// Upstream `Compass::is_calibrating()`.
    pub calibrating: bool,
}

/// Drive MAG_CAL start/cancel on the SITL compass hookup.
#[must_use]
pub fn compass_calibration_tick(
    hookup: &mut SitlCompassHookup,
    inp: CompassCalibrationInputs,
) -> CompassCalibrationOutput {
    if inp.request_cancel {
        hookup.cancel_calibration_all();
        return CompassCalibrationOutput {
            started: false,
            cancelled: true,
            calibrating: hookup.is_calibrating(),
        };
    }
    let started = if inp.request_start {
        hookup.start_calibration_all()
    } else {
        false
    };
    CompassCalibrationOutput {
        started,
        cancelled: false,
        calibrating: hookup.is_calibrating(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sitl_compass_hookup::{SitlCompassHookup, SitlCompassTruth};
    use ap_compass::params::CompassParams;
    use ap_math::matrix3::Matrix3f;

    fn published_hookup() -> SitlCompassHookup {
        let mut hookup = SitlCompassHookup::default();
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
        hookup
    }

    #[test]
    fn no_request_does_not_start() {
        let mut hookup = published_hookup();
        let out = compass_calibration_tick(
            &mut hookup,
            CompassCalibrationInputs {
                request_start: false,
                request_cancel: false,
            },
        );
        assert!(!out.started);
        assert!(!out.cancelled);
        assert!(!out.calibrating);
    }

    #[test]
    fn start_then_cancel() {
        let mut hookup = published_hookup();
        let started = compass_calibration_tick(
            &mut hookup,
            CompassCalibrationInputs {
                request_start: true,
                request_cancel: false,
            },
        );
        assert!(started.started);
        assert!(started.calibrating);
        assert!(hookup.is_calibrating());
        let cancelled = compass_calibration_tick(
            &mut hookup,
            CompassCalibrationInputs {
                request_start: false,
                request_cancel: true,
            },
        );
        assert!(cancelled.cancelled);
        assert!(!cancelled.calibrating);
        assert!(!hookup.is_calibrating());
    }

    #[test]
    fn use_for_yaw_off_does_not_start() {
        let mut hookup = SitlCompassHookup::default();
        let mut params = CompassParams::default();
        params.compass1.use_for_yaw = false;
        hookup.apply_compass_params(params);
        hookup.truth = SitlCompassTruth {
            latitude_deg: 51.875,
            longitude_deg: -0.154,
            now_ms: 10,
        };
        let _ = hookup.publish(Matrix3f::identity(), 0.0025, None);
        let out = compass_calibration_tick(
            &mut hookup,
            CompassCalibrationInputs {
                request_start: true,
                request_cancel: false,
            },
        );
        assert!(!out.started);
        assert!(!out.calibrating);
    }
}
