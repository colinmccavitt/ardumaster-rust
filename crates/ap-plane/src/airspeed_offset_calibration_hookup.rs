//! Pitot offset calibration stub, upstream `AP_Airspeed::calibrate()`.
//!
//! Latches the current raw pitot TAS as `ARSPD_OFFSET` so a biased tube reads
//! ~0 at rest. Dual-instance clusters calibrate every enabled backend.

use crate::sitl_airspeed_hookup::SitlAirspeedHookup;

/// Per-tick inputs for pitot offset calibration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirspeedOffsetCalibrationInputs {
    /// When true, latch raw pitot TAS as the offset this tick.
    pub request_calibrate: bool,
}

/// Result of one pitot offset calibration tick.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedOffsetCalibrationOutput {
    /// True when at least one instance latched an offset.
    pub calibrated: bool,
    /// Primary instance offset after this tick, m/s.
    pub primary_offset_mps: f32,
}

/// Latch pitot offsets when requested, upstream `AP_Airspeed::calibrate()`.
#[must_use]
pub fn airspeed_offset_calibration_tick(
    hookup: &mut SitlAirspeedHookup,
    inp: AirspeedOffsetCalibrationInputs,
) -> AirspeedOffsetCalibrationOutput {
    let calibrated = if inp.request_calibrate {
        hookup.calibrate_offsets()
    } else {
        false
    };
    let primary_offset_mps = hookup
        .backend()
        .map(|backend| backend.config().offset_mps)
        .unwrap_or(0.0);
    AirspeedOffsetCalibrationOutput {
        calibrated,
        primary_offset_mps,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_math::vector3::Vector3f;
    use crate::sitl_airspeed_hookup::{SitlAirspeedHookup, SitlAirspeedTruth};

    #[test]
    fn no_request_does_not_calibrate() {
        let mut hookup = SitlAirspeedHookup::default();
        hookup.truth = SitlAirspeedTruth {
            airspeed_bf: Vector3f::new(3.0, 0.0, 0.0),
            now_ms: 10,
        };
        let _ = hookup.publish(1.0);
        let out = airspeed_offset_calibration_tick(
            &mut hookup,
            AirspeedOffsetCalibrationInputs {
                request_calibrate: false,
            },
        );
        assert!(!out.calibrated);
        assert_eq!(out.primary_offset_mps, 0.0);
    }

    #[test]
    fn request_latches_primary_offset() {
        let mut hookup = SitlAirspeedHookup::default();
        hookup.truth = SitlAirspeedTruth {
            airspeed_bf: Vector3f::new(3.0, 0.0, 0.0),
            now_ms: 10,
        };
        let _ = hookup.publish(1.0);
        let out = airspeed_offset_calibration_tick(
            &mut hookup,
            AirspeedOffsetCalibrationInputs {
                request_calibrate: true,
            },
        );
        assert!(out.calibrated);
        assert!((out.primary_offset_mps - 3.0).abs() < 1e-6);
    }
}
