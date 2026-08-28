//! ARSPD_WIND_WARN airspeed-vs-wind warning, upstream `AP_Airspeed::_wind_warn`.
//!
//! Vehicle-level (not per-instance). A value of 0 falls back to `ARSPD_WIND_MAX`.
//! When the effective threshold is enabled, `|airspeed - groundspeed|` above it
//! flags a GCS warning (it does not by itself disable TAS use).

use crate::wind_max::{airspeed_groundspeed_delta, wind_max_enabled};

/// Upstream `ARSPD_WIND_WARN` default: fall back to `ARSPD_WIND_MAX`.
pub const ARSPD_WIND_WARN_DEFAULT: f32 = 0.0;

/// Effective warning threshold: `ARSPD_WIND_WARN` if set, else `ARSPD_WIND_MAX`.
#[must_use]
pub const fn wind_warn_threshold(wind_warn: f32, wind_max: f32) -> f32 {
    if wind_warn > 0.0 {
        wind_warn
    } else {
        wind_max
    }
}

/// Whether the warning check is enabled (effective threshold > 0).
#[must_use]
pub const fn wind_warn_enabled(wind_warn: f32, wind_max: f32) -> bool {
    wind_max_enabled(wind_warn_threshold(wind_warn, wind_max))
}

/// True when `|EAS - groundspeed|` exceeds the warning threshold.
#[must_use]
pub fn wind_warn_exceeded(
    airspeed_mps: f32,
    groundspeed_mps: f32,
    wind_warn: f32,
    wind_max: f32,
) -> bool {
    let threshold = wind_warn_threshold(wind_warn, wind_max);
    wind_max_enabled(threshold)
        && airspeed_groundspeed_delta(airspeed_mps, groundspeed_mps) > threshold
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wind_max::ARSPD_WIND_MAX_DEFAULT;

    #[test]
    fn default_falls_back_to_disabled_wind_max() {
        assert_eq!(ARSPD_WIND_WARN_DEFAULT, 0.0);
        assert_eq!(
            wind_warn_threshold(ARSPD_WIND_WARN_DEFAULT, ARSPD_WIND_MAX_DEFAULT),
            0.0
        );
        assert!(!wind_warn_enabled(ARSPD_WIND_WARN_DEFAULT, ARSPD_WIND_MAX_DEFAULT));
        assert!(!wind_warn_exceeded(40.0, 5.0, ARSPD_WIND_WARN_DEFAULT, 0.0));
        assert!(!wind_warn_exceeded(40.0, 5.0, -1.0, 0.0));
    }

    #[test]
    fn zero_warn_uses_wind_max_threshold() {
        assert!((wind_warn_threshold(0.0, 10.0) - 10.0).abs() < 1e-6);
        assert!(wind_warn_enabled(0.0, 10.0));
        assert!(wind_warn_exceeded(20.0, 5.0, 0.0, 10.0));
        assert!(!wind_warn_exceeded(20.0, 15.0, 0.0, 10.0));
    }

    #[test]
    fn explicit_warn_is_used_instead_of_wind_max() {
        assert!((wind_warn_threshold(5.0, 20.0) - 5.0).abs() < 1e-6);
        assert!(wind_warn_exceeded(20.0, 10.0, 5.0, 20.0));
        assert!(!wind_warn_exceeded(20.0, 16.0, 5.0, 20.0));
        assert!(!wind_warn_exceeded(20.0, 15.0, 5.0, 20.0));
    }
}
