//! ARSPD_WIND_WARN airspeed-vs-wind warning stub, upstream `AP_Airspeed::_wind_warn`.
//!
//! Vehicle-level threshold. Default 0 falls back to `ARSPD_WIND_MAX`. When the
//! effective threshold is set, a `|airspeed - groundspeed|` larger than it
//! flags a GCS warning (it does not by itself disable TAS use).

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::wind_max::airspeed_groundspeed_delta;
use ap_airspeed::wind_warn::{wind_warn_enabled, wind_warn_exceeded, wind_warn_threshold};

/// Frontend WIND_WARN hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedWindWarnHookup {
    params: AirspeedParams,
}

impl Default for AirspeedWindWarnHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// `ARSPD_WIND_WARN` check published from airspeed vs GPS groundspeed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedWindWarnPublish {
    /// Bound `ARSPD_WIND_WARN` (m/s). Zero falls back to `ARSPD_WIND_MAX`.
    pub wind_warn: f32,
    /// Bound `ARSPD_WIND_MAX` used when `ARSPD_WIND_WARN` is zero.
    pub wind_max: f32,
    /// Effective warning threshold (m/s).
    pub threshold: f32,
    /// True when the effective threshold is greater than zero.
    pub enabled: bool,
    /// `|airspeed - groundspeed|` (m/s).
    pub delta_mps: f32,
    /// True when the enabled warning threshold is exceeded.
    pub exceeded: bool,
}

impl AirspeedWindWarnHookup {
    /// Build a WIND_WARN hookup from vehicle params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply vehicle-level `ARSPD_WIND_WARN` / `ARSPD_WIND_MAX`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_WIND_WARN` (m/s).
    pub fn set_wind_warn(&mut self, wind_warn: f32) {
        let mut params = self.params;
        params.wind_warn = wind_warn;
        self.params = params;
    }

    /// Set `ARSPD_WIND_MAX` used when `ARSPD_WIND_WARN` is zero.
    pub fn set_wind_max(&mut self, wind_max: f32) {
        let mut params = self.params;
        params.wind_max = wind_max;
        self.params = params;
    }

    /// Publish the WIND_WARN check for `airspeed_mps` vs `groundspeed_mps`.
    #[must_use]
    pub fn publish(&self, airspeed_mps: f32, groundspeed_mps: f32) -> AirspeedWindWarnPublish {
        check_airspeed_wind_warn(
            airspeed_mps,
            groundspeed_mps,
            self.params.wind_warn,
            self.params.wind_max,
        )
    }
}

/// Map stored `ARSPD_WIND_WARN` plus speeds to the published warning.
#[must_use]
pub fn check_airspeed_wind_warn(
    airspeed_mps: f32,
    groundspeed_mps: f32,
    wind_warn: f32,
    wind_max: f32,
) -> AirspeedWindWarnPublish {
    AirspeedWindWarnPublish {
        wind_warn,
        wind_max,
        threshold: wind_warn_threshold(wind_warn, wind_max),
        enabled: wind_warn_enabled(wind_warn, wind_max),
        delta_mps: airspeed_groundspeed_delta(airspeed_mps, groundspeed_mps),
        exceeded: wind_warn_exceeded(airspeed_mps, groundspeed_mps, wind_warn, wind_max),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::wind_warn::ARSPD_WIND_WARN_DEFAULT;

    #[test]
    fn default_wind_warn_disables_check() {
        let hookup = AirspeedWindWarnHookup::default();
        assert!((hookup.airspeed_params().wind_warn - ARSPD_WIND_WARN_DEFAULT).abs() < 1e-6);
        let out = hookup.publish(40.0, 5.0);
        assert_eq!(out.wind_warn, 0.0);
        assert!(!out.enabled);
        assert!(!out.exceeded);
    }

    #[test]
    fn explicit_threshold_flags_mismatch() {
        let mut hookup = AirspeedWindWarnHookup::default();
        hookup.set_wind_warn(10.0);
        let fail = hookup.publish(20.0, 5.0);
        assert!(fail.enabled);
        assert!(fail.exceeded);
        assert!((fail.delta_mps - 15.0).abs() < 1e-6);
        let ok = hookup.publish(20.0, 15.0);
        assert!(!ok.exceeded);
    }

    #[test]
    fn zero_warn_falls_back_to_wind_max() {
        let mut hookup = AirspeedWindWarnHookup::default();
        hookup.set_wind_max(8.0);
        let fail = hookup.publish(20.0, 5.0);
        assert!((fail.threshold - 8.0).abs() < 1e-6);
        assert!(fail.exceeded);
    }
}
