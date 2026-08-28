//! ARSPD_WIND_MAX max airspeed check stub, upstream `AP_Airspeed::_wind_max`.
//!
//! Vehicle-level threshold. Default 0 disables the check. When set, a
//! `|airspeed - groundspeed|` larger than `ARSPD_WIND_MAX` flags a pitot
//! mismatch (used with `ARSPD_OPTIONS` bit 0 to disable TAS use).

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::wind_max::{airspeed_groundspeed_delta, wind_max_enabled, wind_max_exceeded};

/// Frontend WIND_MAX hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedWindMaxHookup {
    params: AirspeedParams,
}

impl Default for AirspeedWindMaxHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// `ARSPD_WIND_MAX` check published from airspeed vs GPS groundspeed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedWindMaxPublish {
    /// Bound `ARSPD_WIND_MAX` (m/s). Zero disables the check.
    pub wind_max: f32,
    /// True when `ARSPD_WIND_MAX` is greater than zero.
    pub enabled: bool,
    /// `|airspeed - groundspeed|` (m/s).
    pub delta_mps: f32,
    /// True when the enabled check fails.
    pub exceeded: bool,
}

impl AirspeedWindMaxHookup {
    /// Build a WIND_MAX hookup from vehicle params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply vehicle-level `ARSPD_WIND_MAX`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_WIND_MAX` (m/s).
    pub fn set_wind_max(&mut self, wind_max: f32) {
        let mut params = self.params;
        params.wind_max = wind_max;
        self.params = params;
    }

    /// Publish the WIND_MAX check for `airspeed_mps` vs `groundspeed_mps`.
    #[must_use]
    pub fn publish(&self, airspeed_mps: f32, groundspeed_mps: f32) -> AirspeedWindMaxPublish {
        check_airspeed_wind_max(airspeed_mps, groundspeed_mps, self.params.wind_max)
    }
}

/// Map stored `ARSPD_WIND_MAX` plus speeds to the published check.
#[must_use]
pub fn check_airspeed_wind_max(
    airspeed_mps: f32,
    groundspeed_mps: f32,
    wind_max: f32,
) -> AirspeedWindMaxPublish {
    AirspeedWindMaxPublish {
        wind_max,
        enabled: wind_max_enabled(wind_max),
        delta_mps: airspeed_groundspeed_delta(airspeed_mps, groundspeed_mps),
        exceeded: wind_max_exceeded(airspeed_mps, groundspeed_mps, wind_max),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::wind_max::ARSPD_WIND_MAX_DEFAULT;

    #[test]
    fn default_wind_max_disables_check() {
        let hookup = AirspeedWindMaxHookup::default();
        assert!((hookup.airspeed_params().wind_max - ARSPD_WIND_MAX_DEFAULT).abs() < 1e-6);
        let out = hookup.publish(40.0, 5.0);
        assert_eq!(out.wind_max, 0.0);
        assert!(!out.enabled);
        assert!(!out.exceeded);
    }

    #[test]
    fn enabled_threshold_flags_mismatch() {
        let mut hookup = AirspeedWindMaxHookup::default();
        hookup.set_wind_max(10.0);
        let fail = hookup.publish(20.0, 5.0);
        assert!(fail.enabled);
        assert!(fail.exceeded);
        assert!((fail.delta_mps - 15.0).abs() < 1e-6);
        let ok = hookup.publish(20.0, 15.0);
        assert!(!ok.exceeded);
    }
}
