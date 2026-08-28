//! ARSPD_FBW_MIN / ARSPD_FBW_MAX fly-by-wire airspeed-limit stub.
//!
//! Vehicle-level demanded-airspeed envelope, upstream `aparm.airspeed_min` /
//! `aparm.airspeed_max` (legacy names `ARSPD_FBW_MIN` / `ARSPD_FBW_MAX`).

use ap_airspeed::fbw::{clamp_fbw_airspeed, fbw_envelope};
use ap_airspeed::params::AirspeedParams;

/// Frontend FBW airspeed-limit hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedFbwHookup {
    params: AirspeedParams,
}

impl Default for AirspeedFbwHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// `ARSPD_FBW_MIN` / `ARSPD_FBW_MAX` envelope published for a demanded speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedFbwPublish {
    /// Bound `ARSPD_FBW_MIN` (m/s).
    pub fbw_min: f32,
    /// Bound `ARSPD_FBW_MAX` (m/s).
    pub fbw_max: f32,
    /// Ordered envelope after an inverted-limit swap.
    pub envelope_min: f32,
    /// Ordered envelope after an inverted-limit swap.
    pub envelope_max: f32,
    /// Demanded airspeed before the FBW clamp (m/s).
    pub demanded_mps: f32,
    /// Demanded airspeed constrained into the envelope (m/s).
    pub limited_mps: f32,
}

impl AirspeedFbwHookup {
    /// Build an FBW-limit hookup from vehicle params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply vehicle-level `ARSPD_FBW_MIN` / `ARSPD_FBW_MAX`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_FBW_MIN` (m/s).
    pub fn set_fbw_min(&mut self, fbw_min: f32) {
        let mut params = self.params;
        params.fbw_min = fbw_min;
        self.params = params;
    }

    /// Set `ARSPD_FBW_MAX` (m/s).
    pub fn set_fbw_max(&mut self, fbw_max: f32) {
        let mut params = self.params;
        params.fbw_max = fbw_max;
        self.params = params;
    }

    /// Publish the FBW envelope and the clamped demanded airspeed.
    #[must_use]
    pub fn publish(&self, demanded_mps: f32) -> AirspeedFbwPublish {
        limit_airspeed_fbw(demanded_mps, self.params.fbw_min, self.params.fbw_max)
    }
}

/// Map stored FBW limits plus a demanded speed to the published envelope.
#[must_use]
pub fn limit_airspeed_fbw(demanded_mps: f32, fbw_min: f32, fbw_max: f32) -> AirspeedFbwPublish {
    let (envelope_min, envelope_max) = fbw_envelope(fbw_min, fbw_max);
    AirspeedFbwPublish {
        fbw_min,
        fbw_max,
        envelope_min,
        envelope_max,
        demanded_mps,
        limited_mps: clamp_fbw_airspeed(demanded_mps, fbw_min, fbw_max),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::fbw::{ARSPD_FBW_MAX_DEFAULT, ARSPD_FBW_MIN_DEFAULT};

    #[test]
    fn default_limits_match_upstream() {
        let hookup = AirspeedFbwHookup::default();
        assert!((hookup.airspeed_params().fbw_min - ARSPD_FBW_MIN_DEFAULT).abs() < 1e-6);
        assert!((hookup.airspeed_params().fbw_max - ARSPD_FBW_MAX_DEFAULT).abs() < 1e-6);
        let mid = hookup.publish(15.0);
        assert!((mid.limited_mps - 15.0).abs() < 1e-6);
        assert!((hookup.publish(5.0).limited_mps - 9.0).abs() < 1e-6);
        assert!((hookup.publish(30.0).limited_mps - 22.0).abs() < 1e-6);
    }

    #[test]
    fn custom_limits_clamp_demanded() {
        let mut hookup = AirspeedFbwHookup::default();
        hookup.set_fbw_min(10.0);
        hookup.set_fbw_max(18.0);
        assert!((hookup.publish(8.0).limited_mps - 10.0).abs() < 1e-6);
        assert!((hookup.publish(20.0).limited_mps - 18.0).abs() < 1e-6);
        assert!((hookup.publish(14.0).limited_mps - 14.0).abs() < 1e-6);
    }
}
