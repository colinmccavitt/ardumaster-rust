//! ARSPD_PSI_RANGE sensor pressure-range stub, upstream `AP_Airspeed_Params::psi_range`.
//!
//! Per-instance PSI full-scale. Default 1.0 matches `PSI_RANGE_DEFAULT`.
//! Invalid (non-finite or non-positive) values clamp to the default so analog
//! and MS4525 backends never divide by zero.

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::psi_range::{clamp_psi_range, psi_range_valid};

/// Frontend PSI-range hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedPsiRangeHookup {
    params: AirspeedParams,
}

impl Default for AirspeedPsiRangeHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// `ARSPD_PSI_RANGE` published after clamp / validate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedPsiRangePublish {
    /// Bound `ARSPD_PSI_RANGE` before clamp.
    pub configured: f32,
    /// Usable PSI full-scale after clamp-to-default.
    pub psi_range: f32,
    /// True when the configured value is finite and > 0.
    pub valid: bool,
}

impl AirspeedPsiRangeHookup {
    /// Build a PSI-range hookup from vehicle params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply per-instance `ARSPD_PSI_RANGE`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_PSI_RANGE` on every enabled instance.
    pub fn set_psi_range(&mut self, psi_range: f32) {
        let mut params = self.params;
        params.airspeed1.psi_range = psi_range;
        params.airspeed2.psi_range = psi_range;
        self.params = params;
    }

    /// Publish the clamped / validated primary `ARSPD_PSI_RANGE`.
    #[must_use]
    pub fn publish(&self) -> AirspeedPsiRangePublish {
        validate_airspeed_psi_range(self.params.primary_psi_range())
    }
}

/// Map stored `ARSPD_PSI_RANGE` to the published clamp / validate result.
#[must_use]
pub fn validate_airspeed_psi_range(configured: f32) -> AirspeedPsiRangePublish {
    AirspeedPsiRangePublish {
        configured,
        psi_range: clamp_psi_range(configured),
        valid: psi_range_valid(configured),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::psi_range::ARSPD_PSI_RANGE_DEFAULT;

    #[test]
    fn default_psi_range_matches_upstream() {
        let hookup = AirspeedPsiRangeHookup::default();
        assert!(
            (hookup.airspeed_params().primary_psi_range() - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6
        );
        let out = hookup.publish();
        assert!((out.psi_range - 1.0).abs() < 1e-6);
        assert!(out.valid);
    }

    #[test]
    fn invalid_psi_range_clamps_to_default() {
        let mut hookup = AirspeedPsiRangeHookup::default();
        hookup.set_psi_range(0.0);
        let zero = hookup.publish();
        assert!(!zero.valid);
        assert!((zero.psi_range - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
        hookup.set_psi_range(5.0);
        let ok = hookup.publish();
        assert!(ok.valid);
        assert!((ok.psi_range - 5.0).abs() < 1e-6);
    }
}
