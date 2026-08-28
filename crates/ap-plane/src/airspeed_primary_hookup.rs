//! ARSPD_PRIMARY instance-select stub, upstream `AP_Airspeed::_primary`.
//!
//! Vehicle-level preferred instance (0 / 1). Default 0. Out-of-range values
//! clamp to instance 0. The live cluster uses the configured instance when it
//! is healthy, otherwise the first healthy instance (dual-sensor failover).

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::primary::clamp_primary;

use crate::sitl_airspeed_hookup::SitlAirspeedHookup;

/// Frontend PRIMARY hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedPrimaryHookup {
    params: AirspeedParams,
}

impl Default for AirspeedPrimaryHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// `ARSPD_PRIMARY` selection published from the configured instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirspeedPrimaryPublish {
    /// Bound `ARSPD_PRIMARY`.
    pub configured: u8,
    /// Clamped instance used when health is unknown / unused.
    pub clamped: u8,
}

impl AirspeedPrimaryHookup {
    /// Build a PRIMARY hookup from vehicle params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply vehicle-level `ARSPD_PRIMARY`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_PRIMARY`.
    pub fn set_primary(&mut self, primary: u8) {
        let mut params = self.params;
        params.primary = primary;
        self.params = params;
    }

    /// Publish the clamped `ARSPD_PRIMARY` for `instance_count` backends.
    #[must_use]
    pub fn publish(&self, instance_count: u8) -> AirspeedPrimaryPublish {
        check_airspeed_primary(self.params.primary, instance_count)
    }
}

/// Map stored `ARSPD_PRIMARY` to the clamped instance index.
#[must_use]
pub fn check_airspeed_primary(configured: u8, instance_count: u8) -> AirspeedPrimaryPublish {
    AirspeedPrimaryPublish {
        configured,
        clamped: clamp_primary(configured, instance_count),
    }
}

/// Publish the clamped `ARSPD_PRIMARY` from a SITL hookup.
#[must_use]
pub fn airspeed_primary_tick(hookup: &SitlAirspeedHookup) -> AirspeedPrimaryPublish {
    check_airspeed_primary(
        hookup.airspeed_params().primary,
        hookup.cluster().instance_count(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::primary::ARSPD_PRIMARY_DEFAULT;

    #[test]
    fn default_primary_is_first_instance() {
        let hookup = AirspeedPrimaryHookup::default();
        assert_eq!(hookup.airspeed_params().primary, ARSPD_PRIMARY_DEFAULT);
        let out = hookup.publish(2);
        assert_eq!(out.configured, 0);
        assert_eq!(out.clamped, 0);
    }

    #[test]
    fn configured_secondary_clamps_in_range() {
        let mut hookup = AirspeedPrimaryHookup::default();
        hookup.set_primary(1);
        let out = hookup.publish(2);
        assert_eq!(out.configured, 1);
        assert_eq!(out.clamped, 1);
        let oob = hookup.publish(1);
        assert_eq!(oob.clamped, 0);
    }
}
