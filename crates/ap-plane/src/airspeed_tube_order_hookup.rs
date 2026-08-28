//! ARSPD_TUBE_ORDER stub, upstream `AP_Airspeed::pitot_tube_order`.
//!
//! Remaps analog/SITL differential pressure sign so swapped pitot fittings
//! still publish a usable last-pressure and pitot airspeed.

use ap_airspeed::params::AirspeedParams;
use ap_airspeed::tube_order::{airspeed_from_pressure, last_pressure_pa};

/// Frontend tube-order hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedTubeOrderHookup {
    params: AirspeedParams,
}

impl Default for AirspeedTubeOrderHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// Pressure / airspeed after `ARSPD_TUBE_ORDER`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AirspeedTubeOrderPublish {
    /// Bound `ARSPD_TUBE_ORDER` / `ARSPD2_TUBE_ORDR`.
    pub tube_order: u8,
    /// Signed last pressure (Pa), upstream `state.last_pressure`.
    pub last_pressure_pa: f32,
    /// Pitot airspeed (m/s) after tube-order clamp / abs.
    pub airspeed_mps: f32,
}

impl AirspeedTubeOrderHookup {
    /// Build a tube-order hookup from instance params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` instance params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply `ARSPD_TUBE_ORDER` / `ARSPD2_TUBE_ORDR`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_TUBE_ORDER` on every enabled instance.
    pub fn set_tube_order(&mut self, tube_order: u8) {
        let mut params = self.params;
        params.airspeed1.tube_order = tube_order;
        params.airspeed2.tube_order = tube_order;
        self.params = params;
    }

    /// Remap raw differential pressure through `ARSPD_TUBE_ORDER`.
    #[must_use]
    pub fn publish(&self, raw_pressure_pa: f32) -> AirspeedTubeOrderPublish {
        apply_tube_order(
            raw_pressure_pa,
            self.params.primary_tube_order(),
            self.params.primary_ratio(),
        )
    }
}

/// Apply `ARSPD_TUBE_ORDER` to a raw differential-pressure sample.
#[must_use]
pub fn apply_tube_order(
    raw_pressure_pa: f32,
    tube_order: u8,
    ratio: f32,
) -> AirspeedTubeOrderPublish {
    AirspeedTubeOrderPublish {
        tube_order,
        last_pressure_pa: last_pressure_pa(raw_pressure_pa, tube_order),
        airspeed_mps: airspeed_from_pressure(raw_pressure_pa, ratio, tube_order),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::tube_order::{
        ARSPD_TUBE_ORDER_AUTO, ARSPD_TUBE_ORDER_NEGATIVE, ARSPD_TUBE_ORDER_POSITIVE,
    };

    #[test]
    fn default_order_is_auto() {
        let hookup = AirspeedTubeOrderHookup::default();
        assert_eq!(hookup.airspeed_params().primary_tube_order(), 2);
        let out = hookup.publish(-16.0);
        assert_eq!(out.tube_order, ARSPD_TUBE_ORDER_AUTO);
        assert!((out.last_pressure_pa - 16.0).abs() < 1e-5);
        // default ARSPD_RATIO is 2.0; sqrt(16 * 2) = sqrt(32)
        assert!((out.airspeed_mps - 32.0_f32.sqrt()).abs() < 1e-4);
    }

    #[test]
    fn negative_order_flips_positive_pressure() {
        let mut hookup = AirspeedTubeOrderHookup::default();
        hookup.set_tube_order(ARSPD_TUBE_ORDER_NEGATIVE);
        let out = hookup.publish(16.0);
        assert_eq!(out.tube_order, ARSPD_TUBE_ORDER_NEGATIVE);
        assert!((out.last_pressure_pa + 16.0).abs() < 1e-5);
        assert_eq!(out.airspeed_mps, 0.0);
    }

    #[test]
    fn positive_order_rejects_negative_pressure() {
        let mut hookup = AirspeedTubeOrderHookup::default();
        hookup.set_tube_order(ARSPD_TUBE_ORDER_POSITIVE);
        let out = hookup.publish(-16.0);
        assert!((out.last_pressure_pa + 16.0).abs() < 1e-5);
        assert_eq!(out.airspeed_mps, 0.0);
    }
}
