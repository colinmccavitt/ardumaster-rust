//! Pitot tube order, upstream `AP_Airspeed::pitot_tube_order` / `ARSPD_TUBE_ORDER`.
//!
//! Remaps differential pressure sign before airspeed is taken. Default is
//! auto-detect (2): either connector order is accepted.

use ap_math::scalar::safe_sqrt;

/// Upstream `PITOT_TUBE_ORDER_POSITIVE` / `ARSPD_TUBE_ORDER = 0`.
pub const ARSPD_TUBE_ORDER_POSITIVE: u8 = 0;

/// Upstream `PITOT_TUBE_ORDER_NEGATIVE` / `ARSPD_TUBE_ORDER = 1`.
pub const ARSPD_TUBE_ORDER_NEGATIVE: u8 = 1;

/// Upstream `PITOT_TUBE_ORDER_AUTO` / `ARSPD_TUBE_ORDER = 2`.
pub const ARSPD_TUBE_ORDER_AUTO: u8 = 2;

/// Param-table default, upstream `AP_GROUPINFO("TUBE_ORDR", ..., 2)`.
pub const ARSPD_TUBE_ORDER_DEFAULT: u8 = ARSPD_TUBE_ORDER_AUTO;

/// Upstream `AP_Airspeed::pitot_tube_order`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PitotTubeOrder {
    /// First (often top) connector is stagnation pressure.
    Positive,
    /// Second (often bottom) connector is stagnation pressure.
    Negative,
    /// Accept either order, upstream default.
    Auto,
}

impl Default for PitotTubeOrder {
    fn default() -> Self {
        Self::Auto
    }
}

impl PitotTubeOrder {
    /// Map `ARSPD_TUBE_ORDER` / `ARSPD2_TUBE_ORDR`. Unknown values are Auto.
    #[must_use]
    pub const fn from_param(tube_order: u8) -> Self {
        match tube_order {
            ARSPD_TUBE_ORDER_POSITIVE => Self::Positive,
            ARSPD_TUBE_ORDER_NEGATIVE => Self::Negative,
            _ => Self::Auto,
        }
    }

    /// Param value for this order.
    #[must_use]
    pub const fn as_param(self) -> u8 {
        match self {
            Self::Positive => ARSPD_TUBE_ORDER_POSITIVE,
            Self::Negative => ARSPD_TUBE_ORDER_NEGATIVE,
            Self::Auto => ARSPD_TUBE_ORDER_AUTO,
        }
    }
}

/// Signed last pressure after tube-order, upstream `state.last_pressure`.
#[must_use]
pub fn last_pressure_pa(raw_pressure: f32, tube_order: u8) -> f32 {
    match PitotTubeOrder::from_param(tube_order) {
        PitotTubeOrder::Negative => -raw_pressure,
        PitotTubeOrder::Positive => raw_pressure,
        PitotTubeOrder::Auto => raw_pressure.abs(),
    }
}

/// Pitot airspeed (m/s) from differential pressure, upstream `sqrtf(...)`.
#[must_use]
pub fn airspeed_from_pressure(pressure: f32, ratio: f32, tube_order: u8) -> f32 {
    let q = match PitotTubeOrder::from_param(tube_order) {
        PitotTubeOrder::Negative => (-pressure).max(0.0),
        PitotTubeOrder::Positive => pressure.max(0.0),
        PitotTubeOrder::Auto => pressure.abs(),
    };
    safe_sqrt(q * ratio.max(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_is_auto() {
        assert_eq!(ARSPD_TUBE_ORDER_DEFAULT, 2);
        assert_eq!(PitotTubeOrder::default(), PitotTubeOrder::Auto);
        assert_eq!(PitotTubeOrder::from_param(2), PitotTubeOrder::Auto);
        assert_eq!(PitotTubeOrder::from_param(99), PitotTubeOrder::Auto);
        assert_eq!(PitotTubeOrder::from_param(0), PitotTubeOrder::Positive);
        assert_eq!(PitotTubeOrder::from_param(1), PitotTubeOrder::Negative);
    }

    #[test]
    fn last_pressure_follows_upstream_sign() {
        assert!((last_pressure_pa(819.0, ARSPD_TUBE_ORDER_POSITIVE) - 819.0).abs() < 1e-4);
        assert!((last_pressure_pa(819.0, ARSPD_TUBE_ORDER_NEGATIVE) + 819.0).abs() < 1e-4);
        assert!((last_pressure_pa(-819.0, ARSPD_TUBE_ORDER_AUTO) - 819.0).abs() < 1e-4);
        assert!((last_pressure_pa(-819.0, ARSPD_TUBE_ORDER_POSITIVE) + 819.0).abs() < 1e-4);
    }

    #[test]
    fn airspeed_clamps_wrong_sign() {
        assert!((airspeed_from_pressure(16.0, 1.0, ARSPD_TUBE_ORDER_POSITIVE) - 4.0).abs() < 1e-5);
        assert_eq!(airspeed_from_pressure(-16.0, 1.0, ARSPD_TUBE_ORDER_POSITIVE), 0.0);
        assert!((airspeed_from_pressure(-16.0, 1.0, ARSPD_TUBE_ORDER_NEGATIVE) - 4.0).abs() < 1e-5);
        assert_eq!(airspeed_from_pressure(16.0, 1.0, ARSPD_TUBE_ORDER_NEGATIVE), 0.0);
        assert!((airspeed_from_pressure(-16.0, 1.0, ARSPD_TUBE_ORDER_AUTO) - 4.0).abs() < 1e-5);
    }
}
