//! ARSPD_TYPE backend selection stub, upstream `AP_Airspeed::airspeed_type`.
//!
//! Maps `ARSPD_TYPE` to a backend kind. SITL and analog are implemented;
//! other types stay unported and fall back to None.

use ap_airspeed::backend::{
    active_backend_kind, airspeed_type_enabled, backend_kind_from_type, AirspeedBackendKind,
};
use ap_airspeed::params::AirspeedParams;

/// Frontend type-selection hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedTypeHookup {
    params: AirspeedParams,
}

impl Default for AirspeedTypeHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// Backend selected from `ARSPD_TYPE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirspeedTypePublish {
    /// Bound `ARSPD_TYPE` / `ARSPD2_TYPE`.
    pub sensor_type: u8,
    /// Configured backend before unported fallback.
    pub configured: AirspeedBackendKind,
    /// Active backend after unported types fall back to None.
    pub active: AirspeedBackendKind,
    /// True when a ported, non-None backend is selected.
    pub enabled: bool,
}

impl AirspeedTypeHookup {
    /// Build a type-selection hookup from instance params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` instance params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply `ARSPD_TYPE` / `ARSPD2_TYPE`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_TYPE` on every enabled instance.
    pub fn set_sensor_type(&mut self, sensor_type: u8) {
        let mut params = self.params;
        params.airspeed1.sensor_type = sensor_type;
        params.airspeed2.sensor_type = sensor_type;
        self.params = params;
    }

    /// Publish configured and active backends from `ARSPD_TYPE`.
    #[must_use]
    pub fn publish(&self) -> AirspeedTypePublish {
        select_airspeed_backend(self.params.primary_sensor_type())
    }
}

/// Map `ARSPD_TYPE` to configured/active backends.
#[must_use]
pub fn select_airspeed_backend(sensor_type: u8) -> AirspeedTypePublish {
    let configured = backend_kind_from_type(sensor_type);
    let active = active_backend_kind(configured);
    AirspeedTypePublish {
        sensor_type,
        configured,
        active,
        enabled: airspeed_type_enabled(sensor_type) && active != AirspeedBackendKind::None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::backend::{ARSPD_TYPE_ANALOG, ARSPD_TYPE_NONE, ARSPD_TYPE_SITL};

    #[test]
    fn default_type_is_sitl() {
        let hookup = AirspeedTypeHookup::default();
        assert_eq!(
            hookup.airspeed_params().primary_sensor_type(),
            ARSPD_TYPE_SITL
        );
        let out = hookup.publish();
        assert_eq!(out.configured, AirspeedBackendKind::Sitl);
        assert_eq!(out.active, AirspeedBackendKind::Sitl);
        assert!(out.enabled);
    }

    #[test]
    fn none_disables_backend() {
        let mut hookup = AirspeedTypeHookup::default();
        hookup.set_sensor_type(ARSPD_TYPE_NONE);
        let out = hookup.publish();
        assert_eq!(out.active, AirspeedBackendKind::None);
        assert!(!out.enabled);
    }

    #[test]
    fn analog_type_stays_on_analog() {
        let mut hookup = AirspeedTypeHookup::default();
        hookup.set_sensor_type(ARSPD_TYPE_ANALOG);
        let out = hookup.publish();
        assert_eq!(out.configured, AirspeedBackendKind::Analog);
        assert_eq!(out.active, AirspeedBackendKind::Analog);
        assert!(out.enabled);
    }
}
