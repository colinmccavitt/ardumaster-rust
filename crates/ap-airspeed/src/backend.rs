//! Airspeed backend selection, upstream `AP_Airspeed::airspeed_type` / `ARSPD_TYPE`.
//!
//! SITL and analog are implemented. Other `ARSPD_TYPE` values stay unported and
//! fall back to [`AirspeedBackendKind::None`].

/// Upstream `AP_Airspeed::TYPE_NONE`.
pub const ARSPD_TYPE_NONE: u8 = 0;

/// Upstream `AP_Airspeed::TYPE_I2C_MS4525` (unported).
pub const ARSPD_TYPE_MS4525: u8 = 1;

/// Upstream `AP_Airspeed::TYPE_ANALOG`.
pub const ARSPD_TYPE_ANALOG: u8 = 2;

/// Upstream `AP_Airspeed::TYPE_SITL`.
pub const ARSPD_TYPE_SITL: u8 = 100;

/// SITL plane default for `ARSPD_TYPE` (param-table default is 0 / None).
pub const ARSPD_TYPE_DEFAULT: u8 = ARSPD_TYPE_SITL;

/// Configured airspeed driver, upstream `AP_Airspeed::airspeed_type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AirspeedBackendKind {
    /// No sensor, upstream `TYPE_NONE`.
    None,
    /// Analog pitot, upstream `TYPE_ANALOG` / `AP_Airspeed_Analog`.
    Analog,
    /// SITL pitot, upstream `TYPE_SITL` / `AP_Airspeed_SITL`.
    Sitl,
    /// Unported `ARSPD_TYPE` value.
    Other(u8),
}

impl Default for AirspeedBackendKind {
    fn default() -> Self {
        Self::Sitl
    }
}

impl AirspeedBackendKind {
    /// Map `ARSPD_TYPE` / `ARSPD2_TYPE`, upstream constructor switch.
    #[must_use]
    pub const fn from_type_param(sensor_type: u8) -> Self {
        match sensor_type {
            ARSPD_TYPE_NONE => Self::None,
            ARSPD_TYPE_ANALOG => Self::Analog,
            ARSPD_TYPE_SITL => Self::Sitl,
            other => Self::Other(other),
        }
    }

    /// Whether this backend is implemented in the port.
    #[must_use]
    pub const fn is_available(self) -> bool {
        matches!(self, Self::None | Self::Analog | Self::Sitl)
    }
}

/// Map `ARSPD_TYPE` to a backend kind.
#[must_use]
pub const fn backend_kind_from_type(sensor_type: u8) -> AirspeedBackendKind {
    AirspeedBackendKind::from_type_param(sensor_type)
}

/// Resolve configured type to an allocated backend; unported types become None.
#[must_use]
pub const fn backend_for_kind(kind: AirspeedBackendKind) -> AirspeedBackendKind {
    match kind {
        AirspeedBackendKind::Other(_) => AirspeedBackendKind::None,
        kind => kind,
    }
}

/// Active backend after unported-type fallback, upstream `add_backend`.
#[must_use]
pub const fn active_backend_kind(configured: AirspeedBackendKind) -> AirspeedBackendKind {
    backend_for_kind(configured)
}

/// Upstream `ARSPD_TYPE != TYPE_NONE`.
#[must_use]
pub const fn airspeed_type_enabled(sensor_type: u8) -> bool {
    sensor_type != ARSPD_TYPE_NONE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sitl_and_analog_are_available() {
        assert!(AirspeedBackendKind::Sitl.is_available());
        assert!(AirspeedBackendKind::Analog.is_available());
        assert!(AirspeedBackendKind::None.is_available());
        assert!(!AirspeedBackendKind::Other(ARSPD_TYPE_MS4525).is_available());
    }

    #[test]
    fn from_type_param_matches_upstream() {
        assert_eq!(
            AirspeedBackendKind::from_type_param(ARSPD_TYPE_NONE),
            AirspeedBackendKind::None
        );
        assert_eq!(
            AirspeedBackendKind::from_type_param(ARSPD_TYPE_ANALOG),
            AirspeedBackendKind::Analog
        );
        assert_eq!(
            AirspeedBackendKind::from_type_param(ARSPD_TYPE_SITL),
            AirspeedBackendKind::Sitl
        );
        assert_eq!(
            AirspeedBackendKind::from_type_param(ARSPD_TYPE_MS4525),
            AirspeedBackendKind::Other(ARSPD_TYPE_MS4525)
        );
        assert_eq!(ARSPD_TYPE_DEFAULT, ARSPD_TYPE_SITL);
    }

    #[test]
    fn unported_type_falls_back_to_none() {
        assert_eq!(
            active_backend_kind(AirspeedBackendKind::Other(ARSPD_TYPE_MS4525)),
            AirspeedBackendKind::None
        );
        assert_eq!(
            active_backend_kind(AirspeedBackendKind::Sitl),
            AirspeedBackendKind::Sitl
        );
        assert!(!airspeed_type_enabled(ARSPD_TYPE_NONE));
        assert!(airspeed_type_enabled(ARSPD_TYPE_SITL));
    }
}
