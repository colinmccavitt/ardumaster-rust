//! Analog airspeed pin stub, upstream `AP_Airspeed_Analog` / `ARSPD_PIN`.
//!
//! Binds a HAL analog source to the 3DR-scale voltage-to-pascal conversion so
//! a configured `ARSPD_PIN` publishes differential pressure.

use ap_airspeed::analog::AnalogAirspeedBackend;
use ap_airspeed::params::AirspeedParams;
use ap_airspeed::tube_order::last_pressure_pa;
use ap_hal::analog::MockAnalogSource;

/// Analog pitot hookup for the vehicle loop, fed by a mock/SITL analog source.
#[derive(Debug, Clone)]
pub struct AirspeedAnalogHookup {
    backend: AnalogAirspeedBackend<MockAnalogSource>,
    params: AirspeedParams,
}

impl Default for AirspeedAnalogHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// Differential pressure published from `ARSPD_PIN`.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct AirspeedAnalogPublish {
    /// Differential pressure (Pa), upstream `get_differential_pressure`.
    pub pressure_pa: f32,
    /// True when the analog source returned a sample.
    pub have_pressure: bool,
    /// Bound `ARSPD_PIN`.
    pub pin: i8,
    /// Bound `ARSPD_PSI_RANGE`.
    pub psi_range: f32,
    /// Bound `ARSPD_TUBE_ORDER`.
    pub tube_order: u8,
    /// Bound `ARSPD_BUS`.
    pub bus: u8,
    /// Bound `ARSPD_DEVID`.
    pub devid: i32,
}

impl AirspeedAnalogHookup {
    /// Build a mock analog hookup from instance params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        let mut hookup = Self {
            backend: AnalogAirspeedBackend::new(
                MockAnalogSource::new(),
                params.airspeed1.analog_config(),
            ),
            params,
        };
        let _ = hookup.backend.init();
        hookup
    }

    /// Current `ARSPD_*` instance params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply `ARSPD_PIN` / `ARSPD_PSI_RANGE` to the analog backend.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
        self.backend.set_config(params.airspeed1.analog_config());
        let _ = self.backend.init();
    }

    /// Set `ARSPD_PIN` on the primary analog instance.
    pub fn set_pin(&mut self, pin: i8) {
        let mut params = self.params;
        params.airspeed1.pin = pin;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_TYPE` on the primary analog instance.
    pub fn set_sensor_type(&mut self, sensor_type: u8) {
        let mut params = self.params;
        params.airspeed1.sensor_type = sensor_type;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_TUBE_ORDER` on the primary analog instance.
    pub fn set_tube_order(&mut self, tube_order: u8) {
        let mut params = self.params;
        params.airspeed1.tube_order = tube_order;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_BUS` on the primary analog instance.
    pub fn set_bus(&mut self, bus: u8) {
        let mut params = self.params;
        params.airspeed1.bus = bus;
        self.apply_airspeed_params(params);
    }

    /// Set `ARSPD_DEVID` on the primary analog instance.
    pub fn set_devid(&mut self, devid: i32) {
        let mut params = self.params;
        params.airspeed1.devid = devid;
        self.apply_airspeed_params(params);
    }

    /// Drive the mock analog source to a ratiometric voltage.
    pub fn set_voltage(&mut self, volts: f32) {
        let mut source = MockAnalogSource::new();
        source.set_counts(volts * 4095.0 / 3.3);
        self.backend = AnalogAirspeedBackend::new(source, self.params.airspeed1.analog_config());
        let _ = self.backend.init();
    }

    /// The analog backend bound to the mock pin.
    #[must_use]
    pub const fn backend(&self) -> &AnalogAirspeedBackend<MockAnalogSource> {
        &self.backend
    }

    /// Read `ARSPD_PIN` and publish differential pressure.
    #[must_use]
    pub fn publish(&mut self) -> AirspeedAnalogPublish {
        let pressure = self.backend.get_differential_pressure();
        let tube_order = self.params.primary_tube_order();
        AirspeedAnalogPublish {
            pressure_pa: pressure
                .map(|raw| last_pressure_pa(raw, tube_order))
                .unwrap_or(0.0),
            have_pressure: pressure.is_some(),
            pin: self.backend.config().pin,
            psi_range: self.backend.config().psi_range,
            tube_order,
            bus: self.params.primary_bus(),
            devid: self.params.primary_devid(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::analog::{ARSPD_PIN_DEFAULT, ARSPD_PIN_DISABLED, ARSPD_PSI_RANGE_DEFAULT};

    #[test]
    fn default_pin_is_zero() {
        let hookup = AirspeedAnalogHookup::default();
        assert_eq!(hookup.airspeed_params().primary_pin(), ARSPD_PIN_DEFAULT);
        assert!((hookup.airspeed_params().primary_psi_range() - ARSPD_PSI_RANGE_DEFAULT).abs() < 1e-6);
    }

    #[test]
    fn disabled_pin_publishes_no_pressure() {
        let mut hookup = AirspeedAnalogHookup::default();
        hookup.set_pin(ARSPD_PIN_DISABLED);
        hookup.set_voltage(1.0);
        hookup.set_pin(ARSPD_PIN_DISABLED);
        let out = hookup.publish();
        assert!(!out.have_pressure);
        assert_eq!(out.pin, ARSPD_PIN_DISABLED);
    }
}
