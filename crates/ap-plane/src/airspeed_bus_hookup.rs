//! ARSPD_BUS I2C bus stub, upstream AP_Airspeed_Params::bus.
//!
//! Records which I2C bus digital pitot backends probe. Analog and SITL
//! ignore the bus; unported I2C types still publish the configured bus.

use ap_airspeed::bus::{i2c_probe_bus, uses_i2c_bus};
use ap_airspeed::params::AirspeedParams;

/// Frontend I2C-bus hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedBusHookup {
    params: AirspeedParams,
}

impl Default for AirspeedBusHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// I2C bus selected from ARSPD_BUS.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirspeedBusPublish {
    /// Bound ARSPD_BUS / ARSPD2_BUS.
    pub bus: u8,
    /// Bus passed to hal.i2c_mgr->get_device.
    pub probe_bus: u8,
    /// True when ARSPD_TYPE is an I2C digital backend.
    pub uses_i2c: bool,
}

impl AirspeedBusHookup {
    /// Build an I2C-bus hookup from instance params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current ARSPD_* instance params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply ARSPD_BUS / ARSPD2_BUS.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set ARSPD_BUS on every enabled instance.
    pub fn set_bus(&mut self, bus: u8) {
        let mut params = self.params;
        params.airspeed1.bus = bus;
        params.airspeed2.bus = bus;
        self.params = params;
    }

    /// Set ARSPD_TYPE so I2C-vs-analog/SITL probe can be tested.
    pub fn set_sensor_type(&mut self, sensor_type: u8) {
        let mut params = self.params;
        params.airspeed1.sensor_type = sensor_type;
        params.airspeed2.sensor_type = sensor_type;
        self.params = params;
    }

    /// Publish configured I2C bus from ARSPD_BUS.
    #[must_use]
    pub fn publish(&self) -> AirspeedBusPublish {
        select_airspeed_bus(
            self.params.primary_bus(),
            self.params.primary_sensor_type(),
        )
    }
}

/// Map ARSPD_BUS / ARSPD_TYPE to the I2C probe bus.
#[must_use]
pub fn select_airspeed_bus(bus: u8, sensor_type: u8) -> AirspeedBusPublish {
    AirspeedBusPublish {
        bus,
        probe_bus: i2c_probe_bus(bus),
        uses_i2c: uses_i2c_bus(sensor_type),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::backend::{ARSPD_TYPE_ANALOG, ARSPD_TYPE_MS4525, ARSPD_TYPE_SITL};
    use ap_airspeed::bus::{ARSPD_BUS_DEFAULT, ARSPD_BUS_EXTERNAL2, ARSPD_BUS_INTERNAL};

    #[test]
    fn default_bus_is_external() {
        let hookup = AirspeedBusHookup::default();
        assert_eq!(hookup.airspeed_params().primary_bus(), ARSPD_BUS_DEFAULT);
        let out = hookup.publish();
        assert_eq!(out.bus, ARSPD_BUS_DEFAULT);
        assert_eq!(out.probe_bus, 1);
        assert!(!out.uses_i2c);
        assert_eq!(hookup.airspeed_params().primary_sensor_type(), ARSPD_TYPE_SITL);
    }

    #[test]
    fn internal_bus_is_published() {
        let mut hookup = AirspeedBusHookup::default();
        hookup.set_bus(ARSPD_BUS_INTERNAL);
        let out = hookup.publish();
        assert_eq!(out.bus, ARSPD_BUS_INTERNAL);
        assert_eq!(out.probe_bus, 0);
    }

    #[test]
    fn ms4525_uses_i2c_on_external2() {
        let mut hookup = AirspeedBusHookup::default();
        hookup.set_sensor_type(ARSPD_TYPE_MS4525);
        hookup.set_bus(ARSPD_BUS_EXTERNAL2);
        let out = hookup.publish();
        assert_eq!(out.bus, 2);
        assert!(out.uses_i2c);
        hookup.set_sensor_type(ARSPD_TYPE_ANALOG);
        assert!(!hookup.publish().uses_i2c);
    }
}
