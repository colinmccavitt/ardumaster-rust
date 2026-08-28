//! ARSPD_DEVID device-id stub, upstream `AP_Airspeed_Params::bus_id`.
//!
//! Records the probe-assigned sensor ID (type + bus + instance). Default 0
//! means no sensor found. Digital backends latch `set_bus_id`; a failed
//! probe clears the ID so the next boot can rematch.

use ap_airspeed::devid::{devid_after_probe, devid_is_set};
use ap_airspeed::params::AirspeedParams;

/// Frontend DEVID hookup for the vehicle loop.
#[derive(Debug, Clone)]
pub struct AirspeedDevidHookup {
    params: AirspeedParams,
}

impl Default for AirspeedDevidHookup {
    fn default() -> Self {
        Self::from_params(AirspeedParams::default())
    }
}

/// Device ID published from `ARSPD_DEVID`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AirspeedDevidPublish {
    /// Bound `ARSPD_DEVID` / `ARSPD2_DEVID`.
    pub devid: i32,
    /// True when a sensor ID has been latched.
    pub is_set: bool,
}

impl AirspeedDevidHookup {
    /// Build a DEVID hookup from instance params.
    #[must_use]
    pub fn from_params(params: AirspeedParams) -> Self {
        Self { params }
    }

    /// Current `ARSPD_*` instance params.
    #[must_use]
    pub const fn airspeed_params(&self) -> &AirspeedParams {
        &self.params
    }

    /// Apply `ARSPD_DEVID` / `ARSPD2_DEVID`.
    pub fn apply_airspeed_params(&mut self, params: AirspeedParams) {
        self.params = params;
    }

    /// Set `ARSPD_DEVID` on every enabled instance.
    pub fn set_devid(&mut self, devid: i32) {
        let mut params = self.params;
        params.airspeed1.devid = devid;
        params.airspeed2.devid = devid;
        self.params = params;
    }

    /// Set `ARSPD_TYPE` so probe encoding can be tested.
    pub fn set_sensor_type(&mut self, sensor_type: u8) {
        let mut params = self.params;
        params.airspeed1.sensor_type = sensor_type;
        params.airspeed2.sensor_type = sensor_type;
        self.params = params;
    }

    /// Set `ARSPD_BUS` so I2C DEVID encoding can be tested.
    pub fn set_bus(&mut self, bus: u8) {
        let mut params = self.params;
        params.airspeed1.bus = bus;
        params.airspeed2.bus = bus;
        self.params = params;
    }

    /// Latch or clear DEVID after a backend probe, upstream `set_bus_id`.
    pub fn assign_from_probe(&mut self, found: bool) {
        let devid = devid_after_probe(
            found,
            self.params.primary_sensor_type(),
            self.params.primary_bus(),
            self.params.primary,
        );
        self.set_devid(devid);
    }

    /// Publish configured `ARSPD_DEVID`.
    #[must_use]
    pub fn publish(&self) -> AirspeedDevidPublish {
        select_airspeed_devid(self.params.primary_devid())
    }
}

/// Map stored `ARSPD_DEVID` to the published ID.
#[must_use]
pub fn select_airspeed_devid(devid: i32) -> AirspeedDevidPublish {
    AirspeedDevidPublish {
        devid,
        is_set: devid_is_set(devid),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ap_airspeed::backend::{ARSPD_TYPE_MS4525, ARSPD_TYPE_SITL};
    use ap_airspeed::bus::ARSPD_BUS_EXTERNAL2;
    use ap_airspeed::devid::{
        devid_bus, devid_bus_type, ARSPD_DEVID_DEFAULT, BUS_TYPE_I2C, BUS_TYPE_SITL,
    };

    #[test]
    fn default_devid_is_unset() {
        let hookup = AirspeedDevidHookup::default();
        assert_eq!(hookup.airspeed_params().primary_devid(), ARSPD_DEVID_DEFAULT);
        let out = hookup.publish();
        assert_eq!(out.devid, ARSPD_DEVID_DEFAULT);
        assert!(!out.is_set);
    }

    #[test]
    fn probe_assigns_sitl_and_ms4525_ids() {
        let mut hookup = AirspeedDevidHookup::default();
        hookup.set_sensor_type(ARSPD_TYPE_SITL);
        hookup.assign_from_probe(true);
        let sitl = hookup.publish();
        assert!(sitl.is_set);
        assert_eq!(devid_bus_type(sitl.devid as u32), BUS_TYPE_SITL);

        hookup.set_sensor_type(ARSPD_TYPE_MS4525);
        hookup.set_bus(ARSPD_BUS_EXTERNAL2);
        hookup.assign_from_probe(true);
        let digital = hookup.publish();
        assert!(digital.is_set);
        assert_eq!(devid_bus_type(digital.devid as u32), BUS_TYPE_I2C);
        assert_eq!(devid_bus(digital.devid as u32), ARSPD_BUS_EXTERNAL2);

        hookup.assign_from_probe(false);
        assert!(!hookup.publish().is_set);
    }
}
