//! I2C bus selection, upstream AP_Airspeed_Params::bus / ARSPD_BUS.
//!
//! Digital pitot backends (MS4525 and other I2C types) probe
//! hal.i2c_mgr->get_device(bus, addr). Analog and SITL ignore the bus.

use crate::backend::{ARSPD_TYPE_ANALOG, ARSPD_TYPE_NONE, ARSPD_TYPE_SITL};

/// Internal I2C, ARSPD_BUS = 0.
pub const ARSPD_BUS_INTERNAL: u8 = 0;

/// External I2C, ARSPD_BUS = 1.
pub const ARSPD_BUS_EXTERNAL: u8 = 1;

/// Second external / AHRS bus, ARSPD_BUS = 2.
pub const ARSPD_BUS_EXTERNAL2: u8 = 2;

/// Param-table default, upstream AP_GROUPINFO BUS default 1.
pub const ARSPD_BUS_DEFAULT: u8 = ARSPD_BUS_EXTERNAL;

/// Whether ARSPD_TYPE probes I2C. Analog, SITL, and none do not.
#[must_use]
pub const fn uses_i2c_bus(sensor_type: u8) -> bool {
    !matches!(
        sensor_type,
        ARSPD_TYPE_NONE | ARSPD_TYPE_ANALOG | ARSPD_TYPE_SITL
    )
}

/// Bus number passed to hal.i2c_mgr->get_device, upstream constructor.
#[must_use]
pub const fn i2c_probe_bus(bus: u8) -> u8 {
    bus
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::ARSPD_TYPE_MS4525;

    #[test]
    fn default_bus_is_external() {
        assert_eq!(ARSPD_BUS_DEFAULT, 1);
        assert_eq!(ARSPD_BUS_INTERNAL, 0);
        assert_eq!(ARSPD_BUS_EXTERNAL, 1);
        assert_eq!(ARSPD_BUS_EXTERNAL2, 2);
        assert_eq!(i2c_probe_bus(ARSPD_BUS_DEFAULT), 1);
        assert_eq!(i2c_probe_bus(0), 0);
    }

    #[test]
    fn only_digital_types_use_i2c() {
        assert!(!uses_i2c_bus(ARSPD_TYPE_NONE));
        assert!(!uses_i2c_bus(ARSPD_TYPE_ANALOG));
        assert!(!uses_i2c_bus(ARSPD_TYPE_SITL));
        assert!(uses_i2c_bus(ARSPD_TYPE_MS4525));
    }
}
