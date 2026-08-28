//! Device ID, upstream `AP_Airspeed_Params::bus_id` / `ARSPD_DEVID`.
//!
//! Encodes type, bus, and instance as `AP_HAL::Device::make_bus_id`. Default 0
//! means no sensor found. Digital backends call `set_bus_id` on a successful
//! probe; missing sensors are cleared to 0 so the next boot can rematch.

use crate::backend::{ARSPD_TYPE_ANALOG, ARSPD_TYPE_NONE, ARSPD_TYPE_SITL};

/// Unset / not-found ID, upstream `AP_GROUPINFO DEVID` default 0.
pub const ARSPD_DEVID_DEFAULT: i32 = 0;

/// Upstream `AP_HAL::Device::BUS_TYPE_I2C`.
pub const BUS_TYPE_I2C: u8 = 1;
/// Upstream `AP_HAL::Device::BUS_TYPE_SITL`.
pub const BUS_TYPE_SITL: u8 = 4;

/// First MS4525 I2C address, upstream `MS4525D0_I2C_ADDR1`.
pub const MS4525_I2C_ADDR: u8 = 0x28;

/// Pack a 32-bit DEVID, upstream `AP_HAL::Device::make_bus_id`.
#[must_use]
pub const fn make_bus_id(bus_type: u8, bus: u8, address: u8, devtype: u8) -> u32 {
    let bus_type = bus_type & 0x07;
    let bus = bus & 0x1F;
    (bus_type as u32) | ((bus as u32) << 3) | ((address as u32) << 8) | ((devtype as u32) << 16)
}

/// Bus type nibble, upstream `devid_get_bus_type`.
#[must_use]
pub const fn devid_bus_type(id: u32) -> u8 {
    (id & 0x07) as u8
}

/// Bus instance, upstream `devid_get_bus`.
#[must_use]
pub const fn devid_bus(id: u32) -> u8 {
    ((id >> 3) & 0x1F) as u8
}

/// Address on the bus, upstream `devid_get_address`.
#[must_use]
pub const fn devid_address(id: u32) -> u8 {
    ((id >> 8) & 0xFF) as u8
}

/// Device-class type, upstream `devid_get_devtype`.
#[must_use]
pub const fn devid_devtype(id: u32) -> u8 {
    ((id >> 16) & 0xFF) as u8
}

/// Whether `ARSPD_DEVID` is latched, upstream `bus_id != 0`.
#[must_use]
pub const fn devid_is_set(id: i32) -> bool {
    id != 0
}

/// Clear DEVID when the backend is missing, upstream not-found pass.
#[must_use]
pub const fn clear_devid_if_not_found(found: bool, id: i32) -> i32 {
    if found {
        id
    } else {
        ARSPD_DEVID_DEFAULT
    }
}

/// Probe-assigned DEVID for a configured `ARSPD_TYPE` / `ARSPD_BUS` / instance.
///
/// Analog and `TYPE_NONE` stay 0 (no `Device`). SITL uses `BUS_TYPE_SITL`.
/// Unported I2C types encode bus + the first MS4525 address.
#[must_use]
pub const fn devid_for_configured(sensor_type: u8, bus: u8, instance: u8) -> i32 {
    match sensor_type {
        ARSPD_TYPE_NONE | ARSPD_TYPE_ANALOG => ARSPD_DEVID_DEFAULT,
        ARSPD_TYPE_SITL => make_bus_id(BUS_TYPE_SITL, 0, instance, ARSPD_TYPE_SITL) as i32,
        other => make_bus_id(BUS_TYPE_I2C, bus, MS4525_I2C_ADDR, other) as i32,
    }
}

/// Assign or clear DEVID after probe, upstream `set_bus_id` / clear-if-not-found.
#[must_use]
pub const fn devid_after_probe(found: bool, sensor_type: u8, bus: u8, instance: u8) -> i32 {
    clear_devid_if_not_found(found, devid_for_configured(sensor_type, bus, instance))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::ARSPD_TYPE_MS4525;

    #[test]
    fn default_devid_is_unset() {
        assert_eq!(ARSPD_DEVID_DEFAULT, 0);
        assert!(!devid_is_set(ARSPD_DEVID_DEFAULT));
        assert_eq!(devid_for_configured(ARSPD_TYPE_NONE, 1, 0), 0);
        assert_eq!(devid_for_configured(ARSPD_TYPE_ANALOG, 1, 0), 0);
    }

    #[test]
    fn make_bus_id_packs_hal_bitfields() {
        let id = make_bus_id(BUS_TYPE_I2C, 1, MS4525_I2C_ADDR, ARSPD_TYPE_MS4525);
        assert_eq!(devid_bus_type(id), BUS_TYPE_I2C);
        assert_eq!(devid_bus(id), 1);
        assert_eq!(devid_address(id), MS4525_I2C_ADDR);
        assert_eq!(devid_devtype(id), ARSPD_TYPE_MS4525);
        assert_eq!(id, 1 | (1 << 3) | (0x28 << 8) | (1 << 16));
    }

    #[test]
    fn sitl_and_ms4525_probe_assign_ids() {
        let sitl = devid_after_probe(true, ARSPD_TYPE_SITL, 1, 0);
        assert!(devid_is_set(sitl));
        assert_eq!(devid_bus_type(sitl as u32), BUS_TYPE_SITL);
        assert_eq!(devid_devtype(sitl as u32), ARSPD_TYPE_SITL);

        let ms4525 = devid_after_probe(true, ARSPD_TYPE_MS4525, 2, 0);
        assert_eq!(devid_bus_type(ms4525 as u32), BUS_TYPE_I2C);
        assert_eq!(devid_bus(ms4525 as u32), 2);
        assert_eq!(devid_address(ms4525 as u32), MS4525_I2C_ADDR);

        assert_eq!(devid_after_probe(false, ARSPD_TYPE_MS4525, 2, 0), 0);
    }
}
