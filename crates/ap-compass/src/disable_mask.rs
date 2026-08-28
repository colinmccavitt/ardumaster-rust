//! Compass disable driver-type mask stub, upstream `COMPASS_DISBLMSK`. FW-014.
//!
//! `_driver_type_mask` is a bitmask of `DriverType` values. If a bit is set,
//! that driver is not probed at startup (`Compass::_driver_enabled`). The SITL
//! backend is `DRIVER_SITL = 13`; masking it disables every SITL instance.

/// Upstream `COMPASS_DISBLMSK` default (`AP_GROUPINFO` `0`).
pub const COMPASS_DISBLMSK_DEFAULT: u32 = 0;

/// Upstream `Compass::DriverType` for `COMPASS_DISBLMSK`.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverType {
    /// `0:HMC5883`.
    Hmc5843 = 0,
    /// `1:LSM303D`.
    Lsm303d = 1,
    /// `2:AK8963`.
    Ak8963 = 2,
    /// `3:BMM150`.
    Bmm150 = 3,
    /// `4:LSM9DS1`.
    Lsm9ds1 = 4,
    /// `5:LIS3MDL`.
    Lis3mdl = 5,
    /// `6:AK0991x`.
    Ak09916 = 6,
    /// `7:IST8310`.
    Ist8310 = 7,
    /// `8:ICM20948`.
    Icm20948 = 8,
    /// `9:MMC3416`.
    Mmc3416 = 9,
    /// `11:DroneCAN`.
    Uavcan = 11,
    /// `12:QMC5883`.
    Qmc5883l = 12,
    /// `13:SITL`.
    Sitl = 13,
    /// `14:MAG3110`.
    Mag3110 = 14,
    /// `15:IST8308`.
    Ist8308 = 15,
    /// `16:RM3100`.
    Rm3100 = 16,
    /// `17:MSP`.
    Msp = 17,
    /// `18:ExternalAHRS`.
    ExternalAhrs = 18,
    /// `19:MMC5XX3`.
    Mmc5xx3 = 19,
    /// `20:QMC5883P`.
    Qmc5883p = 20,
    /// `21:BMM350`.
    Bmm350 = 21,
    /// `22:IIS2MDC or LIS2MDL`.
    Iis2mdc = 22,
}

impl DriverType {
    /// Bit in `COMPASS_DISBLMSK` for this driver, `1U << driver_type`.
    #[must_use]
    pub const fn mask_bit(self) -> u32 {
        1u32 << (self as u8)
    }

    /// Decode a known `DriverType` discriminant.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::Hmc5843),
            1 => Some(Self::Lsm303d),
            2 => Some(Self::Ak8963),
            3 => Some(Self::Bmm150),
            4 => Some(Self::Lsm9ds1),
            5 => Some(Self::Lis3mdl),
            6 => Some(Self::Ak09916),
            7 => Some(Self::Ist8310),
            8 => Some(Self::Icm20948),
            9 => Some(Self::Mmc3416),
            11 => Some(Self::Uavcan),
            12 => Some(Self::Qmc5883l),
            13 => Some(Self::Sitl),
            14 => Some(Self::Mag3110),
            15 => Some(Self::Ist8308),
            16 => Some(Self::Rm3100),
            17 => Some(Self::Msp),
            18 => Some(Self::ExternalAhrs),
            19 => Some(Self::Mmc5xx3),
            20 => Some(Self::Qmc5883p),
            21 => Some(Self::Bmm350),
            22 => Some(Self::Iis2mdc),
            _ => None,
        }
    }
}

/// Upstream `Compass::_driver_enabled`: true when the driver bit is clear.
#[must_use]
pub const fn driver_enabled(disable_mask: u32, driver: DriverType) -> bool {
    (disable_mask & driver.mask_bit()) == 0
}

/// True when `DRIVER_SITL` is not masked, so SITL instances may be probed.
#[must_use]
pub const fn sitl_enabled(disable_mask: u32) -> bool {
    driver_enabled(disable_mask, DriverType::Sitl)
}

/// Instance `i` is disabled when its per-instance flag is set **or** SITL
/// is masked. Upstream `_driver_enabled(DRIVER_SITL)` gates the backend.
#[must_use]
pub const fn instance_disabled(disable_mask: u32, instance_disabled: bool) -> bool {
    instance_disabled || !sitl_enabled(disable_mask)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_mask_enables_every_driver() {
        assert_eq!(COMPASS_DISBLMSK_DEFAULT, 0);
        assert!(driver_enabled(COMPASS_DISBLMSK_DEFAULT, DriverType::Sitl));
        assert!(sitl_enabled(COMPASS_DISBLMSK_DEFAULT));
        assert!(driver_enabled(
            COMPASS_DISBLMSK_DEFAULT,
            DriverType::Hmc5843
        ));
        assert!(!instance_disabled(COMPASS_DISBLMSK_DEFAULT, false));
        assert!(instance_disabled(COMPASS_DISBLMSK_DEFAULT, true));
    }

    #[test]
    fn sitl_bit_disables_sitl_only() {
        let mask = DriverType::Sitl.mask_bit();
        assert_eq!(mask, 1u32 << 13);
        assert!(!sitl_enabled(mask));
        assert!(driver_enabled(mask, DriverType::Hmc5843));
        assert!(instance_disabled(mask, false));
        assert!(instance_disabled(mask, true));
    }

    #[test]
    fn from_u8_maps_upstream_ids() {
        assert_eq!(DriverType::from_u8(13), Some(DriverType::Sitl));
        assert_eq!(DriverType::from_u8(5), Some(DriverType::Lis3mdl));
        assert_eq!(DriverType::from_u8(10), None);
        assert_eq!(DriverType::from_u8(23), None);
    }
}
