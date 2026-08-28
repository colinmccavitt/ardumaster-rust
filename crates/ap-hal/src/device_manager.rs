//! I2C/SPI device-manager factory, ported from `AP_HAL/I2CDevice.h` and
//! `SPIDevice.h` `get_device`.
//!
//! Upstream `I2CDeviceManager::get_device` / `SPIDeviceManager::get_device`
//! return a heap `OwnPtr`. The crate is `no_std` with no allocator, so the
//! factory hands back a [`DeviceHandle`] into a fixed slot table — the same
//! substitution [`crate::device::PeriodicHandle`] makes for `void*`.
//!
//! I2C looks up by bus+address (`get_device_ptr(bus, address, ...)`). SPI
//! looks up by name (`get_device_ptr(name)`). A second get of the same key
//! reuses the slot: there is no heap to allocate a fresh handle each call.

use crate::device::{
    BusType, Device, DeviceHandle, I2cDevice, I2cDeviceManager, MockDevice, SpiDevice,
    SpiDeviceManager,
};
use crate::{Error, Result};

/// Shared bus+address factory. Upstream has no common `DeviceManager` class;
/// both I2C and SPI managers expose `get_device`, and the port uses this
/// trait for the no_std table lookup both sides share.
pub trait DeviceManager {
    /// Look up or allocate a device on `bus` at `address`.
    ///
    /// I2C: `bus` is the I2C instance, `address` is the 7-bit address.
    /// SPI: `bus` is the SPI instance, `address` is the chip-select index.
    fn get_device(&mut self, bus: u8, address: u8) -> Result<DeviceHandle>;

    /// Borrow the device at `handle`.
    fn device(&mut self, handle: DeviceHandle) -> Result<&mut dyn Device>;
}

/// Slots in the table-backed manager. SITL exposes four I2C buses; four
/// devices is enough for a stub and makes a full table easy to test.
pub const TABLE_SLOTS: usize = 4;

/// Bytes reserved for an SPI device name (`const char*` in C++).
const NAME_CAP: usize = 16;

/// One occupied slot: identity + the in-memory [`MockDevice`].
#[derive(Debug)]
struct Slot {
    name: [u8; NAME_CAP],
    name_len: u8,
    device: MockDevice,
}

impl Slot {
    fn i2c(bus: u8, address: u8) -> Self {
        Self {
            name: [0; NAME_CAP],
            name_len: 0,
            device: MockDevice::i2c(bus, address),
        }
    }

    fn spi(bus: u8, address: u8, name: &str) -> Result<Self> {
        let bytes = name.as_bytes();
        if bytes.len() > NAME_CAP {
            return Err(Error::Unsupported);
        }
        let mut stored = [0u8; NAME_CAP];
        stored[..bytes.len()].copy_from_slice(bytes);
        Ok(Self {
            name: stored,
            name_len: bytes.len() as u8,
            device: MockDevice::spi(bus, address),
        })
    }

    fn name(&self) -> Option<&str> {
        let n = usize::from(self.name_len);
        if n == 0 {
            return None;
        }
        core::str::from_utf8(self.name.get(..n)?).ok()
    }

    fn matches_bus_address(&self, bus: u8, address: u8) -> bool {
        self.device.bus_num() == bus && self.device.bus_address() == address
    }
}

/// Table-backed [`DeviceManager`] / [`I2cDeviceManager`] / [`SpiDeviceManager`].
///
/// Occupied slots are the OwnPtr stand-in. A bus whose bit is clear in
/// [`I2cDeviceManager::bus_mask`] is rejected the way SITL returns `nullptr`
/// for `bus >= NUM_SITL_I2C_BUSES`.
#[derive(Debug)]
pub struct TableDeviceManager {
    slots: [Option<Slot>; TABLE_SLOTS],
    kind: BusType,
    mask: u32,
    mask_external: u32,
    mask_internal: u32,
}

impl TableDeviceManager {
    /// I2C manager with the C++ base-class masks (`0x0F` / `0x0F` / `0x01`).
    #[inline]
    pub fn i2c() -> Self {
        Self {
            slots: [None, None, None, None],
            kind: BusType::I2c,
            mask: 0x0F,
            mask_external: 0x0F,
            mask_internal: 0x01,
        }
    }

    /// SPI manager. Upstream `get_count()` starts at 0.
    #[inline]
    pub fn spi() -> Self {
        Self {
            slots: [None, None, None, None],
            kind: BusType::Spi,
            mask: 0x0F,
            mask_external: 0x0F,
            mask_internal: 0x01,
        }
    }

    /// Number of occupied slots.
    #[inline]
    pub fn occupied(&self) -> u8 {
        self.slots.iter().filter(|s| s.is_some()).count() as u8
    }

    /// The in-memory device at `handle`, if the slot is live.
    pub fn device_mut(&mut self, handle: DeviceHandle) -> Result<&mut MockDevice> {
        Ok(&mut self.slot_mut(handle)?.device)
    }

    fn slot_index(handle: DeviceHandle) -> Option<usize> {
        let n = handle.0;
        if n == 0 {
            return None;
        }
        let i = usize::from(n) - 1;
        if i < TABLE_SLOTS {
            Some(i)
        } else {
            None
        }
    }

    fn handle_for(index: usize) -> DeviceHandle {
        DeviceHandle((index + 1) as u8)
    }

    fn slot_mut(&mut self, handle: DeviceHandle) -> Result<&mut Slot> {
        let i = Self::slot_index(handle).ok_or(Error::NotPresent)?;
        self.slots
            .get_mut(i)
            .and_then(Option::as_mut)
            .ok_or(Error::NotPresent)
    }

    fn bus_allowed(&self, bus: u8) -> bool {
        if bus >= 32 {
            return false;
        }
        (self.mask & (1u32 << bus)) != 0
    }

    fn find_bus_address(&self, bus: u8, address: u8) -> Option<usize> {
        self.slots.iter().enumerate().find_map(|(i, slot)| {
            slot.as_ref()
                .filter(|s| s.matches_bus_address(bus, address))
                .map(|_| i)
        })
    }

    fn find_name(&self, name: &str) -> Option<usize> {
        self.slots
            .iter()
            .enumerate()
            .find_map(|(i, slot)| slot.as_ref().filter(|s| s.name() == Some(name)).map(|_| i))
    }

    fn first_free(&self) -> Option<usize> {
        self.slots.iter().position(Option::is_none)
    }

    fn alloc_i2c(&mut self, bus: u8, address: u8) -> Result<DeviceHandle> {
        if self.kind != BusType::I2c {
            return Err(Error::Unsupported);
        }
        if !self.bus_allowed(bus) {
            return Err(Error::NotPresent);
        }
        if let Some(i) = self.find_bus_address(bus, address) {
            return Ok(Self::handle_for(i));
        }
        let i = self.first_free().ok_or(Error::NotPresent)?;
        self.slots[i] = Some(Slot::i2c(bus, address));
        Ok(Self::handle_for(i))
    }

    fn alloc_spi_named(&mut self, name: &str) -> Result<DeviceHandle> {
        if self.kind != BusType::Spi {
            return Err(Error::Unsupported);
        }
        if name.is_empty() {
            return Err(Error::Unsupported);
        }
        if let Some(i) = self.find_name(name) {
            return Ok(Self::handle_for(i));
        }
        let i = self.first_free().ok_or(Error::NotPresent)?;
        // CS index is the slot so two names do not collide on (0, 0).
        self.slots[i] = Some(Slot::spi(0, i as u8, name)?);
        Ok(Self::handle_for(i))
    }

    fn alloc_spi_bus_address(&mut self, bus: u8, address: u8) -> Result<DeviceHandle> {
        if self.kind != BusType::Spi {
            return Err(Error::Unsupported);
        }
        if !self.bus_allowed(bus) {
            return Err(Error::NotPresent);
        }
        if let Some(i) = self.find_bus_address(bus, address) {
            return Ok(Self::handle_for(i));
        }
        let i = self.first_free().ok_or(Error::NotPresent)?;
        self.slots[i] = Some(Slot::spi(bus, address, "")?);
        Ok(Self::handle_for(i))
    }
}

impl DeviceManager for TableDeviceManager {
    fn get_device(&mut self, bus: u8, address: u8) -> Result<DeviceHandle> {
        match self.kind {
            BusType::I2c => self.alloc_i2c(bus, address),
            BusType::Spi => self.alloc_spi_bus_address(bus, address),
            _ => Err(Error::Unsupported),
        }
    }

    fn device(&mut self, handle: DeviceHandle) -> Result<&mut dyn Device> {
        Ok(self.device_mut(handle)?)
    }
}

impl I2cDeviceManager for TableDeviceManager {
    fn get_device(&mut self, bus: u8, address: u8) -> Result<DeviceHandle> {
        self.alloc_i2c(bus, address)
    }

    fn bus_mask(&self) -> u32 {
        self.mask
    }

    fn bus_mask_external(&self) -> u32 {
        self.mask_external
    }

    fn bus_mask_internal(&self) -> u32 {
        self.mask_internal
    }
}

impl SpiDeviceManager for TableDeviceManager {
    fn get_device(&mut self, name: &str) -> Result<DeviceHandle> {
        self.alloc_spi_named(name)
    }

    fn get_device_name(&self, idx: u8) -> Option<&str> {
        let mut seen = 0u8;
        for slot in self.slots.iter().flatten() {
            if seen == idx {
                return slot.name();
            }
            seen = seen.saturating_add(1);
        }
        None
    }

    fn count(&self) -> u8 {
        self.occupied()
    }
}

impl TableDeviceManager {
    /// Borrow the slot as [`I2cDevice`] when this is an I2C manager.
    pub fn i2c_device(&mut self, handle: DeviceHandle) -> Result<&mut dyn I2cDevice> {
        if self.kind != BusType::I2c {
            return Err(Error::Unsupported);
        }
        Ok(self.device_mut(handle)?)
    }

    /// Borrow the slot as [`SpiDevice`] when this is an SPI manager.
    pub fn spi_device(&mut self, handle: DeviceHandle) -> Result<&mut dyn SpiDevice> {
        if self.kind != BusType::Spi {
            return Err(Error::Unsupported);
        }
        Ok(self.device_mut(handle)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::device::Speed;

    #[test]
    fn i2c_get_device_bus_address_reuses_slot() {
        let mut mgr = TableDeviceManager::i2c();
        let a = I2cDeviceManager::get_device(&mut mgr, 1, 0x68).unwrap();
        let b = I2cDeviceManager::get_device(&mut mgr, 1, 0x68).unwrap();
        let c = I2cDeviceManager::get_device(&mut mgr, 1, 0x1E).unwrap();
        assert_eq!(a, b);
        assert_ne!(a, c);
        assert_eq!(mgr.occupied(), 2);

        let dev = mgr.device_mut(a).unwrap();
        assert_eq!(dev.bus_type(), BusType::I2c);
        assert_eq!(dev.bus_num(), 1);
        assert_eq!(dev.bus_address(), 0x68);
        dev.write_register(0x0A, 0x5C).unwrap();
        let mut buf = [0u8; 1];
        dev.read_registers(0x0A, &mut buf).unwrap();
        assert_eq!(buf, [0x5C]);
    }

    #[test]
    fn i2c_rejects_bus_outside_mask() {
        let mut mgr = TableDeviceManager::i2c();
        // Default mask 0x0F: buses 0..3. SITL returns nullptr for bus >= 4.
        assert_eq!(
            I2cDeviceManager::get_device(&mut mgr, 4, 0x68),
            Err(Error::NotPresent)
        );
        assert_eq!(
            I2cDeviceManager::get_device(&mut mgr, 5, 0x18),
            Err(Error::NotPresent)
        );
        I2cDeviceManager::get_device(&mut mgr, 3, 0x18).unwrap();
    }

    #[test]
    fn i2c_table_full_is_not_present() {
        let mut mgr = TableDeviceManager::i2c();
        for addr in 0..TABLE_SLOTS {
            I2cDeviceManager::get_device(&mut mgr, 0, addr as u8).unwrap();
        }
        assert_eq!(
            I2cDeviceManager::get_device(&mut mgr, 0, TABLE_SLOTS as u8),
            Err(Error::NotPresent)
        );
    }

    #[test]
    fn spi_get_device_by_name() {
        let mut mgr = TableDeviceManager::spi();
        let h = SpiDeviceManager::get_device(&mut mgr, "ms5611").unwrap();
        assert_eq!(SpiDeviceManager::get_device(&mut mgr, "ms5611").unwrap(), h);
        assert_eq!(mgr.count(), 1);
        assert_eq!(mgr.get_device_name(0), Some("ms5611"));
        assert_eq!(mgr.get_device_name(1), None);

        let imu = SpiDeviceManager::get_device(&mut mgr, "bmi088").unwrap();
        assert_ne!(imu, h);
        assert_eq!(mgr.count(), 2);
        assert_eq!(mgr.get_device_name(1), Some("bmi088"));

        let dev = mgr.device_mut(h).unwrap();
        assert_eq!(dev.bus_type(), BusType::Spi);
        dev.set_speed(Speed::Low).unwrap();
        assert_eq!(dev.speed(), Speed::Low);
    }

    #[test]
    fn spi_rejects_empty_or_overlong_name() {
        let mut mgr = TableDeviceManager::spi();
        assert_eq!(
            SpiDeviceManager::get_device(&mut mgr, ""),
            Err(Error::Unsupported)
        );
        assert_eq!(
            SpiDeviceManager::get_device(&mut mgr, "this_name_is_too_long"),
            Err(Error::Unsupported)
        );
    }

    #[test]
    fn device_manager_bus_address_works_for_both() {
        let mut i2c = TableDeviceManager::i2c();
        let mut spi = TableDeviceManager::spi();

        let ih = DeviceManager::get_device(&mut i2c, 0, 0x76).unwrap();
        let sh = DeviceManager::get_device(&mut spi, 0, 2).unwrap();
        assert_eq!(i2c.device(ih).unwrap().bus_address(), 0x76);
        assert_eq!(spi.device(sh).unwrap().bus_address(), 2);

        // Cross-kind factory methods are rejected.
        assert_eq!(
            I2cDeviceManager::get_device(&mut spi, 0, 0x10),
            Err(Error::Unsupported)
        );
        assert_eq!(
            SpiDeviceManager::get_device(&mut i2c, "ms5611"),
            Err(Error::Unsupported)
        );
    }

    #[test]
    fn managers_are_object_safe() {
        let mut i2c = TableDeviceManager::i2c();
        let mut spi = TableDeviceManager::spi();

        let dm: &mut dyn DeviceManager = &mut i2c;
        let h = dm.get_device(2, 0x1E).unwrap();
        dm.device(h).unwrap().set_device_type(0x07);
        assert_eq!(
            crate::device::DeviceId::from_raw(dm.device(h).unwrap().bus_id()).devtype(),
            0x07
        );

        let im: &mut dyn I2cDeviceManager = &mut i2c;
        assert_eq!(im.bus_mask_internal(), 0x01);
        im.get_device(0, 0x18).unwrap();

        let sm: &mut dyn SpiDeviceManager = &mut spi;
        sm.get_device("icm20602").unwrap();
        assert_eq!(sm.count(), 1);
        assert_eq!(sm.get_device_name(0), Some("icm20602"));
    }

    #[test]
    fn i2c_device_and_spi_device_handles() {
        let mut i2c = TableDeviceManager::i2c();
        let mut spi = TableDeviceManager::spi();
        let ih = I2cDeviceManager::get_device(&mut i2c, 0, 0x40).unwrap();
        let sh = SpiDeviceManager::get_device(&mut spi, "lsm9ds1").unwrap();

        i2c.i2c_device(ih).unwrap().set_split_transfers(true);
        let send = [0xA5u8];
        let mut recv = [0u8; 1];
        spi.spi_device(sh)
            .unwrap()
            .transfer_fullduplex(&send, &mut recv)
            .unwrap();
        assert_eq!(recv, send);

        assert_eq!(i2c.spi_device(ih).err(), Some(Error::Unsupported));
        assert_eq!(spi.i2c_device(sh).err(), Some(Error::Unsupported));
    }

    #[test]
    fn invalid_handle_is_not_present() {
        let mut mgr = TableDeviceManager::i2c();
        assert_eq!(mgr.device(DeviceHandle(0)).err(), Some(Error::NotPresent));
        assert_eq!(mgr.device(DeviceHandle(1)).err(), Some(Error::NotPresent));
        assert_eq!(mgr.device(DeviceHandle(99)).err(), Some(Error::NotPresent));
    }
}
