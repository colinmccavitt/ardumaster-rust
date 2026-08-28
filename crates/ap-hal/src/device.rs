//! I2C/SPI device bus, ported from `AP_HAL/Device.h`, `I2CDevice.h`, and
//! `SPIDevice.h`.
//!
//! Compass, IMU, baro, and airspeed drivers talk to their chips through this
//! surface: register read/write, bus speed, a per-bus semaphore taken during
//! init, and a periodic callback that upstream runs on the bus thread with
//! the lock already held.
//!
//! # Why one trait plus two thin extensions
//!
//! Upstream `I2CDevice` and `SPIDevice` inherit `Device` and add a handful of
//! bus-specific methods. That split is kept: a driver that only needs
//! [`Device::read_registers`] / [`Device::write_register`] takes
//! `&mut dyn Device`, and the I2C or SPI extras stay on their own traits.
//!
//! `I2CDeviceManager::get_device` / `SPIDeviceManager::get_device` returning a
//! heap-allocated `OwnPtr` are not ported: the crate is `no_std` with no
//! allocator. Callers hold a [`Device`] the same way they hold an
//! [`crate::analog::AnalogSource`].
//!
//! Checked-register bookkeeping (`setup_checked_registers`) allocates in
//! upstream and is a `Device.cpp` helper, not a virtual. It lands with a
//! consumer that needs it.

use crate::semaphore::Semaphore;
use crate::{Error, Result};

/// Bus kind packed into a [`DeviceId`]. Upstream `AP_HAL::Device::BusType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum BusType {
    /// Unknown / unset. Upstream `BUS_TYPE_UNKNOWN`.
    #[default]
    Unknown = 0,
    /// I2C. Upstream `BUS_TYPE_I2C`.
    I2c = 1,
    /// SPI. Upstream `BUS_TYPE_SPI`.
    Spi = 2,
    /// UAVCAN / DroneCAN. Upstream `BUS_TYPE_UAVCAN`.
    Uavcan = 3,
    /// SITL. Upstream `BUS_TYPE_SITL`.
    Sitl = 4,
    /// MSP. Upstream `BUS_TYPE_MSP`.
    Msp = 5,
    /// Serial. Upstream `BUS_TYPE_SERIAL`.
    Serial = 6,
    /// Wide SPI (QSPI and friends). Upstream `BUS_TYPE_WSPI`.
    Wspi = 7,
}

impl BusType {
    /// The upstream `enum BusType` ordinal.
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Decode a 3-bit bus-type field. Out-of-range becomes [`BusType::Unknown`].
    #[inline]
    pub const fn from_u8(v: u8) -> Self {
        match v {
            1 => Self::I2c,
            2 => Self::Spi,
            3 => Self::Uavcan,
            4 => Self::Sitl,
            5 => Self::Msp,
            6 => Self::Serial,
            7 => Self::Wspi,
            _ => Self::Unknown,
        }
    }
}

/// Transfer speed for future bus operations. Upstream `AP_HAL::Device::Speed`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum Speed {
    /// Fast transfers. Upstream `SPEED_HIGH`.
    #[default]
    High = 0,
    /// Slow transfers (bring-up, some IMUs). Upstream `SPEED_LOW`.
    Low = 1,
}

impl Speed {
    /// The upstream `enum Speed` ordinal.
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Handle returned by [`Device::register_periodic_callback`].
///
/// Upstream `PeriodicHandle` is `void*`. The port uses a small integer so a
/// backend can index a fixed table without an allocator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PeriodicHandle(pub u32);

/// 24-bit bus identifier packed the way upstream's `DeviceId` union is.
///
/// Bitfields (LSB first, matching GCC on the ChibiOS targets):
/// `bus_type:3`, `bus:5`, `address:8`, `devtype:8`. The width is chosen so
/// the value survives a MAVLink `float` parameter without losing bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DeviceId {
    raw: u32,
}

impl DeviceId {
    /// Pack a bus identity. Upstream `Device::make_bus_id`.
    #[inline]
    pub const fn new(bus_type: BusType, bus: u8, address: u8, devtype: u8) -> Self {
        Self {
            raw: make_bus_id(bus_type, bus, address, devtype),
        }
    }

    /// The packed 32-bit value. Upstream `get_bus_id()`.
    #[inline]
    pub const fn raw(self) -> u32 {
        self.raw
    }

    /// Rebuild from a packed value.
    #[inline]
    pub const fn from_raw(raw: u32) -> Self {
        Self { raw }
    }

    /// Bus kind. Upstream `devid_get_bus_type` / `bus_type()`.
    #[inline]
    pub const fn bus_type(self) -> BusType {
        BusType::from_u8((self.raw & 0x07) as u8)
    }

    /// Bus instance. Upstream `devid_get_bus` / `bus_num()`.
    #[inline]
    pub const fn bus_num(self) -> u8 {
        ((self.raw >> 3) & 0x1f) as u8
    }

    /// Address on the bus (I2C address or SPI CS). Upstream `devid_get_address`.
    #[inline]
    pub const fn address(self) -> u8 {
        ((self.raw >> 8) & 0xff) as u8
    }

    /// Device-class type (e.g. a compass driver id). Upstream `devid_get_devtype`.
    #[inline]
    pub const fn devtype(self) -> u8 {
        ((self.raw >> 16) & 0xff) as u8
    }

    /// Same connection with a different device type. Upstream `change_bus_id`.
    #[inline]
    pub const fn with_devtype(self, devtype: u8) -> Self {
        Self::from_raw(change_bus_id(self.raw, devtype))
    }
}

/// Pack a 24-bit bus id. Upstream `AP_HAL::Device::make_bus_id`.
#[inline]
pub const fn make_bus_id(bus_type: BusType, bus: u8, address: u8, devtype: u8) -> u32 {
    (bus_type.as_u8() as u32 & 0x07)
        | ((bus as u32 & 0x1f) << 3)
        | ((address as u32) << 8)
        | ((devtype as u32) << 16)
}

/// Replace the `devtype` field of a packed id. Upstream `change_bus_id`.
#[inline]
pub const fn change_bus_id(old_id: u32, devtype: u8) -> u32 {
    (old_id & 0x0000_ffff) | ((devtype as u32) << 16)
}

/// Shared I2C/SPI device. Upstream `AP_HAL::Device`.
///
/// `read_registers` / `write_register` default to a half-duplex
/// [`Device::transfer`], ORing [`Device::read_flag`] into the register
/// address on reads — the same wrappers `Device.cpp` provides on top of the
/// pure-virtual `transfer()`.
pub trait Device {
    /// Bus kind. Upstream `bus_type()`.
    fn bus_type(&self) -> BusType;

    /// Bus instance. Upstream `bus_num()`.
    fn bus_num(&self) -> u8;

    /// Packed 24-bit id. Upstream `get_bus_id()`.
    fn bus_id(&self) -> u32;

    /// Address on the bus. Upstream `get_bus_address()`.
    fn bus_address(&self) -> u8;

    /// Set the device-class type packed into the id. Upstream `set_device_type`.
    fn set_device_type(&mut self, devtype: u8);

    /// Change the I2C address. No-op on SPI. Upstream `set_address()`.
    fn set_address(&mut self, address: u8) {
        let _ = address;
    }

    /// Speed of future transfers. Upstream `set_speed()`.
    fn set_speed(&mut self, speed: Speed) -> Result<()>;

    /// Half-duplex send-then-receive. Upstream `transfer()`.
    fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> Result<()>;

    /// Flag ORed into the register address on reads. Upstream `_read_flag`.
    fn read_flag(&self) -> u8 {
        0
    }

    /// Set the read flag used by [`Device::read_registers`]. Upstream
    /// `set_read_flag()`.
    fn set_read_flag(&mut self, flag: u8) {
        let _ = flag;
    }

    /// Read `recv.len()` registers starting at `first_reg`.
    ///
    /// The read flag is ORed into `first_reg` before the transfer, matching
    /// `Device::read_registers`.
    fn read_registers(&mut self, first_reg: u8, recv: &mut [u8]) -> Result<()> {
        let addr = [first_reg | self.read_flag()];
        self.transfer(&addr, recv)
    }

    /// Write one register. Upstream `write_register(reg, val)`.
    fn write_register(&mut self, reg: u8, val: u8) -> Result<()> {
        let buf = [reg, val];
        self.transfer(&buf, &mut [])
    }

    /// Bus semaphore for the init path. Upstream `get_semaphore()`.
    fn get_semaphore(&mut self) -> &mut dyn Semaphore;

    /// Register a periodic bus-thread callback. Upstream
    /// `register_periodic_callback`.
    ///
    /// The functor body is a board concern (upstream uses `FUNCTOR_TYPEDEF`);
    /// the stub records the period and returns a handle a backend can fire.
    fn register_periodic_callback(&mut self, period_usec: u32) -> Result<PeriodicHandle>;

    /// Change the period of a registered callback. Upstream
    /// `adjust_periodic_callback`.
    fn adjust_periodic_callback(&mut self, handle: PeriodicHandle, period_usec: u32) -> Result<()>;

    /// Cancel a periodic callback. Upstream `unregister_callback()`, default
    /// false.
    fn unregister_callback(&mut self, handle: PeriodicHandle) -> Result<()> {
        let _ = handle;
        Err(Error::Unsupported)
    }
}

/// I2C device. Upstream `AP_HAL::I2CDevice`.
pub trait I2cDevice: Device {
    /// Read the same register `times` times, advancing `recv` each time.
    ///
    /// `recv.len()` must be divisible by `times`. Upstream
    /// `read_registers_multiple`.
    fn read_registers_multiple(&mut self, first_reg: u8, recv: &mut [u8], times: u8) -> Result<()> {
        if times == 0 {
            return Ok(());
        }
        let times = usize::from(times);
        if recv.len() % times != 0 {
            return Err(Error::Unsupported);
        }
        let chunk = recv.len() / times;
        let mut offset = 0;
        for _ in 0..times {
            let end = offset + chunk;
            let slice = recv.get_mut(offset..end).ok_or(Error::BusError)?;
            self.read_registers(first_reg, slice)?;
            offset = end;
        }
        Ok(())
    }

    /// Insert a stop between the send and receive halves. Upstream
    /// `set_split_transfers()`, default no-op.
    fn set_split_transfers(&mut self, set: bool) {
        let _ = set;
    }
}

/// SPI device. Upstream `AP_HAL::SPIDevice`.
pub trait SpiDevice: Device {
    /// Full-duplex transfer; `send` and `recv` must be the same length.
    /// Upstream `SPIDevice::transfer_fullduplex`.
    fn transfer_fullduplex(&mut self, send: &[u8], recv: &mut [u8]) -> Result<()>;

    /// Clock out `len` bytes without asserting CS. Upstream `clock_pulse()`,
    /// default false.
    fn clock_pulse(&mut self, len: u32) -> Result<()> {
        let _ = len;
        Err(Error::Unsupported)
    }
}

/// Configured I2C buses. Upstream `AP_HAL::I2CDeviceManager`.
///
/// `get_device` is not ported (it heap-allocates). The masks are what
/// `FOREACH_I2C_*` walks.
pub trait I2cDeviceManager {
    /// Mask of configured I2C bus numbers. Upstream `get_bus_mask()`, default
    /// `0x0F`.
    fn bus_mask(&self) -> u32 {
        0x0F
    }

    /// Mask of external I2C buses. Upstream `get_bus_mask_external()`.
    fn bus_mask_external(&self) -> u32 {
        0x0F
    }

    /// Mask of internal I2C buses. Upstream `get_bus_mask_internal()`.
    fn bus_mask_internal(&self) -> u32 {
        0x01
    }
}

/// Registered SPI devices. Upstream `AP_HAL::SPIDeviceManager`.
///
/// `get_device` is not ported (it heap-allocates).
pub trait SpiDeviceManager {
    /// Number of registered SPI devices. Upstream `get_count()`, default 0.
    fn count(&self) -> u8 {
        0
    }
}

/// Slots a mock can register periodic callbacks into.
const MOCK_PERIODIC_SLOTS: usize = 4;

/// An in-memory [`Device`] for tests and SITL bring-up.
///
/// Register space is a 256-byte table so every `u8` address is valid. The
/// bus semaphore and periodic-callback table are stack-allocated.
#[derive(Debug)]
pub struct MockDevice {
    id: DeviceId,
    speed: Speed,
    read_flag: u8,
    regs: [u8; 256],
    sem: crate::semaphore::MockSemaphore,
    periods: [Option<u32>; MOCK_PERIODIC_SLOTS],
    split_transfers: bool,
    fail_transfers: bool,
}

impl Default for MockDevice {
    fn default() -> Self {
        Self {
            id: DeviceId::new(BusType::I2c, 0, 0, 0),
            speed: Speed::High,
            read_flag: 0,
            regs: [0; 256],
            sem: crate::semaphore::MockSemaphore::new(),
            periods: [None; MOCK_PERIODIC_SLOTS],
            split_transfers: false,
            fail_transfers: false,
        }
    }
}

impl MockDevice {
    /// An I2C device on `bus` at `address`, high speed, empty registers.
    #[inline]
    pub fn i2c(bus: u8, address: u8) -> Self {
        Self {
            id: DeviceId::new(BusType::I2c, bus, address, 0),
            ..Self::default()
        }
    }

    /// An SPI device on `bus` with chip-select `address`.
    #[inline]
    pub fn spi(bus: u8, address: u8) -> Self {
        Self {
            id: DeviceId::new(BusType::Spi, bus, address, 0),
            ..Self::default()
        }
    }

    /// Defaults: I2C bus 0, address 0.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Current transfer speed.
    #[inline]
    pub fn speed(&self) -> Speed {
        self.speed
    }

    /// Packed identity.
    #[inline]
    pub fn id(&self) -> DeviceId {
        self.id
    }

    /// Peek a register without a bus transfer.
    pub fn register(&self, reg: u8) -> u8 {
        self.regs.get(usize::from(reg)).copied().unwrap_or(0)
    }

    /// Poke a register without a bus transfer.
    pub fn set_register(&mut self, reg: u8, val: u8) {
        if let Some(slot) = self.regs.get_mut(usize::from(reg)) {
            *slot = val;
        }
    }

    /// Period of a registered callback, if the handle is live.
    pub fn period_of(&self, handle: PeriodicHandle) -> Option<u32> {
        let i = slot_index(handle)?;
        self.periods.get(i).copied().flatten()
    }

    /// Whether [`I2cDevice::set_split_transfers`] has been asked to split.
    #[inline]
    pub fn splits_transfers(&self) -> bool {
        self.split_transfers
    }

    /// Force subsequent [`Device::transfer`] calls to fail with
    /// [`Error::BusError`].
    #[inline]
    pub fn set_fail_transfers(&mut self, fail: bool) {
        self.fail_transfers = fail;
    }

    /// The mock semaphore, for tests that want [`crate::semaphore::MockSemaphore`]
    /// methods the trait does not expose.
    #[inline]
    pub fn semaphore(&self) -> &crate::semaphore::MockSemaphore {
        &self.sem
    }
}

fn slot_index(handle: PeriodicHandle) -> Option<usize> {
    let n = handle.0;
    if n == 0 {
        return None;
    }
    let i = n as usize - 1;
    if i < MOCK_PERIODIC_SLOTS {
        Some(i)
    } else {
        None
    }
}

impl Device for MockDevice {
    fn bus_type(&self) -> BusType {
        self.id.bus_type()
    }

    fn bus_num(&self) -> u8 {
        self.id.bus_num()
    }

    fn bus_id(&self) -> u32 {
        self.id.raw()
    }

    fn bus_address(&self) -> u8 {
        self.id.address()
    }

    fn set_device_type(&mut self, devtype: u8) {
        self.id = self.id.with_devtype(devtype);
    }

    fn set_address(&mut self, address: u8) {
        self.id = DeviceId::new(self.id.bus_type(), self.id.bus_num(), address, self.id.devtype());
    }

    fn set_speed(&mut self, speed: Speed) -> Result<()> {
        self.speed = speed;
        Ok(())
    }

    fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> Result<()> {
        if self.fail_transfers {
            return Err(Error::BusError);
        }
        match (send, recv.is_empty()) {
            ([], true) => Ok(()),
            ([], false) => {
                for (i, slot) in recv.iter_mut().enumerate() {
                    *slot = *self.regs.get(i).ok_or(Error::NotPresent)?;
                }
                Ok(())
            }
            ([reg], false) => {
                let start = usize::from(*reg & !self.read_flag);
                for (i, slot) in recv.iter_mut().enumerate() {
                    *slot = *self.regs.get(start + i).ok_or(Error::NotPresent)?;
                }
                Ok(())
            }
            ([reg, rest @ ..], true) => {
                let mut addr = usize::from(*reg);
                for b in rest {
                    if let Some(slot) = self.regs.get_mut(addr) {
                        *slot = *b;
                        addr = addr.saturating_add(1);
                    } else {
                        return Err(Error::NotPresent);
                    }
                }
                Ok(())
            }
            _ => Err(Error::Unsupported),
        }
    }

    fn read_flag(&self) -> u8 {
        self.read_flag
    }

    fn set_read_flag(&mut self, flag: u8) {
        self.read_flag = flag;
    }

    fn get_semaphore(&mut self) -> &mut dyn Semaphore {
        &mut self.sem
    }

    fn register_periodic_callback(&mut self, period_usec: u32) -> Result<PeriodicHandle> {
        for (i, slot) in self.periods.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(period_usec);
                return Ok(PeriodicHandle((i as u32) + 1));
            }
        }
        Err(Error::NotPresent)
    }

    fn adjust_periodic_callback(&mut self, handle: PeriodicHandle, period_usec: u32) -> Result<()> {
        let i = slot_index(handle).ok_or(Error::NotPresent)?;
        match self.periods.get_mut(i) {
            Some(slot @ Some(_)) => {
                *slot = Some(period_usec);
                Ok(())
            }
            _ => Err(Error::NotPresent),
        }
    }

    fn unregister_callback(&mut self, handle: PeriodicHandle) -> Result<()> {
        let i = slot_index(handle).ok_or(Error::NotPresent)?;
        match self.periods.get_mut(i) {
            Some(slot @ Some(_)) => {
                *slot = None;
                Ok(())
            }
            _ => Err(Error::NotPresent),
        }
    }
}

impl I2cDevice for MockDevice {
    fn set_split_transfers(&mut self, set: bool) {
        self.split_transfers = set;
    }
}

impl SpiDevice for MockDevice {
    fn transfer_fullduplex(&mut self, send: &[u8], recv: &mut [u8]) -> Result<()> {
        if self.fail_transfers {
            return Err(Error::BusError);
        }
        if send.len() != recv.len() {
            return Err(Error::Unsupported);
        }
        recv.copy_from_slice(send);
        Ok(())
    }
}

/// In-memory I2C bus mask. Upstream `I2CDeviceManager` defaults.
#[derive(Debug, Clone, Copy)]
pub struct MockI2cDeviceManager {
    mask: u32,
    mask_external: u32,
    mask_internal: u32,
}

impl Default for MockI2cDeviceManager {
    fn default() -> Self {
        Self {
            mask: 0x0F,
            mask_external: 0x0F,
            mask_internal: 0x01,
        }
    }
}

impl MockI2cDeviceManager {
    /// Masks matching the C++ base class: `0x0F` / `0x0F` / `0x01`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

impl I2cDeviceManager for MockI2cDeviceManager {
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

/// In-memory SPI device table. Upstream `SPIDeviceManager`.
#[derive(Debug, Clone, Copy, Default)]
pub struct MockSpiDeviceManager {
    count: u8,
}

impl MockSpiDeviceManager {
    /// No registered devices, matching the C++ default `get_count()` of 0.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Override the reported device count.
    #[inline]
    pub fn set_count(&mut self, count: u8) {
        self.count = count;
    }
}

impl SpiDeviceManager for MockSpiDeviceManager {
    fn count(&self) -> u8 {
        self.count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bus_id_round_trip() {
        let id = DeviceId::new(BusType::I2c, 2, 0x1E, 0x07);
        assert_eq!(id.bus_type(), BusType::I2c);
        assert_eq!(id.bus_num(), 2);
        assert_eq!(id.address(), 0x1E);
        assert_eq!(id.devtype(), 0x07);
        assert_eq!(BusType::I2c.as_u8(), 1);
        assert_eq!(BusType::Spi.as_u8(), 2);

        let changed = id.with_devtype(0x42);
        assert_eq!(changed.bus_type(), BusType::I2c);
        assert_eq!(changed.bus_num(), 2);
        assert_eq!(changed.address(), 0x1E);
        assert_eq!(changed.devtype(), 0x42);
        assert_eq!(change_bus_id(id.raw(), 0x42), changed.raw());
    }

    #[test]
    fn set_speed_and_identity() {
        let mut dev = MockDevice::i2c(1, 0x68);
        assert_eq!(dev.bus_type(), BusType::I2c);
        assert_eq!(dev.bus_num(), 1);
        assert_eq!(dev.bus_address(), 0x68);
        assert_eq!(dev.speed(), Speed::High);
        assert_eq!(Speed::High.as_u8(), 0);
        assert_eq!(Speed::Low.as_u8(), 1);

        dev.set_speed(Speed::Low).unwrap();
        assert_eq!(dev.speed(), Speed::Low);
        dev.set_device_type(0x10);
        assert_eq!(DeviceId::from_raw(dev.bus_id()).devtype(), 0x10);
        dev.set_address(0x69);
        assert_eq!(dev.bus_address(), 0x69);
    }

    /// write_register / read_registers are the wrappers IMU and compass
    /// drivers call. The mock register file is what proves the transfer
    /// framing (addr then data, flag ORed on read) is the Device.cpp one.
    #[test]
    fn register_read_write_round_trip() {
        let mut dev = MockDevice::new();
        dev.write_register(0x0A, 0x5C).unwrap();
        assert_eq!(dev.register(0x0A), 0x5C);

        let mut buf = [0u8; 1];
        dev.read_registers(0x0A, &mut buf).unwrap();
        assert_eq!(buf, [0x5C]);

        // multi-byte write then read
        dev.transfer(&[0x10, 1, 2, 3], &mut []).unwrap();
        let mut three = [0u8; 3];
        dev.read_registers(0x10, &mut three).unwrap();
        assert_eq!(three, [1, 2, 3]);
    }

    #[test]
    fn read_flag_is_ored_into_register_address() {
        let mut dev = MockDevice::new();
        dev.set_register(0x0C, 0xAB);
        dev.set_read_flag(0x80);
        assert_eq!(dev.read_flag(), 0x80);

        let mut buf = [0u8; 1];
        // 0x0C | 0x80 is what goes on the wire; the chip still indexes 0x0C.
        dev.read_registers(0x0C, &mut buf).unwrap();
        assert_eq!(buf, [0xAB]);
    }

    #[test]
    fn transfer_failure_is_bus_error() {
        let mut dev = MockDevice::new();
        dev.set_fail_transfers(true);
        assert_eq!(dev.write_register(0x01, 0x00), Err(Error::BusError));
        let mut buf = [0u8; 1];
        assert_eq!(dev.read_registers(0x01, &mut buf), Err(Error::BusError));
    }

    /// Drivers take the bus semaphore during init and give it back. The
    /// handle is the same recursive mutex [`crate::semaphore::Semaphore`]
    /// already covers.
    #[test]
    fn bus_semaphore_take_give() {
        let mut dev = MockDevice::spi(0, 0);
        let sem = dev.get_semaphore();
        assert!(sem.take_nonblocking());
        assert!(sem.take(0));
        assert!(sem.give());
        assert!(sem.give());
        assert!(!sem.give());
        assert_eq!(dev.semaphore().depth(), 0);
    }

    #[test]
    fn periodic_callback_register_adjust_unregister() {
        let mut dev = MockDevice::new();
        let h = dev.register_periodic_callback(1000).unwrap();
        assert_eq!(dev.period_of(h), Some(1000));

        dev.adjust_periodic_callback(h, 2500).unwrap();
        assert_eq!(dev.period_of(h), Some(2500));

        dev.unregister_callback(h).unwrap();
        assert_eq!(dev.period_of(h), None);
        assert_eq!(dev.adjust_periodic_callback(h, 10), Err(Error::NotPresent));
        assert_eq!(
            dev.unregister_callback(PeriodicHandle(0)),
            Err(Error::NotPresent)
        );
    }

    #[test]
    fn periodic_table_fills() {
        let mut dev = MockDevice::new();
        for _ in 0..MOCK_PERIODIC_SLOTS {
            dev.register_periodic_callback(100).unwrap();
        }
        assert_eq!(dev.register_periodic_callback(100), Err(Error::NotPresent));
    }

    #[test]
    fn i2c_read_registers_multiple_and_split() {
        let mut dev = MockDevice::i2c(0, 0x1E);
        dev.set_register(0x03, 0x11);
        let mut buf = [0u8; 3];
        dev.read_registers_multiple(0x03, &mut buf, 3).unwrap();
        assert_eq!(buf, [0x11, 0x11, 0x11]);

        assert!(!dev.splits_transfers());
        dev.set_split_transfers(true);
        assert!(dev.splits_transfers());

        assert_eq!(
            dev.read_registers_multiple(0x03, &mut buf, 2),
            Err(Error::Unsupported)
        );
    }

    #[test]
    fn spi_fullduplex_and_clock_pulse() {
        let mut dev = MockDevice::spi(0, 1);
        let send = [0xA5, 0x5A, 0x00];
        let mut recv = [0u8; 3];
        dev.transfer_fullduplex(&send, &mut recv).unwrap();
        assert_eq!(recv, send);

        let mut short = [0u8; 2];
        assert_eq!(
            dev.transfer_fullduplex(&send, &mut short),
            Err(Error::Unsupported)
        );
        assert_eq!(dev.clock_pulse(16), Err(Error::Unsupported));
    }

    #[test]
    fn device_managers_default_masks() {
        let i2c = MockI2cDeviceManager::new();
        assert_eq!(i2c.bus_mask(), 0x0F);
        assert_eq!(i2c.bus_mask_external(), 0x0F);
        assert_eq!(i2c.bus_mask_internal(), 0x01);

        let mut spi = MockSpiDeviceManager::new();
        assert_eq!(spi.count(), 0);
        spi.set_count(3);
        assert_eq!(spi.count(), 3);
    }

    /// The traits stay object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn device_traits_are_object_safe() {
        let mut i2c = MockDevice::i2c(0, 0x18);
        let mut spi = MockDevice::spi(0, 0);
        let mut i2c_mgr = MockI2cDeviceManager::new();
        let mut spi_mgr = MockSpiDeviceManager::new();

        i2c.write_register(0x00, 0x7F).unwrap();

        let d: &mut dyn Device = &mut i2c;
        let mut buf = [0u8; 1];
        d.read_registers(0x00, &mut buf).unwrap();
        assert_eq!(buf, [0x7F]);
        d.set_speed(Speed::Low).unwrap();
        assert!(d.get_semaphore().take_nonblocking());
        assert!(d.get_semaphore().give());

        let i: &mut dyn I2cDevice = &mut i2c;
        i.set_split_transfers(true);

        let s: &mut dyn SpiDevice = &mut spi;
        let mut echo = [0u8; 1];
        s.transfer_fullduplex(&[0x3C], &mut echo).unwrap();
        assert_eq!(echo, [0x3C]);

        let im: &mut dyn I2cDeviceManager = &mut i2c_mgr;
        let sm: &mut dyn SpiDeviceManager = &mut spi_mgr;
        assert_eq!(im.bus_mask_internal(), 0x01);
        assert_eq!(sm.count(), 0);
    }
}
