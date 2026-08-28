//! Wide/quad SPI device, ported from `AP_HAL/WSPIDevice.h`.
//!
//! Upstream `AP_HAL::WSPIDevice` inherits `Device` (`BUS_TYPE_WSPI`) and adds
//! the command-header / busy surface used by Dual-Quad-Octo SPI flashes.
//! [`WspiDevice`] keeps that split: [`Device::set_speed`],
//! [`Device::transfer`], and the `Device.cpp` register wrappers stay on
//! [`Device`]; this trait only adds the wrap/quad extras.
//!
//! `WSPIDeviceManager::get_device` returning a heap `OwnPtr` is not ported
//! (no allocator). Command-mode bitfields (`WSPI::CFG_*`) are board
//! constants and stay with a ChibiOS consumer.

use crate::device::{BusType, Device, DeviceId, PeriodicHandle, Speed};
use crate::semaphore::{MockSemaphore, Semaphore};
use crate::{Error, Result};

/// Command header for Dual/Quad/Octo SPI transactions.
/// Upstream `AP_HAL::Device::CommandHeader`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CommandHeader {
    /// Command-phase data. Upstream `cmd`.
    pub cmd: u32,
    /// Transfer configuration field. Upstream `cfg`.
    pub cfg: u32,
    /// Address-phase data. Upstream `addr`.
    pub addr: u32,
    /// Alternate-phase data. Upstream `alt`.
    pub alt: u32,
    /// Dummy cycles to insert. Upstream `dummy`.
    pub dummy: u32,
}

/// Wide/quad SPI device. Upstream `AP_HAL::WSPIDevice`.
///
/// `set_speed` / `transfer` / `read_registers` / `write_register` are the
/// inherited [`Device`] methods. This trait is the wrap/quad extras:
/// [`WspiDevice::set_cmd_header`] and [`WspiDevice::is_busy`].
pub trait WspiDevice: Device {
    /// Set the command header for upcoming transfer(s).
    /// Upstream `set_cmd_header`.
    fn set_cmd_header(&mut self, cmd_hdr: CommandHeader);

    /// True while a transfer is in flight. Upstream `is_busy()`.
    fn is_busy(&self) -> bool;

    /// Enter memory-mapped / XIP mode. Upstream `enter_xip_mode()`,
    /// default false.
    fn enter_xip_mode(&mut self) -> Result<()> {
        Err(Error::Unsupported)
    }

    /// Leave XIP mode. Upstream `exit_xip_mode()`, default false.
    fn exit_xip_mode(&mut self) -> Result<()> {
        Err(Error::Unsupported)
    }
}

/// An in-memory [`WspiDevice`] for tests and SITL bring-up.
///
/// Register space is a 256-byte table so every `u8` address is valid. The
/// bus semaphore is stack-allocated. Periodic callbacks are unsupported,
/// matching ChibiOS `WSPIDevice::register_periodic_callback`.
#[derive(Debug)]
pub struct MockWspiDevice {
    id: DeviceId,
    speed: Speed,
    read_flag: u8,
    regs: [u8; 256],
    sem: MockSemaphore,
    cmd_header: CommandHeader,
    busy: bool,
    xip: bool,
    fail_transfers: bool,
}

impl Default for MockWspiDevice {
    fn default() -> Self {
        Self {
            id: DeviceId::new(BusType::Wspi, 0, 0, 0),
            speed: Speed::High,
            read_flag: 0,
            regs: [0; 256],
            sem: MockSemaphore::new(),
            cmd_header: CommandHeader::default(),
            busy: false,
            xip: false,
            fail_transfers: false,
        }
    }
}

impl MockWspiDevice {
    /// A WSPI device on `bus` with chip-select `address`.
    #[inline]
    pub fn new(bus: u8, address: u8) -> Self {
        Self {
            id: DeviceId::new(BusType::Wspi, bus, address, 0),
            ..Self::default()
        }
    }

    /// Current transfer speed.
    #[inline]
    pub fn speed(&self) -> Speed {
        self.speed
    }

    /// Last header passed to [`WspiDevice::set_cmd_header`].
    #[inline]
    pub fn cmd_header(&self) -> CommandHeader {
        self.cmd_header
    }

    /// Whether [`WspiDevice::enter_xip_mode`] is active.
    #[inline]
    pub fn in_xip_mode(&self) -> bool {
        self.xip
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

    /// Mark the peripheral busy or idle. Upstream `is_busy()` is a hardware
    /// poll; the mock exposes the latch so tests can drive it.
    #[inline]
    pub fn set_busy(&mut self, busy: bool) {
        self.busy = busy;
    }

    /// Force subsequent [`Device::transfer`] calls to fail with
    /// [`Error::BusError`].
    #[inline]
    pub fn set_fail_transfers(&mut self, fail: bool) {
        self.fail_transfers = fail;
    }
}

impl Device for MockWspiDevice {
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
        let _ = period_usec;
        Err(Error::Unsupported)
    }

    fn adjust_periodic_callback(&mut self, handle: PeriodicHandle, period_usec: u32) -> Result<()> {
        let _ = (handle, period_usec);
        Err(Error::Unsupported)
    }
}

impl WspiDevice for MockWspiDevice {
    fn set_cmd_header(&mut self, cmd_hdr: CommandHeader) {
        self.cmd_header = cmd_hdr;
    }

    fn is_busy(&self) -> bool {
        self.busy
    }

    fn enter_xip_mode(&mut self) -> Result<()> {
        self.xip = true;
        Ok(())
    }

    fn exit_xip_mode(&mut self) -> Result<()> {
        self.xip = false;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_speed_transfer_and_register_rw() {
        let mut dev = MockWspiDevice::new(0, 1);
        assert_eq!(dev.bus_type(), BusType::Wspi);
        assert_eq!(dev.speed(), Speed::High);
        assert!(dev.set_speed(Speed::Low).is_ok());
        assert_eq!(dev.speed(), Speed::Low);

        assert!(dev.write_register(0x0A, 0x5C).is_ok());
        assert_eq!(dev.register(0x0A), 0x5C);
        let mut buf = [0u8; 1];
        assert!(dev.read_registers(0x0A, &mut buf).is_ok());
        assert_eq!(buf[0], 0x5C);
    }

    #[test]
    fn set_cmd_header_and_busy() {
        let mut dev = MockWspiDevice::default();
        assert!(!dev.is_busy());
        let hdr = CommandHeader {
            cmd: 0x6B,
            cfg: 0x03,
            addr: 0x0010_0000,
            alt: 0,
            dummy: 8,
        };
        dev.set_cmd_header(hdr);
        assert_eq!(dev.cmd_header(), hdr);
        dev.set_busy(true);
        assert!(dev.is_busy());
        assert!(dev.enter_xip_mode().is_ok());
        assert!(dev.in_xip_mode());
        assert!(dev.exit_xip_mode().is_ok());
        assert!(!dev.in_xip_mode());
    }

    #[test]
    fn transfer_failure_is_bus_error() {
        let mut dev = MockWspiDevice::new(0, 0);
        dev.set_fail_transfers(true);
        assert_eq!(dev.transfer(&[0x9F], &mut [0; 3]), Err(Error::BusError));
        assert_eq!(dev.write_register(0x01, 0x00), Err(Error::BusError));
    }

    /// The trait stays object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn wspi_device_trait_is_object_safe() {
        let mut dev = MockWspiDevice::new(1, 0x02);
        let d: &mut dyn WspiDevice = &mut dev;
        assert!(d.set_speed(Speed::High).is_ok());
        d.set_cmd_header(CommandHeader {
            cmd: 0x03,
            ..CommandHeader::default()
        });
        assert!(!d.is_busy());
        assert!(d.write_register(0x20, 0xAA).is_ok());
        let mut buf = [0u8; 1];
        assert!(d.read_registers(0x20, &mut buf).is_ok());
        assert_eq!(buf[0], 0xAA);
        assert_eq!(d.bus_type(), BusType::Wspi);
    }
}
