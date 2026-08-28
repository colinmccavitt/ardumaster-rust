//! GPIO, ported from `AP_HAL/GPIO.h`.
//!
//! Digital pin mode, read/write, and the per-pin [`DigitalSource`] handle.
//! Interrupt attach, PWM input, and USB-connected detection are board
//! surfaces and land with a consumer that needs them.
//!
//! Upstream splits this in two: `DigitalSource` is one configured pin, and
//! `GPIO` is the manager that addresses pins by number. That split is kept,
//! because it is what lets a subsystem hold its own pin without reaching back
//! through a manager singleton on every access — the same reason
//! [`crate::analog::AnalogSource`] is separate from [`crate::analog::AnalogIn`].
//!
//! # Pin values
//!
//! Upstream `read()` / `write()` use `uint8_t` 0/1. The port keeps that
//! encoding rather than collapsing it to `bool`, so a caller that stores the
//! value and later writes it back does the same thing on both sides.

use crate::{Error, Result};

/// Pin direction / alternate function.
///
/// Upstream `HAL_GPIO_INPUT` (0), `HAL_GPIO_OUTPUT` (1), `HAL_GPIO_ALT` (2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PinMode {
    /// High-impedance input. Upstream `HAL_GPIO_INPUT`.
    Input = 0,
    /// Driven output. Upstream `HAL_GPIO_OUTPUT`.
    Output = 1,
    /// Alternate function (peripheral mux). Upstream `HAL_GPIO_ALT`.
    Alt = 2,
}

impl PinMode {
    /// The upstream `#define` value this variant corresponds to.
    #[inline]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// One configured digital pin. Upstream `AP_HAL::DigitalSource`.
pub trait DigitalSource {
    /// Set direction / function. Upstream `mode()`.
    fn set_mode(&mut self, mode: PinMode);

    /// Current level, 0 or 1. Upstream `read()`.
    fn read(&self) -> u8;

    /// Drive the pin. Upstream `write()`.
    fn write(&mut self, value: u8);

    /// Invert the current level. Upstream `toggle()`.
    fn toggle(&mut self);
}

/// The GPIO manager. Upstream `AP_HAL::GPIO`.
///
/// `channel()` is not ported here: it returns a heap-allocated
/// `DigitalSource*` and the crate is `no_std` with no allocator. Callers hold
/// a [`DigitalSource`] the same way they hold an [`crate::analog::AnalogSource`].
pub trait Gpio {
    /// One-time backend setup. Upstream `init()`.
    fn init(&mut self) {}

    /// Set direction / function on `pin`. Upstream `pinMode()`.
    fn set_pin_mode(&mut self, pin: u8, mode: PinMode) -> Result<()>;

    /// Current level on `pin`, 0 or 1. Upstream `read()`.
    fn read(&self, pin: u8) -> Result<u8>;

    /// Drive `pin`. Upstream `write()`.
    fn write(&mut self, pin: u8, value: u8) -> Result<()>;

    /// Invert `pin`. Upstream `toggle()`.
    fn toggle(&mut self, pin: u8) -> Result<()>;

    /// Whether `pin` exists on this board. Upstream `valid_pin()`, default true.
    fn valid_pin(&self, pin: u8) -> bool {
        let _ = pin;
        true
    }
}

/// An in-memory GPIO bank for tests and SITL bring-up.
///
/// Sized by const generics so it stays stack-allocated with no allocator.
#[derive(Debug)]
pub struct MockGpio<const N: usize> {
    modes: [PinMode; N],
    levels: [u8; N],
    inited: bool,
}

impl<const N: usize> Default for MockGpio<N> {
    fn default() -> Self {
        Self {
            modes: [PinMode::Input; N],
            levels: [0; N],
            inited: false,
        }
    }
}

impl<const N: usize> MockGpio<N> {
    /// A bank of `N` pins, all inputs, all low, not yet initialised.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether [`Gpio::init`] has been called.
    #[inline]
    pub fn is_initialized(&self) -> bool {
        self.inited
    }

    /// The configured mode of `pin`, if it exists.
    pub fn mode_of(&self, pin: u8) -> Option<PinMode> {
        self.modes.get(usize::from(pin)).copied()
    }

    fn slot(&self, pin: u8) -> Result<usize> {
        let i = usize::from(pin);
        if i < N {
            Ok(i)
        } else {
            Err(Error::NotPresent)
        }
    }
}

impl<const N: usize> Gpio for MockGpio<N> {
    fn init(&mut self) {
        self.inited = true;
    }

    fn set_pin_mode(&mut self, pin: u8, mode: PinMode) -> Result<()> {
        let i = self.slot(pin)?;
        if let Some(slot) = self.modes.get_mut(i) {
            *slot = mode;
            Ok(())
        } else {
            Err(Error::NotPresent)
        }
    }

    fn read(&self, pin: u8) -> Result<u8> {
        let i = self.slot(pin)?;
        self.levels.get(i).copied().ok_or(Error::NotPresent)
    }

    fn write(&mut self, pin: u8, value: u8) -> Result<()> {
        let i = self.slot(pin)?;
        if let Some(slot) = self.levels.get_mut(i) {
            *slot = if value == 0 { 0 } else { 1 };
            Ok(())
        } else {
            Err(Error::NotPresent)
        }
    }

    fn toggle(&mut self, pin: u8) -> Result<()> {
        let i = self.slot(pin)?;
        if let Some(slot) = self.levels.get_mut(i) {
            *slot = if *slot == 0 { 1 } else { 0 };
            Ok(())
        } else {
            Err(Error::NotPresent)
        }
    }

    fn valid_pin(&self, pin: u8) -> bool {
        usize::from(pin) < N
    }
}

/// A single in-memory digital pin. Upstream `AP_HAL::DigitalSource` for tests.
#[derive(Debug, Clone, Copy)]
pub struct MockDigitalSource {
    pin: u8,
    mode: PinMode,
    level: u8,
}

impl Default for MockDigitalSource {
    fn default() -> Self {
        Self {
            pin: 0,
            mode: PinMode::Input,
            level: 0,
        }
    }
}

impl MockDigitalSource {
    /// A source on `pin`, input, low.
    #[inline]
    pub fn new(pin: u8) -> Self {
        Self {
            pin,
            ..Self::default()
        }
    }

    /// The pin this source is bound to.
    #[inline]
    pub fn pin(&self) -> u8 {
        self.pin
    }

    /// The configured direction / function.
    #[inline]
    pub fn mode(&self) -> PinMode {
        self.mode
    }
}

impl DigitalSource for MockDigitalSource {
    fn set_mode(&mut self, mode: PinMode) {
        self.mode = mode;
    }

    fn read(&self) -> u8 {
        self.level
    }

    fn write(&mut self, value: u8) {
        self.level = if value == 0 { 0 } else { 1 };
    }

    fn toggle(&mut self) {
        self.level = if self.level == 0 { 1 } else { 0 };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pin_mode_read_write_round_trip() {
        let mut gpio = MockGpio::<8>::new();
        gpio.init();
        assert!(gpio.is_initialized());

        gpio.set_pin_mode(3, PinMode::Output).unwrap();
        assert_eq!(gpio.mode_of(3), Some(PinMode::Output));
        assert_eq!(PinMode::Output.as_u8(), 1);

        gpio.write(3, 1).unwrap();
        assert_eq!(gpio.read(3).unwrap(), 1);
        gpio.write(3, 0).unwrap();
        assert_eq!(gpio.read(3).unwrap(), 0);

        // non-zero is treated as high, matching a typical board write()
        gpio.write(3, 7).unwrap();
        assert_eq!(gpio.read(3).unwrap(), 1);
    }

    #[test]
    fn toggle_flips_level() {
        let mut gpio = MockGpio::<4>::new();
        gpio.set_pin_mode(1, PinMode::Output).unwrap();
        assert_eq!(gpio.read(1).unwrap(), 0);
        gpio.toggle(1).unwrap();
        assert_eq!(gpio.read(1).unwrap(), 1);
        gpio.toggle(1).unwrap();
        assert_eq!(gpio.read(1).unwrap(), 0);
    }

    #[test]
    fn missing_pin_is_not_present() {
        let mut gpio = MockGpio::<2>::new();
        assert!(gpio.valid_pin(0));
        assert!(gpio.valid_pin(1));
        assert!(!gpio.valid_pin(2));
        assert_eq!(gpio.set_pin_mode(2, PinMode::Output), Err(Error::NotPresent));
        assert_eq!(gpio.read(9), Err(Error::NotPresent));
        assert_eq!(gpio.write(9, 1), Err(Error::NotPresent));
        assert_eq!(gpio.toggle(9), Err(Error::NotPresent));
    }

    /// DigitalSource is the handle a subsystem keeps, independent of the
    /// manager — the same split as AnalogSource / AnalogIn.
    #[test]
    fn digital_source_mode_read_write_toggle() {
        let mut src = MockDigitalSource::new(13);
        assert_eq!(src.pin(), 13);
        assert_eq!(src.mode(), PinMode::Input);
        assert_eq!(src.read(), 0);

        src.set_mode(PinMode::Output);
        assert_eq!(src.mode(), PinMode::Output);

        src.write(1);
        assert_eq!(src.read(), 1);
        src.toggle();
        assert_eq!(src.read(), 0);
        src.toggle();
        assert_eq!(src.read(), 1);
    }

    /// The traits stay object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn gpio_traits_are_object_safe() {
        let mut gpio = MockGpio::<4>::new();
        let mut src = MockDigitalSource::new(0);
        let g: &mut dyn Gpio = &mut gpio;
        let s: &mut dyn DigitalSource = &mut src;
        g.set_pin_mode(0, PinMode::Alt).unwrap();
        s.set_mode(PinMode::Alt);
        s.write(1);
        assert_eq!(s.read(), 1);
        assert_eq!(g.read(0).unwrap(), 0);
    }
}
