//! QuadPlane / VTOL support, upstream `ArduPlane/quadplane.*` (Plane-4.7.0).
//!
//! Tracked as **VT-001**. This slice is `QuadPlane::setup`: when
//! `Q_ENABLE != 0` it initialises the lift-motor object and sets
//! `initialised`. [`QuadPlane::available`] then returns that flag, not
//! [`QuadPlane::enabled`].
//!
//! Upstream:
//! - `enabled()` is `return enable != 0` (`Q_ENABLE`, `AP_Int8 enable`).
//! - `setup()` returns `true` if already `initialised`; returns `false`
//!   when `enable` is zero; otherwise allocates motors and ends with
//!   `initialised = true`.
//! - `available()` is `return initialised`.

#![no_std]

pub mod tailsitter;
pub mod transition;

/// Default `Q_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, ...)`.
pub const Q_ENABLE_DEFAULT: i8 = 0;

/// The QuadPlane object, upstream `class QuadPlane`.
#[derive(Clone, Copy, Debug)]
pub struct QuadPlane {
    /// `Q_ENABLE`, upstream `AP_Int8 enable`.
    enable: i8,
    /// Upstream `bool initialised` — set true at the end of [`Self::setup`].
    initialised: bool,
    /// Lift-motor object constructed by [`Self::setup`].
    ///
    /// Upstream `AP_MotorsMulticopter *motors`. Frame-class allocation
    /// (`AP_MotorsMatrix` / `AP_MotorsTri` / `AP_MotorsTailsitter`) is a
    /// later slice; this flag is the non-null pointer after motors-init.
    motors_inited: bool,
}

impl QuadPlane {
    /// A disabled QuadPlane (`Q_ENABLE` 0), matching the parameter default.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enable: Q_ENABLE_DEFAULT,
            initialised: false,
            motors_inited: false,
        }
    }

    /// Construct with an explicit `Q_ENABLE` value.
    ///
    /// Does not run [`Self::setup`]; [`Self::available`] stays false
    /// until setup succeeds.
    #[must_use]
    pub const fn with_enable(enable: i8) -> Self {
        Self {
            enable,
            initialised: false,
            motors_inited: false,
        }
    }

    /// Write `Q_ENABLE`.
    ///
    /// Does not clear [`Self::initialised`]; upstream `available()` is
    /// only that flag, not a re-check of `enable`.
    pub fn set_enable(&mut self, enable: i8) {
        self.enable = enable;
    }

    /// Current `Q_ENABLE` value.
    #[must_use]
    pub const fn enable(&self) -> i8 {
        self.enable
    }

    /// Upstream `QuadPlane::enabled` — `return enable != 0`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enable != 0
    }

    /// Upstream `bool initialised`.
    #[must_use]
    pub const fn initialised(&self) -> bool {
        self.initialised
    }

    /// Whether [`Self::setup`] constructed the lift-motor object.
    ///
    /// Upstream `motors != nullptr` after a successful setup.
    #[must_use]
    pub const fn motors_inited(&self) -> bool {
        self.motors_inited
    }

    /// Upstream `QuadPlane::setup`.
    ///
    /// When already initialised, returns `true`. When `Q_ENABLE == 0`,
    /// returns `false` and leaves motors unallocated. Otherwise runs
    /// the motors-init stub and sets `initialised`.
    ///
    /// Soft-armed rejection, the memory check, and frame-class
    /// construction are later slices.
    pub fn setup(&mut self) -> bool {
        if self.initialised {
            return true;
        }
        if !self.enabled() {
            return false;
        }
        // Upstream `motors = NEW_NOTHROW AP_MotorsMatrix(rc_speed)` (default
        // frame class). A null allocation is `allocation_error("motors")`.
        self.motors_inited = true;
        self.initialised = true;
        true
    }

    /// Upstream `QuadPlane::available` — `return initialised`.
    #[must_use]
    pub const fn available(&self) -> bool {
        self.initialised
    }
}

impl Default for QuadPlane {
    fn default() -> Self {
        Self::new()
    }
}
