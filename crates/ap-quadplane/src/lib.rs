//! QuadPlane / VTOL support, upstream `ArduPlane/quadplane.*` (Plane-4.7.0).
//!
//! Tracked as **VT-001** / **VT-007**. Sibling slices land as modules
//! beside [`tailsitter`]. Keep every `mod` declaration in this file so
//! parallel workers do not drop each other's surfaces.
//!
//! Upstream:
//! - `enabled()` is `return enable != 0` (`Q_ENABLE`, `AP_Int8 enable`).
//! - `available()` is `return initialised` (set true at the end of
//!   `QuadPlane::setup()`).
//!
//! `setup()` is the next VT-001 slice. Until then a non-zero `Q_ENABLE` is the
//! live-object check, so `available()` agrees with `enabled()`.

#![no_std]

pub mod tailsitter;

/// Default `Q_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, ...)`.
pub const Q_ENABLE_DEFAULT: i8 = 0;

/// The QuadPlane object, upstream `class QuadPlane`.
#[derive(Clone, Copy, Debug)]
pub struct QuadPlane {
    /// `Q_ENABLE`, upstream `AP_Int8 enable`.
    enable: i8,
}

impl QuadPlane {
    /// A disabled QuadPlane (`Q_ENABLE` 0), matching the parameter default.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enable: Q_ENABLE_DEFAULT,
        }
    }

    /// Construct with an explicit `Q_ENABLE` value.
    #[must_use]
    pub const fn with_enable(enable: i8) -> Self {
        Self { enable }
    }

    /// Write `Q_ENABLE`.
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

    /// Upstream `QuadPlane::available`.
    ///
    /// Upstream returns `initialised`. This stub has no `setup()` yet, so
    /// the object is live when `Q_ENABLE != 0` — the same gate as [`enabled`].
    #[must_use]
    pub const fn available(&self) -> bool {
        self.enabled()
    }
}

impl Default for QuadPlane {
    fn default() -> Self {
        Self::new()
    }
}
