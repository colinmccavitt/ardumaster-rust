//! Tiltrotor enable and type stub, upstream `ArduPlane/tiltrotor.*`.
//!
//! Tracked as **VT-008**. This slice is the gate that decides whether the
//! tiltrotor object is live, and which tilt type it is flying.
//!
//! # When it is enabled
//!
//! Upstream `Tiltrotor::enabled` is `(enable > 0) && setup_complete`.
//! `setup()` fills `enable` from the old heuristic when `Q_TILT_ENABLE`
//! was never written: a QuadPlane whose `Q_TILT_MASK` is non-zero, or
//! whose `Q_TILT_TYPE` is [`TiltType::Bicopter`], is a tiltrotor, so
//! enable is saved as 1. An explicit zero stays off. `setup_complete`
//! is set only when enable ends up positive — calling `setup` on a
//! disabled object does not make `enabled()` true.
//!
//! The comment on `tiltrotor.cpp` says the block is "Enabled by setting
//! `Q_TILT_MASK` to a non-zero value". Bicopter is the other auto-enable
//! path because those airframes do not use a tilt mask.
//!
//! # The four tilt types
//!
//! Upstream `Q_TILT_TYPE` (`Tiltrotor::type`):
//!
//! - [`TiltType::Continuous`] — rotors tilt to any angle on demand.
//! - [`TiltType::Binary`] — retract-style servo, fully forward or fully up.
//! - [`TiltType::VectoredYaw`] — tilt motors vector thrust for yaw in hover.
//! - [`TiltType::Bicopter`] — must use tailsitter frame class (10).
//!
//! [`Tiltrotor::tilt_type`] is `Some` only when the object is live and
//! the stored discriminant is one of those four. The slew / vectored-yaw
//! mix / flap leftover lives in later slices.

/// Default `Q_TILT_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0)`.
pub const TILT_ENABLE_DEFAULT: i8 = 0;

/// Default `Q_TILT_MASK`, upstream `AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0)`.
pub const TILT_MASK_DEFAULT: i16 = 0;

/// Default `Q_TILT_TYPE`, upstream `AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS)`.
pub const TILT_TYPE_DEFAULT: i8 = 0;

/// Types of tilt mechanisms, upstream `Tiltrotor::TILT_TYPE_*`.
///
/// Discriminants match the `@Values` on `Q_TILT_TYPE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i8)]
pub enum TiltType {
    /// `TILT_TYPE_CONTINUOUS` — rotors tilt to any angle on demand.
    Continuous = 0,
    /// `TILT_TYPE_BINARY` — retract-style, fully forward or fully up.
    Binary = 1,
    /// `TILT_TYPE_VECTORED_YAW` — tilt motors control yaw in hover.
    VectoredYaw = 2,
    /// `TILT_TYPE_BICOPTER` — must use tailsitter frame class (10).
    Bicopter = 3,
}

impl TiltType {
    /// Inverse of the upstream discriminant.
    #[must_use]
    pub const fn from_i8(value: i8) -> Option<Self> {
        match value {
            0 => Some(Self::Continuous),
            1 => Some(Self::Binary),
            2 => Some(Self::VectoredYaw),
            3 => Some(Self::Bicopter),
            _ => None,
        }
    }

    /// Upstream discriminant.
    #[must_use]
    pub const fn as_i8(self) -> i8 {
        self as i8
    }
}

/// What `Tiltrotor::setup` reads off QuadPlane and the tiltrotor params.
#[derive(Debug, Clone, Copy)]
pub struct TiltrotorConfig {
    /// `Q_TILT_ENABLE` when the parameter has been written.
    ///
    /// `None` means unconfigured: `setup` applies the mask / bicopter
    /// heuristic.
    pub enable: Option<i8>,
    /// `Q_TILT_MASK`, upstream `tilt_mask`.
    pub tilt_mask: i16,
    /// `Q_TILT_TYPE`, upstream `Tiltrotor::type`.
    pub tilt_type: i8,
}

impl TiltrotorConfig {
    /// A disabled, unconfigured tiltrotor (zero mask, continuous type).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            enable: None,
            tilt_mask: TILT_MASK_DEFAULT,
            tilt_type: TILT_TYPE_DEFAULT,
        }
    }

    /// Unconfigured enable with a non-zero tilt mask (auto-enables).
    #[must_use]
    pub const fn with_tilt_mask(tilt_mask: i16) -> Self {
        Self {
            enable: None,
            tilt_mask,
            tilt_type: TILT_TYPE_DEFAULT,
        }
    }

    /// Unconfigured enable as a bicopter (auto-enables, no tilt mask).
    #[must_use]
    pub const fn bicopter() -> Self {
        Self {
            enable: None,
            tilt_mask: TILT_MASK_DEFAULT,
            tilt_type: TiltType::Bicopter as i8,
        }
    }
}

impl Default for TiltrotorConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// The tiltrotor object, upstream `class Tiltrotor`.
#[derive(Debug, Clone, Copy)]
pub struct Tiltrotor {
    enable: i8,
    setup_complete: bool,
    tilt_mask: i16,
    tilt_type: i8,
}

impl Tiltrotor {
    /// Run upstream `Tiltrotor::setup` and return the resulting object.
    ///
    /// Does not persist parameters (`set_and_save`); the caller owns that.
    /// Servo assignment, thrust-compensation callback, and
    /// `Tiltrotor_Transition` allocation are later slices.
    #[must_use]
    pub fn setup(cfg: TiltrotorConfig) -> Self {
        let mut enable = cfg.enable.unwrap_or(TILT_ENABLE_DEFAULT);
        if cfg.enable.is_none()
            && (cfg.tilt_mask != 0 || cfg.tilt_type == TiltType::Bicopter as i8)
        {
            enable = 1;
        }

        // Upstream returns early when `enable <= 0`, leaving
        // `setup_complete` false.
        let setup_complete = enable > 0;
        Self {
            enable,
            setup_complete,
            tilt_mask: cfg.tilt_mask,
            tilt_type: cfg.tilt_type,
        }
    }

    /// Current `Q_TILT_ENABLE` after setup.
    #[must_use]
    pub const fn enable(&self) -> i8 {
        self.enable
    }

    /// Upstream `Tiltrotor::enabled` — `(enable > 0) && setup_complete`.
    #[must_use]
    pub const fn enabled(&self) -> bool {
        self.enable > 0 && self.setup_complete
    }

    /// Current `Q_TILT_MASK` after setup.
    #[must_use]
    pub const fn tilt_mask(&self) -> i16 {
        self.tilt_mask
    }

    /// Current `Q_TILT_TYPE` discriminant after setup.
    #[must_use]
    pub const fn tilt_type_raw(&self) -> i8 {
        self.tilt_type
    }

    /// Decoded tilt type when the object is live.
    ///
    /// `None` when disabled or the stored discriminant is not one of the
    /// four upstream `TILT_TYPE_*` values.
    #[must_use]
    pub const fn tilt_type(&self) -> Option<TiltType> {
        if !self.enabled() {
            return None;
        }
        TiltType::from_i8(self.tilt_type)
    }

    /// Upstream `Tiltrotor::is_vectored` — `enabled() && _is_vectored`.
    ///
    /// `_is_vectored` is set in `setup` when the tilt mask is non-zero
    /// and the type is [`TiltType::VectoredYaw`]. The yaw mix itself is
    /// a later slice.
    #[must_use]
    pub const fn is_vectored(&self) -> bool {
        self.enabled() && self.tilt_mask != 0 && self.tilt_type == TiltType::VectoredYaw as i8
    }
}
