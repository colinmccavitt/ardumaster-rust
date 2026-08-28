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
//! the stored discriminant is one of those four.
//!
//! # Tilt angle and slew
//!
//! Upstream `current_tilt` is a 0..1 proportion (0 = rotors up / hover,
//! 1 = fully forward). [`Tiltrotor::tilt_angle`] is that as degrees
//! (`current_tilt * 90`), matching the TILT log field. [`Tiltrotor::slew`]
//! walks `current_tilt` toward a target at `Q_TILT_RATE_UP` /
//! `Q_TILT_RATE_DN` (`max_rate_up_dps` / `max_rate_down_dps`). Rate-down
//! of zero uses the up rate. `up` in [`Tiltrotor::tilt_max_change`] is
//! `newtilt < current_tilt` — decreasing tilt is hover-ward.
//!
//! The 90 DPS fast-tilt override (manual / unstabilised FW), flap-range
//! rate (`Q_TILT_WING_FLAP`), vectored-yaw mix, and `continuous_update`
//! leftover live in later slices.

/// Default `Q_TILT_ENABLE`, upstream `AP_GROUPINFO_FLAGS("ENABLE", 1, Tiltrotor, enable, 0)`.
pub const TILT_ENABLE_DEFAULT: i8 = 0;

/// Default `Q_TILT_MASK`, upstream `AP_GROUPINFO("MASK", 2, Tiltrotor, tilt_mask, 0)`.
pub const TILT_MASK_DEFAULT: i16 = 0;

/// Default `Q_TILT_TYPE`, upstream `AP_GROUPINFO("TYPE", 5, Tiltrotor, type, TILT_TYPE_CONTINUOUS)`.
pub const TILT_TYPE_DEFAULT: i8 = 0;

/// Default `Q_TILT_RATE_UP`, upstream `AP_GROUPINFO("RATE_UP", 3, Tiltrotor, max_rate_up_dps, 40)`.
pub const TILT_RATE_UP_DPS_DEFAULT: i16 = 40;

/// Default `Q_TILT_RATE_DN`, upstream `AP_GROUPINFO("RATE_DN", 6, Tiltrotor, max_rate_down_dps, 0)`.
///
/// Zero means "use [`TILT_RATE_UP_DPS_DEFAULT`]" in [`Tiltrotor::tilt_max_change`].
pub const TILT_RATE_DN_DPS_DEFAULT: i16 = 0;

/// Default `Q_TILT_WING_FLAP`, upstream `AP_GROUPINFO("WING_FLAP", 10, Tiltrotor, flap_angle_deg, 0)`.
///
/// Held so [`Tiltrotor::get_fully_forward_tilt`] matches upstream
/// (`1 - flap/90`). Flap mix itself is a later slice.
pub const TILT_FLAP_ANGLE_DEG_DEFAULT: f32 = 0.0;

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
    max_rate_up_dps: i16,
    max_rate_down_dps: i16,
    flap_angle_deg: f32,
    current_tilt: f32,
    angle_achieved: bool,
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
        if cfg.enable.is_none() && (cfg.tilt_mask != 0 || cfg.tilt_type == TiltType::Bicopter as i8)
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
            max_rate_up_dps: TILT_RATE_UP_DPS_DEFAULT,
            max_rate_down_dps: TILT_RATE_DN_DPS_DEFAULT,
            flap_angle_deg: TILT_FLAP_ANGLE_DEG_DEFAULT,
            current_tilt: 0.0,
            angle_achieved: false,
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

    /// Current `Q_TILT_RATE_UP` after setup, degrees per second.
    #[must_use]
    pub const fn max_rate_up_dps(&self) -> i16 {
        self.max_rate_up_dps
    }

    /// Write `Q_TILT_RATE_UP`.
    pub fn set_max_rate_up_dps(&mut self, max_rate_up_dps: i16) {
        self.max_rate_up_dps = max_rate_up_dps;
    }

    /// Current `Q_TILT_RATE_DN` after setup, degrees per second.
    #[must_use]
    pub const fn max_rate_down_dps(&self) -> i16 {
        self.max_rate_down_dps
    }

    /// Write `Q_TILT_RATE_DN`. Zero means "use the up rate".
    pub fn set_max_rate_down_dps(&mut self, max_rate_down_dps: i16) {
        self.max_rate_down_dps = max_rate_down_dps;
    }

    /// Current tilt proportion, upstream `Tiltrotor::current_tilt`.
    ///
    /// `0` is rotors up (hover), `1` is fully forward.
    #[must_use]
    pub const fn current_tilt(&self) -> f32 {
        self.current_tilt
    }

    /// Current tilt in degrees from vertical, `current_tilt * 90`.
    ///
    /// Matches the TILT field `write_log` stores (`current_tilt * 90.0`).
    #[must_use]
    pub const fn tilt_angle(&self) -> f32 {
        self.current_tilt * 90.0
    }

    /// Upstream `bool angle_achieved` after the last [`Self::slew`].
    #[must_use]
    pub const fn angle_achieved(&self) -> bool {
        self.angle_achieved
    }

    /// Upstream `Tiltrotor::tilt_angle_achieved`.
    ///
    /// True when disabled, when the type is not continuous, or when the
    /// last slew reached its target. Slow rates can leave continuous
    /// tilts lagging, so this is not the same as [`Self::fully_fwd`].
    #[must_use]
    pub const fn tilt_angle_achieved(&self) -> bool {
        !self.enabled() || self.tilt_type != TiltType::Continuous as i8 || self.angle_achieved
    }

    /// Upstream `Tiltrotor::get_fully_forward_tilt`.
    ///
    /// `1 - flap_angle_deg/90`. Default flap is zero, so this is `1`.
    #[must_use]
    pub const fn get_fully_forward_tilt(&self) -> f32 {
        1.0 - (self.flap_angle_deg * (1.0 / 90.0))
    }

    /// Upstream `Tiltrotor::fully_fwd`.
    #[must_use]
    pub const fn fully_fwd(&self) -> bool {
        self.enabled() && self.tilt_mask != 0 && self.current_tilt >= self.get_fully_forward_tilt()
    }

    /// Upstream `Tiltrotor::fully_up`.
    #[must_use]
    pub const fn fully_up(&self) -> bool {
        self.enabled() && self.tilt_mask != 0 && self.current_tilt <= 0.0
    }

    /// Maximum tilt-proportion change this tick, upstream `tilt_max_change`.
    ///
    /// `up` is hover-ward (`newtilt < current_tilt`). `dt_s` is
    /// `plane.G_Dt`. The 90 DPS fast-tilt override and flap-range
    /// argument are later slices.
    #[must_use]
    pub const fn tilt_max_change(&self, up: bool, dt_s: f32) -> f32 {
        let rate = if up || self.max_rate_down_dps <= 0 {
            self.max_rate_up_dps as f32
        } else {
            self.max_rate_down_dps as f32
        };
        let dt = if dt_s < 0.0 { 0.0 } else { dt_s };
        rate * dt * (1.0 / 90.0)
    }

    /// Slew `current_tilt` toward `newtilt`, upstream `Tiltrotor::slew`.
    ///
    /// `newtilt` is 0..1. `dt_s` is `plane.G_Dt`. Servo output
    /// (`k_motor_tilt`) is a later slice.
    pub fn slew(&mut self, newtilt: f32, dt_s: f32) {
        let up = newtilt < self.current_tilt;
        let max_change = self.tilt_max_change(up, dt_s);
        self.current_tilt = constrain_f32(
            newtilt,
            self.current_tilt - max_change,
            self.current_tilt + max_change,
        );
        self.angle_achieved = is_equal_f32(newtilt, self.current_tilt);
    }
}

const fn abs_f32(v: f32) -> f32 {
    if v < 0.0 {
        -v
    } else {
        v
    }
}

const fn constrain_f32(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

const fn is_equal_f32(a: f32, b: f32) -> bool {
    abs_f32(a - b) < f32::EPSILON
}
