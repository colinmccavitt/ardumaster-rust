//! `AUTOTUNE_OPTIONS` filter bits and `AUTOTUNE_AXES` single-axis start mask.
//!
//! Upstream `AP_AutoTune::Options` / `has_option` gates the FLTD / FLTT
//! writes at the end of `update`. `ParametersG2::axis_bitmask`
//! (`AUTOTUNE_AXES`) chooses which rate controllers
//! `Plane::autotune_start` actually starts — roll only, pitch only, both,
//! and optionally yaw. I-term / FF coupling stays a later slice.

use crate::state::AtType;

/// Default `AUTOTUNE_OPTIONS` (`ASCALAR(..., 0)`).
pub const AUTOTUNE_OPTIONS_DEFAULT: u32 = 0;

/// Bit 0 — `AP_AutoTune::Options::DISABLE_FLTD_UPDATE`.
pub const AUTOTUNE_OPTION_DISABLE_FLTD_UPDATE: u32 = 1 << 0;

/// Bit 1 — `AP_AutoTune::Options::DISABLE_FLTT_UPDATE`.
pub const AUTOTUNE_OPTION_DISABLE_FLTT_UPDATE: u32 = 1 << 1;

/// Default `AUTOTUNE_AXES` (`AP_GROUPINFO(..., 7)` — roll+pitch+yaw).
pub const AUTOTUNE_AXES_DEFAULT: u8 = 7;

/// Upstream `AutoTuneAxis::ROLL` (`1U << 0`).
pub const AUTOTUNE_AXIS_ROLL: u8 = 1 << 0;

/// Upstream `AutoTuneAxis::PITCH` (`1U << 1`).
pub const AUTOTUNE_AXIS_PITCH: u8 = 1 << 1;

/// Upstream `AutoTuneAxis::YAW` (`1U << 2`).
pub const AUTOTUNE_AXIS_YAW: u8 = 1 << 2;

/// Upstream `AP_AutoTune::Options` bit index.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AutotuneOption {
    /// Bit 0 — do not rewrite `filt_D_hz` during the tune.
    DisableFltdUpdate = 0,
    /// Bit 1 — do not rewrite `filt_T_hz` during the tune.
    DisableFlttUpdate = 1,
}

impl AutotuneOption {
    /// Decode an upstream `Options` discriminant.
    #[must_use]
    pub const fn from_u8(raw: u8) -> Option<Self> {
        match raw {
            0 => Some(Self::DisableFltdUpdate),
            1 => Some(Self::DisableFlttUpdate),
            _ => None,
        }
    }

    /// Stored bit index, matching the C++ enum.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Mask bit stored in `AUTOTUNE_OPTIONS`.
    ///
    /// Upstream `uint32_t(1 << uint32_t(option))`.
    #[must_use]
    pub const fn bit(self) -> u32 {
        1 << (self as u32)
    }
}

/// Decoded `AUTOTUNE_OPTIONS`, upstream `AP_FixedWing::autotune_options`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutotuneOptions {
    /// Raw `AUTOTUNE_OPTIONS` integer.
    pub bits: u32,
}

impl Default for AutotuneOptions {
    fn default() -> Self {
        Self {
            bits: AUTOTUNE_OPTIONS_DEFAULT,
        }
    }
}

impl AutotuneOptions {
    /// Wrap a stored `AUTOTUNE_OPTIONS` value.
    #[must_use]
    pub const fn from_bits(bits: u32) -> Self {
        Self { bits }
    }

    /// Upstream `AP_AutoTune::has_option`.
    #[must_use]
    pub const fn has_option(self, option: AutotuneOption) -> bool {
        (self.bits & option.bit()) != 0
    }

    /// Bit 0: leave the operator's `FLTD` alone.
    #[must_use]
    pub const fn disable_fltd_update(self) -> bool {
        self.has_option(AutotuneOption::DisableFltdUpdate)
    }

    /// Bit 1: leave the operator's `FLTT` alone.
    #[must_use]
    pub const fn disable_fltt_update(self) -> bool {
        self.has_option(AutotuneOption::DisableFlttUpdate)
    }
}

/// FLTT target, upstream `10.0 / (current.tau * 2 * M_PI)`.
#[must_use]
pub fn fltt_hz(tau: f32) -> f32 {
    10.0 / (tau * 2.0 * core::f32::consts::PI)
}

/// FLTD target, upstream `AP::ins().get_gyro_filter_hz() * 0.5`.
#[must_use]
pub const fn fltd_hz(gyro_filter_hz: f32) -> f32 {
    gyro_filter_hz * 0.5
}

/// Filter writes after `has_option` is applied.
///
/// `filt_E_hz` is always forced to 0. `fltt_hz` / `fltd_hz` are `None`
/// when the matching disable bit is set.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilterUpdate {
    /// New `filt_T_hz` when bit 1 is clear.
    pub fltt_hz: Option<f32>,
    /// `filt_E_hz`, always 0.
    pub flte_hz: f32,
    /// New `filt_D_hz` when bit 0 is clear.
    pub fltd_hz: Option<f32>,
}

/// Apply `AUTOTUNE_OPTIONS` to one update's filter rewrite.
#[must_use]
pub fn apply_filter_options(
    options: AutotuneOptions,
    tau: f32,
    gyro_filter_hz: f32,
) -> FilterUpdate {
    FilterUpdate {
        fltt_hz: if options.disable_fltt_update() {
            None
        } else {
            Some(fltt_hz(tau))
        },
        flte_hz: 0.0,
        fltd_hz: if options.disable_fltd_update() {
            None
        } else {
            Some(fltd_hz(gyro_filter_hz))
        },
    }
}

/// Upstream `Plane::AutoTuneAxis`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AutotuneAxis {
    /// Bit 0 — start the roll tuner.
    Roll = 1 << 0,
    /// Bit 1 — start the pitch tuner.
    Pitch = 1 << 1,
    /// Bit 2 — start the yaw tuner.
    Yaw = 1 << 2,
}

impl AutotuneAxis {
    /// Mask bit stored in `AUTOTUNE_AXES`.
    #[must_use]
    pub const fn bit(self) -> u8 {
        self as u8
    }
}

/// Decoded `AUTOTUNE_AXES`, upstream `ParametersG2::axis_bitmask`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AutotuneAxes {
    /// Raw `AUTOTUNE_AXES` integer.
    pub bits: u8,
}

impl Default for AutotuneAxes {
    fn default() -> Self {
        Self {
            bits: AUTOTUNE_AXES_DEFAULT,
        }
    }
}

impl AutotuneAxes {
    /// Wrap a stored `AUTOTUNE_AXES` value.
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        Self { bits }
    }

    /// Roll-only mask, `AUTOTUNE_AXES = 1`.
    #[must_use]
    pub const fn roll_only() -> Self {
        Self {
            bits: AUTOTUNE_AXIS_ROLL,
        }
    }

    /// Pitch-only mask, `AUTOTUNE_AXES = 2`.
    #[must_use]
    pub const fn pitch_only() -> Self {
        Self {
            bits: AUTOTUNE_AXIS_PITCH,
        }
    }

    /// Roll+pitch mask (both attitude axes), `AUTOTUNE_AXES = 3`.
    #[must_use]
    pub const fn roll_and_pitch() -> Self {
        Self {
            bits: AUTOTUNE_AXIS_ROLL | AUTOTUNE_AXIS_PITCH,
        }
    }

    /// Whether `autotune_start` will start `axis`.
    #[must_use]
    pub const fn axis_enabled(self, axis: AutotuneAxis) -> bool {
        (self.bits & axis.bit()) != 0
    }

    /// Upstream `tune_roll`.
    #[must_use]
    pub const fn tune_roll(self) -> bool {
        self.axis_enabled(AutotuneAxis::Roll)
    }

    /// Upstream `tune_pitch`.
    #[must_use]
    pub const fn tune_pitch(self) -> bool {
        self.axis_enabled(AutotuneAxis::Pitch)
    }

    /// Upstream `tune_yaw`.
    #[must_use]
    pub const fn tune_yaw(self) -> bool {
        self.axis_enabled(AutotuneAxis::Yaw)
    }

    /// Any axis selected — otherwise GCS reports "No axis selected".
    #[must_use]
    pub const fn any_selected(self) -> bool {
        self.tune_roll() || self.tune_pitch() || self.tune_yaw()
    }

    /// Whether `Plane::autotune_start` starts the tuner for `axis`.
    #[must_use]
    pub const fn starts_type(self, axis: AtType) -> bool {
        match axis {
            AtType::Roll => self.tune_roll(),
            AtType::Pitch => self.tune_pitch(),
            AtType::Yaw => self.tune_yaw(),
        }
    }
}
