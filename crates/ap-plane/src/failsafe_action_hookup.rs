//! Short vs long RC failsafe action table, `FS_SHORT_ACTN` / `FS_LONG_ACTN`.
//!
//! Upstream `ArduPlane/events.cpp` (`rc_failsafe_short_on_event`,
//! `failsafe_long_on_event`), `Plane::check_short_rc_failsafe` in
//! `ArduPlane/system.cpp`, and the enums in `ArduPlane/defines.h`.
//!
//! This stub maps a mode plus the parameter to the mode change the vehicle
//! would ask for. Landing-sequence / emergency-landing / takeoff-pending
//! gates and Q_OPTIONS RTL/QRTL overrides are left for a later slice.

use crate::mode_table::ModeNumber;

/// Upstream `failsafe_action_short` / `FS_SHORT_ACTN`.
///
/// Default is [`Self::BestGuess`]. Value 3 disables the short event entirely
/// (`check_short_rc_failsafe` never calls `rc_failsafe_short_on_event`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FailsafeActionShort {
    /// 0 — CIRCLE in stick modes; no change in AUTO/GUIDED/LOITER.
    BestGuess = 0,
    /// 1 — CIRCLE.
    Circle = 1,
    /// 2 — FBWA (zero throttle once FBWA is entered).
    Fbwa = 2,
    /// 3 — short failsafe disabled.
    Disabled = 3,
    /// 4 — FBWB.
    Fbwb = 4,
}

impl FailsafeActionShort {
    /// Decode `FS_SHORT_ACTN`. Unknown values are `None`.
    #[must_use]
    pub const fn from_param(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::BestGuess),
            1 => Some(Self::Circle),
            2 => Some(Self::Fbwa),
            3 => Some(Self::Disabled),
            4 => Some(Self::Fbwb),
            _ => None,
        }
    }

    /// Upstream `FS_SHORT_ACTN` default, `FS_ACTION_SHORT_BESTGUESS`.
    #[must_use]
    pub const fn default_param() -> Self {
        Self::BestGuess
    }

    /// Whether `check_short_rc_failsafe` will enter the short event.
    #[must_use]
    pub const fn is_enabled(self) -> bool {
        !matches!(self, Self::Disabled)
    }
}

/// Upstream `failsafe_action_long` / `FS_LONG_ACTN`.
///
/// Default is [`Self::Continue`]. In stick/manual modes Continue still
/// switches to RTL; only AUTO-like modes treat 0 as stay-put.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FailsafeActionLong {
    /// 0 — Continue (AUTO-like) or RTL (stick/manual).
    Continue = 0,
    /// 1 — RTL.
    Rtl = 1,
    /// 2 — Glide: switch to FBWA.
    Glide = 2,
    /// 3 — Deploy parachute; no mode change in this stub.
    Parachute = 3,
    /// 4 — Switch to AUTO at the current waypoint.
    Auto = 4,
    /// 5 — AUTOLAND, or RTL if that mode cannot start.
    Autoland = 5,
}

impl FailsafeActionLong {
    /// Decode `FS_LONG_ACTN`. Unknown values are `None`.
    #[must_use]
    pub const fn from_param(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Continue),
            1 => Some(Self::Rtl),
            2 => Some(Self::Glide),
            3 => Some(Self::Parachute),
            4 => Some(Self::Auto),
            5 => Some(Self::Autoland),
            _ => None,
        }
    }

    /// Upstream `FS_LONG_ACTN` default, `FS_ACTION_LONG_CONTINUE`.
    #[must_use]
    pub const fn default_param() -> Self {
        Self::Continue
    }
}

/// What the action table asks the vehicle to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FailsafeActionResult {
    /// Stay in the current mode (no `set_mode`).
    Continue,
    /// `set_mode` to this number.
    Switch(ModeNumber),
    /// `parachute_release()`; mode unchanged.
    Parachute,
}

/// Mode groups used by `rc_failsafe_short_on_event`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ShortGroup {
    /// MANUAL / STABILIZE / ACRO / FBWA / AUTOTUNE / FBWB / CRUISE / TRAINING.
    Stick,
    /// AUTO / AUTOLAND / AVOID_ADSB / GUIDED / LOITER / THERMAL.
    AutoLike,
    /// CIRCLE / TAKEOFF / RTL / QLAND / QRTL / LOITER_ALT_QLAND / INITIALISING.
    Never,
    /// QSTABILIZE / QLOITER / QHOVER / QAUTOTUNE / QACRO — default QLAND.
    Quadplane,
}

/// Mode groups used by `failsafe_long_on_event`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LongGroup {
    /// Stick/manual plus CIRCLE / LOITER / THERMAL / TAKEOFF.
    StickOrHold,
    /// AUTO / AVOID_ADSB / GUIDED.
    AutoLike,
    /// RTL: only AUTO and AUTOLAND change mode.
    Rtl,
    /// QSTABILIZE / QHOVER / QLOITER / QACRO / QAUTOTUNE — default QLAND.
    Quadplane,
    /// QLAND / QRTL / LOITER_ALT_QLAND / INITIALISING / AUTOLAND.
    Never,
}

fn short_group(mode: ModeNumber) -> ShortGroup {
    match mode {
        ModeNumber::Manual
        | ModeNumber::Stabilize
        | ModeNumber::Acro
        | ModeNumber::FlyByWireA
        | ModeNumber::Autotune
        | ModeNumber::FlyByWireB
        | ModeNumber::Cruise
        | ModeNumber::Training => ShortGroup::Stick,
        ModeNumber::Auto
        | ModeNumber::Autoland
        | ModeNumber::AvoidAdsb
        | ModeNumber::Guided
        | ModeNumber::Loiter
        | ModeNumber::Thermal => ShortGroup::AutoLike,
        ModeNumber::Circle
        | ModeNumber::Takeoff
        | ModeNumber::Rtl
        | ModeNumber::QLand
        | ModeNumber::QRtl
        | ModeNumber::LoiterAltQLand
        | ModeNumber::Initialising => ShortGroup::Never,
        ModeNumber::QStabilize
        | ModeNumber::QLoiter
        | ModeNumber::QHover
        | ModeNumber::QAutotune
        | ModeNumber::QAcro => ShortGroup::Quadplane,
    }
}

fn long_group(mode: ModeNumber) -> LongGroup {
    match mode {
        ModeNumber::Manual
        | ModeNumber::Stabilize
        | ModeNumber::Acro
        | ModeNumber::FlyByWireA
        | ModeNumber::Autotune
        | ModeNumber::FlyByWireB
        | ModeNumber::Cruise
        | ModeNumber::Training
        | ModeNumber::Circle
        | ModeNumber::Loiter
        | ModeNumber::Thermal
        | ModeNumber::Takeoff => LongGroup::StickOrHold,
        ModeNumber::Auto | ModeNumber::AvoidAdsb | ModeNumber::Guided => LongGroup::AutoLike,
        ModeNumber::Rtl => LongGroup::Rtl,
        ModeNumber::QStabilize
        | ModeNumber::QHover
        | ModeNumber::QLoiter
        | ModeNumber::QAcro
        | ModeNumber::QAutotune => LongGroup::Quadplane,
        ModeNumber::QLand
        | ModeNumber::QRtl
        | ModeNumber::LoiterAltQLand
        | ModeNumber::Initialising
        | ModeNumber::Autoland => LongGroup::Never,
    }
}

fn short_requested_mode(action: FailsafeActionShort) -> ModeNumber {
    match action {
        FailsafeActionShort::Fbwa => ModeNumber::FlyByWireA,
        FailsafeActionShort::Fbwb => ModeNumber::FlyByWireB,
        FailsafeActionShort::BestGuess
        | FailsafeActionShort::Circle
        | FailsafeActionShort::Disabled => ModeNumber::Circle,
    }
}

fn long_requested(
    action: FailsafeActionLong,
    autoland_available: bool,
    continue_is_rtl: bool,
) -> FailsafeActionResult {
    match action {
        FailsafeActionLong::Parachute => FailsafeActionResult::Parachute,
        FailsafeActionLong::Glide => FailsafeActionResult::Switch(ModeNumber::FlyByWireA),
        FailsafeActionLong::Auto => FailsafeActionResult::Switch(ModeNumber::Auto),
        FailsafeActionLong::Autoland => {
            if autoland_available {
                FailsafeActionResult::Switch(ModeNumber::Autoland)
            } else {
                FailsafeActionResult::Switch(ModeNumber::Rtl)
            }
        }
        FailsafeActionLong::Rtl => FailsafeActionResult::Switch(ModeNumber::Rtl),
        FailsafeActionLong::Continue => {
            if continue_is_rtl {
                FailsafeActionResult::Switch(ModeNumber::Rtl)
            } else {
                FailsafeActionResult::Continue
            }
        }
    }
}

/// Resolve `FS_SHORT_ACTN` for `mode`, upstream `rc_failsafe_short_on_event`.
///
/// [`FailsafeActionShort::Disabled`] never enters the event, so the table
/// returns [`FailsafeActionResult::Continue`] for every mode.
#[must_use]
pub fn short_failsafe_action(
    mode: ModeNumber,
    action: FailsafeActionShort,
) -> FailsafeActionResult {
    if !action.is_enabled() {
        return FailsafeActionResult::Continue;
    }
    match short_group(mode) {
        ShortGroup::Never => FailsafeActionResult::Continue,
        ShortGroup::Quadplane => FailsafeActionResult::Switch(ModeNumber::QLand),
        ShortGroup::Stick => FailsafeActionResult::Switch(short_requested_mode(action)),
        ShortGroup::AutoLike => {
            if matches!(action, FailsafeActionShort::BestGuess) {
                FailsafeActionResult::Continue
            } else {
                FailsafeActionResult::Switch(short_requested_mode(action))
            }
        }
    }
}

/// Resolve `FS_LONG_ACTN` for `mode`, upstream `failsafe_long_on_event`.
///
/// `autoland_available` is whether `set_mode(AUTOLAND)` would succeed. RTL
/// plus AUTOLAND still asks for AUTOLAND even when that start would fail,
/// because upstream does not fall back from RTL.
#[must_use]
pub fn long_failsafe_action(
    mode: ModeNumber,
    action: FailsafeActionLong,
    autoland_available: bool,
) -> FailsafeActionResult {
    match long_group(mode) {
        LongGroup::Never => FailsafeActionResult::Continue,
        LongGroup::Quadplane => FailsafeActionResult::Switch(ModeNumber::QLand),
        LongGroup::StickOrHold => long_requested(action, autoland_available, true),
        LongGroup::AutoLike => long_requested(action, autoland_available, false),
        LongGroup::Rtl => match action {
            FailsafeActionLong::Auto => FailsafeActionResult::Switch(ModeNumber::Auto),
            FailsafeActionLong::Autoland => FailsafeActionResult::Switch(ModeNumber::Autoland),
            _ => FailsafeActionResult::Continue,
        },
    }
}
