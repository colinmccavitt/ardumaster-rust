//! The mode number table and the mode-exit decision.
//!
//! Upstream `ArduPlane/control_modes.cpp:6`, `Plane::mode_from_mode_num`, and
//! `ArduPlane/mode.cpp:16`, `Mode::exit`.

/// A Plane flight mode, by upstream's number.
///
/// The numbers are MAVLink's `custom_mode` for this vehicle: a ground station
/// sends them and every log records them, so they are the port's contract
/// rather than an internal choice. Gaps in the sequence are real — the numbers
/// are historical, and several are held by modes this build does not compile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ModeNumber {
    /// 0
    Manual,
    /// 1
    Circle,
    /// 2
    Stabilize,
    /// 3
    Training,
    /// 4
    Acro,
    /// 5
    FlyByWireA,
    /// 6
    FlyByWireB,
    /// 7
    Cruise,
    /// 8
    Autotune,
    /// 10
    Auto,
    /// 11
    Rtl,
    /// 12
    Loiter,
    /// 13
    Takeoff,
    /// 14 — and see [`from_number`](ModeNumber::from_number) on why 14 may not
    /// mean this.
    AvoidAdsb,
    /// 15
    Guided,
    /// 16
    Initialising,
    /// 17
    QStabilize,
    /// 18
    QHover,
    /// 19
    QLoiter,
    /// 20
    QLand,
    /// 21
    QRtl,
    /// 22
    QAutotune,
    /// 23
    QAcro,
    /// 24
    Thermal,
    /// 25
    LoiterAltQLand,
    /// 26
    Autoland,
}

impl ModeNumber {
    /// Upstream's number for this mode.
    #[must_use]
    pub fn as_number(self) -> u8 {
        match self {
            Self::Manual => 0,
            Self::Circle => 1,
            Self::Stabilize => 2,
            Self::Training => 3,
            Self::Acro => 4,
            Self::FlyByWireA => 5,
            Self::FlyByWireB => 6,
            Self::Cruise => 7,
            Self::Autotune => 8,
            Self::Auto => 10,
            Self::Rtl => 11,
            Self::Loiter => 12,
            Self::Takeoff => 13,
            Self::AvoidAdsb => 14,
            Self::Guided => 15,
            Self::Initialising => 16,
            Self::QStabilize => 17,
            Self::QHover => 18,
            Self::QLoiter => 19,
            Self::QLand => 20,
            Self::QRtl => 21,
            Self::QAutotune => 22,
            Self::QAcro => 23,
            Self::Thermal => 24,
            Self::LoiterAltQLand => 25,
            Self::Autoland => 26,
        }
    }

    /// The mode a number denotes, or `None` if no mode does.
    ///
    /// # The features are not a formality
    ///
    /// Upstream's switch is threaded with `#if`s, and a number whose case is
    /// compiled out returns null rather than a mode. So which numbers are
    /// valid is a property of the build, not of the protocol, and a ground
    /// station asking for a mode this firmware does not have gets a refusal
    /// rather than a wrong mode.
    ///
    /// # Except once
    ///
    /// `AVOID_ADSB` (14) is the exception, and it is deliberate: its `break`
    /// sits *inside* the `#if`, so with ADSB compiled out the case falls
    /// through to `GUIDED`. Upstream marks it with a comment. A vehicle asked
    /// to avoid traffic it cannot see is put into guided flight rather than
    /// refused — which keeps it under the ground station's control instead of
    /// leaving it in whatever it was doing.
    ///
    /// That fallthrough is the one place in this table where a number means a
    /// different mode depending on the build, so it is a parameter here rather
    /// than a compile-time choice.
    #[must_use]
    pub fn from_number(number: u8, features: &BuildFeatures) -> Option<Self> {
        let mode = match number {
            0 => Self::Manual,
            1 => Self::Circle,
            2 => Self::Stabilize,
            3 => Self::Training,
            4 => Self::Acro,
            5 => Self::FlyByWireA,
            6 => Self::FlyByWireB,
            7 => Self::Cruise,
            8 => Self::Autotune,
            10 => Self::Auto,
            11 => Self::Rtl,
            12 => Self::Loiter,
            13 => Self::Takeoff,
            14 => {
                if features.adsb {
                    Self::AvoidAdsb
                } else {
                    // The fallthrough, not a mistake. See above.
                    Self::Guided
                }
            }
            15 => Self::Guided,
            16 => Self::Initialising,
            17 if features.quadplane => Self::QStabilize,
            18 if features.quadplane => Self::QHover,
            19 if features.quadplane => Self::QLoiter,
            20 if features.quadplane => Self::QLand,
            21 if features.quadplane => Self::QRtl,
            22 if features.quadplane && features.qautotune => Self::QAutotune,
            23 if features.quadplane => Self::QAcro,
            24 if features.soaring => Self::Thermal,
            25 if features.quadplane => Self::LoiterAltQLand,
            26 if features.autoland => Self::Autoland,
            _ => return None,
        };
        Some(mode)
    }
}

/// Which optional modes this firmware was built with.
///
/// Upstream expresses these as `#if`s around the table's cases. They are
/// runtime data here because a port that baked them in could not be tested
/// against a build configured differently, and because the ADSB one changes
/// what an existing number means rather than only whether it is valid.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct BuildFeatures {
    /// `HAL_ADSB_ENABLED`. When false, 13 means Guided.
    pub adsb: bool,
    /// `HAL_QUADPLANE_ENABLED`.
    pub quadplane: bool,
    /// `QAUTOTUNE_ENABLED`, itself inside the quadplane block.
    pub qautotune: bool,
    /// `HAL_SOARING_ENABLED`.
    pub soaring: bool,
    /// `MODE_AUTOLAND_ENABLED`.
    pub autoland: bool,
}

/// Whether leaving a mode should put the autotuned gains back, upstream the
/// second half of `Mode::exit`.
///
/// # It reads the mode being entered, not the one being left
///
/// `Mode::exit` runs after `set_mode` has already assigned `control_mode`, so
/// the mode it compares against is the *new* one. Reading it as "the mode
/// being left is not autotune" would restore the gains on every exit from
/// autotune — which is the exact opposite of what the line does, and would
/// discard a tune the moment it finished.
///
/// The condition is: having just entered something other than autotune, put
/// the original gains back. Entering autotune from autotune — a mode change
/// that does not change mode — never reaches here at all, because `set_mode`
/// returns early.
#[must_use]
pub fn restores_autotune_gains(entered_mode: ModeNumber) -> bool {
    entered_mode != ModeNumber::Autotune
}
