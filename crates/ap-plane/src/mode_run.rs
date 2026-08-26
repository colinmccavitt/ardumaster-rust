//! What a mode does each iteration, and what resets it.
//!
//! Upstream `ArduPlane/mode.cpp`: `Mode::run`, `Mode::reset_controllers`,
//! `Mode::pre_arm_checks`, `Mode::output_pilot_throttle`.

/// How much of the pilot's stick reaches a stabilised mode's output.
///
/// Upstream `StickMixing` in `defines.h`, and the numbers are a parameter
/// (`STICK_MIXING`), so they are stored in aircraft and cannot be renumbered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StickMixing {
    /// 0 — no mixing at all.
    None,
    /// 1 — fly-by-wire mixing on roll and pitch.
    Fbw,
    /// 2 — a removed option, kept because aircraft in the field still have it
    /// stored. See [`applies_fbw_stick_mixing`].
    DirectRemoved,
    /// 3 — VTOL yaw mixing, which is not fixed-wing stick mixing.
    VtolYaw,
    /// 4 — fly-by-wire mixing on roll only.
    FbwNoPitch,
}

impl StickMixing {
    /// The stored parameter value.
    #[must_use]
    pub fn as_number(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Fbw => 1,
            Self::DirectRemoved => 2,
            Self::VtolYaw => 3,
            Self::FbwNoPitch => 4,
        }
    }

    /// The setting a stored parameter value denotes, or `None` if it is not a
    /// setting.
    ///
    /// Upstream's switch has no default case, so a value outside the enum
    /// simply matches nothing and no mixing is applied. That is represented
    /// here as `None` rather than silently folded into `StickMixing::None`,
    /// because "the parameter is out of range" and "the pilot asked for no
    /// mixing" are different facts even though the aircraft flies the same.
    #[must_use]
    pub fn from_number(number: u8) -> Option<Self> {
        match number {
            0 => Some(Self::None),
            1 => Some(Self::Fbw),
            2 => Some(Self::DirectRemoved),
            3 => Some(Self::VtolYaw),
            4 => Some(Self::FbwNoPitch),
            _ => None,
        }
    }
}

/// Whether this iteration applies fly-by-wire stick mixing, upstream the
/// switch at the top of `Mode::run`.
///
/// # A removed option that still does something
///
/// `DIRECT_REMOVED` was direct stick mixing, which was taken out. It maps to
/// FBW mixing rather than to nothing, and upstream's comment says why: an
/// aircraft that had direct mixing configured would otherwise lose stick
/// authority entirely at the next firmware update, which is a worse surprise
/// than a different flavour of mixing. Reusing the value is cheaper than a
/// parameter conversion.
///
/// So three of the five values mix and two do not, and the two that do not
/// are the ones that mean something else: no mixing, and VTOL yaw mixing,
/// which is not this.
#[must_use]
pub fn applies_fbw_stick_mixing(setting: Option<StickMixing>) -> bool {
    matches!(
        setting,
        Some(StickMixing::Fbw | StickMixing::FbwNoPitch | StickMixing::DirectRemoved)
    )
}

/// The steering state `Mode::reset_controllers` clears.
///
/// The integrators it also resets belong to the three attitude controllers
/// and to TECS, which own their own reset and are not duplicated here.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SteerReset {
    /// Upstream `steer_state.locked_course`.
    pub locked_course: bool,
    /// Upstream `steer_state.locked_course_err`.
    pub locked_course_err: f32,
}

impl SteerReset {
    /// Upstream `Mode::reset_controllers`' effect on the steering state.
    ///
    /// A locked course is a heading the aircraft was told to hold against
    /// crosswind. Carrying one into a reset would have the vehicle correcting
    /// towards a course that belonged to a manoeuvre that has ended.
    pub fn reset(&mut self) {
        self.locked_course = false;
        self.locked_course_err = 0.0;
    }
}

/// What a pre-arm check decided, and what to tell the pilot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PreArmResult<'a> {
    /// Armable in this mode.
    Allowed,
    /// Not armable, with the message to show.
    Refused(&'a str),
}

/// The generic message used when a mode refuses without saying why.
///
/// Upstream fills this in rather than showing an empty reason, because a
/// refusal with no text reads to a pilot as a bug in the ground station
/// rather than as a decision by the aircraft.
pub const GENERIC_REFUSAL: &str = "mode not armable";

/// Upstream `Mode::pre_arm_checks`.
///
/// The mode's own `_pre_arm_checks` decides; this only ensures the decision
/// arrives with an explanation attached.
#[must_use]
pub fn pre_arm_checks(mode_allows: bool, mode_message: &str) -> PreArmResult<'_> {
    if mode_allows {
        return PreArmResult::Allowed;
    }
    if mode_message.is_empty() {
        return PreArmResult::Refused(GENERIC_REFUSAL);
    }
    PreArmResult::Refused(mode_message)
}

/// Which throttle input a stabilised mode passes through, upstream
/// `Mode::output_pilot_throttle`.
///
/// `THR_PASS_STAB` maps the stick straight to the output. Otherwise the input
/// is adjusted so that centre stick produces `TRIM_THROTTLE` rather than half
/// travel — which is what a pilot expects from a trimmed aircraft, and which
/// a direct mapping cannot give them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PilotThrottleSource {
    /// The raw stick, `THR_PASS_STAB` set.
    Direct,
    /// The stick adjusted around the throttle trim.
    TrimAdjusted,
}

/// Upstream `Mode::output_pilot_throttle`'s choice.
#[must_use]
pub fn pilot_throttle_source(throttle_passthru_stabilize: bool) -> PilotThrottleSource {
    if throttle_passthru_stabilize {
        PilotThrottleSource::Direct
    } else {
        PilotThrottleSource::TrimAdjusted
    }
}
