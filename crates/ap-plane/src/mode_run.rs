//! What a mode does each iteration, and what resets it.
//!
//! Upstream `ArduPlane/mode.cpp`: `Mode::run`, `Mode::reset_controllers`,
//! `Mode::pre_arm_checks`, `Mode::output_pilot_throttle`, `Mode::is_taking_
//! off`, `Mode::output_rudder_and_steering`.

use ap_servo::function::Function;
use ap_tecs::params::FlightStage;

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

/// Upstream `Mode::is_taking_off`: is the vehicle in the takeoff flight
/// stage?
///
/// A one-line equality check upstream, kept as a pure predicate over an
/// explicit `flight_stage` rather than reaching into vehicle state, matching
/// this port's own `ADR-0012` convention (see e.g.
/// [`crate::failsafe_in_landing_sequence_hookup`]'s own `flight_stage`
/// parameter). [`FlightStage`] is `ap-tecs`'s own port of
/// `AP_FixedWing::FlightStage` — reused directly rather than reinvented,
/// since it already carries a real `Takeoff` variant with the correct
/// upstream discriminant.
///
/// `ap-plane`'s `throttle_rules::allow_fw_systemid` already ported upstream's
/// `if (is_taking_off() || is_landing())` gate (real `mode.cpp:394`) as
/// `SystemIdContext.taking_off`, an externally-supplied `bool` — this
/// function is the real computation that value was always meant to come
/// from. Wiring `allow_fw_systemid`'s caller to actually call this is a
/// separate integration concern, left untouched here.
#[must_use]
pub fn is_taking_off(flight_stage: FlightStage) -> bool {
    flight_stage == FlightStage::Takeoff
}

/// What `Mode::output_rudder_and_steering` writes to the servo registry:
/// the same value, broadcast to both `k_rudder` and `k_steering`.
///
/// Upstream performs the two `SRV_Channels::set_output_scaled` calls
/// directly; this only decides what should be written; matching this port's
/// own "decide, don't act" convention (see [`pilot_throttle_source`] above,
/// upstream `Mode::output_pilot_throttle`'s own port). The caller applies
/// each pair with `ap_servo::registry::Registry::set_output_scaled`.
///
/// Both entries reuse [`Function`], `ap-servo`'s own real representation of
/// `SRV_Channel::Function` (`Function::RUDDER` / `Function::STEERING`,
/// upstream `k_rudder` / `k_steering`), rather than a bare pair of `f32`s —
/// keeping the channel identity attached to the value instead of leaving the
/// caller to already know which two functions this describes.
///
/// `vtol-rust`'s `ModeQAutotune::run` already ported its own call to
/// `output_rudder_and_steering(0.0)` as a bare decision flag,
/// `QAutotuneRun::rudder_centered: bool` (`crates/ap-quadplane/src/mode_
/// qautotune.rs`), without implementing the underlying mechanism. It is a
/// candidate that could now be wired to this function, but doing so is
/// `VT-009`'s own already-closed scope, not this ticket's — left untouched.
#[must_use]
pub fn output_rudder_and_steering(val: f32) -> RudderSteeringOutput {
    RudderSteeringOutput {
        rudder: (Function::RUDDER, val),
        steering: (Function::STEERING, val),
    }
}

/// The two servo-function writes [`output_rudder_and_steering`] decided on.
///
/// `rudder` and `steering` are always `(Function::RUDDER, val)` and
/// `(Function::STEERING, val)` for the same `val` — upstream's real function
/// has no asymmetry between the two channels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RudderSteeringOutput {
    /// `(Function::RUDDER, val)`, upstream's `k_rudder` write.
    pub rudder: (Function, f32),
    /// `(Function::STEERING, val)`, upstream's `k_steering` write.
    pub steering: (Function, f32),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn is_taking_off_true_for_takeoff_stage() {
        assert!(is_taking_off(FlightStage::Takeoff));
    }

    #[test]
    fn is_taking_off_false_for_normal_stage() {
        assert!(!is_taking_off(FlightStage::Normal));
    }

    #[test]
    fn is_taking_off_false_for_land_stage() {
        assert!(!is_taking_off(FlightStage::Land));
    }

    #[test]
    fn is_taking_off_false_for_vtol_stage() {
        assert!(!is_taking_off(FlightStage::Vtol));
    }

    #[test]
    fn is_taking_off_false_for_abort_landing_stage() {
        assert!(!is_taking_off(FlightStage::AbortLanding));
    }

    #[test]
    fn output_rudder_and_steering_broadcasts_identical_fractional_value() {
        let out = output_rudder_and_steering(0.37);

        // Both channels carry the exact same value the caller passed in --
        // not swapped, not independently derived.
        assert_eq!(out.rudder, (Function::RUDDER, 0.37));
        assert_eq!(out.steering, (Function::STEERING, 0.37));
        assert_eq!(out.rudder.1, out.steering.1);

        // And it is genuinely the two distinct real channel functions, not
        // the same function written twice.
        assert_ne!(out.rudder.0, out.steering.0);
    }
}
