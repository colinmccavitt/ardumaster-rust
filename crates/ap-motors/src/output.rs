//! Turning an actuator demand into a PWM pulse, upstream
//! `AP_MotorsMulticopter`'s output stage. COP-004.
//!
//! By this point the mixer has decided what fraction of its range each motor
//! should run at. Three things still have to happen: the demand is slew
//! limited so a step in the controller does not become a step at the ESC, it
//! is mapped onto the configured pulse width, and while the rotors are coming
//! up to idle it is driven by the spin ramp rather than by the mixer.

use ap_math::scalar::is_positive;

use crate::spool::SpoolState;

/// The pulse widths the ESCs are configured for, upstream `MOT_PWM_MIN` and
/// `MOT_PWM_MAX`, plus whether PWM is suppressed entirely while disarmed.
#[derive(Debug, Clone, Copy)]
pub struct PwmParams {
    /// `MOT_PWM_MIN`, the pulse for zero output.
    pub pwm_min: i16,
    /// `MOT_PWM_MAX`, the pulse for full output.
    pub pwm_max: i16,
    /// `MOT_SAFE_DISARM`: whether to stop sending pulses at all when disarmed.
    pub disarm_disable_pwm: bool,
    /// The parameter *default* behind [`Self::pwm_min`].
    ///
    /// Carried because `update_throttle_range` uses `set_and_default`, which
    /// writes the default as well as the value. Only the value affects the
    /// pulse widths, so it would be easy to drop -- but the default is what a
    /// parameter reset restores, and a port that silently kept the old one
    /// would hand a digital-output vehicle back its analog endpoints.
    pub pwm_min_default: i16,
    /// The parameter default behind [`Self::pwm_max`]. See
    /// [`Self::pwm_min_default`].
    pub pwm_max_default: i16,
}

/// How the outputs are driven, upstream `MOT_PWM_TYPE`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PwmType {
    /// Ordinary servo-rate PWM.
    #[default]
    Normal = 0,
    /// One pulse per loop, sent as soon as the value is known.
    OneShot = 1,
    /// OneShot with the pulse width divided by eight.
    OneShot125 = 2,
    /// A raw duty cycle for brushed motors.
    Brushed = 3,
    /// Digital, 150 kbit/s.
    DShot150 = 4,
    /// Digital, 300 kbit/s.
    DShot300 = 5,
    /// Digital, 600 kbit/s.
    DShot600 = 6,
    /// Digital, 1200 kbit/s.
    DShot1200 = 7,
    /// Scaled output mapped to PWM by the servo layer.
    PwmRange = 8,
    /// Scaled angle output mapped to PWM by the servo layer.
    PwmAngle = 9,
}

impl PwmParams {
    /// Whether the endpoints make sense, upstream `check_mot_pwm_params`.
    ///
    /// A minimum below 1 would be indistinguishable from "no pulse", and a
    /// maximum at or below the minimum leaves no range to command.
    pub fn valid(&self) -> bool {
        self.pwm_min >= 1 && self.pwm_min < self.pwm_max
    }
}

/// How fast the output may move, upstream `MOT_SLEW_UP_TIME` and
/// `MOT_SLEW_DN_TIME`.
#[derive(Debug, Clone, Copy)]
pub struct SlewParams {
    /// Seconds to travel the whole range upward. Zero disables up-limiting.
    pub slew_up_time: f32,
    /// Seconds to travel the whole range downward. Zero disables
    /// down-limiting.
    pub slew_dn_time: f32,
}

/// The longest slew time upstream will honour.
///
/// Half a second to cross the range is already slow enough to be a handling
/// problem; the clamp stops a mistyped parameter from making the aircraft
/// unflyable rather than merely sluggish.
const SLEW_TIME_MAX: f32 = 0.5;

/// Map an actuator demand onto a pulse width, upstream `output_to_pwm`.
///
/// In `SHUT_DOWN` the demand is ignored: the output is either the minimum
/// pulse or no pulse at all, depending on whether the vehicle is configured to
/// stop driving the ESCs while disarmed.
///
/// The result is truncated, not rounded — upstream computes a `float` and
/// returns it through an `int16_t`, so 1499.97 is 1499. Rust's `as`
/// additionally saturates where C++ would be undefined, which only differs for
/// endpoints far outside any pulse width an ESC would accept.
#[expect(
    clippy::cast_possible_truncation,
    reason = "the truncation is the ported behaviour: upstream returns a float \
through an int16_t return type"
)]
pub fn output_to_pwm(state: SpoolState, armed: bool, params: &PwmParams, actuator: f32) -> i16 {
    let pwm_output = if state == SpoolState::ShutDown {
        if params.disarm_disable_pwm && !armed {
            0.0
        } else {
            f32::from(params.pwm_min)
        }
    } else {
        f32::from(params.pwm_min) + f32::from(params.pwm_max - params.pwm_min) * actuator
    };

    pwm_output as i16
}

/// Move an actuator output toward `input`, no faster than the slew limits
/// allow. Upstream `set_actuator_with_slew`.
///
/// Each limit is only applied when its time is positive; zero — the default —
/// means that direction is unlimited. The limits are computed from the
/// *current* output, so they bound the step rather than the destination.
///
/// There is no `SHUT_DOWN` check here even though upstream's comment above the
/// function mentions one. That is not an omission: `output_to_motors` assigns
/// zero to the actuator directly in that state and never calls this, so the
/// unlimited de-energisation happens at the call site. Adding a check here
/// would be dead code that reads like a safety property.
pub fn set_actuator_with_slew(
    actuator_output: &mut f32,
    input: f32,
    dt_s: f32,
    params: &SlewParams,
) {
    // Unlimited unless a slew time says otherwise.
    let mut output_slew_limit_up = 1.0_f32;
    let mut output_slew_limit_dn = 0.0_f32;

    if is_positive(params.slew_up_time) {
        let output_delta_up_max = dt_s / params.slew_up_time.clamp(0.0, SLEW_TIME_MAX);
        output_slew_limit_up = (*actuator_output + output_delta_up_max).clamp(0.0, 1.0);
    }

    if is_positive(params.slew_dn_time) {
        let output_delta_dn_max = dt_s / params.slew_dn_time.clamp(0.0, SLEW_TIME_MAX);
        output_slew_limit_dn = (*actuator_output - output_delta_dn_max).clamp(0.0, 1.0);
    }

    *actuator_output = input.clamp(output_slew_limit_dn, output_slew_limit_up);
}

/// The actuator demand while the rotors are coming up to idle, upstream
/// `actuator_spin_up_to_ground_idle`.
///
/// The spin ramp runs 0 to 1 across the range between stopped and `SPIN_MIN`,
/// so scaling it by `SPIN_MIN` gives the actual output to send. The clamp
/// matters because the ramp is stepped before it is checked, so it can be
/// fractionally past 1 for one iteration.
pub fn actuator_spin_up_to_ground_idle(spin_up_ratio: f32, spin_min: f32) -> f32 {
    spin_up_ratio.clamp(0.0, 1.0) * spin_min
}

/// Set the pulse endpoints for the output type, upstream
/// `update_throttle_range`.
///
/// Digital protocols and the two scaled types do not use the endpoints as
/// microseconds at all -- the servo layer maps a normalised value onto
/// whatever the protocol wants -- so upstream pins them to a plain 1000-2000
/// range rather than leaving whatever an analog setup was configured with.
///
/// Returns the ESC scaling to hand the RC output layer, which upstream applies
/// through `hal.rcout->set_esc_scaling`.
pub fn update_throttle_range(
    params: &mut PwmParams,
    pwm_type: PwmType,
    have_digital_outputs: bool,
) -> (i16, i16) {
    if have_digital_outputs || pwm_type == PwmType::PwmRange || pwm_type == PwmType::PwmAngle {
        // `set_and_default`, not a plain assignment: the default moves too, so
        // a later parameter reset restores 1000-2000 rather than the analog
        // endpoints this vehicle never used.
        params.pwm_min = 1000;
        params.pwm_min_default = 1000;
        params.pwm_max = 2000;
        params.pwm_max_default = 2000;
    }

    (params.pwm_min, params.pwm_max)
}

/// The value written to the boost-throttle channel, upstream
/// `output_boost_throttle`.
///
/// A boost motor runs proportionally to the main throttle, scaled by
/// `MOT_BOOST_SCALE`. A scale of zero or less means no boost motor, which
/// writes zero rather than skipping the write -- an unwritten channel would
/// hold its last value.
///
/// The `* 1000.0` is upstream's: the channel is scaled in thousandths.
pub fn boost_throttle_output(throttle: f32, boost_scale: f32) -> f32 {
    if boost_scale > 0.0 {
        (throttle * boost_scale).clamp(0.0, 1.0) * 1000.0
    } else {
        0.0
    }
}

/// The roll, pitch, yaw and thrust values written to their channels, upstream
/// `output_rpyt`.
///
/// Returned as `(roll, pitch, yaw, thrust)`. The angular three are scaled in
/// centidegrees against a 45 degree full-scale; thrust, like the boost
/// channel, is in thousandths.
pub fn rpyt_outputs(
    roll_in_ff: f32,
    pitch_in_ff: f32,
    yaw_in_ff: f32,
    throttle: f32,
) -> (f32, f32, f32, f32) {
    (
        roll_in_ff * 4500.0,
        pitch_in_ff * 4500.0,
        yaw_in_ff * 4500.0,
        throttle * 1000.0,
    )
}
