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
