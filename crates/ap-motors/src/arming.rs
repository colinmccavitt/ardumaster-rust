//! Pre-arm checks and the motor test sequence, upstream `AP_Motors`'
//! `arming_checks`, `motor_test_checks` and `output_test_seq`. COP-004.
//!
//! These are the last thing between a configuration mistake and a spinning
//! propeller, so the port carries *which* check failed rather than a rendered
//! string. Upstream writes a message into a caller's buffer; a caller here can
//! render the same words, but it can also count how often a particular check
//! trips, which a formatted string makes needlessly hard.

use ap_servo::function::Function;
use ap_servo::registry::Registry;

use crate::output::PwmParams;

/// The highest `SPIN_MIN` upstream will arm with.
///
/// Above this the motors are already turning fast enough at "idle" that the
/// margin between armed and flying has largely gone.
const SPIN_MIN_MAX: f32 = 0.3;

/// Why a pre-arm check refused.
///
/// Ordered as upstream orders the checks: the first failure is the one
/// reported, and re-ordering them would change which message an operator sees
/// for a vehicle with more than one problem.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArmingFailure {
    /// The frame class and type did not produce a usable mixing table.
    /// Upstream: "Check frame class and type".
    FrameNotInitialised,
    /// A fitted motor has no output channel assigned to its function.
    /// Upstream: "no SERVOx_FUNCTION set to MotorN" — with `motor` here being
    /// the zero-based index, which upstream renders one-based.
    MotorWithoutChannel {
        /// The zero-based motor slot.
        motor: u8,
    },
    /// `MOT_SPIN_MIN` is above [`SPIN_MIN_MAX`].
    SpinMinTooHigh {
        /// The offending value.
        spin_min: f32,
    },
    /// `MOT_SPIN_ARM` is above `MOT_SPIN_MIN`, which would make the armed idle
    /// faster than the point thrust begins.
    SpinArmAboveSpinMin,
    /// `MOT_PWM_MIN` and `MOT_PWM_MAX` do not describe a usable range.
    BadPwmEndpoints,
}

/// What the checks need to know about the vehicle.
#[derive(Debug, Clone, Copy)]
pub struct ArmingContext<'a> {
    /// Whether `setup_motors` produced a usable frame.
    pub initialised_ok: bool,
    /// Which motor slots are fitted.
    pub motor_enabled: &'a [bool],
    /// `MOT_SPIN_MIN`.
    pub spin_min: f32,
    /// `MOT_SPIN_ARM`.
    pub spin_arm: f32,
    /// The PWM endpoints.
    pub pwm: PwmParams,
}

/// The full pre-arm checks, upstream `AP_MotorsMulticopter::arming_checks`.
///
/// The order is upstream's and is load-bearing: frame first, then output
/// assignment, then the parameter sanity checks. An operator with an
/// unconfigured frame *and* a bad `SPIN_MIN` should be told about the frame,
/// because fixing that may change which outputs are wanted.
pub fn arming_checks(ctx: &ArmingContext, registry: &Registry) -> Result<(), ArmingFailure> {
    if !ctx.initialised_ok {
        return Err(ArmingFailure::FrameNotInitialised);
    }

    for (i, &enabled) in ctx.motor_enabled.iter().enumerate() {
        if !enabled {
            continue;
        }
        let motor = u8::try_from(i).unwrap_or(u8::MAX);
        if registry.output_channel_mask(Function::motor(motor)) == 0 {
            return Err(ArmingFailure::MotorWithoutChannel { motor });
        }
    }

    if ctx.spin_min > SPIN_MIN_MAX {
        return Err(ArmingFailure::SpinMinTooHigh {
            spin_min: ctx.spin_min,
        });
    }
    if ctx.spin_arm > ctx.spin_min {
        return Err(ArmingFailure::SpinArmAboveSpinMin);
    }
    if !ctx.pwm.valid() {
        return Err(ArmingFailure::BadPwmEndpoints);
    }

    Ok(())
}

/// The checks a motor test runs, upstream `AP_Motors::motor_test_checks`.
///
/// Only the frame check, deliberately. Upstream says why: a motor test is less
/// strict than arming because not every output has to be assigned — the point
/// of the test may be to find out which ones are.
pub fn motor_test_checks(ctx: &ArmingContext) -> Result<(), ArmingFailure> {
    if ctx.initialised_ok {
        Ok(())
    } else {
        Err(ArmingFailure::FrameNotInitialised)
    }
}

/// What `output_test_seq` decided, upstream `AP_Motors::output_test_seq`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TestSeq {
    /// Drive the numbered motor at this pulse.
    Run {
        /// Which motor in test order.
        motor_seq: u8,
        /// The pulse to drive it at.
        pwm: i16,
    },
    /// Refuse, and put the outputs to their minimum.
    ///
    /// Upstream calls `output_min()` on this path rather than simply returning
    /// false — a refused test still leaves the aircraft safe rather than
    /// wherever the last command left it.
    RefuseAndMinimise,
}

/// Decide whether a motor test may run, upstream `output_test_seq`.
///
/// Both armed *and* the interlock. Testing a motor is exactly the situation
/// where a half-satisfied safety condition is dangerous, so upstream requires
/// both and so does this.
#[must_use]
pub fn output_test_seq(armed: bool, interlock: bool, motor_seq: u8, pwm: i16) -> TestSeq {
    if armed && interlock {
        TestSeq::Run { motor_seq, pwm }
    } else {
        TestSeq::RefuseAndMinimise
    }
}
