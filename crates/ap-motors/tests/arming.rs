//! Pre-arm checks and the motor test gate.
//!
//! These are the last thing between a configuration mistake and a spinning
//! propeller, so each check gets its own test, and the *order* gets one too: a
//! vehicle with several problems must be told about them in upstream's order,
//! because fixing an early one can change what the later ones should say.

use ap_motors::arming::{
    arming_checks, motor_test_checks, output_test_seq, ArmingContext, ArmingFailure, TestSeq,
};
use ap_motors::output::PwmParams;
use ap_servo::function::Function;
use ap_servo::registry::Registry;

fn pwm() -> PwmParams {
    PwmParams {
        pwm_min: 1000,
        pwm_max: 2000,
        disarm_disable_pwm: false,
        pwm_min_default: 1000,
        pwm_max_default: 2000,
    }
}

/// Four motors assigned straight through.
fn registry() -> Registry {
    let mut registry = Registry::new();
    let assignments: Vec<Function> = (0..32_u8)
        .map(|c| {
            if c < 4 {
                Function::motor(c)
            } else {
                Function::NONE
            }
        })
        .collect();
    registry.update_aux_servo_function(&assignments);
    registry
}

fn healthy() -> ([bool; 32], f32, f32) {
    let mut enabled = [false; 32];
    for slot in enabled.iter_mut().take(4) {
        *slot = true;
    }
    (enabled, 0.15, 0.10)
}

fn ctx<'a>(
    enabled: &'a [bool],
    spin_min: f32,
    spin_arm: f32,
    initialised_ok: bool,
) -> ArmingContext<'a> {
    ArmingContext {
        initialised_ok,
        motor_enabled: enabled,
        spin_min,
        spin_arm,
        pwm: pwm(),
    }
}

#[test]
fn a_healthy_vehicle_passes() {
    let (enabled, spin_min, spin_arm) = healthy();
    assert_eq!(
        arming_checks(&ctx(&enabled, spin_min, spin_arm, true), &registry()),
        Ok(())
    );
}

#[test]
fn an_unconfigured_frame_is_refused() {
    let (enabled, spin_min, spin_arm) = healthy();
    assert_eq!(
        arming_checks(&ctx(&enabled, spin_min, spin_arm, false), &registry()),
        Err(ArmingFailure::FrameNotInitialised)
    );
}

/// A fitted motor with no output channel is refused, and named.
#[test]
fn a_motor_without_an_output_is_refused() {
    let (mut enabled, spin_min, spin_arm) = healthy();
    enabled[5] = true; // fitted, but nothing assigned to motor 5

    assert_eq!(
        arming_checks(&ctx(&enabled, spin_min, spin_arm, true), &registry()),
        Err(ArmingFailure::MotorWithoutChannel { motor: 5 })
    );
}

#[test]
fn an_excessive_spin_min_is_refused() {
    let (enabled, _, spin_arm) = healthy();
    assert_eq!(
        arming_checks(&ctx(&enabled, 0.31, spin_arm, true), &registry()),
        Err(ArmingFailure::SpinMinTooHigh { spin_min: 0.31 })
    );
    // Exactly at the bound is allowed: upstream tests `>`, not `>=`.
    assert_eq!(
        arming_checks(&ctx(&enabled, 0.3, spin_arm, true), &registry()),
        Ok(())
    );
}

/// An armed idle above the point thrust begins is refused.
#[test]
fn spin_arm_above_spin_min_is_refused() {
    let (enabled, _, _) = healthy();
    assert_eq!(
        arming_checks(&ctx(&enabled, 0.15, 0.20, true), &registry()),
        Err(ArmingFailure::SpinArmAboveSpinMin)
    );
    // Equal is allowed: upstream tests `>`.
    assert_eq!(
        arming_checks(&ctx(&enabled, 0.15, 0.15, true), &registry()),
        Ok(())
    );
}

#[test]
fn unusable_pwm_endpoints_are_refused() {
    let (enabled, spin_min, spin_arm) = healthy();
    let mut c = ctx(&enabled, spin_min, spin_arm, true);
    c.pwm.pwm_max = 900; // below the minimum

    assert_eq!(
        arming_checks(&c, &registry()),
        Err(ArmingFailure::BadPwmEndpoints)
    );
}

/// With several problems, the earliest check is the one reported.
///
/// Order is load-bearing. An operator with an unconfigured frame and a bad
/// SPIN_MIN should be told about the frame, because fixing that can change
/// which outputs are wanted — and being told about the second problem first
/// sends them the wrong way.
#[test]
fn the_first_failure_is_the_one_reported() {
    let (mut enabled, _, _) = healthy();
    enabled[5] = true;

    // Frame, motor and parameters all bad: frame wins.
    let mut c = ctx(&enabled, 0.9, 0.95, false);
    c.pwm.pwm_max = 900;
    assert_eq!(
        arming_checks(&c, &registry()),
        Err(ArmingFailure::FrameNotInitialised)
    );

    // Frame fixed: the unassigned motor is next, ahead of the parameters.
    c.initialised_ok = true;
    assert_eq!(
        arming_checks(&c, &registry()),
        Err(ArmingFailure::MotorWithoutChannel { motor: 5 })
    );

    // Motor fixed: SPIN_MIN, ahead of SPIN_ARM and the endpoints.
    enabled[5] = false;
    let mut c = ctx(&enabled, 0.9, 0.95, true);
    c.pwm.pwm_max = 900;
    assert_eq!(
        arming_checks(&c, &registry()),
        Err(ArmingFailure::SpinMinTooHigh { spin_min: 0.9 })
    );
}

/// A motor test checks only the frame.
///
/// Deliberately less strict than arming: not every output has to be assigned,
/// because finding out which ones are may be the point of the test.
#[test]
fn a_motor_test_does_not_require_assigned_outputs() {
    let (mut enabled, _, _) = healthy();
    enabled[5] = true; // unassigned, which arming would refuse

    assert_eq!(motor_test_checks(&ctx(&enabled, 0.9, 0.95, true)), Ok(()));
    assert_eq!(
        motor_test_checks(&ctx(&enabled, 0.15, 0.1, false)),
        Err(ArmingFailure::FrameNotInitialised)
    );
}

/// A motor test needs armed *and* the interlock, and a refusal minimises.
///
/// Testing a motor is exactly where a half-satisfied safety condition is
/// dangerous. And a refused test does not merely return false — it puts the
/// outputs to their minimum, leaving the aircraft safe rather than wherever
/// the last command left it.
#[test]
fn a_motor_test_needs_both_conditions_and_minimises_on_refusal() {
    assert_eq!(
        output_test_seq(true, true, 2, 1500),
        TestSeq::Run {
            motor_seq: 2,
            pwm: 1500
        }
    );

    for (armed, interlock) in [(false, true), (true, false), (false, false)] {
        assert_eq!(
            output_test_seq(armed, interlock, 2, 1500),
            TestSeq::RefuseAndMinimise,
            "armed={armed} interlock={interlock}"
        );
    }
}
