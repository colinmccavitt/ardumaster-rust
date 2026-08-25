//! Behaviour of a channel's output state.
//!
//! The interesting content is a priority order between three things that can
//! decide a pulse: a direct pulse write, an emergency stop, and an override.
//! Each pair of them has a defined winner, and collapsing any pair changes
//! what an emergency stop can reach — so each is pinned separately rather
//! than through one composite scenario that would pass on several wrong
//! orderings.

use ap_servo::function::Function;
use ap_servo::output_channel::{OutputChannel, OutputContext};
use ap_servo::{OutputType, ServoChannel};

fn config() -> ServoChannel {
    ServoChannel {
        servo_min: 1000,
        servo_max: 2000,
        servo_trim: 1500,
        reversed: false,
        output_type: OutputType::Range,
        high_out: 1000,
    }
}

/// A motor is an E-stop function; a flap is not.
fn motor() -> OutputChannel {
    OutputChannel::new(config(), Function::MOTOR1, 0)
}

#[test]
fn the_normal_path_converts_the_scaled_value() {
    let mut ch = motor();
    ch.calc_pwm(500.0, &OutputContext::default());
    assert_eq!(ch.output_pwm(), 1500, "half range should be mid pulse");
}

/// A direct pulse write outranks the scaled path entirely.
#[test]
fn a_direct_pulse_write_is_not_overwritten() {
    let mut ch = motor();
    assert!(ch.set_output_pwm(1234, false));

    let ctx = OutputContext {
        have_pwm_mask: 1 << 0,
        emergency_stop: false,
    };
    ch.calc_pwm(500.0, &ctx);

    assert_eq!(ch.output_pwm(), 1234, "the scaled path must not touch it");
}

/// ...including during an emergency stop.
///
/// Upstream flags this as a wart rather than a design, and says why it is
/// awkward to fix: E-stopping such a channel would have to stop it to
/// `SERVOn_MIN` rather than `MOT_PWM_MIN`, which is the wrong value on a
/// multirotor. Reproduced deliberately, not inherited by accident — a port
/// that "fixed" it would diverge from every vehicle in the field.
#[test]
fn an_emergency_stop_does_not_reach_a_directly_written_pulse() {
    let mut ch = motor();
    assert!(ch.set_output_pwm(1900, false));

    let ctx = OutputContext {
        have_pwm_mask: 1 << 0,
        emergency_stop: true,
    };
    ch.calc_pwm(800.0, &ctx);

    assert_eq!(
        ch.output_pwm(),
        1900,
        "upstream leaves this alone; see the doc comment"
    );
}

/// An emergency stop beats an override.
#[test]
fn an_emergency_stop_overrides_an_override() {
    let mut ch = motor();
    ch.set_override(true);

    let ctx = OutputContext {
        have_pwm_mask: 0,
        emergency_stop: true,
    };
    ch.calc_pwm(800.0, &ctx);

    assert_eq!(ch.output_pwm(), 1000, "forced to the zero-scaled pulse");
}

/// An override beats the normal path.
#[test]
fn an_override_holds_against_the_scaled_path() {
    let mut ch = motor();
    assert!(ch.set_output_pwm(1700, false));
    ch.set_override(true);

    // No pulse-width mask this time: the override alone must hold it.
    ch.calc_pwm(200.0, &OutputContext::default());

    assert_eq!(ch.output_pwm(), 1700, "the override should hold");
}

/// An emergency stop only reaches functions it applies to.
///
/// A flap is not something an E-stop should switch off, and a port
/// that forced every function to zero would retract one mid-approach.
#[test]
fn an_emergency_stop_leaves_non_estop_functions_alone() {
    let mut flap = OutputChannel::new(config(), Function::FLAP, 0);
    flap.set_override(true);

    let ctx = OutputContext {
        have_pwm_mask: 0,
        emergency_stop: true,
    };
    flap.calc_pwm(800.0, &ctx);

    // The override holds, because the E-stop never applied and so never
    // forced past it.
    assert_eq!(flap.output_pwm(), 0, "nothing should have been written");
}

/// An unforced write is refused while an override is active, and says so.
///
/// The return value is load-bearing: upstream sets the shared pulse-width mask
/// only when the write actually happened, so a caller that assumed success
/// would mark a channel as directly written when it was not — and the scaled
/// path would then skip a channel it should be driving.
#[test]
fn a_refused_write_reports_that_it_was_refused() {
    let mut ch = motor();
    ch.set_override(true);

    assert!(!ch.set_output_pwm(1234, false), "should be refused");
    assert_eq!(ch.output_pwm(), 0, "and should not have written");

    assert!(ch.set_output_pwm(1234, true), "forced should succeed");
    assert_eq!(ch.output_pwm(), 1234);
}

/// Every motor function is an E-stop function, including the high ones.
///
/// `should_e_stop` covers `k_motor13 ... k_motor32` through a GCC case-range,
/// which is easy to read as two entries and expand to two. That would leave
/// eighteen motors an emergency stop could not reach on a large frame.
#[test]
fn every_motor_is_an_estop_function() {
    for channel in 0..32_u8 {
        let f = Function::motor(channel);
        assert!(
            f.should_e_stop(),
            "motor channel {channel} (function {}) must be E-stoppable",
            f.0
        );
    }
}

/// And things that are not throttles are not.
#[test]
fn a_control_surface_is_not_an_estop_function() {
    for f in [
        Function::NONE,
        Function::AILERON,
        Function::ELEVATOR,
        Function::RUDDER,
        Function::FLAP,
    ] {
        assert!(!f.should_e_stop(), "function {} should not E-stop", f.0);
    }
}
