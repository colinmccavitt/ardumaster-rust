//! Function classification and channel travel limits.
//!
//! The motor-number pair is the interesting part. `Function::motor` maps a
//! motor to a function and `Function::motor_num` maps it back, across three
//! disjoint ranges — and upstream writes those ranges out twice, once in each
//! direction. Two transcriptions of the same non-obvious mapping is two
//! chances to get it wrong, so the port writes it twice too but ties them
//! together with a round trip, which no amount of re-reading would.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an index fault is a test failure, which is the desired outcome"
)]

use ap_servo::function::Function;
use ap_servo::{Limit, OutputType, ServoChannel};

/// Every motor round-trips through the function mapping and back.
///
/// This is what makes writing the inverse safe. The ranges start at 33, 82 and
/// 160 with large gaps, so an off-by-one in either direction lands on a real
/// but wrong function — an output that exists and does something else, not an
/// error.
#[test]
fn every_motor_round_trips_through_its_function() {
    for channel in 0..32_u8 {
        let f = Function::motor(channel);
        assert_eq!(
            f.motor_num(),
            Some(channel),
            "motor {channel} maps to function {} which maps back to {:?}",
            f.0,
            f.motor_num()
        );
        assert!(f.is_motor(), "function {} should be a motor", f.0);
    }
}

/// Nothing outside the three ranges claims to be a motor.
#[test]
fn non_motor_functions_have_no_motor_number() {
    for f in [
        Function::NONE,
        Function::AILERON,
        Function::ELEVATOR,
        Function::RUDDER,
        Function::FLAP,
        Function::ACTUATOR1,
    ] {
        assert_eq!(f.motor_num(), None, "function {} is not a motor", f.0);
        assert!(!f.is_motor(), "function {} should not be a motor", f.0);
    }
}

/// `is_motor` and `motor_num` agree with each other everywhere.
///
/// Two predicates over the same three ranges; a discrepancy between them would
/// mean one range was edited and the other was not.
#[test]
fn the_two_motor_predicates_agree() {
    for value in 0..=u8::MAX {
        let f = Function(value);
        assert_eq!(
            f.is_motor(),
            f.motor_num().is_some(),
            "function {value}: is_motor and motor_num disagree"
        );
    }
}

/// Control surfaces and motors are disjoint sets.
#[test]
fn nothing_is_both_a_motor_and_a_control_surface() {
    for value in 0..=u8::MAX {
        let f = Function(value);
        assert!(
            !(f.is_motor() && f.is_control_surface()),
            "function {value} claims to be both"
        );
    }
}

/// Surfaces are classified as such.
#[test]
fn the_usual_surfaces_are_control_surfaces() {
    for f in [
        Function::AILERON,
        Function::ELEVATOR,
        Function::RUDDER,
        Function::FLAP,
    ] {
        assert!(
            f.is_control_surface(),
            "function {} should be a surface",
            f.0
        );
    }
}

fn channel(reversed: bool) -> ServoChannel {
    ServoChannel {
        servo_min: 1100,
        servo_max: 1900,
        servo_trim: 1450,
        reversed,
        output_type: OutputType::Range,
        high_out: 1000,
    }
}

/// Reversing swaps the commanded ends but not the trim.
///
/// `Min` and `Max` name the ends of the *commanded* range, not of the pulse
/// range, so a reversed channel returns the larger pulse for `Min`. Trim is a
/// single configured position with nothing to exchange it with — reversing it
/// too would move the neutral point of every reversed channel, which is the
/// kind of error that shows up as a trimmed-out airframe rather than as a
/// failure.
#[test]
fn reversing_swaps_the_ends_but_leaves_trim_alone() {
    let forward = channel(false);
    let reversed = channel(true);

    assert_eq!(forward.limit_pwm(Limit::Min), 1100);
    assert_eq!(forward.limit_pwm(Limit::Max), 1900);

    assert_eq!(
        reversed.limit_pwm(Limit::Min),
        1900,
        "reversed min is the high pulse"
    );
    assert_eq!(
        reversed.limit_pwm(Limit::Max),
        1100,
        "reversed max is the low pulse"
    );

    assert_eq!(forward.limit_pwm(Limit::Trim), 1450);
    assert_eq!(
        reversed.limit_pwm(Limit::Trim),
        1450,
        "trim must not be reversed"
    );
}

/// Zero pwm is zero regardless of anything else.
#[test]
fn zero_pwm_is_zero() {
    assert_eq!(channel(false).limit_pwm(Limit::ZeroPwm), 0);
    assert_eq!(channel(true).limit_pwm(Limit::ZeroPwm), 0);
}

/// Parity: every function's classification against upstream.
///
/// All 190 values swept through `is_motor`, `should_e_stop` and
/// `is_control_surface`. These are the three tables that decide whether an
/// emergency stop reaches an output, whether a function counts as a motor, and
/// whether it moves a surface — each generated from a switch or a range test,
/// each with a case-range that reads naturally as two entries.
///
/// `motor_num` is not swept: it is an instance method and would need a channel
/// per function, which the 32-channel limit rules out. It is covered
/// transitively — the round-trip test above pins it as the exact inverse of
/// `motor`, and `is_motor`, verified here for every value, pins the three
/// ranges both are built from.
#[test]
fn every_function_classification_matches_upstream() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/srv_predicates.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_srv_setup_fixture.py",
            path.display()
        )
    });

    let mut checked = 0_usize;
    let mut motors = 0_usize;
    let mut estops = 0_usize;
    let mut surfaces = 0_usize;

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("function,") {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        assert_eq!(c.len(), 4, "malformed row: {line}");

        let f = ap_servo::function::Function(c[0].parse::<u8>().expect("function"));
        let want_motor = c[1] == "1";
        let want_estop = c[2] == "1";
        let want_surface = c[3] == "1";

        assert_eq!(f.is_motor(), want_motor, "function {}: is_motor", f.0);
        assert_eq!(
            f.should_e_stop(),
            want_estop,
            "function {}: should_e_stop",
            f.0
        );
        assert_eq!(
            f.is_control_surface(),
            want_surface,
            "function {}: is_control_surface",
            f.0
        );

        checked += 3;
        motors += usize::from(want_motor);
        estops += usize::from(want_estop);
        surfaces += usize::from(want_surface);
    }

    // A sweep where every answer is false would pass against a port that
    // returned false for everything, so assert the tables are populated.
    assert!(motors >= 32, "expected at least 32 motors, found {motors}");
    assert!(
        estops > motors,
        "e-stop should cover more than the motors alone"
    );
    assert!(surfaces > 0, "no control surfaces found");

    println!(
        "{checked} classifications over {} functions, all exact          ({motors} motors, {estops} e-stop, {surfaces} surfaces)",
        checked / 3
    );
}
