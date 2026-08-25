//! Parity test: the per-function default output shapes against upstream.
//!
//! Thirty-two functions swept across five scaled values, with the resulting
//! pulse compared. The shapes themselves are private — `high_out` and
//! `type_angle` cannot be read — so they are checked through the conversion,
//! which is what the shape is *for*.
//!
//! The swept set is chosen by `tools/parity/pick_funcs.py` from the same parse
//! that builds the table: every actuator (the case-range easiest to get
//! wrong), two representatives of each other group, and twenty functions with
//! no default at all. An earlier version picked them by hand from remembered
//! enum values and several were wrong, which is why the selection is derived
//! rather than written down.
//!
//! # What the groups look like
//!
//! With `SERVOn_MIN` 1100, `MAX` 1900 and `TRIM` 1500:
//!
//! - A range maps 0 to the minimum: `1100 + scaled/high * 800`.
//! - An angle maps 0 to the trim: `1500 + scaled/high * 400`.
//! - A function with no default has no shape at all, so every input maps to
//!   the minimum — which is upstream measuring what the table asserts by
//!   omission, and the reason motors need an explicit `set_range`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_servo::function::{DefaultOutput, Function};
use ap_servo::{OutputType, ServoChannel};

/// The `SERVOn_*` defaults the fixture ran with.
const SERVO_MIN: u16 = 1100;
const SERVO_MAX: u16 = 1900;
const SERVO_TRIM: u16 = 1500;

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

/// The channel a function's default shape produces.
///
/// A function with no default leaves the channel unconfigured, which upstream
/// represents as a zero `high_out` — and a zero-width output maps everything
/// to the minimum rather than dividing by it.
fn channel_for(function: Function) -> ServoChannel {
    let (output_type, high_out) = match function.default_output() {
        Some(DefaultOutput::Range(high)) => (OutputType::Range, high),
        Some(DefaultOutput::Angle(high)) => (
            OutputType::Angle,
            u16::try_from(high).expect("a non-negative angle"),
        ),
        None => (OutputType::Range, 0),
    };

    ServoChannel {
        servo_min: SERVO_MIN,
        servo_max: SERVO_MAX,
        servo_trim: SERVO_TRIM,
        reversed: false,
        output_type,
        high_out,
    }
}

#[test]
fn the_default_output_shapes_match_upstream() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/srv_setup_sweep.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_motors_fixture.py",
            path.display()
        )
    });

    let mut checked = 0_usize;
    let mut seen_range = false;
    let mut seen_angle = false;
    let mut seen_none = false;

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("function,") {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        assert_eq!(c.len(), 3, "malformed row: {line}");

        let function = Function(c[0].parse::<u8>().expect("function"));
        let scaled = f(c[1]);
        let want: u16 = c[2].parse().expect("pwm");

        let got = channel_for(function).pwm_from_scaled_value(scaled);
        assert_eq!(
            got,
            want,
            "function {} at scaled {scaled}: {got} != upstream {want} \
             (default {:?})",
            function.0,
            function.default_output()
        );
        checked += 1;

        match function.default_output() {
            Some(DefaultOutput::Range(_)) => seen_range = true,
            Some(DefaultOutput::Angle(_)) => seen_angle = true,
            None => seen_none = true,
        }
    }

    assert!(
        seen_range && seen_angle && seen_none,
        "the sweep must cover ranges, angles and functions with no default: \
         range={seen_range} angle={seen_angle} none={seen_none}"
    );
    println!("{checked} conversions, all exact");
}

/// Every actuator is in the swept set and comes back as the unit angle.
///
/// Stated separately from the sweep because the case-range is the specific
/// thing this fixture was built to prove, and a sweep that quietly stopped
/// including the actuators would still pass the test above.
#[test]
fn the_sweep_covers_every_actuator() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/srv_setup_sweep.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let swept: std::collections::BTreeSet<u8> = text
        .lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("function,"))
        .filter_map(|l| l.split(',').next())
        .filter_map(|f| f.parse().ok())
        .collect();

    for actuator in [
        Function::ACTUATOR1,
        Function::ACTUATOR2,
        Function::ACTUATOR3,
        Function::ACTUATOR4,
        Function::ACTUATOR5,
        Function::ACTUATOR6,
    ] {
        assert!(
            swept.contains(&actuator.0),
            "actuator function {} is not in the swept set",
            actuator.0
        );
        assert_eq!(actuator.default_output(), Some(DefaultOutput::Angle(1)));
    }
}
