//! Parity test: the servo function registry against upstream.
//!
//! Two sweeps. `#motorfn` is `get_motor_function` across 40 motor slots, and
//! `#digital` is `have_digital_outputs` against a digital mask grown in steps.
//!
//! # Why the motor mapping is worth a fixture of its own
//!
//! It is three disjoint ranges, not one. Motors 1-8 start at function 33,
//! motors 9-12 at 82, and motor 13 onward at 160 — because the later motor
//! functions were added long after the first eight and took whatever enum
//! values were free. A port that assumed `k_motor1 + channel` throughout
//! compiles, passes anything with four or six motors, and silently drives the
//! wrong outputs on an octa-quad. These numbers also end up in `SERVOn_FUNCTION`
//! parameters, so getting one wrong is a configuration a user could be flying.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_servo::function::Function;
use ap_servo::registry::Registry;

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/srv_functions.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_motors_fixture.py",
            path.display()
        )
    })
}

fn section(text: &str, name: &str) -> Vec<Vec<String>> {
    let mut rows = Vec::new();
    let mut inside = false;
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            inside = tag == name;
            continue;
        }
        if !inside || line.is_empty() {
            continue;
        }
        if line
            .split(',')
            .next()
            .is_some_and(|f| f.parse::<u64>().is_err())
        {
            continue;
        }
        rows.push(line.split(',').map(str::to_owned).collect());
    }
    assert!(!rows.is_empty(), "fixture section #{name} is empty");
    rows
}

#[test]
fn the_motor_function_mapping_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "motorfn");

    let mut distinct_bases = std::collections::BTreeSet::new();

    for r in &rows {
        assert_eq!(r.len(), 2);
        let channel: u8 = r[0].parse().expect("channel");
        let want: u8 = r[1].parse().expect("function");

        let got = Function::motor(channel);
        assert_eq!(
            got.0, want,
            "motor channel {channel}: function {} != upstream {want}",
            got.0
        );

        // Record which of the three ranges each channel fell into, so the
        // assertion below can prove the sweep actually crossed all of them.
        distinct_bases.insert(u16::from(want) - u16::from(channel));
    }

    assert_eq!(
        distinct_bases.len(),
        3,
        "the sweep must cross all three motor-function ranges, saw offsets \
         {distinct_bases:?}"
    );
    println!("{} motor-function mappings, all exact", rows.len());
}

#[test]
fn the_digital_output_test_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "digital");

    // The effective digital mask is recorded per row rather than
    // reconstructed here. set_digital_outputs accumulates and cannot be
    // cleared, and an earlier fixture section already added to it, so any
    // reconstruction would be a guess that breaks the next time a section is
    // added ahead of this one.
    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 4);
        let step: usize = r[0].parse().expect("step");
        let digital_mask: u32 = r[1].parse().expect("digital");
        let mask: u32 = r[2].parse().expect("mask");
        let want = r[3] == "1";

        let got = Registry::have_digital_outputs(mask, digital_mask);
        assert_eq!(
            got, want,
            "step {step} mask {mask:#x} against digital {digital_mask:#x}"
        );
        checked += 1;
    }

    println!("{checked} digital-output tests, all exact");
}

/// An empty mask is false, not vacuously true.
///
/// "All of no channels are digital" is the mathematically tidy answer and the
/// wrong one: it would send a vehicle with no motors assigned down the digital
/// path. Upstream tests `mask != 0` first for exactly this reason.
#[test]
fn an_empty_mask_is_not_digital() {
    for digital in [0_u32, 0xF, u32::MAX] {
        assert!(
            !Registry::have_digital_outputs(0, digital),
            "empty mask against digital {digital:#x} should be false"
        );
    }
}

/// A function nobody assigned and a function that does not exist are different
/// answers.
///
/// Unassigned is an empty channel mask; undefined is `INVALID_MASK`. A port
/// that returned zero for both would let a typo in `SERVOn_FUNCTION` look like
/// a channel that simply is not driven.
#[test]
fn an_undefined_function_is_distinguishable_from_an_unassigned_one() {
    let registry = Registry::new();

    let unassigned = registry.output_channel_mask(Function::MOTOR1);
    assert_eq!(unassigned, 0, "an unassigned function drives no channels");

    let undefined = registry.output_channel_mask(Function(250));
    assert_eq!(
        undefined,
        ap_servo::registry::INVALID_MASK,
        "a function this build does not define is not merely unassigned"
    );
}

/// Writing a scaled value clears those channels from the pulse-width mask.
///
/// That side effect is the load-bearing part: it records that the channels are
/// now driven by a scaled value, so the conversion to pulses happens rather
/// than a stale width being passed through.
#[test]
fn writing_a_scaled_value_clears_the_pulse_width_mask() {
    let mut registry = Registry::new();
    registry.assign(Function::MOTOR1, 0b1010);
    registry.set_have_pwm(0b1111);
    assert_eq!(registry.have_pwm_mask(), 0b1111);

    registry.set_output_scaled(Function::MOTOR1, 0.5);

    assert_eq!(
        registry.have_pwm_mask(),
        0b0101,
        "only this function's channels should be cleared"
    );
    assert!((registry.output_scaled(Function::MOTOR1) - 0.5).abs() < f32::EPSILON);
}

/// The three motor ranges are contiguous within themselves.
#[test]
fn each_motor_range_is_contiguous() {
    for ch in 0..7_u8 {
        assert_eq!(
            Function::motor(ch + 1).0,
            Function::motor(ch).0 + 1,
            "motors 1-8 should be contiguous, broke at {ch}"
        );
    }
    for ch in 8..11_u8 {
        assert_eq!(
            Function::motor(ch + 1).0,
            Function::motor(ch).0 + 1,
            "motors 9-12 should be contiguous, broke at {ch}"
        );
    }
    for ch in 12..31_u8 {
        assert_eq!(
            Function::motor(ch + 1).0,
            Function::motor(ch).0 + 1,
            "motors 13+ should be contiguous, broke at {ch}"
        );
    }

    // And the ranges are genuinely disjoint, which is the whole point.
    assert_ne!(Function::motor(8).0, Function::motor(7).0 + 1);
    assert_ne!(Function::motor(12).0, Function::motor(11).0 + 1);
}
