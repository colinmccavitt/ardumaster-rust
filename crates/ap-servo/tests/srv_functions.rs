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

/// `invalid_mask` is a real set of channels, not a sentinel.
///
/// This test previously asserted that an undefined function returns an
/// all-ones sentinel. That was wrong, and it passed anyway because the port and
/// the test were wrong together — the exact failure a fixture against upstream
/// exists to catch, and the reason this one now checks against
/// `update_aux_servo_function`'s actual output.
///
/// What upstream does: `invalid_mask` accumulates the channels whose *own*
/// `SERVOn_FUNCTION` is not a function this build defines, and
/// `get_output_channel_mask` answers with it when asked about an undefined
/// function. So asking a meaningless question returns the channels that are
/// themselves meaningless.
#[test]
fn an_undefined_function_answers_with_the_invalid_channels() {
    let mut registry = Registry::new();

    // Channels 0 and 2 are assigned; 1 and 3 hold a function this build does
    // not define.
    let assignments = [
        Function::MOTOR1,
        Function(250),
        Function::MOTOR2,
        Function(251),
    ];
    registry.update_aux_servo_function(&assignments);

    assert_eq!(
        registry.invalid_mask(),
        0b1010,
        "channels 1 and 3 are invalid"
    );
    assert_eq!(registry.output_channel_mask(Function::MOTOR1), 0b0001);
    assert_eq!(registry.output_channel_mask(Function::MOTOR2), 0b0100);

    // An assigned-to-nobody function is empty; an undefined one is the invalid
    // set. Those are different answers, which is the whole point.
    assert_eq!(registry.output_channel_mask(Function::MOTOR3), 0);
    assert_eq!(registry.output_channel_mask(Function(250)), 0b1010);
}

/// One function can drive several channels.
///
/// Two servos on one surface is ordinary, and writing the function has to
/// write both. A port storing a single channel per function would work on
/// every simple airframe and fail silently on the ones that need this.
#[test]
fn one_function_can_drive_several_channels() {
    let mut registry = Registry::new();
    let assignments = [Function::MOTOR1, Function::MOTOR1, Function::MOTOR2];
    registry.update_aux_servo_function(&assignments);

    assert_eq!(registry.output_channel_mask(Function::MOTOR1), 0b011);
    assert_eq!(registry.output_channel_mask(Function::MOTOR2), 0b100);
}

/// Rebuilding clears what came before.
///
/// A channel moved from one function to another must leave no trace on the
/// old one, or the old function keeps driving an output nobody assigned to it.
#[test]
fn rebuilding_the_masks_is_not_a_merge() {
    let mut registry = Registry::new();

    registry.update_aux_servo_function(&[Function::MOTOR1, Function::MOTOR2]);
    assert_eq!(registry.output_channel_mask(Function::MOTOR1), 0b01);

    // Channel 0 moves to MOTOR3.
    registry.update_aux_servo_function(&[Function::MOTOR3, Function::MOTOR2]);
    assert_eq!(
        registry.output_channel_mask(Function::MOTOR1),
        0,
        "the old function must not keep the channel"
    );
    assert_eq!(registry.output_channel_mask(Function::MOTOR3), 0b01);
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
