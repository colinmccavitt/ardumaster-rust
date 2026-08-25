//! Motor numbers, output functions and output channels.
//!
//! Three different spaces that coincide on an ordinary vehicle and come apart
//! on a configured one. Motor 0 drives function `k_motor1`, which drives
//! whichever channels the operator assigned it to — usually channel 0, which
//! is exactly why treating the three as interchangeable survives testing on a
//! default airframe and addresses the wrong outputs on a real one.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an index fault is a test failure, which is the desired outcome"
)]

use ap_motors::output::motor_mask_to_channel_mask;
use ap_motors::MotorMatrix;
use ap_servo::function::Function;
use ap_servo::registry::Registry;

/// A registry where motor N sits on channel N, as a default airframe has it.
fn straight_through(motors: u8) -> Registry {
    let mut registry = Registry::new();
    let assignments: Vec<Function> = (0..32_u8)
        .map(|c| {
            if c < motors {
                Function::motor(c)
            } else {
                Function::NONE
            }
        })
        .collect();
    registry.update_aux_servo_function(&assignments);
    registry
}

#[test]
fn a_default_airframe_maps_motors_straight_through() {
    let registry = straight_through(4);
    assert_eq!(motor_mask_to_channel_mask(0b1111, &registry), 0b1111);
}

/// A motor moved to another output moves its bit with it.
///
/// This is the case that makes the indirection necessary. Motor 0 on channel 7
/// must produce bit 7, not bit 0 — and a port that returned the motor mask
/// unchanged would agree with upstream on every default airframe and be wrong
/// here.
#[test]
fn a_relocated_motor_moves_its_channel_bit() {
    let mut registry = Registry::new();
    let mut assignments = [Function::NONE; 32];
    assignments[7] = Function::motor(0);
    assignments[1] = Function::motor(1);
    registry.update_aux_servo_function(&assignments);

    assert_eq!(
        motor_mask_to_channel_mask(0b01, &registry),
        1 << 7,
        "motor 0 is on channel 7"
    );
    assert_eq!(
        motor_mask_to_channel_mask(0b11, &registry),
        (1 << 7) | (1 << 1)
    );
}

/// One motor function driving two channels produces two bits.
///
/// Not a curiosity: it is how a frame with paired outputs is wired, and a port
/// that assumed one channel per motor would leave the second one unconfigured
/// — running at whatever rate and mode the board defaulted to.
#[test]
fn a_motor_on_two_channels_produces_two_bits() {
    let mut registry = Registry::new();
    let mut assignments = [Function::NONE; 32];
    assignments[3] = Function::motor(0);
    assignments[9] = Function::motor(0);
    registry.update_aux_servo_function(&assignments);

    assert_eq!(
        motor_mask_to_channel_mask(0b1, &registry),
        (1 << 3) | (1 << 9)
    );
}

/// An unassigned motor contributes nothing.
#[test]
fn an_unassigned_motor_contributes_no_channels() {
    let registry = straight_through(2);
    // Motors 2 and 3 are in the mask but on no channel.
    assert_eq!(motor_mask_to_channel_mask(0b1111, &registry), 0b0011);
}

/// The frame's mask is its fitted motors, and it survives the round trip.
#[test]
fn the_frame_mask_covers_exactly_the_fitted_motors() {
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(1, 1), "quad X should be supported");

    let mask = m.motor_mask();
    assert_eq!(
        mask.count_ones(),
        4,
        "a quad has four motors, got {mask:#b}"
    );

    let registry = straight_through(8);
    assert_eq!(
        motor_mask_to_channel_mask(mask, &registry),
        mask,
        "straight through, the channel mask should match the motor mask"
    );
}

/// The boost throttle's channel joins the frame's, even though it is not a
/// mixed motor.
///
/// Callers use this to decide which outputs the motor library owns — update
/// rates, output modes — and a boost motor needs the same treatment as the
/// rest despite being absent from the mixing table.
#[test]
fn the_output_mask_includes_the_boost_throttle() {
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(1, 1));

    let mut registry = Registry::new();
    let mut assignments = [Function::NONE; 32];
    for (c, slot) in assignments.iter_mut().enumerate().take(4) {
        *slot = Function::motor(u8::try_from(c).expect("under four"));
    }
    assignments[11] = Function::BOOST_THROTTLE;
    registry.update_aux_servo_function(&assignments);

    let mask = m.output_channel_mask(&registry);
    assert_eq!(mask, 0b1111 | (1 << 11), "got {mask:#b}");
}

/// Claiming a free channel works; claiming a taken one does not.
#[test]
fn a_default_assignment_does_not_overwrite_a_configured_channel() {
    let mut registry = Registry::new();
    let mut assignments = [Function::NONE; 32];
    assignments[0] = Function::AILERON;
    registry.update_aux_servo_function(&assignments);

    // Channel 0 is taken by something else: refused.
    assert!(
        !registry.set_aux_channel_default(&mut assignments, Function::motor(0), 0),
        "must not overwrite a configured channel"
    );
    assert_eq!(assignments[0], Function::AILERON, "and must leave it alone");

    // Channel 1 is free: claimed.
    assert!(registry.set_aux_channel_default(&mut assignments, Function::motor(0), 1));
    assert_eq!(assignments[1], Function::motor(0));

    // Now that the function is assigned somewhere, a second call is a no-op
    // that still reports success rather than claiming another channel.
    assert!(registry.set_aux_channel_default(&mut assignments, Function::motor(0), 2));
    assert_eq!(
        assignments[2],
        Function::NONE,
        "the function was already assigned; channel 2 should be untouched"
    );
}

/// Parity: the motor-to-channel mapping against upstream.
///
/// The harness records the channel assignment as it finds it and then asks the
/// real `motor_mask_to_srv_channel_mask` what it makes of several motor masks.
/// The assignment is not a tidy four-in-a-row: it has motors on channels 0-3
/// *and* on channels 8-11, which reach through the second motor function range
/// (82-85). That is the part a port collapsing motor numbers and channels gets
/// wrong, and it only shows up because the fixture takes the state it finds
/// rather than building a convenient one.
#[test]
fn the_motor_channel_mapping_matches_upstream() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/motor_mask_map.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_motors_fixture.py",
            path.display()
        )
    });

    let mut assignments = [Function::NONE; 32];
    let mut mask_cases: Vec<(u32, u32)> = Vec::new();
    let mut frame_mask: Option<u32> = None;

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("case,") {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        assert_eq!(c.len(), 4, "malformed row: {line}");

        match c[1] {
            "ch" => {
                let channel: usize = c[2].parse().expect("channel");
                let function: i32 = c[3].parse().expect("function");
                assignments[channel] = if function < 0 {
                    Function(u8::MAX)
                } else {
                    Function(u8::try_from(function).expect("function fits"))
                };
            }
            "mm" => mask_cases.push((
                c[2].parse().expect("motor mask"),
                c[3].parse().expect("channel mask"),
            )),
            "gm" => frame_mask = Some(c[3].parse().expect("frame mask")),
            other => panic!("unknown row kind {other}"),
        }
    }

    assert!(!mask_cases.is_empty(), "no mask cases in the fixture");

    let mut registry = Registry::new();
    registry.update_aux_servo_function(&assignments);

    for (motor_mask, want) in &mask_cases {
        let got = motor_mask_to_channel_mask(*motor_mask, &registry);
        assert_eq!(
            got, *want,
            "motor mask {motor_mask:#x}: channels {got:#x} != upstream {want:#x}"
        );
    }

    // The fixture must reach past the first motor range, or it is not testing
    // the thing it exists for.
    let reaches_second_range = assignments
        .iter()
        .any(|f| f.motor_num().is_some_and(|n| n >= 8));
    assert!(
        reaches_second_range,
        "the recorded assignment must include a motor from the second function          range, or a port that conflates motor numbers with channels would pass"
    );

    // And the frame's own mask, for the quad the harness leaves configured.
    let want_frame = frame_mask.expect("no gm row");
    let mut m = MotorMatrix::new();
    assert!(m.setup_motors(1, 1), "quad X");
    assert_eq!(m.output_channel_mask(&registry), want_frame, "frame mask");

    println!("{} mask mappings, all exact", mask_cases.len());
}
