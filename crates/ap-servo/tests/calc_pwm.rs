//! Parity test: `calc_pwm`'s priority order against upstream.
//!
//! Four channels carrying the four combinations of the two channel-level
//! states — held by a direct pulse write, held by an override, both, neither —
//! swept across four scaled values and both emergency-stop states.
//!
//! A separate channel per combination rather than one channel reset between
//! cases: neither an override nor the pulse-width mask can be cleared through
//! the public API, so a reset would have to be faked, and then the fixture
//! measures the fake.
//!
//! # What the rows show
//!
//! The unheld channel tracks the scaled value (0 → 1000, 1000 → 2000) and goes
//! to 1000 under an emergency stop. The directly-written channel holds 1234
//! throughout — *including* under emergency stop, which is upstream's
//! documented wart and the reason this file exists rather than a unit test
//! asserting what the priority order ought to be.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_servo::function::Function;
use ap_servo::output_channel::OutputChannel;
use ap_servo::registry::Registry;
use ap_servo::{OutputType, ServoChannel};

/// The functions the fixture assigns to channels 0..3.
const FN: [u8; 4] = [33, 34, 35, 36];

/// Matches the `set_range(function, 1000)` the fixture performs.
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

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

#[test]
fn the_calc_pwm_priority_matches_upstream() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/srv_calc_pwm.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_motors_fixture.py",
            path.display()
        )
    });

    // case -> rows
    let mut cases: std::collections::BTreeMap<usize, Vec<Vec<String>>> =
        std::collections::BTreeMap::new();
    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("case,") {
            continue;
        }
        let c: Vec<String> = line.split(',').map(str::to_owned).collect();
        assert_eq!(c.len(), 7, "malformed row: {line}");
        let case: usize = c[0].parse().expect("case");
        cases.entry(case).or_default().push(c);
    }
    assert!(!cases.is_empty(), "fixture has no cases");

    // Build the same four channels the fixture did, and hold 2 and 3 with an
    // override for the whole sweep.
    let mut channels: Vec<OutputChannel> = (0..4_u8)
        .map(|i| OutputChannel::new(config(), Function(FN[usize::from(i)]), i))
        .collect();
    channels[2].set_override(true);
    channels[3].set_override(true);

    let mut registry = Registry::new();
    registry.update_aux_servo_function(&[
        Function(FN[0]),
        Function(FN[1]),
        Function(FN[2]),
        Function(FN[3]),
    ]);

    // The override write itself, matching set_output_pwm_chan_timeout, which
    // forces past the override it is in the act of setting.
    channels[2].set_output_pwm(1777, true);
    channels[3].set_output_pwm(1777, true);

    let mut checked = 0_usize;
    let mut saw_estop_bypassed = false;

    for (case, rows) in &cases {
        assert_eq!(rows.len(), 4, "case {case}: expected four channels");
        let estop = rows[0][2] == "1";
        let scaled = f(&rows[0][5]);

        for &function in &FN {
            registry.set_output_scaled(Function(function), scaled);
        }
        // After the scaled writes, which clear the pulse-width mask.
        registry.set_output_pwm(&mut channels, Function(FN[1]), 1234);
        registry.set_output_pwm(&mut channels, Function(FN[3]), 1234);

        registry.calc_pwm(&mut channels, estop);

        for r in rows {
            let ch: usize = r[1].parse().expect("channel");
            let want: u16 = r[6].parse().expect("pwm");
            assert_eq!(
                channels[ch].output_pwm(),
                want,
                "case {case} channel {ch} (estop {estop}, scaled {scaled}): \
                 {} != upstream {want}",
                channels[ch].output_pwm()
            );
            checked += 1;

            // A directly written channel keeping its pulse through an
            // emergency stop is the wart this test exists to hold in place.
            if estop && ch == 1 && want == 1234 {
                saw_estop_bypassed = true;
            }
        }
    }

    assert!(
        saw_estop_bypassed,
        "the sweep must include an emergency stop failing to reach a directly \
         written channel, or the wart it pins is not covered"
    );
    println!("{} cases, {checked} pulses, all exact", cases.len());
}
