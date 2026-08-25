//! Parity test: `DerivativeFilter` against upstream's own implementation.
//!
//! All four instantiated sizes, through the sequences that reach the
//! interesting paths: an unfilled buffer, a straight line, irregular spacing,
//! a zig-zag the smoothing is supposed to reject, a repeated timestamp that
//! must be dropped, and a microsecond counter wrapping past its maximum
//! mid-buffer.
//!
//! Samples and timestamps are read from the fixture, so the port replays
//! exactly the sequence upstream saw.
//!
//! Values are raw bit patterns, so every comparison is exact.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact comparison against upstream is the point of the test"
)]

use ap_filter::derivative::DerivativeFilter;

const SCENARIOS: &[&str] = &["line", "irregular", "zigzag", "repeat", "wrap"];
const STEPS: usize = 40;

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/derivative_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_derivative_fixture.py",
            path.display()
        )
    })
}

fn rows(text: &str) -> Vec<Vec<String>> {
    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("scenario,"))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

/// Replay one scenario at one size and compare every slope.
fn check<const N: usize>(rows: &[Vec<String>], scenario: &str) -> usize {
    let mut filter = DerivativeFilter::<N>::new();
    let mut checked = 0;
    let want_size = N.to_string();

    let mine: Vec<&Vec<String>> = rows
        .iter()
        .filter(|r| r[0] == scenario && r[1] == want_size)
        .collect();
    assert_eq!(mine.len(), STEPS, "{scenario}/{N}: wrong row count");

    for (step, row) in mine.iter().enumerate() {
        assert_eq!(row.len(), 6);
        assert_eq!(row[2].parse::<usize>().expect("step"), step);

        let sample = f(&row[3]);
        let timestamp: u32 = row[4].parse().expect("timestamp");
        filter.update(sample, timestamp);

        let got = filter.slope();
        let want = f(&row[5]);
        assert!(
            same(got, want),
            "{scenario}/{N} step {step}: {got:e} ({:#010x}) != upstream {want:e} ({:#010x})",
            got.to_bits(),
            want.to_bits()
        );
        checked += 1;
    }
    checked
}

#[test]
fn the_differentiator_matches_upstream_at_every_size() {
    let text = fixture();
    let rows = rows(&text);
    assert_eq!(rows.len(), SCENARIOS.len() * STEPS * 4);

    let mut total = 0;
    for scenario in SCENARIOS {
        total += check::<5>(&rows, scenario);
        total += check::<7>(&rows, scenario);
        total += check::<9>(&rows, scenario);
        total += check::<11>(&rows, scenario);
    }
    println!(
        "{total} slopes compared bit-exactly across {} scenarios and 4 sizes",
        SCENARIOS.len()
    );
    assert_eq!(total, SCENARIOS.len() * STEPS * 4);
}

/// The unfilled-buffer guard is worth calling out separately: it is the thing
/// D-005's uninitialised `_timestamps` would defeat, and the fixture's early
/// steps are where it shows.
#[test]
fn the_unfilled_buffer_reports_no_slope_in_both() {
    let text = fixture();
    let rows = rows(&text);

    // Seven-sample form, straight line: nothing until the buffer fills.
    let mine: Vec<&Vec<String>> = rows
        .iter()
        .filter(|r| r[0] == "line" && r[1] == "7")
        .collect();

    let mut filter = DerivativeFilter::<7>::new();
    let mut upstream_zero_steps = 0;
    for row in mine.iter().take(6) {
        filter.update(f(&row[3]), row[4].parse().expect("timestamp"));
        assert_eq!(filter.slope(), 0.0);
        if f(&row[5]) == 0.0 {
            upstream_zero_steps += 1;
        }
    }
    assert_eq!(
        upstream_zero_steps, 6,
        "upstream should also report nothing while the buffer fills — if it does \
         not, its _timestamps were not zero and the harness lost its static storage"
    );
}
