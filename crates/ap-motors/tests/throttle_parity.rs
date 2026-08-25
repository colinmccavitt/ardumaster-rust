//! Parity test: the throttle input path against upstream.
//!
//! Two sweeps. `#filter` runs `update_throttle_filter` across three cutoff
//! pairs, four stick profiles and both arming states, 400 steps each, checking
//! both the filtered throttle and the slew estimate every step. `#hover` runs
//! `update_throttle_hover` across the learn modes, four starting values, four
//! targets and two rates.
//!
//! The slew estimate is a seven-sample derivative, so it only agrees if the
//! whole history agrees — a single step fed at the wrong time, or fed when it
//! should have been skipped, shows up several steps later and never recovers.
//!
//! # Reading the inputs rather than recomputing them
//!
//! One stick profile is a sine. Rust's `f32::sin` and C's `sinf` are both
//! correctly-rounded-ish but not required to agree in the last bit, so
//! recomputing the profile here would inject a transcendental disagreement
//! into a comparison that is otherwise pure arithmetic. The fixture dumps the
//! input it actually used, and this reads it back — the test compares the
//! filter, not the two standard libraries' sine.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_motors::throttle::{HoverLearn, HoverThrottle, ThrottleInput};

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/motors_throttle.csv"))
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
            .is_some_and(|f| f.parse::<f64>().is_err())
        {
            continue;
        }
        rows.push(line.split(',').map(str::to_owned).collect());
    }
    assert!(!rows.is_empty(), "fixture section #{name} is empty");
    rows
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

#[test]
fn the_throttle_filter_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "filter");

    let mut i = 0_usize;
    let mut cases = 0_usize;
    let mut compared = 0_usize;

    // ONE input path across every case, because the harness drives a singleton
    // and never resets the slew detector. It cannot: DerivativeFilter::reset()
    // leaves its own timestamps behind, so slope() afterwards computes across
    // stale timestamps and cleared samples. That is a registered divergence the
    // port fixes, which means resetting either side here would be comparing
    // against behaviour the port deliberately refuses to have.
    //
    // Keeping one continuous history on both sides sidesteps it entirely, and
    // is a stronger test besides: 9,600 consecutive steps with no chance to
    // resynchronise.
    let mut input = ThrottleInput::new();
    let mut now_us = 0_u32;
    let mut feeds = 0_i32;

    while i < rows.len() {
        let case: usize = rows[i][0].parse().expect("case");
        let cutoff = f(&rows[i][2]);
        let slew_cutoff = f(&rows[i][3]);
        let armed = rows[i][4] == "1";

        input.set_filter_cutoff(cutoff);
        input.set_slew_filter_cutoff(slew_cutoff);

        // The harness runs one disarmed step before each case, which clears the
        // throttle filter the way a real disarm does.
        now_us += 2500;
        let pre = input.throttle_raw();
        input.update(0.0, false, 0.0025, now_us);
        if !ap_math::scalar::is_equal(pre, input.throttle_raw()) {
            feeds += 1;
        }

        let mut step = 0_usize;
        while i < rows.len() && rows[i][0].parse::<usize>().expect("case") == case {
            let r = &rows[i];
            assert_eq!(r.len(), 11);
            assert_eq!(r[1].parse::<usize>().expect("step"), step);

            let throttle_in = f(&r[5]);
            now_us += 2500;
            assert_eq!(
                now_us,
                r[6].parse::<u32>().expect("now_us"),
                "case {case} step {step}: the clocks have drifted apart"
            );

            let before = input.throttle_raw();
            input.update(throttle_in, armed, 0.0025, now_us);

            // Whether the slew detector was fed, compared on the step it
            // happens. The detector is seven samples wide, so a disagreement
            // about the count would otherwise surface seven steps later as an
            // unexplained slope mismatch.
            let fed = !ap_math::scalar::is_equal(before, input.throttle_raw());
            assert_eq!(
                i32::from(fed),
                r[9].parse::<i32>().expect("fed"),
                "case {case} step {step}: fed the slew detector?"
            );
            feeds += i32::from(fed);
            assert_eq!(
                feeds,
                r[10].parse::<i32>().expect("feeds"),
                "case {case} step {step}: samples fed to the slew detector so far"
            );
            compared += 2;

            let want_filtered = f(&r[7]);
            let want_slew = f(&r[8]);

            // `throttle()` clamps; the fixture dumps the filter's raw value,
            // so compare through a path that does not clamp twice.
            let got_filtered = input.throttle_raw();
            assert!(
                same(got_filtered, want_filtered),
                "case {case} step {step} (cutoff {cutoff}, armed {armed}) \
                 filtered: {got_filtered} ({:#010x}) != upstream {want_filtered} \
                 ({:#010x})",
                got_filtered.to_bits(),
                want_filtered.to_bits()
            );
            assert!(
                same(input.slew_rate(), want_slew),
                "case {case} step {step} (cutoff {cutoff}, armed {armed}) \
                 slew rate: {} ({:#010x}) != upstream {want_slew} ({:#010x})",
                input.slew_rate(),
                input.slew_rate().to_bits(),
                want_slew.to_bits()
            );
            compared += 2;

            i += 1;
            step += 1;
        }
        cases += 1;
    }

    println!("{cases} filter cases, {compared} values, all bit-exact");
}

#[test]
fn the_hover_learning_matches_upstream() {
    let text = fixture();
    let rows = section(&text, "hover");

    let mut i = 0_usize;
    let mut compared = 0_usize;

    while i < rows.len() {
        let case: usize = rows[i][0].parse().expect("case");
        let learn = match rows[i][2].parse::<i32>().expect("learn") {
            0 => HoverLearn::Disabled,
            1 => HoverLearn::LearnOnly,
            2 => HoverLearn::LearnAndSave,
            n => panic!("unknown learn mode {n}"),
        };
        let dt = f(&rows[i][3]);
        let throttle = f(&rows[i][4]);

        // The starting value is not in the row; it is whatever the first
        // update produced from it. Recover it by stepping the same lag
        // backwards is not possible, so the harness dumps the post-update
        // value and the start is reconstructed from the case index the same
        // way the harness built it.
        let starts = [0.35_f32, 0.125, 0.6875, 0.5];
        let start = starts[(case / 8) % 4];

        let mut hover = HoverThrottle::new(start);

        let mut step = 0_usize;
        while i < rows.len() && rows[i][0].parse::<usize>().expect("case") == case {
            let r = &rows[i];
            assert_eq!(r.len(), 7);
            assert_eq!(r[1].parse::<usize>().expect("step"), step);

            hover.update(throttle, dt, learn);

            for (label, got, want) in [
                ("raw", hover.raw(), f(&r[5])),
                ("get", hover.get(), f(&r[6])),
            ] {
                assert!(
                    same(got, want),
                    "case {case} step {step} (learn {learn:?}, dt {dt}, \
                     throttle {throttle}, start {start}) {label}: {got} \
                     ({:#010x}) != upstream {want} ({:#010x})",
                    got.to_bits(),
                    want.to_bits()
                );
                compared += 1;
            }

            i += 1;
            step += 1;
        }
    }

    println!("{compared} hover values, all bit-exact");
}

/// Disarmed, the filter is reset rather than run.
///
/// This is why a throttle slew after arming starts from zero instead of
/// resuming from wherever the stick was left — and it is load-bearing, since
/// `SPOOLING_UP` compares its ceiling against the filtered throttle.
#[test]
fn disarming_resets_the_throttle_filter() {
    let mut input = ThrottleInput::new();
    input.set_filter_cutoff(2.0);
    input.set_slew_filter_cutoff(25.0);

    for step in 0..400_u32 {
        input.update(0.8, true, 0.0025, (step + 1) * 2500);
    }
    assert!(input.throttle() > 0.5, "filter should have risen");

    input.update(0.8, false, 0.0025, 1_002_500);
    assert!(
        same(input.throttle(), 0.0),
        "disarming should zero the filter, got {}",
        input.throttle()
    );
}

/// Learning is skipped entirely when the mode is disabled.
#[test]
fn a_disabled_hover_learn_never_moves_the_estimate() {
    let mut hover = HoverThrottle::new(0.35);
    for _ in 0..10_000 {
        hover.update(0.6, 0.0025, HoverLearn::Disabled);
    }
    assert!(same(hover.raw(), 0.35), "estimate moved to {}", hover.raw());
}

/// The estimate converges toward the throttle, bounded by the reachable range.
#[test]
fn the_hover_estimate_converges_within_its_bounds() {
    // A target inside the range is reached.
    let mut hover = HoverThrottle::new(0.35);
    for _ in 0..40_000 {
        hover.update(0.45, 0.0025, HoverLearn::LearnOnly);
    }
    assert!((hover.get() - 0.45).abs() < 1e-3, "got {}", hover.get());

    // A target outside it is clamped, not chased.
    let mut hover = HoverThrottle::new(0.35);
    for _ in 0..40_000 {
        hover.update(0.95, 0.0025, HoverLearn::LearnOnly);
    }
    assert!(same(hover.get(), 0.6875), "got {}", hover.get());
}
