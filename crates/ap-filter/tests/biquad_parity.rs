//! Parity test: `LowPassFilter2p` against upstream's own implementation.
//!
//! Harness-driven rather than replayed from a flight log. The filter is
//! stateful and every awkward path in it is configuration-dependent — the
//! Nyquist clamp, a zero cutoff falling through to pass-through, a zero sample
//! rate doing the same, a cutoff under `FLT_EPSILON` that `is_positive`
//! rejects, first-sample seeding, reset, reset-to-value, and retuning
//! mid-stream. A flight log exercises exactly one configuration.
//!
//! The input sequence is **read from the fixture** rather than reconstructed.
//! It contains a `sinf` call, and D-017 records that upstream links glibc while
//! the port links libm, so a reconstruction could differ by an ulp and fail for
//! a reason that has nothing to do with the filter. Taking the recorded input
//! makes the two runs identical by construction.
//!
//! Both template instantiations are checked. `LowPassFilter2pFloat` and
//! `LowPassFilter2pVector3f` are separately instantiated in upstream's `.cpp`,
//! and a port that got the scalar case right could still have the vector one
//! wrong.
//!
//! The coefficients are compared too, from a second fixture. Comparing only
//! outputs would let a wrong coefficient hide behind a compensating state
//! error.
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

use ap_filter::biquad::{BiquadParams, LowPassFilter2p};
use ap_math::vector3::Vector3f;

/// What happens at step 40. Duplicated from
/// `tools/parity/gen_biquad_fixture.py` — this is the configuration under
/// test, not data, so it is stated here rather than read.
#[derive(Clone, Copy, PartialEq)]
enum Action {
    None,
    Reset,
    ResetTo(f32),
    Retune(f32),
}

struct Scenario {
    name: &'static str,
    sample_freq: f32,
    cutoff_freq: f32,
    act: Action,
}

const SCENARIOS: &[Scenario] = &[
    s("ins_gyro", 8000.0, 188.0, Action::None),
    s("ins_accel", 1000.0, 20.0, Action::None),
    s("loop_rate", 400.0, 10.0, Action::None),
    s("slow", 50.0, 5.0, Action::None),
    s("clamped", 1000.0, 900.0, Action::None),
    s("at_boundary", 1000.0, 400.0, Action::None),
    s("zero_cutoff", 1000.0, 0.0, Action::None),
    s("zero_rate", 0.0, 20.0, Action::None),
    s("neg_cutoff", 1000.0, -20.0, Action::None),
    s("sub_epsilon", 1000.0, 1.0e-8, Action::None),
    s("reset", 1000.0, 20.0, Action::Reset),
    s("reset_to", 1000.0, 20.0, Action::ResetTo(3.5)),
    s("retune", 1000.0, 20.0, Action::Retune(60.0)),
    s("retune_to_zero", 1000.0, 20.0, Action::Retune(0.0)),
];

const fn s(name: &'static str, sample_freq: f32, cutoff_freq: f32, act: Action) -> Scenario {
    Scenario {
        name,
        sample_freq,
        cutoff_freq,
        act,
    }
}

fn fixture(name: &str) -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures").join(name))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_biquad_fixture.py",
            path.display()
        )
    })
}

/// Data rows, header and comments dropped.
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

/// Distance in representable floats. Only meaningful for same-signed finite
/// values, which is asserted by the caller.
fn ulps(a: f32, b: f32) -> i64 {
    let key = |x: f32| -> i64 {
        let b = i64::from(x.to_bits() as i32);
        if b < 0 {
            i64::from(i32::MIN) - b
        } else {
            b
        }
    };
    (key(a) - key(b)).abs()
}

/// Which scenarios upstream and the port agree about exactly.
///
/// A scenario whose coefficients come out bit-identical has no transcendental
/// disagreement in it, so its samples must be bit-identical too. Computing the
/// split here rather than hardcoding it means the test keeps its own bar
/// honest if libm or glibc ever changes.
fn bit_exact_scenarios() -> Vec<bool> {
    let text = fixture("biquad_params.csv");
    let rows = rows(&text);
    SCENARIOS
        .iter()
        .enumerate()
        .map(|(i, sc)| {
            let row = &rows[i];
            let mut p = BiquadParams::default();
            p.update(sc.sample_freq, sc.cutoff_freq);
            if let Action::Retune(c) = sc.act {
                p.update(sc.sample_freq, c);
            }
            [
                (p.cutoff_freq, f(&row[1])),
                (p.sample_freq, f(&row[2])),
                (p.a1, f(&row[3])),
                (p.a2, f(&row[4])),
                (p.b0, f(&row[5])),
                (p.b1, f(&row[6])),
                (p.b2, f(&row[7])),
            ]
            .iter()
            .all(|&(a, b)| same(a, b))
        })
        .collect()
}

/// The coefficients themselves.
///
/// This is what catches a compensating error: a filter whose coefficients are
/// wrong but whose state has drifted to hide it would pass the sample
/// comparison for a while and then diverge somewhere else entirely.
///
/// Differences are bounded at 8 ulps. Measured worst is 3, in `loop_rate`'s
/// `b0`, and every one of them is downstream of `tanf` or `cosf` -- D-017.
#[test]
fn the_coefficients_match_upstream() {
    let text = fixture("biquad_params.csv");
    let rows = rows(&text);
    assert_eq!(rows.len(), SCENARIOS.len());

    let mut worst = 0_i64;
    let mut note = String::new();

    for (i, sc) in SCENARIOS.iter().enumerate() {
        let row = &rows[i];
        assert_eq!(row.len(), 8);
        assert_eq!(row[0], sc.name);

        let mut p = BiquadParams::default();
        p.update(sc.sample_freq, sc.cutoff_freq);
        if let Action::Retune(c) = sc.act {
            p.update(sc.sample_freq, c);
        }

        for (label, got, want) in [
            ("cutoff_freq", p.cutoff_freq, f(&row[1])),
            ("sample_freq", p.sample_freq, f(&row[2])),
            ("a1", p.a1, f(&row[3])),
            ("a2", p.a2, f(&row[4])),
            ("b0", p.b0, f(&row[5])),
            ("b1", p.b1, f(&row[6])),
            ("b2", p.b2, f(&row[7])),
        ] {
            if same(got, want) {
                continue;
            }
            let u = ulps(got, want);
            assert!(
                u <= 8,
                "{}: {label} = {got:e} ({:#010x}) against upstream {want:e} ({:#010x}), \
                 {u} ulps — too far to be D-017",
                sc.name,
                got.to_bits(),
                want.to_bits()
            );
            if u > worst {
                worst = u;
                note = format!("{} {label}", sc.name);
            }
        }
    }
    println!("coefficients: worst {worst} ulps ({note}), bound 8");
}

/// Every sample of every scenario, both instantiations.
///
/// Scenarios whose coefficients are bit-identical must produce bit-identical
/// output — no tolerance, because there is nothing in them that could
/// legitimately differ. The rest are allowed a relative error, because a
/// three-ulp coefficient seed feeds a recursive filter and grows.
#[test]
fn the_filter_matches_upstream_sample_for_sample() {
    /// Measured worst is 5.57e-6, at `retune` step 52.
    const REL_TOLERANCE: f64 = 1.0e-5;

    let text = fixture("biquad_parity.csv");
    let rows = rows(&text);
    assert_eq!(
        rows.len(),
        SCENARIOS.len() * 100,
        "fixture and scenario table disagree about how many runs there are"
    );

    let exact = bit_exact_scenarios();
    let exact_count = exact.iter().filter(|&&e| e).count();
    assert!(
        exact_count >= SCENARIOS.len() / 2,
        "most scenarios should agree exactly; only {exact_count} of {} did, which \
         suggests something other than D-017 is wrong",
        SCENARIOS.len()
    );

    let mut worst_rel = 0.0_f64;
    let mut worst_where = String::new();

    for (i, sc) in SCENARIOS.iter().enumerate() {
        let mut port_f = LowPassFilter2p::<f32>::new();
        let mut port_v = LowPassFilter2p::<Vector3f>::new();
        port_f.set_cutoff_frequency(sc.sample_freq, sc.cutoff_freq);
        port_v.set_cutoff_frequency(sc.sample_freq, sc.cutoff_freq);

        for step in 0..100 {
            let row = &rows[i * 100 + step];
            assert_eq!(row.len(), 7, "row {step} of {}", sc.name);
            assert_eq!(row[0], sc.name, "scenario order must match the fixture");
            assert_eq!(
                row[1].parse::<usize>().expect("step"),
                step,
                "step order must match the fixture"
            );

            if step == 40 {
                match sc.act {
                    Action::None => {}
                    Action::Reset => {
                        port_f.reset();
                        port_v.reset();
                    }
                    Action::ResetTo(v) => {
                        port_f.reset_to(v);
                        port_v.reset_to(Vector3f::new(v, 2.0 * v, -v));
                    }
                    Action::Retune(c) => {
                        port_f.set_cutoff_frequency(sc.sample_freq, c);
                        port_v.set_cutoff_frequency(sc.sample_freq, c);
                    }
                }
            }

            // The recorded input, not a reconstruction — see the module docs.
            let input = f(&row[2]);
            let got_f = port_f.apply(input);
            let got_v = port_v.apply(Vector3f::new(input, 2.0 * input, -input));

            for (axis, got, want) in [
                ("float", got_f, f(&row[3])),
                ("x", got_v.x, f(&row[4])),
                ("y", got_v.y, f(&row[5])),
                ("z", got_v.z, f(&row[6])),
            ] {
                if same(got, want) {
                    continue;
                }
                assert!(
                    !exact[i],
                    "{} step {step} {axis}: {got:e} ({:#010x}) != upstream {want:e} \
                     ({:#010x}). This scenario's coefficients are bit-identical, so \
                     there is nothing here that could legitimately differ — this is a \
                     porting bug, not D-017",
                    sc.name,
                    got.to_bits(),
                    want.to_bits()
                );

                let rel = ((f64::from(got) - f64::from(want)) / f64::from(want).abs()).abs();
                assert!(
                    rel < REL_TOLERANCE,
                    "{} step {step} {axis}: {got:e} against upstream {want:e}, relative \
                     {rel:e} — beyond what a three-ulp coefficient seed explains",
                    sc.name
                );
                if rel > worst_rel {
                    worst_rel = rel;
                    worst_where = format!("{} step {step} {axis}", sc.name);
                }
            }
        }
    }
    println!(
        "{exact_count} of {} scenarios bit-exact; worst relative error elsewhere {worst_rel:e} ({worst_where})",
        SCENARIOS.len()
    );
}
