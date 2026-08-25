//! Parity test: `NotchFilter` against upstream's own implementation.
//!
//! Harness-driven. Every interesting path here is configuration- and
//! sequence-dependent: the five-percent slew limit on retuning, the
//! unchanged-configuration early return, a rejected configuration that
//! disables the filter while keeping its cached centre, `Q` coming out zero
//! for a bandwidth reaching down through zero, the deferred reset, and the
//! ringing transient a notch shows when brought into service without one.
//!
//! The input sequence is read from the fixture rather than reconstructed — it
//! contains `sinf` calls, and D-017 records that upstream links glibc while the
//! port links libm, so a reconstruction could differ by an ulp for a reason
//! that has nothing to do with the filter.
//!
//! Both template instantiations are checked, and the coefficients are compared
//! directly as well as the outputs. Comparing only outputs would let a wrong
//! coefficient hide behind a compensating state error.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact comparison against upstream is the point of the test"
)]

use ap_filter::notch::NotchFilter;
use ap_math::vector3::Vector3f;

/// What happens, and when. Duplicated from
/// `tools/parity/gen_notch_fixture.py` — this is the configuration under test,
/// not data, so it is stated rather than read.
#[derive(Clone, Copy, PartialEq)]
enum Action {
    None,
    ResetAtStart,
    ResetMidway,
    Retune(f32),
    RetuneSame,
    Disable,
}

struct Scenario {
    name: &'static str,
    sample_freq: f32,
    center_freq: f32,
    bandwidth: f32,
    attenuation_db: f32,
    act: Action,
}

const fn s(
    name: &'static str,
    sample_freq: f32,
    center_freq: f32,
    bandwidth: f32,
    attenuation_db: f32,
    act: Action,
) -> Scenario {
    Scenario {
        name,
        sample_freq,
        center_freq,
        bandwidth,
        attenuation_db,
        act,
    }
}

const SCENARIOS: &[Scenario] = &[
    s("plain", 1000.0, 100.0, 40.0, 30.0, Action::None),
    s("serviced", 1000.0, 100.0, 40.0, 30.0, Action::ResetAtStart),
    s("ins_rate", 8000.0, 188.0, 60.0, 30.0, Action::ResetAtStart),
    s("narrow", 1000.0, 100.0, 10.0, 40.0, Action::ResetAtStart),
    s("wide", 1000.0, 200.0, 150.0, 20.0, Action::ResetAtStart),
    s("degenerate", 1000.0, 20.0, 40.0, 30.0, Action::None),
    s("too_high", 1000.0, 600.0, 40.0, 30.0, Action::None),
    s("slew_up", 1000.0, 100.0, 40.0, 30.0, Action::Retune(400.0)),
    s("slew_down", 1000.0, 100.0, 40.0, 30.0, Action::Retune(1.0)),
    s("retune_same", 1000.0, 100.0, 40.0, 30.0, Action::RetuneSame),
    s(
        "retune_bad",
        1000.0,
        100.0,
        40.0,
        30.0,
        Action::Retune(900.0),
    ),
    s(
        "reset_midway",
        1000.0,
        100.0,
        40.0,
        30.0,
        Action::ResetMidway,
    ),
    s("disabled", 1000.0, 100.0, 40.0, 30.0, Action::Disable),
];

const STEPS: usize = 120;

fn fixture(name: &str) -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures").join(name))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_notch_fixture.py",
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

/// Build the port's filter for a scenario and run it to just before the
/// midway action, returning it ready for the rest.
fn configured(sc: &Scenario) -> (NotchFilter<f32>, NotchFilter<Vector3f>, f32, f32) {
    let mut ff = NotchFilter::<f32>::new();
    let mut fv = NotchFilter::<Vector3f>::new();
    ff.init(
        sc.sample_freq,
        sc.center_freq,
        sc.bandwidth,
        sc.attenuation_db,
    );
    fv.init(
        sc.sample_freq,
        sc.center_freq,
        sc.bandwidth,
        sc.attenuation_db,
    );
    if sc.act == Action::ResetAtStart {
        ff.reset();
        fv.reset();
    }
    let (a, q) =
        NotchFilter::<f32>::calculate_a_and_q(sc.center_freq, sc.bandwidth, sc.attenuation_db);
    (ff, fv, a, q)
}

fn apply_midway(
    sc: &Scenario,
    ff: &mut NotchFilter<f32>,
    fv: &mut NotchFilter<Vector3f>,
    a: f32,
    q: f32,
) {
    match sc.act {
        Action::None | Action::ResetAtStart => {}
        Action::ResetMidway => {
            ff.reset();
            fv.reset();
        }
        Action::Retune(target) => {
            ff.init_with_a_and_q(sc.sample_freq, target, a, q);
            fv.init_with_a_and_q(sc.sample_freq, target, a, q);
        }
        Action::RetuneSame => {
            ff.init_with_a_and_q(sc.sample_freq, sc.center_freq, a, q);
            fv.init_with_a_and_q(sc.sample_freq, sc.center_freq, a, q);
        }
        Action::Disable => {
            ff.disable();
            fv.disable();
        }
    }
}

/// Which scenarios agree with upstream exactly on coefficients, and therefore
/// must agree exactly on output too.
fn bit_exact_scenarios() -> Vec<bool> {
    let text = fixture("notch_coeffs.csv");
    let rows = rows(&text);
    SCENARIOS
        .iter()
        .enumerate()
        .map(|(i, sc)| {
            let row = &rows[i];
            let (mut ff, mut fv, a, q) = configured(sc);
            apply_midway(sc, &mut ff, &mut fv, a, q);
            let (b0, b1, b2, a1, a2) = ff.coefficients();
            [
                (ff.center_freq(), f(&row[2])),
                (ff.sample_freq(), f(&row[3])),
                (b0, f(&row[4])),
                (b1, f(&row[5])),
                (b2, f(&row[6])),
                (a1, f(&row[7])),
                (a2, f(&row[8])),
            ]
            .iter()
            .all(|&(x, y)| same(x, y))
        })
        .collect()
}

/// The coefficients and the enabled flag, after each scenario's action.
///
/// Differences are bounded at 32 ulps. `calculate_A_and_Q` runs `powf`, `log2f`
/// and `sqrtf`, and the coefficients then run `sinf` and `cosf` — five
/// transcendentals deep, against D-017's ulp-level disagreement at each.
#[test]
fn the_coefficients_match_upstream() {
    let text = fixture("notch_coeffs.csv");
    let rows = rows(&text);
    assert_eq!(rows.len(), SCENARIOS.len());

    let mut worst = 0_i64;
    let mut note = String::new();

    for (i, sc) in SCENARIOS.iter().enumerate() {
        let row = &rows[i];
        assert_eq!(row.len(), 9);
        assert_eq!(row[0], sc.name);

        let (mut ff, mut fv, a, q) = configured(sc);
        apply_midway(sc, &mut ff, &mut fv, a, q);

        assert_eq!(
            i32::from(ff.is_initialised()),
            row[1].parse::<i32>().expect("flag"),
            "{}: enabled flag disagrees with upstream",
            sc.name
        );

        let (b0, b1, b2, a1, a2) = ff.coefficients();
        for (label, got, want) in [
            ("center_freq", ff.center_freq(), f(&row[2])),
            ("sample_freq", ff.sample_freq(), f(&row[3])),
            ("b0", b0, f(&row[4])),
            ("b1", b1, f(&row[5])),
            ("b2", b2, f(&row[6])),
            ("a1", a1, f(&row[7])),
            ("a2", a2, f(&row[8])),
        ] {
            if same(got, want) {
                continue;
            }
            let u = ulps(got, want);
            assert!(
                u <= 32,
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
    println!("coefficients: worst {worst} ulps ({note}), bound 32");
}

/// Every sample of every scenario, both instantiations.
///
/// Scenarios whose coefficients are bit-identical must produce bit-identical
/// output; there is nothing in them that could legitimately differ.
#[test]
fn the_filter_matches_upstream_sample_for_sample() {
    const REL_TOLERANCE: f64 = 1.0e-3;

    let text = fixture("notch_parity.csv");
    let rows = rows(&text);
    assert_eq!(rows.len(), SCENARIOS.len() * STEPS);

    let exact = bit_exact_scenarios();
    let exact_count = exact.iter().filter(|&&e| e).count();

    let mut worst_rel = 0.0_f64;
    let mut worst_where = String::new();

    for (i, sc) in SCENARIOS.iter().enumerate() {
        let (mut ff, mut fv, a, q) = configured(sc);

        for step in 0..STEPS {
            let row = &rows[i * STEPS + step];
            assert_eq!(row.len(), 7, "row {step} of {}", sc.name);
            assert_eq!(row[0], sc.name, "scenario order must match the fixture");
            assert_eq!(
                row[1].parse::<usize>().expect("step"),
                step,
                "step order must match the fixture"
            );

            if step == 50 {
                apply_midway(sc, &mut ff, &mut fv, a, q);
            }

            let input = f(&row[2]);
            let got_f = ff.apply(input);
            let got_v = fv.apply(Vector3f::new(input, 2.0 * input, -input));

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
                     nothing here could legitimately differ — this is a porting bug, \
                     not D-017",
                    sc.name,
                    got.to_bits(),
                    want.to_bits()
                );

                let denom = f64::from(want).abs().max(1.0e-3);
                let rel = ((f64::from(got) - f64::from(want)) / denom).abs();
                assert!(
                    rel < REL_TOLERANCE,
                    "{} step {step} {axis}: {got:e} against upstream {want:e}, relative \
                     {rel:e} — beyond what a few-ulp coefficient seed explains",
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
