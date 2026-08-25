//! Parity test: `HarmonicNotchFilter` against upstream's own bank.
//!
//! Unlike the biquad and notch fixtures, this one cannot compare coefficients:
//! `HarmonicNotchFilter` keeps its bank and shaping constants private, and
//! there is no accessor. So it compares what the filter actually *does* — a
//! deterministic broadband signal through the whole bank, sample for sample. A
//! notch placed at the wrong frequency changes the output, so the placement
//! arithmetic is covered even though it is never read directly.
//!
//! Every scenario runs in `Fixed` tracking mode. `_tracking_mode` is private
//! with no setter, so the harness cannot set it; retunes are driven by calling
//! `update` explicitly, which does not consult the mode. Only `init` does, to
//! decide whether to place the notches itself.
//!
//! # D-021 is pinned here, not merely asserted
//!
//! The `quintuple_h1` scenario asks upstream for a quintuple notch. Upstream
//! clamps the composite count to three, so what it produces should be
//! *identical* to its own `triple_h1` output — and
//! [`upstream_quintuple_is_actually_a_triple`] asserts exactly that, bit for
//! bit. That is a far sharper statement of the defect than "the port differs
//! from upstream", and it would fail immediately if upstream ever fixed it.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact comparison against upstream is the point of the test"
)]

use ap_filter::harmonic::{
    CompositeNotches, HarmonicNotchFilter, HarmonicNotchParams, TrackingMode,
};
use ap_math::vector3::Vector3f;

struct Scenario {
    name: &'static str,
    sample_freq: f32,
    center_freq: f32,
    bandwidth: f32,
    attenuation_db: f32,
    freq_min_ratio: f32,
    harmonics: u32,
    composite: CompositeNotches,
    treat_low_as_min: bool,
    num_centers: u8,
    retune: bool,
    centers: &'static [f32],
    centers2: &'static [f32],
}

#[allow(
    clippy::too_many_arguments,
    reason = "a positional constructor for a data table that mirrors the C struct \
array in gen_harmonic_fixture.py; keeping the two in the same order is what makes them \
checkable against each other by eye"
)]
const fn s(
    name: &'static str,
    sample_freq: f32,
    center_freq: f32,
    bandwidth: f32,
    attenuation_db: f32,
    freq_min_ratio: f32,
    harmonics: u32,
    composite: CompositeNotches,
    treat_low_as_min: bool,
    num_centers: u8,
    retune: bool,
    centers: &'static [f32],
    centers2: &'static [f32],
) -> Scenario {
    Scenario {
        name,
        sample_freq,
        center_freq,
        bandwidth,
        attenuation_db,
        freq_min_ratio,
        harmonics,
        composite,
        treat_low_as_min,
        num_centers,
        retune,
        centers,
        centers2,
    }
}

const N: &[f32] = &[];

/// Mirrors the table in `tools/parity/gen_harmonic_fixture.py`.
const SCENARIOS: &[Scenario] = &[
    s(
        "single_h1",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x1,
        CompositeNotches::Single,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "single_h12",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x3,
        CompositeNotches::Single,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "single_h124",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0xB,
        CompositeNotches::Single,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "double_h1",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x1,
        CompositeNotches::Double,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "triple_h1",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x1,
        CompositeNotches::Triple,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "triple_h12",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x3,
        CompositeNotches::Triple,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "quintuple_h1",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x1,
        CompositeNotches::Quintuple,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "ins_rate",
        8000.0,
        188.0,
        60.0,
        30.0,
        1.0,
        0x3,
        CompositeNotches::Double,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "past_nyquist",
        1000.0,
        200.0,
        40.0,
        30.0,
        1.0,
        0x7,
        CompositeNotches::Single,
        false,
        1,
        false,
        N,
        N,
    ),
    s(
        "tracked",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x1,
        CompositeNotches::Single,
        false,
        1,
        true,
        &[150.0],
        &[90.0],
    ),
    s(
        "fade_out",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x1,
        CompositeNotches::Single,
        false,
        1,
        true,
        &[50.0],
        &[10.0],
    ),
    s(
        "low_as_min",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x1,
        CompositeNotches::Single,
        true,
        1,
        true,
        &[50.0],
        &[10.0],
    ),
    s(
        "multi",
        1000.0,
        100.0,
        40.0,
        30.0,
        1.0,
        0x3,
        CompositeNotches::Single,
        false,
        3,
        true,
        &[100.0, 150.0, 200.0],
        &[120.0, 170.0, 220.0],
    ),
];

const STEPS: usize = 120;

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/harmonic_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_harmonic_fixture.py",
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

/// Build the port's bank for a scenario, brought into service the way the
/// harness does.
fn build(
    sc: &Scenario,
    composite: CompositeNotches,
) -> (HarmonicNotchFilter<f32>, HarmonicNotchFilter<Vector3f>) {
    let params = HarmonicNotchParams {
        center_freq_hz: sc.center_freq,
        bandwidth_hz: sc.bandwidth,
        attenuation_db: sc.attenuation_db,
        freq_min_ratio: sc.freq_min_ratio,
        harmonics: sc.harmonics,
        composite_notches: composite,
        tracking_mode: TrackingMode::Fixed,
        treat_low_as_min: sc.treat_low_as_min,
    };
    let mut hf = HarmonicNotchFilter::<f32>::new();
    let mut hv = HarmonicNotchFilter::<Vector3f>::new();
    hf.allocate_filters(sc.num_centers, sc.harmonics, composite);
    hv.allocate_filters(sc.num_centers, sc.harmonics, composite);
    hf.init(sc.sample_freq, params);
    hv.init(sc.sample_freq, params);
    hf.reset();
    hv.reset();
    (hf, hv)
}

/// Run one scenario through the port and return its outputs, given the inputs
/// upstream actually saw.
fn run_port(sc: &Scenario, composite: CompositeNotches, inputs: &[f32]) -> Vec<(f32, Vector3f)> {
    let (mut hf, mut hv) = build(sc, composite);
    let mut out = Vec::with_capacity(STEPS);
    for (step, &input) in inputs.iter().enumerate() {
        if sc.retune && step == 40 {
            hf.update_multi(sc.centers);
            hv.update_multi(sc.centers);
        }
        if sc.retune && step == 80 {
            hf.update_multi(sc.centers2);
            hv.update_multi(sc.centers2);
        }
        out.push((
            hf.apply(input),
            hv.apply(Vector3f::new(input, 2.0 * input, -input)),
        ));
    }
    out
}

/// The fixture's inputs and outputs for one scenario, by name.
fn scenario_rows<'a>(rows: &'a [Vec<String>], name: &str) -> Vec<&'a Vec<String>> {
    rows.iter().filter(|r| r[0] == name).collect()
}

/// Every sample of every scenario, both instantiations.
///
/// `quintuple_h1` is expected to differ — that is D-021, and it is pinned
/// precisely by the test below rather than waved at here.
#[test]
fn the_bank_matches_upstream_sample_for_sample() {
    let text = fixture();
    let rows = rows(&text);
    assert_eq!(rows.len(), SCENARIOS.len() * STEPS);

    let mut compared = 0_usize;
    for sc in SCENARIOS {
        let sr = scenario_rows(&rows, sc.name);
        assert_eq!(sr.len(), STEPS, "{}: wrong row count", sc.name);

        let inputs: Vec<f32> = sr.iter().map(|r| f(&r[2])).collect();
        let got = run_port(sc, sc.composite, &inputs);

        if sc.composite == CompositeNotches::Quintuple {
            // D-021: the port deliberately places five where upstream places
            // three. Asserting equality here would be asserting the bug.
            continue;
        }

        for (step, row) in sr.iter().enumerate() {
            assert_eq!(row.len(), 7);
            let (gf, gv) = got[step];
            for (axis, g, w) in [
                ("float", gf, f(&row[3])),
                ("x", gv.x, f(&row[4])),
                ("y", gv.y, f(&row[5])),
                ("z", gv.z, f(&row[6])),
            ] {
                assert!(
                    same(g, w),
                    "{} step {step} {axis}: {g:e} ({:#010x}) != upstream {w:e} ({:#010x})",
                    sc.name,
                    g.to_bits(),
                    w.to_bits()
                );
            }
            compared += 1;
        }
    }
    println!(
        "{compared} samples compared bit-exactly across {} scenarios",
        SCENARIOS.len() - 1
    );
}

/// D-021, stated as sharply as it can be: upstream's *quintuple* notch and
/// upstream's *triple* notch produce byte-identical output.
///
/// Both scenarios are configured identically apart from the composite option,
/// so if the quintuple option did anything at all the two would differ. They
/// do not, because `allocate_filters` clamps the count to three and the branch
/// that would place the outer pair is unreachable.
///
/// This compares upstream against upstream — no port code is involved — so it
/// is a statement about ArduPilot, and it will start failing the day upstream
/// fixes it.
#[test]
fn upstream_quintuple_is_actually_a_triple() {
    let text = fixture();
    let rows = rows(&text);

    let quint = scenario_rows(&rows, "quintuple_h1");
    let triple = scenario_rows(&rows, "triple_h1");
    assert_eq!(quint.len(), STEPS);
    assert_eq!(triple.len(), STEPS);

    for (step, (q, t)) in quint.iter().zip(triple.iter()).enumerate() {
        for col in 2..7 {
            assert!(
                same(f(&q[col]), f(&t[col])),
                "step {step} column {col}: upstream's quintuple {} differs from its \
                 triple {} — upstream may have fixed D-021, in which case the port's \
                 divergence should be retired",
                f(&q[col]),
                f(&t[col])
            );
        }
    }

    // And the port, asked for a quintuple, does something different — five
    // notches rather than three.
    let sc = SCENARIOS
        .iter()
        .find(|s| s.name == "quintuple_h1")
        .expect("scenario");
    let inputs: Vec<f32> = quint.iter().map(|r| f(&r[2])).collect();
    let port_quint = run_port(sc, CompositeNotches::Quintuple, &inputs);
    let differs = port_quint
        .iter()
        .zip(quint.iter())
        .any(|((gf, _), row)| !same(*gf, f(&row[3])));
    assert!(
        differs,
        "the port should place five notches where upstream places three"
    );

    // Asked for a triple, it agrees with upstream exactly — so the difference
    // above is the quintuple handling and nothing else.
    let port_triple = run_port(sc, CompositeNotches::Triple, &inputs);
    for (step, ((gf, _), row)) in port_triple.iter().zip(quint.iter()).enumerate() {
        assert!(
            same(*gf, f(&row[3])),
            "step {step}: the port's triple should match upstream's clamped quintuple \
             exactly, {gf:e} against {:e}",
            f(&row[3])
        );
    }
}
