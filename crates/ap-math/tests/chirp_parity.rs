//! Parity test: the chirp frequency sweep against upstream.
//!
//! Nine configurations over a dense time grid running from before each record
//! starts to after it ends, so every branch of the window and every branch of
//! the waveform is crossed: the silence either side, both raised-cosine fades,
//! the unity middle, the dwell, the exponential sweep, and the
//! constant-frequency case where the exponent is zero and upstream tests for
//! it rather than dividing by it.
//!
//! A descending sweep is in there because the exponent is then negative and
//! nothing in the code says it must not be, and a near-equal pair because the
//! exponent is tiny and the phase multiplier correspondingly enormous.
//!
//! The completion flag is compared as well as the output and the frequency.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::chirp::Chirp;

struct Config {
    name: &'static str,
    record: f32,
    f_start: f32,
    f_stop: f32,
    fade_in: f32,
    fade_out: f32,
    const_freq: f32,
    magnitude: f32,
}

#[allow(
    clippy::too_many_arguments,
    reason = "a positional constructor for a data table mirroring the C struct array \nin gen_chirp_fixture.py; keeping the two in the same order is what makes them checkable \nagainst each other by eye"
)]
const fn c(
    name: &'static str,
    record: f32,
    f_start: f32,
    f_stop: f32,
    fade_in: f32,
    fade_out: f32,
    const_freq: f32,
    magnitude: f32,
) -> Config {
    Config {
        name,
        record,
        f_start,
        f_stop,
        fade_in,
        fade_out,
        const_freq,
        magnitude,
    }
}

/// Mirrors the table in `tools/parity/gen_chirp_fixture.py`.
/// The configuration whose waveform is not reproducible across float
/// implementations. See `a_near_equal_sweep_is_not_reproducible`.
const NEAR_EQUAL: &str = "near_equal";

const CONFIGS: &[Config] = &[
    c("sweep", 30.0, 0.5, 10.0, 2.0, 2.0, 3.0, 1.0),
    c("no_dwell", 20.0, 1.0, 20.0, 1.0, 1.0, 0.0, 0.5),
    c("steady", 10.0, 2.0, 2.0, 1.0, 1.0, 0.0, 1.0),
    c("steady_dwell", 10.0, 3.0, 3.0, 1.0, 1.0, 4.0, 0.25),
    c("wide", 60.0, 0.1, 40.0, 5.0, 5.0, 8.0, 1.0),
    c("descending", 25.0, 10.0, 0.5, 2.0, 2.0, 2.0, 1.0),
    c("no_fade", 15.0, 1.0, 5.0, 0.0, 0.0, 0.0, 1.0),
    c("all_dwell", 10.0, 1.0, 10.0, 1.0, 1.0, 12.0, 1.0),
    c("near_equal", 20.0, 2.0, 2.0001, 1.0, 1.0, 0.0, 1.0),
];

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/chirp_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_chirp_fixture.py",
            path.display()
        )
    })
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

#[test]
fn the_chirp_matches_upstream_sample_for_sample() {
    /// Fraction of the commanded magnitude the output may differ by.
    ///
    /// The phase of a long sweep reaches several hundred radians, so the few
    /// ulps D-017 guarantees in `expf` and `sinf` arrive amplified. Measured
    /// worst is well inside this.
    const BOUND: f64 = 1.0e-3;

    let text = fixture();
    let mut rows: Vec<Vec<&str>> = Vec::new();
    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("config,") {
            continue;
        }
        rows.push(line.split(',').collect());
    }

    let mut checked = 0_usize;
    let mut exact = 0_usize;
    let mut worst_rel = 0.0_f64;
    let mut worst_where = String::new();

    for cfg in CONFIGS {
        if cfg.name == NEAR_EQUAL {
            // Checked separately below; see `a_near_equal_sweep_is_not_reproducible`.
            continue;
        }
        let mut chirp = Chirp::new();
        chirp.init(
            cfg.record,
            cfg.f_start,
            cfg.f_stop,
            cfg.fade_in,
            cfg.fade_out,
            cfg.const_freq,
        );

        let mine: Vec<&Vec<&str>> = rows.iter().filter(|r| r[0] == cfg.name).collect();
        assert!(
            mine.len() > 100,
            "{}: only {} rows in the fixture",
            cfg.name,
            mine.len()
        );

        for row in mine {
            assert_eq!(row.len(), 5);
            let time = f(row[1]);
            let want_out = f(row[2]);
            let want_freq = f(row[3]);
            let want_complete = row[4] == "1";

            let got_out = chirp.update(time, cfg.magnitude);

            assert_eq!(
                chirp.completed(),
                want_complete,
                "{} at t={time}: completion flag",
                cfg.name
            );

            // The output is judged against the magnitude it was commanded
            // with, not against its own instantaneous value. The phase of a
            // long exponential sweep reaches several hundred radians, so the
            // one-ulp difference D-017 guarantees in expf and sinf moves the
            // sine by a few times 1e-5 — negligible against a signal of scale
            // 1.0, and enormous against a value that happens to be near a
            // zero crossing.
            //
            // The frequency never approaches zero, so it gets a relative
            // bound.
            for (label, g, w, scale) in [
                ("output", got_out, want_out, f64::from(cfg.magnitude)),
                (
                    "frequency",
                    chirp.frequency_rads(),
                    want_freq,
                    f64::from(want_freq).abs().max(1.0e-3),
                ),
            ] {
                if same(g, w) {
                    exact += 1;
                } else {
                    let rel = ((f64::from(g) - f64::from(w)) / scale).abs();
                    assert!(
                        rel < BOUND,
                        "{} {label} at t={time}: {g} against upstream {w} \
                         ({rel:e} of the {scale} scale)",
                        cfg.name
                    );
                    if rel > worst_rel {
                        worst_rel = rel;
                        worst_where = format!("{} {label} at t={time}", cfg.name);
                    }
                }
                checked += 1;
            }
        }
    }

    assert!(
        checked > 40_000,
        "fixture looks truncated: {checked} values"
    );
    println!(
        "{checked} values across {} configurations, {exact} bit-exact ({:.2}%); worst relative {worst_rel:e} {worst_where}",
        CONFIGS.len(),
        100.0 * exact as f64 / checked as f64
    );
}

/// Start and stop frequencies that are nearly but not exactly equal produce a
/// waveform no two float implementations agree on, and this pins that.
///
/// `B = ln(wMax/wMin)` is 5e-5 for 2.0 against 2.0001 Hz, and the phase is
/// multiplied by `wMin * (record - dwell) / B` — about five million. The last
/// bits of `logf` and `expf` become thousands of radians, and the sine of that
/// is arbitrary.
///
/// Upstream handles *exactly* equal frequencies with `is_equal` and a separate
/// branch. Nearly equal falls through to the sweep, where this happens. The
/// frequency the generator reports stays correct throughout; only the waveform
/// is affected, so a frequency-response measurement using it would be reading
/// a signal it cannot reproduce.
#[test]
fn a_near_equal_sweep_is_not_reproducible() {
    let text = fixture();
    let cfg = CONFIGS
        .iter()
        .find(|c| c.name == NEAR_EQUAL)
        .expect("the near-equal configuration");

    let mut chirp = Chirp::new();
    chirp.init(
        cfg.record,
        cfg.f_start,
        cfg.f_stop,
        cfg.fade_in,
        cfg.fade_out,
        cfg.const_freq,
    );

    let mut worst_output = 0.0_f64;
    let mut worst_freq = 0.0_f64;
    let mut samples = 0_usize;

    for line in text.lines() {
        if !line.starts_with(NEAR_EQUAL) {
            continue;
        }
        let row: Vec<&str> = line.split(',').collect();
        let time = f(row[1]);
        let got = chirp.update(time, cfg.magnitude);

        worst_output = worst_output.max(f64::from(got - f(row[2])).abs());
        worst_freq = worst_freq.max(f64::from(chirp.frequency_rads() - f(row[3])).abs());
        samples += 1;
    }

    assert!(samples > 100, "expected the configuration in the fixture");
    println!(
        "near_equal: worst output difference {worst_output:e}, worst frequency difference {worst_freq:e} over {samples} samples"
    );

    // The frequency is a single exp and stays right.
    assert!(
        worst_freq < 1.0e-3,
        "the reported frequency should still track upstream, got {worst_freq}"
    );
    // The waveform does not, and saying so is the point of this test. If this
    // ever starts holding, the amplification has gone and the note above is
    // stale.
    assert!(
        worst_output > 0.1,
        "the waveform was expected to diverge; if it no longer does, the \
         five-million-fold phase amplification has changed and this test and \
         its documentation need revisiting"
    );
}
