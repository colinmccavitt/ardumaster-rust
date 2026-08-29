//! Parity test: `SCurve::calculate_path` against upstream.
//!
//! The time-solver is the mathematical core of the 23-segment track — snap,
//! jerk, accel, speed and length in, five durations out. The grids cross
//! every branch: non-positive limits, a start already at cruise, a length
//! too short to accelerate, the small-`At` shrink-`tj` path from rest, the
//! speed-change path with a non-zero start, and the four solutions
//! (0 / 2 / 5 / 7).
//!
//! The function is public and takes every piece of state as an argument, so
//! the harness calls it directly. Nothing depends on class layout.
//!
//! `powf` comes from libm here and glibc upstream — D-017. A 1-ULP gap on
//! the Cardano / shrink-`tj` roots is that, not a transcription error; a
//! relative bound of 1e-5 would still catch a formula that was wrong.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::scurve::calculate_path;

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/scurve_path_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_scurve_path_fixture.py",
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
fn calculate_path_matches_upstream() {
    let text = fixture();

    let mut checked = 0_usize;
    let mut exact = 0_usize;
    let mut worst = 0.0_f64;
    let mut worst_where = String::new();
    for line in text.lines() {
        if line.is_empty()
            || line.starts_with('#')
            || line.starts_with("sm,")
            || !line.as_bytes()[0].is_ascii_digit()
        {
            continue;
        }
        let cols: Vec<&str> = line.split(',').collect();
        assert_eq!(cols.len(), 11, "row: {line}");
        let sm = f(cols[0]);
        let jm = f(cols[1]);
        let v0 = f(cols[2]);
        let am = f(cols[3]);
        let vm = f(cols[4]);
        let length = f(cols[5]);
        let exp = (f(cols[6]), f(cols[7]), f(cols[8]), f(cols[9]), f(cols[10]));
        let got = calculate_path(sm, jm, v0, am, vm, length);
        for (label, g, w) in [
            ("jm", got.jm, exp.0),
            ("tj", got.tj, exp.1),
            ("t2", got.t2, exp.2),
            ("t4", got.t4, exp.3),
            ("t6", got.t6, exp.4),
        ] {
            if same(g, w) {
                exact += 1;
            } else {
                let denom = f64::from(w).abs().max(1.0);
                let rel = ((f64::from(g) - f64::from(w)) / denom).abs();
                assert!(
                    rel < 1.0e-5,
                    "sm={sm} jm={jm} v0={v0} am={am} vm={vm} L={length} {label}: {g} against upstream {w}"
                );
                if rel > worst {
                    worst = rel;
                    worst_where = format!("sm={sm} {label}");
                }
            }
            checked += 1;
        }
    }
    assert!(
        checked > 200_000,
        "fixture looks truncated: {checked} values"
    );
    println!(
        "{checked} values, {exact} bit-exact ({:.2}%); worst relative {worst:e} {worst_where}",
        100.0 * exact as f64 / checked as f64
    );
}
