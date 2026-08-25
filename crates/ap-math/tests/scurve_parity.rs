//! Parity test: SCurve's segment kinematics against upstream.
//!
//! The three closed-form evaluators are the mathematical core of the
//! snap-limited trajectory — a raised-cosine jerk profile integrated up to
//! position.
//!
//! The grids cross every branch: the non-positive-`tj` early return, times
//! inside the segment and past its end, negative jerk references, and start
//! states with non-zero acceleration, velocity *and* position so no term can
//! be dropped without something moving.
//!
//! # How the harness reaches private functions
//!
//! All three are declared `private` upstream. Access specifiers are not part
//! of C++ name mangling, so `tools/parity/gen_scurve_fixture.py` relaxes only
//! the compiler's access check in its own view of the class — the object it
//! links is the one waf built and the vehicle runs, unchanged.
//!
//! That is safe for these three specifically because each is `const` and takes
//! every piece of state as an argument. None reads a member, so nothing
//! depends on the class layout. It is deliberately not extended to the
//! segment-array functions, which do touch members.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::scurve::{javp_const_jerk, javp_decr_jerk, javp_incr_jerk, Javp, SegmentStart};

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/scurve_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_scurve_fixture.py",
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
fn the_segment_kinematics_match_upstream() {
    let text = fixture();

    let mut checked = 0_usize;
    let mut exact = 0_usize;
    let mut worst = 0.0_f64;
    let mut worst_where = String::new();
    let mut kinds = std::collections::BTreeMap::<&str, usize>::new();

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("kind,") {
            continue;
        }
        let r: Vec<&str> = line.split(',').collect();
        assert_eq!(r.len(), 11, "malformed row: {line}");
        *kinds.entry(r[0]).or_default() += 1;

        let t = f(r[1]);
        let tj = f(r[2]);
        let jm = f(r[3]);
        let start = SegmentStart {
            accel: f(r[4]),
            vel: f(r[5]),
            pos: f(r[6]),
        };
        let want = Javp {
            jerk: f(r[7]),
            accel: f(r[8]),
            vel: f(r[9]),
            pos: f(r[10]),
        };

        let got = match r[0] {
            "const" => javp_const_jerk(t, jm, start),
            "incr" => javp_incr_jerk(t, tj, jm, start),
            "decr" => javp_decr_jerk(t, tj, jm, start),
            other => panic!("unknown fixture kind {other}"),
        };

        for (label, g, w) in [
            ("jerk", got.jerk, want.jerk),
            ("accel", got.accel, want.accel),
            ("vel", got.vel, want.vel),
            ("pos", got.pos, want.pos),
        ] {
            if same(g, w) {
                exact += 1;
            } else {
                // sin and cos come from libm here and glibc upstream — D-017.
                let denom = f64::from(w).abs().max(1.0);
                let rel = ((f64::from(g) - f64::from(w)) / denom).abs();
                assert!(
                    rel < 1.0e-5,
                    "{} {label} at t={t} tj={tj} jm={jm}: {g} against upstream {w}",
                    r[0]
                );
                if rel > worst {
                    worst = rel;
                    worst_where = format!("{} {label}", r[0]);
                }
            }
            checked += 1;
        }
    }

    for kind in ["const", "incr", "decr"] {
        assert!(
            kinds.get(kind).copied().unwrap_or(0) > 100,
            "the fixture is missing {kind} rows"
        );
    }
    assert!(
        checked > 60_000,
        "fixture looks truncated: {checked} values"
    );

    println!(
        "{checked} values, {exact} bit-exact ({:.2}%); worst relative {worst:e} {worst_where}",
        100.0 * exact as f64 / checked as f64
    );
}
