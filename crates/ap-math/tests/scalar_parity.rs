//! Parity test: the scalar helpers against upstream `AP_Math.cpp`.
//!
//! These were ported early, before the parity harness existed, and until now
//! had never been compared against upstream. They are used by nearly every
//! other module, so an error here would be inherited widely and would surface
//! looking like a defect in whatever module happened to expose it.
//!
//! Values are raw bit patterns, so every comparison is exact. Inputs sit on the
//! awkward parts of each domain — either side of the wrap discontinuities,
//! across zero, at the edges of `asin`'s and `sqrt`'s domains, and either side
//! of `FLT_EPSILON` where the `is_*` predicates switch.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact comparison against upstream is the point of the test"
)]

use ap_math::scalar::*;

fn fixture_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/scalar_parity.csv"))
        .expect("workspace root")
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.parse::<u32>().expect("bit pattern"))
}

struct Row {
    section: String,
    c: Vec<String>,
}

fn parse(text: &str) -> Vec<Row> {
    let mut rows = Vec::new();
    let mut section = String::new();
    let mut header_pending = false;
    for line in text.lines() {
        if line.is_empty() {
            continue;
        }
        if let Some(name) = line.strip_prefix('#') {
            section = name.to_string();
            header_pending = true;
            continue;
        }
        if header_pending {
            header_pending = false;
            continue;
        }
        rows.push(Row {
            section: section.clone(),
            c: line.split(',').map(str::to_string).collect(),
        });
    }
    rows
}

/// Compare bit patterns, treating NaN as equal to NaN.
///
/// `safe_sqrt` and `constrain_value` can both produce NaN, and a plain `==`
/// would call two NaNs different and fail for the wrong reason.
fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

#[test]
fn scalar_helpers_match_upstream() {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: {} not present", path.display());
        return;
    }
    let rows = parse(&std::fs::read_to_string(&path).expect("read fixture"));
    assert!(!rows.is_empty(), "fixture has no rows");

    let mut counts = std::collections::BTreeMap::<String, usize>::new();
    let mut bump = |k: &str| *counts.entry(k.to_string()).or_default() += 1;

    for r in &rows {
        match r.section.as_str() {
            "wrap" => {
                assert_eq!(r.c.len(), 3, "wrap row");
                let inp = f(&r.c[1]);
                let want = f(&r.c[2]);
                let got = match r.c[0].as_str() {
                    "wrap_360" => wrap_360(inp),
                    "wrap_180" => wrap_180(inp),
                    "wrap_2PI" => wrap_2pi(inp),
                    "wrap_PI" => wrap_pi(inp),
                    other => panic!("unhandled wrap fn {other}"),
                };
                assert!(
                    same(got, want),
                    "{}({inp:?}): port {got:?} ({:#x}), upstream {want:?} ({:#x})",
                    r.c[0],
                    got.to_bits(),
                    want.to_bits()
                );
                bump(&r.c[0]);
            }
            "unary" => {
                assert_eq!(r.c.len(), 3, "unary row");
                let inp = f(&r.c[1]);
                let want = f(&r.c[2]);
                let got = match r.c[0].as_str() {
                    "safe_sqrt" => safe_sqrt(inp),
                    "safe_asin" => safe_asin(inp),
                    "sq" => sq(inp),
                    "degrees" => degrees(inp),
                    "radians" => radians(inp),
                    other => panic!("unhandled unary fn {other}"),
                };
                assert!(
                    same(got, want),
                    "{}({inp:?}): port {got:?}, upstream {want:?}",
                    r.c[0]
                );
                bump(&r.c[0]);
            }
            "predicate" => {
                assert_eq!(r.c.len(), 3, "predicate row");
                let inp = f(&r.c[1]);
                let want = r.c[2] == "1";
                let got = match r.c[0].as_str() {
                    "is_zero" => is_zero(inp),
                    "is_positive" => is_positive(inp),
                    "is_negative" => is_negative(inp),
                    other => panic!("unhandled predicate {other}"),
                };
                assert_eq!(got, want, "{}({inp:?})", r.c[0]);
                bump(&r.c[0]);
            }
            "binary" => {
                assert_eq!(r.c.len(), 4, "binary row");
                let a = f(&r.c[1]);
                let b = f(&r.c[2]);
                match r.c[0].as_str() {
                    "is_equal" => {
                        assert_eq!(is_equal(a, b), r.c[3] == "1", "is_equal({a:?}, {b:?})");
                    }
                    "norm2" => {
                        let want = f(&r.c[3]);
                        let got = norm2(a, b);
                        assert!(
                            same(got, want),
                            "norm2({a:?}, {b:?}): port {got:?}, upstream {want:?}"
                        );
                    }
                    other => panic!("unhandled binary fn {other}"),
                }
                bump(&r.c[0]);
            }
            "constrain" => {
                assert_eq!(r.c.len(), 5, "constrain row");
                let (v, lo, hi) = (f(&r.c[0]), f(&r.c[1]), f(&r.c[2]));
                let want = f(&r.c[3]);
                let got = constrain_value(v, lo, hi);
                assert!(
                    same(got, want),
                    "constrain_value({v:?}, {lo:?}, {hi:?}): port {got:?}, upstream {want:?}"
                );
                bump("constrain_value");
            }
            "interp" => {
                assert_eq!(r.c.len(), 6, "interp row");
                let got =
                    linear_interpolate(f(&r.c[0]), f(&r.c[1]), f(&r.c[2]), f(&r.c[3]), f(&r.c[4]));
                let want = f(&r.c[5]);
                assert!(
                    same(got, want),
                    "linear_interpolate({}, {}, {}, {}, {}): port {got:?}, upstream {want:?}",
                    f(&r.c[0]),
                    f(&r.c[1]),
                    f(&r.c[2]),
                    f(&r.c[3]),
                    f(&r.c[4])
                );
                bump("linear_interpolate");
            }
            other => panic!("unhandled section {other}"),
        }
    }

    let total: usize = counts.values().sum();
    println!("{total} scalar cases matched upstream exactly:");
    for (k, v) in &counts {
        println!("  {k:<20} {v}");
    }

    // A parity test that matched nothing would pass, and every helper must
    // actually be represented rather than one carrying the count.
    for name in [
        "wrap_360",
        "wrap_180",
        "wrap_2PI",
        "wrap_PI",
        "safe_sqrt",
        "safe_asin",
        "sq",
        "degrees",
        "radians",
        "is_zero",
        "is_positive",
        "is_negative",
        "is_equal",
        "norm2",
        "constrain_value",
        "linear_interpolate",
    ] {
        assert!(
            counts.get(name).copied().unwrap_or(0) > 0,
            "{name} contributed no cases"
        );
    }
    assert!(total > 300, "expected the whole fixture, got {total}");
}
