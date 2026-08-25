//! Parity test: polygon geometry against upstream `AP_Math/polygon.cpp`.
//!
//! Expected values come from compiling a harness with the flags waf used and
//! linking the objects waf already built, so upstream's own code produced every
//! one of them — see `tools/parity/gen_polygon_fixture.py`. Regenerate after an
//! upstream re-baseline; a diff in the fixture is an upstream behaviour change.
//!
//! Float results are carried as raw bit patterns. These functions end in
//! `sqrtf`, and a decimal round-trip would blur the last bit — exactly where a
//! port difference would first appear. Reading them with `f32::from_bits` keeps
//! the comparison exact, so the tolerance below is a deliberate choice rather
//! than an artefact of printing.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "comparing against upstream's logged values is what the test is for"
)]

use ap_math::polygon::*;
use ap_math::vector2::Vector2f;

fn fixture_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/polygon_parity.csv"))
        .expect("workspace root")
}

/// The fixture, split into its named sections.
struct Fixture {
    polys: Vec<(String, Vec<Vector2f>)>,
    rows: Vec<(String, Vec<String>)>,
}

fn parse(text: &str) -> Fixture {
    let mut polys = Vec::new();
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
            continue; // column names
        }
        let f: Vec<String> = line.split(',').map(str::to_string).collect();
        if section == "polys" {
            let coords: Vec<f32> = f[2]
                .split_whitespace()
                .map(|s| s.parse::<f32>().expect("coord"))
                .collect();
            assert_eq!(
                coords.len(),
                f[1].parse::<usize>().expect("n") * 2,
                "polygon {} coordinate count",
                f[0]
            );
            polys.push((
                f[0].clone(),
                coords
                    .chunks(2)
                    .map(|c| Vector2f::new(c[0], c[1]))
                    .collect(),
            ));
        } else {
            rows.push((section.clone(), f));
        }
    }
    Fixture { polys, rows }
}

fn f32_of(s: &str) -> f32 {
    f32::from_bits(s.parse::<u32>().expect("bit pattern"))
}

#[test]
fn polygon_matches_upstream() {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: {} not present", path.display());
        return;
    }
    let fx = parse(&std::fs::read_to_string(&path).expect("read fixture"));
    assert!(!fx.polys.is_empty(), "fixture has no polygons");

    let poly = |name: &str| -> &[Vector2f] {
        fx.polys
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, v)| v.as_slice())
            .unwrap_or_else(|| panic!("fixture has no polygon {name}"))
    };

    let mut counts = std::collections::BTreeMap::<&str, usize>::new();

    for (section, f) in &fx.rows {
        match section.as_str() {
            "complete" => {
                let v = poly(&f[0]);
                let n: usize = f[1].parse().expect("n");
                let want = f[2] == "1";
                assert_eq!(
                    polygon_complete(&v[..n]),
                    want,
                    "polygon_complete({}, n={n})",
                    f[0]
                );
                *counts.entry("complete").or_default() += 1;
            }
            "outside" => {
                let v = poly(&f[0]);
                let p = Vector2f::new(f[1].parse().expect("px"), f[2].parse().expect("py"));
                let want = f[3] == "1";
                assert_eq!(
                    polygon_outside(p, v),
                    want,
                    "polygon_outside({}, {:?})",
                    f[0],
                    p
                );
                *counts.entry("outside").or_default() += 1;
            }
            "intersects" => {
                let v = poly(&f[0]);
                let p1 = Vector2f::new(f[1].parse().expect("x"), f[2].parse().expect("y"));
                let p2 = Vector2f::new(f[3].parse().expect("x"), f[4].parse().expect("y"));
                let want_found = f[5] == "1";
                let got = polygon_intersects(v, p1, p2);
                assert_eq!(
                    got.is_some(),
                    want_found,
                    "polygon_intersects({}, {p1:?}..{p2:?})",
                    f[0]
                );
                if let Some(hit) = got {
                    assert_eq!(
                        (hit.x, hit.y),
                        (f32_of(&f[6]), f32_of(&f[7])),
                        "intersection point for {} {p1:?}..{p2:?}",
                        f[0]
                    );
                }
                *counts.entry("intersects").or_default() += 1;
            }
            "dist_line" => {
                let v = poly(&f[0]);
                let p1 = Vector2f::new(f[1].parse().expect("x"), f[2].parse().expect("y"));
                let p2 = Vector2f::new(f[3].parse().expect("x"), f[4].parse().expect("y"));
                assert_eq!(
                    polygon_closest_distance_line(v, p1, p2),
                    f32_of(&f[5]),
                    "polygon_closest_distance_line({}, {p1:?}..{p2:?})",
                    f[0]
                );
                *counts.entry("dist_line").or_default() += 1;
            }
            "dist_point" => {
                let v = poly(&f[0]);
                let p = Vector2f::new(f[1].parse().expect("px"), f[2].parse().expect("py"));
                let want_ok = f[3] == "1";
                let got = polygon_closest_distance_point(v, p);
                assert_eq!(
                    got.is_some(),
                    want_ok,
                    "polygon_closest_distance_point({}, {p:?})",
                    f[0]
                );
                if let Some(c) = got {
                    assert_eq!(
                        (c.x, c.y),
                        (f32_of(&f[4]), f32_of(&f[5])),
                        "closest vector for {} {p:?}",
                        f[0]
                    );
                }
                *counts.entry("dist_point").or_default() += 1;
            }
            other => panic!("fixture has an unhandled section {other}"),
        }
    }

    let total: usize = counts.values().sum();
    println!("{total} cases matched upstream polygon.cpp exactly");
    for (k, v) in &counts {
        println!("  {k:<12} {v}");
    }

    // A parity test that quietly matched nothing would pass, and each section
    // must actually be represented rather than one carrying the whole count.
    for section in [
        "complete",
        "outside",
        "intersects",
        "dist_line",
        "dist_point",
    ] {
        assert!(
            counts.get(section).copied().unwrap_or(0) > 0,
            "section {section} contributed no cases"
        );
    }
    assert!(total > 1500, "expected the whole fixture, got {total}");
}

/// Upstream's integer instantiation is not ported yet, but the fixture already
/// carries its results. This checks the oracle is intact so the day an integer
/// `Vector2` lands, the comparison is available rather than needing a re-run.
#[test]
fn integer_instantiation_oracle_is_present_but_unported() {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: fixture not present");
        return;
    }
    let fx = parse(&std::fs::read_to_string(&path).expect("read fixture"));
    let int_rows = fx
        .rows
        .iter()
        .filter(|(s, f)| s == "outside" && (f[4] == "0" || f[4] == "1"))
        .count();
    assert!(
        int_rows > 1000,
        "expected upstream's Vector2l results to be recorded, got {int_rows}"
    );

    // Where the float and integer forms disagree is worth knowing about when
    // the integer port lands: on whole-number coordinates they should not.
    let disagreements = fx
        .rows
        .iter()
        .filter(|(s, f)| s == "outside" && f[3] != f[4])
        .count();
    assert_eq!(
        disagreements, 0,
        "upstream's float and integer forms disagree on {disagreements} \
         whole-number cases; the integer port must reproduce that, not the float result"
    );
}
