//! Parity test: bearing, distance and coordinate checks against upstream
//! `AP_Math/location.cpp`.
//!
//! Values are raw bit patterns, so every float comparison is exact.
//!
//! # D-016 is pinned here, not merely described
//!
//! The integer coordinate checks are the one place the port deliberately
//! differs. Upstream compares `labs(lat) <= 90*1e7` with a `float` bound, so at
//! 9e8 — where representable floats are 64 apart — values just past 90 degrees
//! round onto the bound and are accepted. Rather than assert a hard-coded
//! window, the test requires that wherever the two disagree it is always in the
//! same direction (upstream accepts, the port rejects) and always just outside
//! the true bound. That stays correct if upstream ever changes the constant.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact comparison against upstream is the point of the test"
)]

use ap_math::location::*;
use ap_math::scalar::{cd_to_rad, rad_to_cd};
use ap_math::vector2::Vector2f;

fn fixture_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/location_parity.csv"))
        .expect("workspace root")
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.parse::<u32>().expect("bit pattern"))
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

fn rows(text: &str) -> Vec<(String, Vec<String>)> {
    let mut out = Vec::new();
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
        out.push((
            section.clone(),
            line.split(',').map(str::to_string).collect(),
        ));
    }
    out
}

#[test]
fn location_matches_upstream() {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: {} not present", path.display());
        return;
    }
    let all = rows(&std::fs::read_to_string(&path).expect("read fixture"));
    assert!(!all.is_empty(), "fixture has no rows");

    let mut checked = 0usize;
    // D-016: values where the port rejects and upstream accepts
    let mut diverged: Vec<i64> = Vec::new();

    for (section, c) in &all {
        match section.as_str() {
            "bearing" => {
                assert_eq!(c.len(), 7, "bearing row");
                let o = Vector2f::new(f(&c[0]), f(&c[1]));
                let d = Vector2f::new(f(&c[2]), f(&c[3]));

                let rad = get_bearing_rad(o, d);
                assert!(same(rad, f(&c[4])), "get_bearing_rad({o:?}, {d:?})");
                let cd = get_bearing_cd(o, d);
                assert!(same(cd, f(&c[5])), "get_bearing_cd({o:?}, {d:?})");
                let dist = get_horizontal_distance(o, d);
                assert!(
                    same(dist, f(&c[6])),
                    "get_horizontal_distance({o:?}, {d:?})"
                );
                checked += 3;
            }
            "checkdeg" => {
                assert_eq!(c.len(), 3, "checkdeg row");
                let v = f(&c[0]);
                assert_eq!(check_lat_deg(v), c[1] == "1", "check_lat({v})");
                assert_eq!(check_lng_deg(v), c[2] == "1", "check_lng({v})");
                checked += 2;
            }
            "checkint" => {
                assert_eq!(c.len(), 3, "checkint row");
                let v: i32 = c[0].parse().expect("value");
                let up_lat = c[1] == "1";
                let up_lng = c[2] == "1";

                for (got, up, bound) in [
                    (check_lat_1e7(v), up_lat, 900_000_000i64),
                    (check_lng_1e7(v), up_lng, 1_800_000_000i64),
                ] {
                    if got == up {
                        checked += 1;
                        continue;
                    }
                    // The only permitted disagreement is D-016: upstream
                    // accepting a value the port rejects, just past the bound.
                    assert!(
                        up && !got,
                        "at {v} the port accepted something upstream rejected, \
                         which is not the registered divergence"
                    );
                    let mag = i64::from(v).abs();
                    assert!(
                        mag > bound,
                        "at {v} the two disagree inside the valid range, which \
                         is not D-016"
                    );
                    diverged.push(v.into());
                }
            }
            "cdconv" => {
                assert_eq!(c.len(), 3, "cdconv row");
                let v = f(&c[0]);
                assert!(same(cd_to_rad(v), f(&c[1])), "cd_to_rad({v})");
                assert!(same(rad_to_cd(v), f(&c[2])), "rad_to_cd({v})");
                checked += 2;
            }
            other => panic!("unhandled section {other}"),
        }
    }

    println!("{checked} location cases matched upstream exactly");
    println!(
        "D-016: {} integer coordinates upstream accepts and the port rejects: {:?}",
        diverged.len(),
        diverged
    );

    assert!(checked > 70, "expected the whole fixture, got {checked}");
    // The divergence must actually be exercised, or the fixture has stopped
    // probing the boundary and the test has quietly become weaker.
    assert!(
        !diverged.is_empty(),
        "the fixture no longer straddles the coordinate bound, so D-016 is untested"
    );
}
