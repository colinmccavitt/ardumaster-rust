//! Parity test: every sensor rotation against upstream `Vector3::rotate`.
//!
//! Exhaustive over the enum's whole numeric range, not just the values that
//! name a rotation, so the port's handling of `ROTATION_MAX`,
//! `ROTATION_CUSTOM_OLD` and `ROTATION_CUSTOM_END` is compared rather than
//! assumed. `ROTATION_CUSTOM_1` and `_2` are the only values skipped: upstream
//! delegates them to a library that is not ported.
//!
//! Values are raw bit patterns, so every comparison is exact. That matters
//! here more than usual — the rotations are generated, and the generator has to
//! reproduce upstream's float promotion precisely. One probe vector pairs a
//! huge component with a tiny one specifically so that "subtract then promote"
//! and "promote then subtract" give different answers; if the generator got
//! that backwards, this test says so.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact comparison against upstream is the point of the test"
)]

use ap_math::rotations_gen::{rotate, BadRotation, Rotation};
use ap_math::vector3::Vector3f;
use std::collections::HashMap;

fn fixture_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/rotations_parity.csv"))
        .expect("workspace root")
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.parse::<u32>().expect("bit pattern"))
}

struct Case {
    rotation: u8,
    probe: usize,
    rejected: bool,
    out: [f32; 3],
}

fn parse(text: &str) -> (Vec<Case>, HashMap<usize, [f32; 3]>) {
    let mut cases = Vec::new();
    let mut probes = HashMap::new();
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
        let c: Vec<&str> = line.split(',').collect();
        match section.as_str() {
            "rot" => {
                assert_eq!(c.len(), 6, "rot row: {line}");
                cases.push(Case {
                    rotation: c[0].parse().expect("rotation"),
                    probe: c[1].parse().expect("probe"),
                    rejected: c[2] == "1",
                    out: [f(c[3]), f(c[4]), f(c[5])],
                });
            }
            "probes" => {
                assert_eq!(c.len(), 4, "probe row: {line}");
                probes.insert(
                    c[0].parse::<usize>().expect("probe"),
                    [f(c[1]), f(c[2]), f(c[3])],
                );
            }
            other => panic!("unhandled section {other}"),
        }
    }
    (cases, probes)
}

#[test]
fn every_rotation_matches_upstream() {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: {} not present", path.display());
        return;
    }
    let (cases, probes) = parse(&std::fs::read_to_string(&path).expect("read fixture"));
    assert!(!cases.is_empty(), "fixture has no cases");

    let mut applied = 0usize;
    let mut rejected = 0usize;
    let mut seen: Vec<u8> = Vec::new();

    for c in &cases {
        let start = *probes.get(&c.probe).expect("probe");
        let mut v = Vector3f::new(start[0], start[1], start[2]);

        match Rotation::from_u8(c.rotation) {
            None => {
                // Not in the enum at all. Upstream's switch falls through to
                // INTERNAL_ERROR and leaves the vector untouched.
                assert!(
                    c.rejected,
                    "upstream accepted {} but it is not in the enum",
                    c.rotation
                );
                assert_eq!(
                    c.out, start,
                    "upstream should leave the vector alone for {}",
                    c.rotation
                );
                rejected += 1;
            }
            Some(r) => {
                let got = rotate(&mut v, r);
                match got {
                    Ok(()) => {
                        assert!(
                            !c.rejected,
                            "port applied {:?} but upstream reported an internal error",
                            r
                        );
                        assert_eq!(
                            [v.x, v.y, v.z],
                            c.out,
                            "{:?} (raw {}) on probe {:?}",
                            r,
                            c.rotation,
                            start
                        );
                        applied += 1;
                        if !seen.contains(&c.rotation) {
                            seen.push(c.rotation);
                        }
                    }
                    Err(BadRotation::NotARotation) => {
                        assert!(c.rejected, "port rejected {:?} but upstream accepted it", r);
                        assert_eq!(
                            [v.x, v.y, v.z],
                            start,
                            "a rejected rotation must leave the vector alone"
                        );
                        rejected += 1;
                    }
                    Err(BadRotation::CustomUnsupported) => {
                        panic!("custom rotations should not be in the fixture")
                    }
                }
            }
        }
    }

    println!(
        "{applied} rotation applications matched upstream exactly across {} distinct \
         rotations; {rejected} non-rotations rejected by both",
        seen.len()
    );

    // Exhaustive means exhaustive: every concrete rotation must have been
    // exercised, not merely most of them.
    assert_eq!(
        seen.len(),
        44,
        "expected all 44 concrete rotations, saw {}",
        seen.len()
    );
    assert!(rejected > 0, "the non-rotation cases were not covered");
}

/// Every rotation must preserve length — an independent check on the generated
/// switch that does not consult upstream at all.
///
/// A parity test agrees with upstream even where upstream is wrong; this says
/// the transformations are genuinely rotations. Together the two are much
/// stronger than either alone.
#[test]
fn every_rotation_preserves_length() {
    for raw in 0..=103u8 {
        let Some(r) = Rotation::from_u8(raw) else {
            continue;
        };
        if matches!(r, Rotation::Custom1 | Rotation::Custom2) {
            continue;
        }
        let mut v = Vector3f::new(3.0, -4.0, 12.0);
        let before = v.length();
        if rotate(&mut v, r).is_err() {
            continue; // the non-rotations
        }
        let after = v.length();
        assert!(
            (after - before).abs() < 1e-4,
            "{r:?} changed length from {before} to {after}"
        );
    }
}

/// The enum's discriminants must match upstream's, since drivers and
/// parameters carry the raw value and MAVLink's `MAV_SENSOR_ORIENTATION` is
/// expected to agree.
///
/// Written as literals from `rotations.h` rather than through the port's own
/// constants — the FlightStage bug in `ap-tecs` was a test that asserted the
/// port against itself and passed while every value was wrong.
#[test]
fn discriminants_match_upstream() {
    assert_eq!(Rotation::None as u8, 0);
    assert_eq!(Rotation::Yaw45 as u8, 1);
    assert_eq!(Rotation::Yaw90 as u8, 2);
    assert_eq!(Rotation::Yaw180 as u8, 4);
    assert_eq!(Rotation::Roll180 as u8, 8);
    assert_eq!(Rotation::Pitch180 as u8, 12);
    assert_eq!(Rotation::Roll45 as u8, 42);
    assert_eq!(Rotation::Roll315 as u8, 43);
    // implicit values, which follow the preceding entry
    assert_eq!(
        Rotation::Max as u8,
        44,
        "ROTATION_MAX has no explicit value"
    );
    assert_eq!(Rotation::CustomOld as u8, 100);
    assert_eq!(Rotation::Custom1 as u8, 101);
    assert_eq!(Rotation::Custom2 as u8, 102);
    assert_eq!(
        Rotation::CustomEnd as u8,
        103,
        "ROTATION_CUSTOM_END has no explicit value either"
    );

    // and values between the blocks name nothing
    assert_eq!(Rotation::from_u8(50), None);
    assert_eq!(Rotation::from_u8(99), None);
    assert_eq!(Rotation::from_u8(104), None);
}
