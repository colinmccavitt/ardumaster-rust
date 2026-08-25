//! Parity test: every multirotor frame table against upstream.
//!
//! The frame tables are ~700 lines of angles behind build-time conditionals, so
//! the port generates them from the C++ (`tools/parity/gen_frames.py`) rather
//! than transcribing them. That turns the risk from "someone mistyped an angle"
//! into "the generator misparsed something", which is the risk this test is
//! here to close.
//!
//! The fixture is not read off the source. It is the factor array dumped from
//! the compiled ArduCopter object after `setup_motors`, swept across every
//! frame class and every frame type. So the two sides come from genuinely
//! different places: one from parsing text, one from running code.
//!
//! # What the sweep covers
//!
//! Classes 0..=17 and types 0..=25 — past the end of `motor_frame_type`, which
//! stops at 21. The out-of-range types are not padding. A class that answers
//! for a type the enum does not define is a class with a productive `default:`
//! branch, and that is exactly how Y6's fallback layout was found: it answers
//! for all 24 types it does not name.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_motors::{MotorMatrix, MAX_NUM_MOTORS};

const CLASSES: std::ops::RangeInclusive<u8> = 0..=17;
const TYPES: std::ops::RangeInclusive<u8> = 0..=25;

struct Row {
    motor: usize,
    roll: f32,
    pitch: f32,
    yaw: f32,
    throttle: f32,
    order: u8,
}

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/motors_frames.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_motors_fixture.py",
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
fn every_frame_matches_upstream() {
    let text = fixture();

    // (class, type) -> rows, absent entry meaning upstream built nothing.
    let mut want: std::collections::BTreeMap<(u8, u8), Vec<Row>> =
        std::collections::BTreeMap::new();
    let mut seen: std::collections::BTreeSet<(u8, u8)> = std::collections::BTreeSet::new();

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("class,") {
            continue;
        }
        let c: Vec<&str> = line.split(',').collect();
        assert_eq!(c.len(), 9, "malformed fixture row: {line}");
        let key = (
            c[0].parse::<u8>().expect("class"),
            c[1].parse::<u8>().expect("type"),
        );
        seen.insert(key);
        if c[2] != "1" {
            continue;
        }
        want.entry(key).or_default().push(Row {
            motor: c[3].parse().expect("motor"),
            roll: f(c[4]),
            pitch: f(c[5]),
            yaw: f(c[6]),
            throttle: f(c[7]),
            order: c[8].parse().expect("order"),
        });
    }

    // The sweep in the test and the sweep in the harness must agree, or a whole
    // region of the space would go unchecked without anything saying so.
    let expected: std::collections::BTreeSet<(u8, u8)> =
        CLASSES.flat_map(|c| TYPES.map(move |t| (c, t))).collect();
    assert_eq!(
        seen, expected,
        "the fixture sweep and the test sweep cover different ranges"
    );

    let mut frames = 0_usize;
    let mut values = 0_usize;

    for class in CLASSES {
        for ty in TYPES {
            let mut m = MotorMatrix::new();
            let ok = m.setup_motors(class, ty);
            let rows = want.get(&(class, ty));

            assert_eq!(
                ok,
                rows.is_some(),
                "class {class} type {ty}: port says supported={ok}, upstream \
                 built {} motors",
                rows.map_or(0, Vec::len)
            );

            let Some(rows) = rows else {
                // Unsupported: every slot must be cleared, not merely unused.
                for i in 0..MAX_NUM_MOTORS {
                    assert!(
                        !m.is_enabled(i),
                        "class {class} type {ty}: motor {i} left enabled"
                    );
                }
                continue;
            };
            frames += 1;

            assert_eq!(
                m.num_motors(),
                rows.len(),
                "class {class} type {ty}: motor count"
            );

            for r in rows {
                assert!(
                    m.is_enabled(r.motor),
                    "class {class} type {ty}: motor {} should be enabled",
                    r.motor
                );
                let got = m.motor(r.motor).expect("enabled motor has factors");
                for (label, g, w) in [
                    ("roll", got.roll, r.roll),
                    ("pitch", got.pitch, r.pitch),
                    ("yaw", got.yaw, r.yaw),
                    ("throttle", got.throttle, r.throttle),
                ] {
                    assert!(
                        same(g, w),
                        "class {class} type {ty} motor {} {label}: \
                         {g} ({:#010x}) != upstream {w} ({:#010x})",
                        r.motor,
                        g.to_bits(),
                        w.to_bits()
                    );
                    values += 1;
                }
                assert_eq!(
                    m.test_order(r.motor),
                    Some(r.order),
                    "class {class} type {ty} motor {}: test order",
                    r.motor
                );
                values += 1;
            }
        }
    }

    assert_eq!(frames, 64, "the Copter build defines 64 frames");
    println!("{frames} frames, {values} values, all bit-exact");
}

/// Y6 answers for frame types it never names, and the others refuse.
///
/// This is the one place a class disagrees with its siblings, and it is easy to
/// lose in a regeneration: a `default:` that stopped being parsed would turn 24
/// working frames into refusals, and every one of those is a vehicle that would
/// no longer arm.
#[test]
fn only_y6_falls_back_for_unknown_frame_types() {
    const UNNAMED: u8 = 25; // past the end of motor_frame_type

    let mut y6 = MotorMatrix::new();
    assert!(y6.setup_motors(5, UNNAMED), "Y6 should fall back");
    assert_eq!(y6.num_motors(), 6);

    for class in [1_u8, 2, 3, 4, 12, 14] {
        let mut m = MotorMatrix::new();
        assert!(
            !m.setup_motors(class, UNNAMED),
            "class {class} should refuse an unknown frame type"
        );
        assert_eq!(m.num_motors(), 0, "class {class} left motors behind");
    }
}
