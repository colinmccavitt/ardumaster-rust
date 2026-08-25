//! Parity test: `SplineCurve` against upstream, through the public API.
//!
//! Deliberately public-only. The internals could be reached the way the SCurve
//! harness reaches SCurve's, but `update_solution` and `calc_target_pos_vel`
//! read and write `_hermite_solution` — a member — so that would put a
//! class-layout assumption in the middle of the test. The public surface is
//! the whole observable behaviour anyway.
//!
//! Each scenario sets limits and a segment, checks the two speed maxima the
//! setup computed, then walks the curve to completion comparing position and
//! velocity at every step. Walking it is what exercises `calc_dt_speed_max`,
//! including the braking clamp whose behaviour differs between the setup and
//! stepping call sites because upstream's out-parameter aliases a member at
//! the former.
//!
//! Positions are compared as `f64` because that is what they are; velocities
//! as `f32`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::spline::{SplineCurve, Vector3p};
use ap_math::vector3::Vector3f;

struct Scenario {
    name: &'static str,
    limits: (f32, f32, f32, f32, f32),
    origin: (f64, f64, f64),
    dest: (f64, f64, f64),
    origin_vel: (f32, f32, f32),
    dest_vel: (f32, f32, f32),
    start_speed: f32,
}

const fn s(
    name: &'static str,
    limits: (f32, f32, f32, f32, f32),
    origin: (f64, f64, f64),
    dest: (f64, f64, f64),
    origin_vel: (f32, f32, f32),
    dest_vel: (f32, f32, f32),
    start_speed: f32,
) -> Scenario {
    Scenario {
        name,
        limits,
        origin,
        dest,
        origin_vel,
        dest_vel,
        start_speed,
    }
}

/// Mirrors the table in `tools/parity/gen_spline_fixture.py`.
const SCENARIOS: &[Scenario] = &[
    s(
        "straight",
        (500.0, 250.0, 150.0, 250.0, 100.0),
        (0.0, 0.0, 0.0),
        (2000.0, 0.0, 0.0),
        (200.0, 0.0, 0.0),
        (200.0, 0.0, 0.0),
        200.0,
    ),
    s(
        "right_turn",
        (500.0, 250.0, 150.0, 250.0, 100.0),
        (0.0, 0.0, 0.0),
        (2000.0, 2000.0, 0.0),
        (300.0, 0.0, 0.0),
        (0.0, 300.0, 0.0),
        300.0,
    ),
    s(
        "climb",
        (500.0, 250.0, 150.0, 250.0, 100.0),
        (0.0, 0.0, 0.0),
        (2000.0, 0.0, -500.0),
        (200.0, 0.0, 0.0),
        (200.0, 0.0, -50.0),
        200.0,
    ),
    s(
        "descend",
        (500.0, 250.0, 150.0, 250.0, 100.0),
        (0.0, 0.0, 0.0),
        (2000.0, 0.0, 500.0),
        (200.0, 0.0, 0.0),
        (200.0, 0.0, 50.0),
        200.0,
    ),
    s(
        "short_leg",
        (500.0, 250.0, 150.0, 250.0, 100.0),
        (0.0, 0.0, 0.0),
        (100.0, 0.0, 0.0),
        (500.0, 0.0, 0.0),
        (500.0, 0.0, 0.0),
        100.0,
    ),
    s(
        "stop_at_end",
        (500.0, 250.0, 150.0, 250.0, 100.0),
        (0.0, 0.0, 0.0),
        (1500.0, 500.0, 0.0),
        (250.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        250.0,
    ),
    s(
        "zero_length",
        (500.0, 250.0, 150.0, 250.0, 100.0),
        (100.0, 200.0, -50.0),
        (100.0, 200.0, -50.0),
        (100.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        100.0,
    ),
    s(
        "tight",
        (500.0, 250.0, 150.0, 250.0, 100.0),
        (0.0, 0.0, 0.0),
        (500.0, 50.0, 0.0),
        (400.0, 0.0, 0.0),
        (-400.0, 0.0, 0.0),
        400.0,
    ),
    s(
        "slow_limits",
        (100.0, 50.0, 50.0, 50.0, 25.0),
        (0.0, 0.0, 0.0),
        (1000.0, 1000.0, 0.0),
        (80.0, 0.0, 0.0),
        (0.0, 80.0, 0.0),
        80.0,
    ),
];

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/spline_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_spline_fixture.py",
            path.display()
        )
    })
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

fn d(s: &str) -> f64 {
    f64::from_bits(s.trim().parse::<u64>().expect("bit pattern"))
}

#[test]
fn the_spline_matches_upstream_step_for_step() {
    let text = fixture();
    let mut rows: Vec<Vec<&str>> = Vec::new();
    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("scenario,") {
            continue;
        }
        rows.push(line.split(',').collect());
    }

    let mut checked = 0_usize;
    let mut worst_pos = 0.0_f64;
    let mut worst_vel = 0.0_f64;
    let mut worst_where = String::new();

    for sc in SCENARIOS {
        let mut curve = SplineCurve::new();
        let (sxy, sup, sdn, axy, az) = sc.limits;
        curve.set_speed_accel(sxy, sup, sdn, axy, az);
        curve.set_origin_and_destination(
            Vector3p::new(sc.origin.0, sc.origin.1, sc.origin.2),
            Vector3p::new(sc.dest.0, sc.dest.1, sc.dest.2),
            Vector3f::new(sc.origin_vel.0, sc.origin_vel.1, sc.origin_vel.2),
            Vector3f::new(sc.dest_vel.0, sc.dest_vel.1, sc.dest_vel.2),
        );

        let mine: Vec<&Vec<&str>> = rows.iter().filter(|r| r[0] == sc.name).collect();
        assert!(!mine.is_empty(), "{}: nothing in the fixture", sc.name);

        // Step -1 carries the two speed maxima the setup computed.
        let setup = mine[0];
        assert_eq!(setup[1], "-1", "{}: expected the setup row first", sc.name);
        let want_origin_speed = f(setup[2]);
        let want_dest_speed = f(setup[3]);
        assert!(
            (curve.origin_speed_max() - want_origin_speed).abs()
                <= 1.0e-4 * want_origin_speed.abs().max(1.0),
            "{}: origin_speed_max {} against upstream {want_origin_speed}",
            sc.name,
            curve.origin_speed_max()
        );
        assert!(
            (curve.destination_speed_max() - want_dest_speed).abs()
                <= 1.0e-4 * want_dest_speed.abs().max(1.0),
            "{}: destination_speed_max {} against upstream {want_dest_speed} — \
             if this is zero against a non-zero upstream, the out-parameter \
             aliasing in calc_dt_speed_max has not been reproduced",
            sc.name,
            curve.destination_speed_max()
        );
        assert_eq!(
            curve.reached_destination(),
            setup[8] == "1",
            "{}: reached_destination after setup",
            sc.name
        );
        checked += 1;

        let mut vel = Vector3f::new(sc.origin_vel.0, sc.origin_vel.1, sc.origin_vel.2);
        if vel.length() > 0.0 {
            if let Some(unit) = vel.normalized() {
                vel = unit * sc.start_speed;
            }
        }

        for row in mine.iter().skip(1) {
            assert_eq!(row.len(), 9);
            let target = curve.advance_target_along_track(0.01, vel);
            vel = target.velocity;

            let want_pos = Vector3p::new(d(row[2]), d(row[3]), d(row[4]));
            let want_vel = Vector3f::new(f(row[5]), f(row[6]), f(row[7]));

            let pos_err = (target.position - want_pos).length();
            let vel_err = f64::from((vel - want_vel).length());
            let scale = want_pos.length().max(1.0);

            assert!(
                pos_err / scale < 1.0e-6,
                "{} step {}: position {:?} against upstream {want_pos:?}",
                sc.name,
                row[1],
                target.position
            );
            assert!(
                vel_err < 1.0e-3 * f64::from(want_vel.length()).max(1.0),
                "{} step {}: velocity {vel:?} against upstream {want_vel:?}",
                sc.name,
                row[1]
            );

            if pos_err / scale > worst_pos {
                worst_pos = pos_err / scale;
                worst_where = format!("{} step {}", sc.name, row[1]);
            }
            worst_vel = worst_vel.max(vel_err);

            assert_eq!(
                curve.reached_destination(),
                row[8] == "1",
                "{} step {}: reached_destination",
                sc.name,
                row[1]
            );
            checked += 1;
        }

        assert!(
            curve.reached_destination(),
            "{}: should have arrived by the end of the fixture",
            sc.name
        );
    }

    assert!(checked > 4000, "fixture looks truncated: {checked} rows");
    println!(
        "{checked} steps across {} scenarios; worst relative position error {worst_pos:e} ({worst_where}), worst velocity error {worst_vel:e}",
        SCENARIOS.len()
    );
}
