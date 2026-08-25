//! Parity test: `AP_Math/control.cpp`'s scalar shaping against upstream.
//!
//! Grids over every branch rather than a plausible trajectory. `sqrt_controller`
//! has three top-level branches and two sub-branches inside the third, and the
//! join between the linear and square-root regions is where a transcription
//! error would be least visible and most damaging — the response would still
//! look sensible either side of it.
//!
//! The closed-loop functions are driven as loops too: a thousand steps of
//! `shape_pos_vel_accel` feeding `update_pos_vel_accel`. Their state compounds,
//! so a per-step difference too small to see in isolation shows up there.
//!
//! Only valid arguments are driven. Upstream answers a malformed limit with
//! `INTERNAL_ERROR` and an untouched output, so there is nothing numeric to
//! compare — the harness stubs that to abort, and the port's own tests cover
//! that it refuses and leaves the value alone.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::vector3::Vector3f;

use ap_math::control::{
    inv_sqrt_controller, kinematic_limit, shape_accel, shape_angle_vel_accel, shape_pos_vel_accel,
    shape_vel_accel, sqrt_controller, sqrt_controller_accel, stopping_distance,
    update_pos_vel_accel, update_vel_accel, Postype,
};

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/control_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_control_fixture.py",
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

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

#[test]
fn the_control_shaping_matches_upstream() {
    let text = fixture();

    let mut counts: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
    let mut exact = 0_usize;
    let mut checked = 0_usize;
    let mut worst = 0.0_f64;
    let mut worst_where = String::new();

    // Closed-loop state, advanced in step with the fixture's own loops.
    let mut loop_vel = (0.0_f32, 0.0_f32);
    let mut loop_pos = (0.0_f64, 0.0_f32, 0.0_f32);
    let mut loop_angle = (3.0_f32, 0.0_f32, 0.0_f32);

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("kind,") {
            continue;
        }
        let r: Vec<&str> = line.split(',').collect();
        assert_eq!(r.len(), 9, "malformed row: {line}");
        let kind = r[0];
        *counts.entry(kind).or_default() += 1;

        // (label, got, want) pairs for this row.
        let mut pairs: Vec<(&str, f32, f32)> = Vec::new();

        match kind {
            "sqrt" => {
                pairs.push((
                    "sqrt_controller",
                    sqrt_controller(f(r[1]), f(r[2]), f(r[3]), f(r[4])),
                    f(r[7]),
                ));
            }
            "inv" => {
                pairs.push((
                    "inv_sqrt_controller",
                    inv_sqrt_controller(f(r[1]), f(r[2]), f(r[3])),
                    f(r[7]),
                ));
                pairs.push((
                    "stopping_distance",
                    stopping_distance(f(r[1]), f(r[2]), f(r[3])),
                    f(r[8]),
                ));
            }
            "sqrt_accel" => {
                pairs.push((
                    "sqrt_controller_accel",
                    sqrt_controller_accel(f(r[1]), f(r[2]), f(r[3]), f(r[4]), f(r[5])),
                    f(r[7]),
                ));
            }
            "upd_vel" => {
                let mut vel = f(r[1]);
                update_vel_accel(&mut vel, f(r[2]), 0.02, f(r[3]), f(r[4]));
                pairs.push(("update_vel_accel", vel, f(r[7])));
            }
            "upd_pos" => {
                // Position is compared as Postype, not as a float: it
                // accumulates for a whole flight and the type exists to stop
                // that drifting.
                let mut pos: Postype = 7.5;
                let mut vel = f(r[1]);
                update_pos_vel_accel(&mut pos, &mut vel, f(r[2]), 0.02, f(r[3]), f(r[4]), f(r[5]));
                pairs.push(("update_pos_vel_accel_vel", vel, f(r[7])));
                let want: Postype = r[8].trim().parse().expect("position");
                assert!(
                    (pos - want).abs() < 1e-12,
                    "update_pos_vel_accel position: {pos} != upstream {want}"
                );
            }
            "shape_accel" => {
                let mut accel = f(r[2]);
                shape_accel(f(r[1]), &mut accel, 5.0, 0.02).expect("valid");
                pairs.push(("shape_accel", accel, f(r[7])));
            }
            "kin" => {
                let dir = Vector3f::new(f(r[1]), f(r[2]), f(r[3]));
                pairs.push((
                    "kinematic_limit",
                    kinematic_limit(dir, f(r[4]), f(r[5]), f(r[6])),
                    f(r[7]),
                ));
            }
            "loop_vel" => {
                shape_vel_accel(
                    5.0,
                    0.0,
                    loop_vel.0,
                    &mut loop_vel.1,
                    -3.0,
                    3.0,
                    10.0,
                    0.01,
                    true,
                )
                .expect("valid");
                update_vel_accel(&mut loop_vel.0, loop_vel.1, 0.01, 0.0, 0.0);
                pairs.push(("loop vel", loop_vel.0, f(r[7])));
                pairs.push(("loop vel accel", loop_vel.1, f(r[8])));
            }
            "loop_pos" => {
                shape_pos_vel_accel(
                    100.0,
                    0.0,
                    0.0,
                    loop_pos.0,
                    loop_pos.1,
                    &mut loop_pos.2,
                    -10.0,
                    10.0,
                    -3.0,
                    3.0,
                    10.0,
                    0.01,
                    true,
                )
                .expect("valid");
                update_pos_vel_accel(
                    &mut loop_pos.0,
                    &mut loop_pos.1,
                    loop_pos.2,
                    0.01,
                    0.0,
                    0.0,
                    0.0,
                );
                // Position is a double; compared at that width.
                let want_pos = d(r[7]);
                let rel = ((loop_pos.0 - want_pos) / want_pos.abs().max(1.0)).abs();
                assert!(
                    rel < 1.0e-9,
                    "loop position at step {}: {} against upstream {want_pos}",
                    r[1],
                    loop_pos.0
                );
                checked += 1;
                if loop_pos.0.to_bits() == want_pos.to_bits() {
                    exact += 1;
                }
                pairs.push(("loop pos accel", loop_pos.2, f(r[8])));
            }
            "loop_angle" => {
                shape_angle_vel_accel(
                    -3.0,
                    0.0,
                    0.0,
                    loop_angle.0,
                    loop_angle.1,
                    &mut loop_angle.2,
                    -1.0,
                    1.0,
                    2.0,
                    5.0,
                    0.01,
                    true,
                )
                .expect("valid");
                let mut p = f64::from(loop_angle.0);
                update_pos_vel_accel(&mut p, &mut loop_angle.1, loop_angle.2, 0.01, 0.0, 0.0, 0.0);
                #[allow(
                    clippy::cast_possible_truncation,
                    reason = "the harness narrows the postype back to float the same way"
                )]
                {
                    loop_angle.0 = p as f32;
                }
                pairs.push(("loop angle", loop_angle.0, f(r[7])));
                pairs.push(("loop angle accel", loop_angle.2, f(r[8])));
            }
            other => panic!("unknown fixture kind {other}"),
        }

        for (label, got, want) in pairs {
            if same(got, want) {
                exact += 1;
            } else {
                let denom = f64::from(want).abs().max(1.0e-3);
                let rel = ((f64::from(got) - f64::from(want)) / denom).abs();
                assert!(
                    rel < 1.0e-5,
                    "{label} in row `{line}`: {got} against upstream {want}"
                );
                if rel > worst {
                    worst = rel;
                    worst_where = label.to_owned();
                }
            }
            checked += 1;
        }
    }

    assert!(
        checked > 25_000,
        "fixture looks truncated: {checked} values"
    );
    for kind in [
        "sqrt",
        "inv",
        "sqrt_accel",
        "upd_vel",
        "upd_pos",
        "shape_accel",
        "loop_vel",
        "loop_pos",
        "loop_angle",
        "kin",
    ] {
        assert!(
            counts.get(kind).copied().unwrap_or(0) > 0,
            "the fixture is missing every {kind} row"
        );
    }

    println!(
        "{checked} values, {exact} bit-exact ({:.2}%); worst relative {worst:e} {worst_where}",
        100.0 * exact as f64 / checked as f64
    );
}
