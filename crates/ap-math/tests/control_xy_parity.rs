//! The 2-D shaping family against the real firmware.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::control::{
    limit_accel_corner_xy, limit_accel_xy, shape_accel_xy, shape_accel_xy_3d, shape_vel_accel_xy,
    update_pos_vel_accel_xy, update_vel_accel_xy, Postype,
};
use ap_math::vector2::{Vector2, Vector2f};
use ap_math::vector3::Vector3f;

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn d(s: &str) -> Postype {
    s.trim().parse().expect("decimal")
}

const TOL: f32 = 3e-5;

fn sections() -> std::collections::HashMap<String, Vec<Vec<String>>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/control_xy.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out: std::collections::HashMap<String, Vec<Vec<String>>> = Default::default();
    let mut current = String::new();
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag.to_owned();
            continue;
        }
        if line.is_empty() || line.chars().next().is_some_and(char::is_alphabetic) {
            continue;
        }
        out.entry(current.clone())
            .or_default()
            .push(line.split(',').map(str::to_owned).collect());
    }
    out
}

/// Both acceleration limiters, swept over the angle between velocity and
/// acceleration.
///
/// That angle is what they decide on. `limit_accel_xy` always gives up
/// along-track to keep cross-track — path over schedule. `limit_accel_corner_xy`
/// makes the same trade only while accelerating; under braking it reverses and
/// keeps the deceleration instead, because a vehicle that asked to slow down
/// usually has a reason and holding a curve matters least exactly then.
///
/// The sweep crosses 90 degrees, where the along-track component changes sign
/// and the cornering limiter switches regime.
#[test]
fn the_acceleration_limiters_match_upstream() {
    let s = sections();
    let rows = s.get("limits").expect("limits section");

    let mut largest = 0.0_f32;
    let mut plain_fired = 0_usize;
    let mut corner_fired = 0_usize;
    let mut disagreed = 0_usize;

    for r in rows {
        assert_eq!(r.len(), 12, "malformed limits row");
        let idx: usize = r[0].parse().expect("idx");
        let vel = Vector2f::new(f(&r[1]), f(&r[2]));
        let acc = Vector2f::new(f(&r[3]), f(&r[4]));
        let accel_max = f(&r[5]);

        let mut plain = acc;
        let plain_ret = limit_accel_xy(vel, &mut plain, accel_max);
        let mut corner = acc;
        let corner_ret = limit_accel_corner_xy(vel, &mut corner, accel_max);

        for (label, got, want) in [
            ("plain_x", plain.x, f(&r[6])),
            ("plain_y", plain.y, f(&r[7])),
            ("corner_x", corner.x, f(&r[9])),
            ("corner_y", corner.y, f(&r[10])),
        ] {
            let diff = (got - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < TOL,
                "row {idx} {label}: {got} != upstream {want} (diff {diff})"
            );
        }
        assert_eq!(plain_ret, r[8].trim() == "1", "row {idx} plain return");
        assert_eq!(corner_ret, r[11].trim() == "1", "row {idx} corner return");

        if plain_ret {
            plain_fired += 1;
        }
        if corner_ret {
            corner_fired += 1;
        }
        if (plain.x - corner.x).abs() > 1e-6 || (plain.y - corner.y).abs() > 1e-6 {
            disagreed += 1;
        }
    }

    // If the two never diverged the sweep would pass with one implementing
    // the other, which is the mistake worth catching.
    assert!(
        disagreed > 20,
        "the two limiters only differed on {disagreed} rows; the braking \
         regime is barely covered"
    );
    assert!(
        plain_fired > 50 && corner_fired > 50,
        "both limiters must actually engage ({plain_fired}, {corner_fired})"
    );

    println!(
        "{} limiter rows, largest difference {largest:e}, they differ on {disagreed}",
        rows.len()
    );
}

/// The two shapers, stepped, with a demand that turns a corner partway.
///
/// Both carry acceleration forward, so what is compared is the trajectory
/// rather than any single value. The demand turns ninety degrees at 0.5 s and
/// again at 1.0 s, which swings the shaped acceleration through the cornering
/// limiter's regimes with real state behind it.
#[test]
fn the_shapers_match_upstream() {
    let s = sections();
    let rows = s.get("shape").expect("shape section");

    let mut accel_a = Vector2f::new(0.0, 0.0);
    let mut accel_b = Vector2f::new(0.0, 0.0);
    let mut accel_3d = Vector3f::new(0.0, 0.0, -7.5);
    let mut largest = 0.0_f32;
    let mut both_limits = [0_usize; 2];
    let mut linear_region = 0_usize;

    for r in rows {
        assert_eq!(r.len(), 20, "malformed shape row");
        let step: usize = r[0].parse().expect("step");
        let dt = f(&r[1]);
        let accel_max = f(&r[2]);
        let jerk_max = f(&r[3]);
        let vel_des = Vector2f::new(f(&r[4]), f(&r[5]));
        let accel_ff = Vector2f::new(f(&r[6]), f(&r[7]));
        let vel = Vector2f::new(f(&r[8]), f(&r[9]));
        let limit_total = r[10].trim() == "1";
        both_limits[usize::from(limit_total)] += 1;

        shape_accel_xy(vel_des, &mut accel_a, jerk_max, dt);
        shape_accel_xy_3d(
            Vector3f::new(vel_des.x, vel_des.y, 99.0),
            &mut accel_3d,
            jerk_max,
            dt,
        );

        // Track whether the correction is inside its bound. While it
        // saturates, the gain and the length normalisation are invisible --
        // any wrong value clips to the same answer.
        if (vel_des - vel).length() < 1.0 {
            linear_region += 1;
        }
        shape_vel_accel_xy(
            vel_des,
            accel_ff,
            vel,
            &mut accel_b,
            accel_max,
            jerk_max,
            dt,
            limit_total,
        );

        for (label, got, want) in [
            ("sa_x", accel_a.x, f(&r[11])),
            ("sa_y", accel_a.y, f(&r[12])),
            ("sva_x", accel_b.x, f(&r[13])),
            ("sva_y", accel_b.y, f(&r[14])),
            ("3d_x", accel_3d.x, f(&r[15])),
            ("3d_y", accel_3d.y, f(&r[16])),
            ("3d_z", accel_3d.z, f(&r[17])),
        ] {
            let diff = (got - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < TOL,
                "step {step} {label}: {got} != upstream {want} (diff {diff})"
            );
        }
    }

    assert!(
        both_limits[0] > 100 && both_limits[1] > 100,
        "limit_total_accel must be exercised both ways, got {both_limits:?}"
    );

    // Without this the sequence proves nothing about the controller's gain:
    // a saturated correction is the same value whatever produced it.
    assert!(
        linear_region > 50,
        "only {linear_region} steps had the correction inside its bound; the \
         sqrt controller never left saturation and its gain is untested"
    );

    // The three-dimensional form must have left z exactly alone.
    assert!(
        (accel_3d.z + 7.5).abs() < 1e-9,
        "shape_accel_xy_3d must not touch the vertical axis, z is {}",
        accel_3d.z
    );

    println!(
        "{} shaper steps, largest difference {largest:e}",
        rows.len()
    );
}

/// The forward-projection helpers and their directional suppression.
///
/// A non-zero `limit` says the vehicle cannot go further that way. The
/// velocity step is dropped only when the step, the error, and the current
/// velocity all point along it — any one pointing back means the step is
/// helping. The third test is `!is_negative` rather than `is_positive`, so a
/// velocity of exactly zero still counts as "not moving away" and a
/// stationary vehicle at a limit stays suppressed.
///
/// Position is `Postype`, compared as such: it accumulates for the whole
/// flight, and a float would start losing centimetres a few kilometres out.
#[test]
fn the_forward_projection_matches_upstream() {
    let s = sections();
    let rows = s.get("update").expect("update section");

    let mut largest = 0.0_f32;
    let mut largest_pos = 0.0_f64;
    let mut suppressed_vel = 0_usize;
    let mut suppressed_pos = 0_usize;

    for r in rows {
        assert_eq!(r.len(), 20, "malformed update row");
        let idx: usize = r[0].parse().expect("idx");
        let dt = f(&r[1]);
        let vel0 = Vector2f::new(f(&r[4]), f(&r[5]));
        let accel = Vector2f::new(f(&r[6]), f(&r[7]));
        let limit = Vector2f::new(f(&r[8]), f(&r[9]));
        let pos_error = Vector2f::new(f(&r[10]), f(&r[11]));
        let vel_error = Vector2f::new(f(&r[12]), f(&r[13]));

        let mut vel_a = vel0;
        update_vel_accel_xy(&mut vel_a, accel, dt, limit, vel_error);

        let mut pos_b = Vector2::new(d(&r[2]), d(&r[3]));
        let mut vel_b = vel0;
        update_pos_vel_accel_xy(
            &mut pos_b, &mut vel_b, accel, dt, limit, pos_error, vel_error,
        );

        for (label, got, want) in [
            ("uva_vel_x", vel_a.x, f(&r[14])),
            ("uva_vel_y", vel_a.y, f(&r[15])),
            ("upva_vel_x", vel_b.x, f(&r[18])),
            ("upva_vel_y", vel_b.y, f(&r[19])),
        ] {
            let diff = (got - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < TOL,
                "row {idx} {label}: {got} != upstream {want} (diff {diff})"
            );
        }

        for (label, got, want) in [
            ("upva_pos_x", pos_b.x, d(&r[16])),
            ("upva_pos_y", pos_b.y, d(&r[17])),
        ] {
            let diff = (got - want).abs();
            largest_pos = largest_pos.max(diff);
            assert!(
                diff < 1e-12,
                "row {idx} {label}: {got} != upstream {want} (diff {diff})"
            );
        }

        // Was the step suppressed? Compare against the unlimited projection.
        let mut free = vel0;
        update_vel_accel_xy(&mut free, accel, dt, Vector2f::new(0.0, 0.0), vel_error);
        if (free.x - vel_a.x).abs() > 1e-9 || (free.y - vel_a.y).abs() > 1e-9 {
            suppressed_vel += 1;
        }
        if (pos_b.x - d(&r[2])).abs() < 1e-15 && (pos_b.y - d(&r[3])).abs() < 1e-15 {
            suppressed_pos += 1;
        }
    }

    // Both suppressions must fire and both must sometimes not, or the sweep
    // passes with the whole mechanism removed.
    assert!(
        suppressed_vel > 5 && suppressed_vel < rows.len() - 5,
        "velocity suppression fired on {suppressed_vel} of {}",
        rows.len()
    );
    assert!(
        suppressed_pos > 5 && suppressed_pos < rows.len() - 5,
        "position suppression fired on {suppressed_pos} of {}",
        rows.len()
    );

    println!(
        "{} projection rows, largest velocity difference {largest:e}, \
         position {largest_pos:e}; suppression fired on {suppressed_vel} velocity \
         and {suppressed_pos} position steps",
        rows.len()
    );
}

/// A nonsensical acceleration or jerk limit leaves the command untouched.
///
/// Upstream raises an internal error and returns. The port returns quietly,
/// which is a difference in *reporting* rather than in the value — and it
/// cannot be recorded, because raising that error aborts the harness.
///
/// Both halves of the guard are checked separately, so neither can be dropped.
#[test]
fn a_degenerate_limit_leaves_the_command_alone() {
    let vel_des = Vector2f::new(5.0, -3.0);
    let accel_ff = Vector2f::new(0.5, 0.25);
    let vel = Vector2f::new(1.0, 1.0);
    let start = Vector2f::new(1.25, -0.75);

    for (accel_max, jerk_max) in [(0.0, 30.0), (6.0, 0.0), (0.0, 0.0), (-6.0, 30.0)] {
        let mut accel = start;
        shape_vel_accel_xy(
            vel_des, accel_ff, vel, &mut accel, accel_max, jerk_max, 0.0025, true,
        );
        assert_eq!(
            (accel.x, accel.y),
            (start.x, start.y),
            "accel_max {accel_max}, jerk_max {jerk_max} should have been refused"
        );
    }

    // And a sane pair does move it, so the test is not passing because
    // nothing ever shapes.
    let mut accel = start;
    shape_vel_accel_xy(vel_des, accel_ff, vel, &mut accel, 6.0, 30.0, 0.0025, true);
    assert_ne!(
        (accel.x, accel.y),
        (start.x, start.y),
        "a valid pair must shape the command"
    );
}
