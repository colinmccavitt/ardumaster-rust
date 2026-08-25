//! The landing descent demand, against the real firmware.
//!
//! The demand is a local variable inside `Mode::land_run_vertical_control`,
//! so the recording intercepts the call it is passed to rather than reading
//! any stored state — see `tools/parity/gen_land_descent.py`. What is compared
//! is the actual argument the firmware handed its position controller.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::land::{land_descent, LandDescentConfig};

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/land_descent.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter(|l| !l.starts_with(|c: char| c.is_alphabetic()))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

#[test]
fn the_landing_descent_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut largest = 0.0_f32;
    let mut distinct = std::collections::BTreeSet::new();
    let mut lifted = 0_usize;
    let mut paused = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 13, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let pause = b(&r[1]);
        let config = LandDescentConfig {
            land_alt_low_m: f(&r[3]),
            land_speed_high_ms: f(&r[4]),
            land_speed_ms: f(&r[5]),
            max_speed_down_ms: f(&r[6]),
            pos_p_kp: f(&r[7]),
            max_accel_mss: f(&r[8]),
        };

        let got = land_descent(pause, f(&r[2]), b(&r[10]), &config, f(&r[9]));

        let want_rate = f(&r[11]);
        let want_lift = b(&r[12]);

        let diff = (got.climb_rate_ms - want_rate).abs();
        largest = largest.max(diff);
        assert!(
            diff < 1e-6,
            "row {idx}: climb rate {} against upstream {want_rate} (diff {diff}), \
             alt {} config {config:?}",
            got.climb_rate_ms,
            f(&r[2])
        );
        assert_eq!(
            got.ignore_descent_limit, want_lift,
            "row {idx}: descent limit lift {} against upstream {want_lift}",
            got.ignore_descent_limit
        );

        distinct.insert(want_rate.to_bits());
        if want_lift {
            lifted += 1;
        }
        if pause {
            paused += 1;
        }
    }

    // A sweep that produced one answer, or never took a branch, would pin far
    // less than its row count suggests.
    assert!(
        distinct.len() > 20,
        "only {} distinct demands across {} rows",
        distinct.len(),
        rows.len()
    );
    assert!(
        lifted > 0 && lifted < rows.len(),
        "the lift branch never varies"
    );
    assert!(paused > 0, "the paused branch is never exercised");

    println!(
        "{} rows, {} distinct demands, {lifted} with the limit lifted, \
         {paused} paused, largest difference {largest:e}",
        rows.len(),
        distinct.len()
    );
}

/// The aircraft never hovers at the slowdown height, which is the point of
/// the clamp.
///
/// The proportional term drives towards `land_alt_low_m` rather than towards
/// the ground, so on its own it would settle there and stop. The clamp keeps
/// the demand at or below `-|LAND_SPEED|`, so it can never reach zero.
#[test]
fn the_descent_never_stalls_at_the_slowdown_height() {
    let config = LandDescentConfig {
        land_alt_low_m: 3.0,
        land_speed_high_ms: 2.0,
        land_speed_ms: 0.5,
        max_speed_down_ms: 1.5,
        pos_p_kp: 1.0,
        max_accel_mss: 2.5,
    };

    // Straddling the slowdown height, and right at it.
    for alt in [10.0_f32, 3.1, 3.0, 2.9, 1.0, 0.0, -0.5] {
        let d = land_descent(false, alt, false, &config, 0.0025);
        assert!(
            d.climb_rate_ms <= -0.5,
            "at {alt} m the demand was {}, which does not descend at the \
             final speed",
            d.climb_rate_ms
        );
    }
}

/// `LAND_ALT_LOW` below a metre behaves as a metre.
#[test]
fn the_slowdown_height_has_a_floor() {
    let mut config = LandDescentConfig {
        land_alt_low_m: 1.0,
        land_speed_high_ms: 2.0,
        land_speed_ms: 0.3,
        max_speed_down_ms: 1.5,
        pos_p_kp: 1.0,
        max_accel_mss: 2.5,
    };
    let at_one = land_descent(false, 5.0, false, &config, 0.0025);

    for low in [0.0_f32, 0.25, 0.999] {
        config.land_alt_low_m = low;
        let below = land_descent(false, 5.0, false, &config, 0.0025);
        assert_eq!(
            below, at_one,
            "a slowdown height of {low} should behave as one metre"
        );
    }

    // And above a metre it does move, so the floor is a floor and not a
    // constant.
    config.land_alt_low_m = 4.0;
    assert_ne!(land_descent(false, 5.0, false, &config, 0.0025), at_one);
}

/// A landing does not speed up as it nears the ground, even if the parameters
/// ask it to.
///
/// `LAND_SPEED` set faster than `LAND_SPEED_HIGH` is a misconfiguration that
/// would otherwise make the aircraft accelerate into the ground. The ceiling
/// is raised to the final speed before it is used as a bound.
#[test]
fn a_final_speed_faster_than_the_approach_does_not_accelerate_the_landing() {
    let config = LandDescentConfig {
        land_alt_low_m: 3.0,
        // Slower than the final speed below — the misconfiguration.
        land_speed_high_ms: 0.4,
        land_speed_ms: 1.2,
        max_speed_down_ms: 1.5,
        pos_p_kp: 1.0,
        max_accel_mss: 2.5,
    };

    // High up, where the demand runs into the ceiling.
    let high = land_descent(false, 40.0, false, &config, 0.0025);
    assert!(
        (high.climb_rate_ms + 1.2).abs() < 1e-6,
        "the ceiling should have been raised to the final speed, got {}",
        high.climb_rate_ms
    );
}

/// A paused descent commands nothing and leaves the limit in place.
///
/// The second half matters: a pause is not an arrival, so lifting the descent
/// limit for it would let the next unpaused iteration start from a state that
/// says the aircraft is already down.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "exactness is the assertion: a paused descent commands literally nothing, and a demand within an epsilon of zero is still a demand"
)]
fn a_paused_descent_commands_nothing() {
    let config = LandDescentConfig {
        land_alt_low_m: 3.0,
        land_speed_high_ms: 2.0,
        land_speed_ms: 0.5,
        max_speed_down_ms: 1.5,
        pos_p_kp: 1.0,
        max_accel_mss: 2.5,
    };

    for alt in [0.0_f32, 1.0, 50.0] {
        for maybe in [false, true] {
            let d = land_descent(true, alt, maybe, &config, 0.0025);
            assert_eq!(d.climb_rate_ms, 0.0);
            assert!(
                !d.ignore_descent_limit,
                "a paused descent lifted the limit at {alt} m"
            );
        }
    }
}
