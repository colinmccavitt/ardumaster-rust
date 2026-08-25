//! The NE position controller's kinematic layer against the real firmware.

#![allow(
    clippy::float_cmp,
    reason = "the magnitude test compares exactly on purpose: fabsf of a \
negative is bit-identical to the positive, and a tolerance would let an \
approximate implementation through"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_control::pos_control_ne::{AttitudeCapability, NeLimits};

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/pos_control_ne.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out = Vec::new();
    let mut current = "";
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag;
            continue;
        }
        if line.is_empty() || line.chars().next().is_some_and(char::is_alphabetic) {
            continue;
        }
        if current == section {
            out.push(line.split(',').map(str::to_owned).collect());
        }
    }
    out
}

/// The horizontal limit derivation, swept over the attitude controller's
/// capability.
///
/// The jerk limit is the interesting one, and it is derived rather than
/// configured. A multirotor accelerates horizontally by leaning, so changing
/// its horizontal acceleration means changing its lean angle — and the rate it
/// can do that at is an angular rate. Hence a jerk bound of the attitude
/// controller's rate limit times gravity, and a second bound from angular
/// acceleration, since the vehicle cannot reach its maximum lean rate
/// instantly.
///
/// Both apply only with body-frame feedforward on. The sweep covers both
/// settings, and the assertions below check the two produce visibly different
/// spreads — with the bounds disabled the jerk should simply be whatever was
/// configured.
#[test]
fn the_ne_limit_derivation_matches_upstream() {
    let rows = rows("limits");
    assert!(!rows.is_empty(), "no limit rows");

    let mut largest = 0.0_f32;
    let mut with_ff = std::collections::BTreeSet::new();
    let mut without_ff = std::collections::BTreeSet::new();

    for r in &rows {
        assert_eq!(r.len(), 12, "malformed limits row");
        let idx: usize = r[0].parse().expect("idx");
        let ff = r[8].trim() == "1";

        let attitude = AttitudeCapability {
            ang_vel_roll_max_rads: f(&r[4]),
            ang_vel_pitch_max_rads: f(&r[5]),
            accel_roll_max_radss: f(&r[6]),
            accel_pitch_max_radss: f(&r[7]),
            bf_feedforward: ff,
        };

        let got = NeLimits::derive(f(&r[1]), f(&r[2]), f(&r[3]), &attitude);

        for (label, value, want) in [
            ("vel_max", got.vel_max_ne_ms, f(&r[9])),
            ("accel_max", got.accel_max_ne_mss, f(&r[10])),
            ("jerk_max", got.jerk_max_ne_msss, f(&r[11])),
        ] {
            let diff = (value - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < 3e-5,
                "row {idx} {label}: {value} != upstream {want} (diff {diff})"
            );
        }

        let key = got.jerk_max_ne_msss.to_bits();
        if ff {
            with_ff.insert(key);
        } else {
            without_ff.insert(key);
        }
    }

    // With the bounds disabled the jerk is whatever was configured, so it
    // takes only as many values as the sweep configured. With them enabled it
    // takes many more. If those two counts matched, the feedforward branch
    // would be doing nothing and the test could not tell.
    assert!(
        with_ff.len() > without_ff.len() * 2,
        "the feedforward branch barely changes the result: {} distinct jerk \
         limits with it on against {} with it off",
        with_ff.len(),
        without_ff.len()
    );

    println!(
        "{} limit rows, largest difference {largest:e}; {} distinct jerk limits \
         with feedforward, {} without",
        rows.len(),
        with_ff.len(),
        without_ff.len()
    );
}

/// A negative speed or acceleration limit is taken as its magnitude.
///
/// Upstream calls `fabsf` on both. It matters because a caller that computes a
/// limit from a signed quantity and forgets the absolute value would otherwise
/// configure a controller that can never satisfy its own bound — every
/// comparison against a negative maximum fails.
#[test]
fn negative_limits_are_taken_as_magnitudes() {
    let attitude = AttitudeCapability {
        ang_vel_roll_max_rads: 0.0,
        ang_vel_pitch_max_rads: 0.0,
        accel_roll_max_radss: 0.0,
        accel_pitch_max_radss: 0.0,
        bf_feedforward: false,
    };

    let negative = NeLimits::derive(-7.5, -2.5, 5.0, &attitude);
    let positive = NeLimits::derive(7.5, 2.5, 5.0, &attitude);

    assert_eq!(negative.vel_max_ne_ms, positive.vel_max_ne_ms);
    assert_eq!(negative.accel_max_ne_mss, positive.accel_max_ne_mss);
    assert_eq!(negative.vel_max_ne_ms, 7.5);
    assert_eq!(negative.accel_max_ne_mss, 2.5);
}
