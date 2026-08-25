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

use ap_control::pos_control_ne::{stopping_point_d, AttitudeCapability, DLimits, NeLimits};

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

/// The vertical limit derivation, including its filter-bandwidth jerk bound.
///
/// Zero means *leave unchanged* here, the opposite of the horizontal setter
/// which takes whatever it is given. So the sweep passes zeros deliberately
/// and each row starts from the same prior state.
///
/// The jerk bound is a filter-bandwidth argument. The acceleration PID
/// low-passes its target and its error, and commanding jerk faster than those
/// can follow buys nothing — the command is smoothed away and only phase lag
/// remains. Hence a cap at one fifth of the filter's corner rate, and the
/// `min` with gravity, because a jerk budget derived from an acceleration the
/// aircraft cannot reach is not a budget.
#[test]
fn the_vertical_limit_derivation_matches_upstream() {
    let rows = rows("dlimits");
    assert!(!rows.is_empty(), "no dlimits rows");

    // What the harness restores before each row.
    let prior = DLimits {
        vel_max_down_ms: 2.0,
        vel_max_up_ms: 2.5,
        accel_max_d_mss: 2.5,
        jerk_max_d_msss: 0.0,
    };

    let mut largest = 0.0_f32;
    let mut distinct_jerk = std::collections::BTreeSet::new();
    let mut left_unchanged = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 11, "malformed dlimits row");
        let idx: usize = r[0].parse().expect("idx");

        let got = DLimits::derive(
            prior,
            f(&r[1]),
            f(&r[2]),
            f(&r[3]),
            f(&r[4]),
            f(&r[5]),
            f(&r[6]),
        );

        for (label, value, want) in [
            ("vel_down", got.vel_max_down_ms, f(&r[7])),
            ("vel_up", got.vel_max_up_ms, f(&r[8])),
            ("accel_max", got.accel_max_d_mss, f(&r[9])),
            ("jerk_max", got.jerk_max_d_msss, f(&r[10])),
        ] {
            let diff = (value - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < 3e-5,
                "row {idx} {label}: {value} != upstream {want} (diff {diff})"
            );
        }

        distinct_jerk.insert(got.jerk_max_d_msss.to_bits());
        if f(&r[1]) == 0.0 && got.vel_max_down_ms == prior.vel_max_down_ms {
            left_unchanged += 1;
        }
    }

    assert!(
        left_unchanged > 100,
        "only {left_unchanged} rows exercised the leave-unchanged path; a port \
         treating zero as 'no limit' would pass"
    );
    assert!(
        distinct_jerk.len() > 5,
        "the jerk bound produced only {} distinct values; the filter cutoffs \
         are not biting",
        distinct_jerk.len()
    );

    println!(
        "{} vertical-limit rows, largest difference {largest:e}, {} distinct \
         jerk limits, {left_unchanged} rows left a limit unchanged",
        rows.len(),
        distinct_jerk.len()
    );
}

/// The overspeed gain, swept across both limits and through zero.
///
/// A vehicle already travelling faster than permitted needs *more* authority,
/// not less: it has further to slow down and no more distance to do it in. The
/// gain grows in proportion, so twice the permitted speed gets four times the
/// acceleration budget.
///
/// Both branches guard against a zero limit, which would otherwise divide — a
/// zero limit means unconfigured, and an unconfigured axis should not be told
/// it is over speed.
#[test]
fn the_overspeed_gain_matches_upstream() {
    let rows = rows("overspeed");
    assert!(!rows.is_empty(), "no overspeed rows");

    let mut largest = 0.0_f32;
    let mut engaged = 0_usize;
    let mut with_zero_limit = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 5, "malformed overspeed row");
        let idx: usize = r[0].parse().expect("idx");
        let vel_desired = f(&r[1]);
        let limits = DLimits {
            vel_max_down_ms: f(&r[2]),
            vel_max_up_ms: f(&r[3]),
            accel_max_d_mss: 1.0,
            jerk_max_d_msss: 1.0,
        };

        let got = limits.overspeed_gain(vel_desired);
        let want = f(&r[4]);
        let diff = (got - want).abs();
        largest = largest.max(diff);
        assert!(
            diff < 3e-5,
            "row {idx}: gain {got} != upstream {want} (diff {diff})"
        );

        if (got - 1.0).abs() > 1e-6 {
            engaged += 1;
        }
        if limits.vel_max_down_ms == 0.0 || limits.vel_max_up_ms == 0.0 {
            with_zero_limit += 1;
        }
    }

    assert!(
        engaged > 50,
        "the gain only left unity on {engaged} rows; the sweep is not going \
         over speed"
    );
    assert!(
        with_zero_limit > 50,
        "the zero-limit guard is barely covered ({with_zero_limit} rows)"
    );

    println!(
        "{} overspeed rows, largest difference {largest:e}, gain engaged on \
         {engaged}, {with_zero_limit} with an unconfigured limit",
        rows.len()
    );
}

/// The vertical stopping point and its asymmetric bounds.
///
/// Three metres up against two metres down, and the asymmetry is not
/// arbitrary: overshooting upward costs altitude the vehicle can recover,
/// overshooting downward may cost the vehicle. The tighter bound is on the
/// direction where being wrong is unrecoverable.
#[test]
fn the_vertical_stopping_point_matches_upstream() {
    let rows = rows("dstop");
    assert!(!rows.is_empty(), "no dstop rows");

    let mut largest = 0.0_f32;
    let mut clamped_up = 0_usize;
    let mut clamped_down = 0_usize;
    let mut unconfigured = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 8, "malformed dstop row");
        let idx: usize = r[0].parse().expect("idx");
        let limits = DLimits {
            vel_max_down_ms: 2.0,
            vel_max_up_ms: 2.5,
            accel_max_d_mss: f(&r[6]),
            jerk_max_d_msss: 5.0,
        };

        let got = stopping_point_d(f(&r[1]), f(&r[2]), f(&r[3]), f(&r[4]), f(&r[5]), &limits);
        let want: f32 = r[7].trim().parse::<f64>().expect("stop") as f32;
        let diff = (got - want).abs();
        largest = largest.max(diff);
        assert!(
            diff < 3e-5,
            "row {idx}: stopping point {got} != upstream {want} (diff {diff})"
        );

        let base = f(&r[1]) - f(&r[2]);
        if (got - (base - 3.0)).abs() < 1e-4 {
            clamped_up += 1;
        }
        if (got - (base + 2.0)).abs() < 1e-4 {
            clamped_down += 1;
        }
        if f(&r[5]) <= 0.0 || f(&r[6]) <= 0.0 {
            unconfigured += 1;
        }
    }

    // All three behaviours must appear, or the bounds could be swapped or
    // dropped without the comparison noticing.
    assert!(
        clamped_up > 0 && clamped_down > 0,
        "both bounds must engage ({clamped_up} up, {clamped_down} down)"
    );
    assert!(
        unconfigured > 5,
        "the unconfigured-axis path is barely covered ({unconfigured} rows)"
    );

    println!(
        "{} stopping-point rows, largest difference {largest:e}; clamped up on \
         {clamped_up}, down on {clamped_down}, unconfigured on {unconfigured}",
        rows.len()
    );
}
