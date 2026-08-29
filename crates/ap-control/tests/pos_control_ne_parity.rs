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

use ap_control::pos_control_ne::{
    accel_ne_to_lean_angles, controller_is_active, init_terrain, lean_angles_to_accel_ned,
    offset_target_timed_out, stopping_point_d, stopping_point_ne, thrust_vector, update_terrain,
    yaw_from_ne_motion, AttitudeCapability, DEkfReset, DEkfTargets, DEstimates, DInitInputs,
    DLimits, DOffsetState, DOffsets, DTerrain, DUpdateInputs, EkfResetMethod, NeDisturbance,
    NeEkfReset, NeEkfTargets, NeEstimates, NeInitInputs, NeLimits, NeOffsetState, NeOffsets,
    NeUpdateInputs, PosControlD, PosControlNe, NE_POS_P,
};
use ap_math::vector2::{Vector2, Vector2f};
use ap_math::vector3::Vector3f;
use ap_pid::{AcP1d, AcP2d, AcPid, AcPid2d, AcPidBasic, PidGains};

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

/// Lean angles to horizontal acceleration, swept past the divisor's floor.
///
/// The divisor `cos(roll)·cos(pitch)` is the vertical component of the thrust
/// axis; dividing by it turns "where the aircraft points" into "what a vehicle
/// holding altitude actually accelerates at", since a leaning aircraft must
/// push harder to stay level and pushes harder horizontally too.
///
/// Floored at a tenth, capping implied thrust at ten times hover. Past about
/// 84 degrees the true value diverges — an aircraft on its side cannot hold
/// altitude at any horizontal acceleration — and the floor turns that into a
/// large finite number the limits downstream can bound. 132 of the 256
/// recorded rows are past it.
#[test]
fn the_lean_to_accel_map_matches_upstream() {
    let rows = rows("leanaccel");
    assert!(!rows.is_empty(), "no leanaccel rows");

    let mut largest = 0.0_f32;
    let mut floored = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 7, "malformed leanaccel row");
        let idx: usize = r[0].parse().expect("idx");
        let euler = Vector3f::new(f(&r[1]), f(&r[2]), f(&r[3]));

        let got = lean_angles_to_accel_ned(euler);
        for (label, value, want) in [
            ("acc_n", got.x, f(&r[4])),
            ("acc_e", got.y, f(&r[5])),
            ("acc_d", got.z, f(&r[6])),
        ] {
            let diff = (value - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < 3e-5,
                "row {idx} {label}: {value} != upstream {want} (diff {diff})"
            );
        }

        if libm::cosf(euler.x) * libm::cosf(euler.y) < 0.1 {
            floored += 1;
        }
    }

    assert!(
        floored > 50 && floored < rows.len() - 50,
        "the divisor floor must both bind and not: it bound on {floored} of {}",
        rows.len()
    );

    println!(
        "{} lean-to-accel rows, largest difference {largest:e}, floor bound on \
         {floored}",
        rows.len()
    );
}

/// Acceleration to lean angles, against the recorded heading.
///
/// The recorded heading is due north — the harness AHRS cannot be driven — so
/// every row here has an identity rotation into the body frame. The rotation
/// itself is covered by [`the_body_rotation_uses_the_heading`] below; this
/// checks the conversion and the `cos(pitch)` correction.
#[test]
fn the_accel_to_lean_map_matches_upstream() {
    let rows = rows("accellean");
    assert!(!rows.is_empty(), "no accellean rows");

    let mut largest = 0.0_f32;
    for r in &rows {
        assert_eq!(r.len(), 7, "malformed accellean row");
        let idx: usize = r[0].parse().expect("idx");

        let (roll, pitch) = accel_ne_to_lean_angles(f(&r[1]), f(&r[2]), f(&r[3]), f(&r[4]));

        for (label, value, want) in [("roll", roll, f(&r[5])), ("pitch", pitch, f(&r[6]))] {
            let diff = (value - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < 3e-5,
                "row {idx} {label}: {value} != upstream {want} (diff {diff})"
            );
        }
    }

    println!(
        "{} accel-to-lean rows, largest difference {largest:e}",
        rows.len()
    );
}

/// The heading rotation, which the recording cannot exercise.
///
/// The harness AHRS reports due north, so `cos_yaw` is one and `sin_yaw` zero
/// in every recorded row and the rotation into forward-right is the identity.
/// A port that dropped it entirely would pass the parity test above.
///
/// Checked here against the property rather than a recorded value: a pure
/// north demand must become pure pitch when heading north, and pure roll when
/// heading east.
#[test]
fn the_body_rotation_uses_the_heading() {
    // Heading north: a northward demand is straight ahead, so pitch only.
    let (roll, pitch) = accel_ne_to_lean_angles(4.0, 0.0, 1.0, 0.0);
    assert!(
        roll.abs() < 1e-6 && pitch < -0.1,
        "heading north, a northward demand is pure pitch: got roll {roll}, \
         pitch {pitch}"
    );

    // Heading east: the same demand is now off the left wing, so roll only.
    let (roll, pitch) = accel_ne_to_lean_angles(4.0, 0.0, 0.0, 1.0);
    assert!(
        pitch.abs() < 1e-6 && roll < -0.1,
        "heading east, a northward demand is pure roll: got roll {roll}, \
         pitch {pitch}"
    );

    // And the cos(pitch) correction: with the aircraft pitched hard, the same
    // lateral demand needs more roll than it would when level, because rolling
    // while pitched moves the thrust sideways by only cos(pitch)·sin(roll).
    let (roll_level, _) = accel_ne_to_lean_angles(0.0, 3.0, 1.0, 0.0);
    let (roll_pitched, _) = accel_ne_to_lean_angles(25.0, 3.0, 1.0, 0.0);
    assert!(
        roll_pitched.abs() < roll_level.abs(),
        "the cos(pitch) correction should shrink the roll demand when pitched: \
         level {roll_level}, pitched {roll_pitched}"
    );
}

/// The thrust vector hands the attitude controller a direction, not a force.
///
/// The vertical component is replaced by `-g` regardless of what the vertical
/// controller is doing. That is not an approximation: only the ratio of
/// horizontal to vertical sets the lean angle, so passing the real vertical
/// acceleration would make the commanded bank depend on the climb rate — the
/// aircraft would lean differently through the same horizontal manoeuvre while
/// climbing than while descending. Fixing it at gravity decouples the axes,
/// which is what allows them separate controllers at all.
#[test]
fn the_thrust_vector_is_independent_of_the_vertical_command() {
    let horizontal = Vector3f::new(2.5, -1.5, 0.0);

    let climbing = thrust_vector(Vector3f::new(horizontal.x, horizontal.y, -4.0));
    let descending = thrust_vector(Vector3f::new(horizontal.x, horizontal.y, 6.0));

    assert_eq!(
        (climbing.x, climbing.y, climbing.z),
        (descending.x, descending.y, descending.z),
        "the commanded direction must not depend on the vertical command"
    );
    assert!(
        (climbing.z + 9.80665).abs() < 1e-6,
        "the vertical component is fixed at gravity, got {}",
        climbing.z
    );
}

/// The scalar angle/acceleration pair and `input_expo`.
///
/// The expo guard is on `expo` alone, so a negative expo is allowed and
/// inverts the shaping — coarse near centre, fine at the stops. Unusual but
/// well defined, and the sweep includes it.
#[test]
fn the_angle_conversions_match_upstream() {
    use ap_math::control::{
        accel_mss_to_angle_deg, accel_mss_to_angle_rad, angle_deg_to_accel_mss,
        angle_rad_to_accel_mss, input_expo,
    };

    let rows = rows("angleconv");
    assert!(!rows.is_empty(), "no angleconv rows");

    let mut largest = 0.0_f32;
    let mut passthrough = 0_usize;
    let mut inverted = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 8, "malformed angleconv row");
        let idx: usize = r[0].parse().expect("idx");
        let x = f(&r[1]);
        let expo = f(&r[6]);

        for (label, value, want) in [
            ("accel_from_rad", angle_rad_to_accel_mss(x), f(&r[2])),
            ("accel_from_deg", angle_deg_to_accel_mss(x), f(&r[3])),
            ("rad_from_accel", accel_mss_to_angle_rad(x), f(&r[4])),
            ("deg_from_accel", accel_mss_to_angle_deg(x), f(&r[5])),
            ("shaped", input_expo(x, expo), f(&r[7])),
        ] {
            let diff = (value - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < 3e-4,
                "row {idx} {label}: {value} != upstream {want} (diff {diff})"
            );
        }

        if expo >= 0.95 {
            passthrough += 1;
        }
        if expo < 0.0 {
            inverted += 1;
        }
    }

    assert!(
        passthrough > 50,
        "the expo passthrough guard is barely covered ({passthrough} rows)"
    );
    assert!(
        inverted > 50,
        "negative expo is allowed and inverts the shaping; only {inverted} \
         rows cover it"
    );

    println!(
        "{} angle-conversion rows, largest difference {largest:e}; \
         {passthrough} passthrough, {inverted} inverted",
        rows.len()
    );
}
// var_info and update_terrain are in.
// Lean-angle conversions, get_thrust_vector, NE/D update_controller,
// and NE/D init/offset/EKF reset are done.

fn update_inputs() -> NeUpdateInputs {
    NeUpdateInputs {
        dt: 0.02,
        ahrs_control_scale_xy: 1.0,
        ne_control_scale_factor: 1.0,
        vel_max_ne_ms: 10.0,
        estimates: NeEstimates {
            pos_m: Vector2::new(0.0, 0.0),
            vel_ms: Vector2f::new(0.0, 0.0),
        },
        offsets: NeOffsets::default(),
        lean_angle_max_rad: 0.8,
        cos_yaw: 1.0,
        sin_yaw: 0.0,
        att_yaw_target_rad: 0.3,
    }
}

/// A one-metre north error at kp=1, no filters, no I, no D, must produce a
/// 1 m/s north velocity demand and then a 1 m/s^2 north acceleration
/// demand when the velocity PID is also kp=1 with I and D off.
#[test]
fn the_pid_path_is_p_then_velocity_pid_then_feedforward() {
    let mut ne = PosControlNe::new();
    ne.pos_desired_m = Vector2::new(1.0, 0.0);
    let mut pos_p = AcP2d::new(NE_POS_P);
    let mut vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let inp = update_inputs();
    let mut disturb = NeDisturbance::default();

    let out = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);

    assert!((out.vel_target_ms.x - 1.0).abs() < 1e-5);
    assert!(out.vel_target_ms.y.abs() < 1e-5);
    assert!((out.accel_target_mss.x - 1.0).abs() < 1e-5);
    assert!(out.accel_target_mss.y.abs() < 1e-5);
    assert_eq!(out.ne_control_scale_factor, 1.0);
}

/// The AHRS scale is applied to the P output *and* the PID output. A
/// port that scaled only one of them would still look plausible on a
/// quiet hover and fail here: with scale 0.5 the velocity demand is
/// halved and the acceleration demand is quartered.
#[test]
fn the_ahrs_scale_applies_to_both_loops() {
    let mut ne = PosControlNe::new();
    ne.pos_desired_m = Vector2::new(1.0, 0.0);
    let mut pos_p = AcP2d::new(1.0);
    let mut vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut inp = update_inputs();
    inp.ahrs_control_scale_xy = 0.5;
    let mut disturb = NeDisturbance::default();

    let out = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);

    assert!(
        (out.vel_target_ms.x - 0.5).abs() < 1e-5,
        "P output must be scaled"
    );
    assert!(
        (out.accel_target_mss.x - 0.25).abs() < 1e-5,
        "PID output must be scaled again, got {}",
        out.accel_target_mss.x
    );
}

/// `_ne_control_scale_factor` is a one-shot. After the call it is 1,
/// and it multiplies the same places the AHRS scale does.
#[test]
fn the_one_shot_scale_is_consumed() {
    let mut ne = PosControlNe::new();
    ne.pos_desired_m = Vector2::new(1.0, 0.0);
    let mut pos_p = AcP2d::new(1.0);
    let mut vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut inp = update_inputs();
    inp.ne_control_scale_factor = 2.0;
    let mut disturb = NeDisturbance::default();

    let out = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);
    assert!((out.vel_target_ms.x - 2.0).abs() < 1e-5);
    assert!((out.accel_target_mss.x - 4.0).abs() < 1e-5);
    assert_eq!(out.ne_control_scale_factor, 1.0);
}

/// Offsets are added to desired to form the absolute target, and to the
/// feed-forward velocity and acceleration. A port that treated desired
/// as already-absolute would double-count them or drop them.
#[test]
fn offsets_are_added_to_the_absolute_target_and_the_feedforward() {
    let mut ne = PosControlNe::new();
    ne.vel_desired_ms = Vector2f::new(0.5, 0.0);
    ne.accel_desired_mss = Vector2f::new(0.25, 0.0);
    let mut pos_p = AcP2d::new(0.0);
    let mut vel_pid = AcPid2d::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut inp = update_inputs();
    inp.offsets = NeOffsets {
        pos_m: Vector2::new(3.0, -1.0),
        vel_ms: Vector2f::new(0.1, 0.2),
        accel_mss: Vector2f::new(0.05, -0.05),
    };
    let mut disturb = NeDisturbance::default();

    let out = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);
    assert!((out.pos_target_m.x - 3.0).abs() < 1e-9);
    assert!((out.pos_target_m.y + 1.0).abs() < 1e-9);
    assert!((out.vel_target_ms.x - 0.6).abs() < 1e-5);
    assert!((out.vel_target_ms.y - 0.2).abs() < 1e-5);
    assert!((out.accel_target_mss.x - 0.30).abs() < 1e-5);
    assert!((out.accel_target_mss.y + 0.05).abs() < 1e-5);
}

/// A disturbance is added to the estimate for this cycle and then
/// cleared. The P controller therefore sees a different error than the
/// raw estimate, and a second call with the same desired must not.
#[test]
fn a_disturbance_is_applied_once_and_then_cleared() {
    let mut ne = PosControlNe::new();
    ne.pos_desired_m = Vector2::new(0.0, 0.0);
    let mut pos_p = AcP2d::new(1.0);
    let mut vel_pid = AcPid2d::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let inp = update_inputs();
    let mut disturb = NeDisturbance {
        pos_m: Vector2f::new(-2.0, 0.0),
        vel_ms: Vector2f::zero(),
    };

    let first = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);
    assert!((first.vel_target_ms.x - 2.0).abs() < 1e-5);
    assert_eq!(disturb.pos_m, Vector2f::zero());
    assert_eq!(disturb.vel_ms, Vector2f::zero());

    let second = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);
    assert!(
        second.vel_target_ms.x.abs() < 1e-5,
        "a cleared disturbance must not keep correcting"
    );
}

/// When the lean-angle budget binds, the limit vector is the
/// *unbounded* acceleration — that is what the next PID step uses for
/// anti-windup. When it does not bind, the limit vector is zero.
#[test]
fn the_limit_vector_is_the_unbounded_accel_only_when_clipped() {
    let mut ne = PosControlNe::new();
    ne.pos_desired_m = Vector2::new(50.0, 0.0);
    let mut pos_p = AcP2d::new(2.0);
    let mut vel_pid = AcPid2d::new(2.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut inp = update_inputs();
    inp.lean_angle_max_rad = 0.1;
    let mut disturb = NeDisturbance::default();

    let out = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);
    assert!(
        out.limited,
        "a 50 m error at kp=2 must exceed 0.1 rad of lean"
    );
    assert!(
        ne.limit_vector.length() > out.accel_target_mss.length(),
        "the stored limit is the unbounded demand, not the clipped one"
    );

    ne.pos_desired_m = Vector2::new(0.01, 0.0);
    ne.limit_vector = Vector2f::zero();
    pos_p = AcP2d::new(1.0);
    vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    inp.lean_angle_max_rad = 0.8;
    let out = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);
    assert!(!out.limited);
    assert_eq!(ne.limit_vector, Vector2f::zero());
}

/// The lean angles are exactly [`accel_ne_to_lean_angles`] of the
/// (possibly limited) acceleration target. This is composition, not a
/// second conversion.
#[test]
fn lean_angles_come_from_the_existing_conversion() {
    let mut ne = PosControlNe::new();
    ne.pos_desired_m = Vector2::new(1.0, 0.5);
    let mut pos_p = AcP2d::new(1.0);
    let mut vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut inp = update_inputs();
    inp.cos_yaw = 0.0;
    inp.sin_yaw = 1.0;
    let mut disturb = NeDisturbance::default();

    let out = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);
    let (roll, pitch) = accel_ne_to_lean_angles(
        out.accel_target_mss.x,
        out.accel_target_mss.y,
        inp.cos_yaw,
        inp.sin_yaw,
    );
    assert!((out.roll_target_rad - roll).abs() < 1e-7);
    assert!((out.pitch_target_rad - pitch).abs() < 1e-7);
}

/// Yaw follows the velocity vector once speed exceeds five percent of
/// the configured maximum, and keeps the attitude target below that.
#[test]
fn yaw_follows_the_velocity_vector_only_when_moving() {
    let (yaw, rate) =
        yaw_from_ne_motion(Vector2f::new(2.0, 0.0), Vector2f::new(0.0, 4.0), 10.0, 0.3);
    assert!(
        (yaw - 0.0).abs() < 1e-5,
        "due east? wait north is +x so heading 0"
    );
    // vel = (2, 0) heading atan2(0, 2) = 0.
    // accel_turn is (0, 4), speed 2, turn rate 2 rad/s.
    // vel.cross(accel_turn) = 2*4 - 0*0 = 8 > 0, so rate stays positive.
    assert!((rate - 2.0).abs() < 1e-5, "got rate {rate}");

    let (yaw, rate) =
        yaw_from_ne_motion(Vector2f::new(0.1, 0.0), Vector2f::new(0.0, 4.0), 10.0, 0.3);
    assert_eq!(yaw, 0.3, "below 5% of 10 m/s the attitude yaw is kept");
    assert_eq!(rate, 0.0);
}

/// A P-controller clamp rewrites the absolute target and therefore the
/// desired position (target minus offset). That is how a clamp on the
/// *absolute* target becomes a clamp on the trajectory.
#[test]
fn a_position_clamp_rewrites_desired() {
    let mut ne = PosControlNe::new();
    ne.pos_desired_m = Vector2::new(100.0, 0.0);
    let mut pos_p = AcP2d::new(1.0);
    pos_p.set_limits(2.0, 0.0, 0.0);
    let mut vel_pid = AcPid2d::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut inp = update_inputs();
    inp.offsets.pos_m = Vector2::new(1.0, 0.0);
    let mut disturb = NeDisturbance::default();

    let out = ne.update_controller(&mut pos_p, &mut vel_pid, &inp, &mut disturb);
    assert!(
        out.pos_target_m.x < 10.0,
        "the absolute target must have been pulled in, got {}",
        out.pos_target_m.x
    );
    assert!(
        (ne.pos_desired_m.x - (out.pos_target_m.x - 1.0)).abs() < 1e-9,
        "desired is the (clamped) target minus the offset"
    );
}

fn d_update_inputs() -> DUpdateInputs {
    DUpdateInputs {
        dt: 0.02,
        now_ms: 0,
        ahrs_control_scale_z: 1.0,
        estimates: DEstimates::default(),
        offsets: DOffsets::default(),
        terrain: DTerrain::default(),
        estimated_accel_d_mss: 0.0,
        throttle_lower: false,
        throttle_upper: false,
        throttle_hover: 0.0,
        vibe_comp_enabled: false,
        vel_max_down_ms: 2.5,
    }
}

fn accel_p(p: f32) -> AcPid {
    AcPid::new(PidGains {
        p,
        imax: 1.0,
        ..PidGains::default()
    })
}

/// A one-metre down error at kp=1, no filters, no I, no D, must produce a
/// 1 m/s down velocity demand and then a 1 m/s^2 down acceleration
/// demand when the velocity PID is also kp=1 with I and D off. The
/// acceleration PID, also kp=1 against a zero measurement, then produces
/// a unit down-positive thrust which is negated for the attitude
/// controller.
#[test]
fn the_d_pid_path_is_p_then_velocity_pid_then_accel_pid() {
    let mut d = PosControlD::new();
    d.pos_desired_m = 1.0;
    let mut pos_p = AcP1d::new(1.0);
    let mut vel_pid = AcPidBasic::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(1.0);
    let inp = d_update_inputs();

    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);

    assert!((out.vel_target_ms - 1.0).abs() < 1e-5);
    assert!((out.accel_target_mss - 1.0).abs() < 1e-5);
    assert!((out.thrust_d_norm - 1.0).abs() < 1e-5);
    assert!((out.throttle_out + 1.0).abs() < 1e-5);
}

/// The AHRS Z scale is applied to the P output *and* the velocity-PID
/// output. A port that scaled only one of them would still look
/// plausible on a quiet hover and fail here: with scale 0.5 the velocity
/// demand is halved and the acceleration demand is quartered. The
/// acceleration PID is *not* scaled — it works in thrust.
#[test]
fn the_d_ahrs_scale_applies_to_both_outer_loops() {
    let mut d = PosControlD::new();
    d.pos_desired_m = 1.0;
    let mut pos_p = AcP1d::new(1.0);
    let mut vel_pid = AcPidBasic::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(1.0);
    let mut inp = d_update_inputs();
    inp.ahrs_control_scale_z = 0.5;

    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);

    assert!(
        (out.vel_target_ms - 0.5).abs() < 1e-5,
        "P output must be scaled"
    );
    assert!(
        (out.accel_target_mss - 0.25).abs() < 1e-5,
        "velocity PID output must be scaled again, got {}",
        out.accel_target_mss
    );
    assert!(
        (out.thrust_d_norm - 0.25).abs() < 1e-5,
        "accel PID sees the already-scaled target, not a third scale"
    );
}

/// Offsets and terrain are added to desired to form the absolute target,
/// and to the feed-forward velocity and acceleration. A port that
/// treated desired as already-absolute would double-count them or drop
/// them; a port that added only offsets would miss terrain.
#[test]
fn d_offsets_and_terrain_are_added_to_the_target_and_the_feedforward() {
    let mut d = PosControlD::new();
    d.vel_desired_ms = 0.5;
    d.accel_desired_mss = 0.25;
    let mut pos_p = AcP1d::new(0.0);
    let mut vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(0.0);
    let mut inp = d_update_inputs();
    inp.offsets = DOffsets {
        pos_m: 3.0,
        vel_ms: 0.1,
        accel_mss: 0.05,
    };
    inp.terrain = DTerrain {
        pos_m: 1.0,
        vel_ms: 0.2,
        accel_mss: -0.05,
    };

    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    assert!((out.pos_target_m - 4.0).abs() < 1e-9);
    assert!((out.vel_target_ms - 0.8).abs() < 1e-5);
    assert!((out.accel_target_mss - 0.25).abs() < 1e-5);
}

/// A P-controller clamp rewrites the absolute target and therefore the
/// desired position (target minus offset minus terrain). That is how a
/// clamp on the *absolute* target becomes a clamp on the trajectory.
#[test]
fn a_d_position_clamp_rewrites_desired() {
    let mut d = PosControlD::new();
    d.pos_desired_m = 100.0;
    let mut pos_p = AcP1d::new(1.0);
    pos_p.set_limits(-2.0, 2.0, 0.0, 0.0);
    let mut vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(0.0);
    let mut inp = d_update_inputs();
    inp.offsets.pos_m = 1.0;
    inp.terrain.pos_m = 0.5;

    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    assert!(
        out.pos_target_m.abs() < 10.0,
        "the absolute target must have been pulled in, got {}",
        out.pos_target_m
    );
    assert!(
        (d.pos_desired_m - (out.pos_target_m - 1.5)).abs() < 1e-9,
        "desired is the (clamped) target minus offset minus terrain"
    );
}

/// Hover throttle is subtracted from the accel-PID output so the
/// attitude controller sees a delta from hover. On target with a
/// 0.4 hover that delta is -0.4 (down-positive), and the throttle
/// sent upward is therefore +0.4.
#[test]
fn hover_throttle_is_subtracted_and_the_sign_is_flipped() {
    let mut d = PosControlD::new();
    let mut pos_p = AcP1d::new(0.0);
    let mut vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(0.0);
    let mut inp = d_update_inputs();
    inp.throttle_hover = 0.4;

    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    assert!((out.thrust_d_norm + 0.4).abs() < 1e-5);
    assert!((out.throttle_out - 0.4).abs() < 1e-5);
}

/// If the configured accel-PID IMAX is below hover, it is raised.
/// Without that the integrator cannot produce enough thrust to hold
/// the vehicle up.
#[test]
fn hover_raises_the_accel_pid_imax() {
    let mut d = PosControlD::new();
    let mut pos_p = AcP1d::new(0.0);
    let mut vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(0.0);
    accel_pid.gains.imax = 0.1;
    let mut inp = d_update_inputs();
    inp.throttle_hover = 0.35;

    let _ = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    assert!((accel_pid.gains.imax - 0.35).abs() < 1e-6);
}

/// Throttle-upper stores -1 on the limit vector (cannot accelerate
/// *up*, so down-positive limit is negative). Throttle-lower stores
/// +1. Neither stores zero.
#[test]
fn throttle_limits_set_the_vertical_limit_vector() {
    let mut d = PosControlD::new();
    let mut pos_p = AcP1d::new(0.0);
    let mut vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(0.0);
    let mut inp = d_update_inputs();

    inp.throttle_upper = true;
    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    assert_eq!(out.limit, -1.0);
    assert_eq!(d.limit, -1.0);

    inp.throttle_upper = false;
    inp.throttle_lower = true;
    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    assert_eq!(out.limit, 1.0);

    inp.throttle_lower = false;
    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    assert_eq!(out.limit, 0.0);
}

/// Vibration compensation ignores the acceleration measurement and
/// builds throttle from a scaled feed-forward on the *target* plus the
/// accel-PID integrator. A port that still ran the acceleration PID
/// would respond to `estimated_accel_d_mss` here; this test sets that
/// measurement far from the target so the two paths cannot agree.
#[test]
fn vibe_compensation_ignores_the_acceleration_measurement() {
    let mut d = PosControlD::new();
    d.pos_desired_m = 1.0;
    let mut pos_p = AcP1d::new(1.0);
    let mut vel_pid = AcPidBasic::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(1.0);
    let mut inp = d_update_inputs();
    inp.vibe_comp_enabled = true;
    inp.throttle_hover = 0.4;
    inp.estimated_accel_d_mss = 50.0;

    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    // vel_target = 1, accel_target = 1 (P=1, I=D=0).
    // vibe thrust = 0.250 * 0.4 * 1 + I, I walks by
    // dt * hover * vel_error * vel_kp * 0.125 = 0.02 * 0.4 * 1 * 1 * 0.125
    // = 0.001. Then hover is subtracted.
    let vibe_i = 0.02 * 0.4 * 1.0 * 1.0 * 0.125;
    let expected_thrust = 0.250 * 0.4 * 1.0 + vibe_i - 0.4;
    assert!(
        (out.thrust_d_norm - expected_thrust).abs() < 1e-5,
        "got {} want {expected_thrust}",
        out.thrust_d_norm
    );
}

/// The health ratio walks toward 0.5 of the configured descent speed
/// of error and is clamped to 0..2. Starting at the default 2, a large
/// positive velocity error pulls it down.
#[test]
fn the_health_ratio_walks_and_clamps() {
    let mut d = PosControlD::new();
    assert_eq!(d.vel_d_control_ratio, 2.0);
    d.pos_desired_m = 10.0;
    let mut pos_p = AcP1d::new(1.0);
    let mut vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(0.0);
    let mut inp = d_update_inputs();
    inp.vel_max_down_ms = 1.0;

    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    // error = 10, error_ratio = 10, delta = 0.02 * 0.1 * (0.5 - 10) = -0.019
    assert!((out.vel_d_control_ratio - (2.0 - 0.019)).abs() < 1e-5);

    d.vel_d_control_ratio = 0.0;
    d.pos_desired_m = -10.0;
    pos_p = AcP1d::new(1.0);
    vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    // error = -10, error_ratio = -10, delta = 0.02 * 0.1 * (0.5 - -10) = 0.021
    // 0 + 0.021 stays above 0.
    let out = d.update_controller(&mut pos_p, &mut vel_pid, &mut accel_pid, &inp);
    assert!((out.vel_d_control_ratio - 0.021).abs() < 1e-5);
}

// var_info and update_terrain are in.

fn ne_limits() -> NeLimits {
    NeLimits {
        vel_max_ne_ms: 5.0,
        accel_max_ne_mss: 2.0,
        jerk_max_ne_msss: 5.0,
    }
}

fn d_limits() -> DLimits {
    DLimits {
        vel_max_down_ms: 1.5,
        vel_max_up_ms: 2.5,
        accel_max_d_mss: 2.5,
        jerk_max_d_msss: 5.0,
    }
}

fn ne_init_inputs() -> NeInitInputs {
    NeInitInputs {
        estimates: NeEstimates {
            pos_m: Vector2::new(10.0, -4.0),
            vel_ms: Vector2f::new(1.0, 0.5),
        },
        att_target_euler_rad: Vector3f::new(0.1, -0.2, 0.4),
        ahrs_yaw: 0.0,
        lean_angle_max_rad: 0.8,
        now_ms: 1_000,
        ticks: 50,
        last_update_ticks: 50,
        ahrs_ekf_reset_ms: 7,
        accel_target_mss: Vector2f::new(0.3, 0.0),
    }
}

fn d_init_inputs() -> DInitInputs {
    DInitInputs {
        estimates: DEstimates {
            pos_m: 8.0,
            vel_ms: 0.4,
        },
        now_ms: 1_000,
        ticks: 50,
        ahrs_ekf_reset_ms: 7,
        estimated_accel_d_mss: 0.2,
        accel_max_d_mss: 2.5,
        throttle_in: 0.45,
        throttle_hover: 0.35,
    }
}

/// Three seconds is still live; one millisecond more is abandoned.
#[test]
fn the_offset_timeout_is_strictly_greater_than_three_seconds() {
    assert!(!offset_target_timed_out(3_000, 0));
    assert!(offset_target_timed_out(3_001, 0));
    // Unsigned wrap: a clock that rolled past zero still measures 3001 ms.
    assert!(offset_target_timed_out(100, 100u32.wrapping_sub(3_001)));
    assert!(!offset_target_timed_out(100, 100u32.wrapping_sub(3_000)));
}

/// Active means this tick or the previous one. Two ticks is a timeout.
#[test]
fn the_controller_is_active_for_at_most_one_tick() {
    assert!(controller_is_active(10, 10));
    assert!(controller_is_active(11, 10));
    assert!(!controller_is_active(12, 10));
    assert!(controller_is_active(0, u32::MAX));
}

/// Init snaps current to target. A stale target is zeroed first so a
/// crashed script cannot seed the controller with a leftover tow.
#[test]
fn ne_init_offsets_snaps_current_to_target_or_zero() {
    let mut state = NeOffsetState {
        current: NeOffsets {
            pos_m: Vector2::new(1.0, 2.0),
            vel_ms: Vector2f::new(0.1, 0.0),
            accel_mss: Vector2f::zero(),
        },
        target: NeOffsets {
            pos_m: Vector2::new(3.0, -1.0),
            vel_ms: Vector2f::new(0.2, 0.3),
            accel_mss: Vector2f::new(0.05, 0.0),
        },
        target_ms: 1_000,
    };
    state.init(1_500);
    assert!((state.current.pos_m.x - 3.0).abs() < 1e-9);
    assert!((state.current.vel_ms.y - 0.3).abs() < 1e-9);

    state.target_ms = 0;
    state.target.pos_m = Vector2::new(9.0, 9.0);
    state.init(4_000);
    assert_eq!(state.current.pos_m.x, 0.0);
    assert_eq!(state.target.pos_m.x, 0.0);
}

/// A live offset with matching current and target and zero kinematics
/// must not move. A port that always integrated something would fail.
#[test]
fn ne_update_offsets_is_idle_when_current_equals_target_at_rest() {
    let mut state = NeOffsetState {
        current: NeOffsets {
            pos_m: Vector2::new(2.0, -1.0),
            ..NeOffsets::default()
        },
        target: NeOffsets {
            pos_m: Vector2::new(2.0, -1.0),
            ..NeOffsets::default()
        },
        target_ms: 1_000,
    };
    state.update(
        &ne_limits(),
        0.02,
        1_000,
        Vector2f::zero(),
        Vector2f::zero(),
        Vector2f::zero(),
    );
    assert!((state.current.pos_m.x - 2.0).abs() < 1e-6);
    assert!((state.current.pos_m.y + 1.0).abs() < 1e-6);
    assert!(state.current.vel_ms.x.abs() < 1e-6);
    assert!(state.current.accel_mss.x.abs() < 1e-6);
}

/// A timeout zeros the target and the shaper then pulls current toward
/// the origin. After one step the acceleration must point home.
#[test]
fn ne_update_offsets_times_out_and_shapes_home() {
    let mut state = NeOffsetState {
        current: NeOffsets {
            pos_m: Vector2::new(4.0, 0.0),
            ..NeOffsets::default()
        },
        target: NeOffsets {
            pos_m: Vector2::new(4.0, 0.0),
            ..NeOffsets::default()
        },
        target_ms: 0,
    };
    state.update(
        &ne_limits(),
        0.02,
        4_000,
        Vector2f::zero(),
        Vector2f::zero(),
        Vector2f::zero(),
    );
    assert_eq!(state.target.pos_m.x, 0.0);
    assert!(
        state.current.accel_mss.x < 0.0,
        "must accelerate toward zero, got {}",
        state.current.accel_mss.x
    );
}

/// Matching timestamps is a no-op. A port that always reconstructed
/// would move the target on every loop.
#[test]
fn ne_ekf_reset_is_idle_when_the_timestamp_is_unchanged() {
    let mut ekf = NeEkfReset::init(42);
    let mut targets = NeEkfTargets {
        pos_target_m: Vector2::new(1.0, 0.0),
        vel_target_ms: Vector2f::new(0.5, 0.0),
        pos_desired_m: Vector2::new(1.0, 0.0),
        vel_desired_ms: Vector2f::new(0.5, 0.0),
        offsets: NeOffsets::default(),
    };
    let fired = ekf.handle(
        42,
        EkfResetMethod::MoveTarget,
        &mut targets,
        Vector2f::zero(),
        Vector2f::zero(),
        NeEstimates {
            pos_m: Vector2::new(0.0, 0.0),
            vel_ms: Vector2f::zero(),
        },
    );
    assert!(!fired);
    assert!((targets.pos_target_m.x - 1.0).abs() < 1e-12);
}

/// When the stored error equals `target - old_estimate` and the estimate
/// jumps, MoveTarget shifts desired and target by that jump so the
/// error is unchanged.
#[test]
fn ne_ekf_move_target_preserves_the_stored_error() {
    let mut ekf = NeEkfReset::init(1);
    let mut targets = NeEkfTargets {
        pos_target_m: Vector2::new(5.0, 1.0),
        vel_target_ms: Vector2f::new(2.0, 0.0),
        pos_desired_m: Vector2::new(4.0, 1.0),
        vel_desired_ms: Vector2f::new(1.5, 0.0),
        offsets: NeOffsets {
            pos_m: Vector2::new(1.0, 0.0),
            vel_ms: Vector2f::new(0.5, 0.0),
            accel_mss: Vector2f::zero(),
        },
    };
    // Stored error is target - old_estimate = (5,1) - (3,1) = (2,0)
    // New estimate is (8,1); jump is +5 N.
    let fired = ekf.handle(
        99,
        EkfResetMethod::MoveTarget,
        &mut targets,
        Vector2f::new(2.0, 0.0),
        Vector2f::new(0.5, 0.0),
        NeEstimates {
            pos_m: Vector2::new(8.0, 1.0),
            vel_ms: Vector2f::new(3.0, 0.0),
        },
    );
    assert!(fired);
    assert_eq!(ekf.last_reset_ms, 99);
    // delta_pos = error - (target - new_est) = 2 - (5-8) = 5
    assert!((targets.pos_target_m.x - 10.0).abs() < 1e-6);
    assert!((targets.pos_desired_m.x - 9.0).abs() < 1e-6);
    // new_error = 10 - 8 = 2, unchanged
    assert!(((targets.pos_target_m.x - 8.0) - 2.0).abs() < 1e-6);
    // offsets must not move
    assert!((targets.offsets.pos_m.x - 1.0).abs() < 1e-12);
}

/// MoveVehicle puts the jump into the offset so Auto can slew onto the
/// new origin instead of dragging the trajectory.
#[test]
fn ne_ekf_move_vehicle_shifts_the_offset_not_desired() {
    let mut ekf = NeEkfReset::init(1);
    let mut targets = NeEkfTargets {
        pos_target_m: Vector2::new(5.0, 0.0),
        vel_target_ms: Vector2f::new(0.0, 0.0),
        pos_desired_m: Vector2::new(5.0, 0.0),
        vel_desired_ms: Vector2f::zero(),
        offsets: NeOffsets::default(),
    };
    let fired = ekf.handle(
        3,
        EkfResetMethod::MoveVehicle,
        &mut targets,
        Vector2f::new(1.0, 0.0),
        Vector2f::zero(),
        NeEstimates {
            pos_m: Vector2::new(7.0, 0.0),
            vel_ms: Vector2f::zero(),
        },
    );
    assert!(fired);
    // delta = 1 - (5-7) = 3
    assert!((targets.pos_desired_m.x - 5.0).abs() < 1e-12);
    assert!((targets.offsets.pos_m.x - 3.0).abs() < 1e-6);
}

/// NE fires on a zero timestamp if the latched value is not zero. The
/// vertical handler does not; a port that shared the guard would fail.
#[test]
fn ne_ekf_reset_fires_on_a_zero_timestamp() {
    let mut ekf = NeEkfReset::init(5);
    let mut targets = NeEkfTargets {
        pos_target_m: Vector2::new(0.0, 0.0),
        vel_target_ms: Vector2f::zero(),
        pos_desired_m: Vector2::new(0.0, 0.0),
        vel_desired_ms: Vector2f::zero(),
        offsets: NeOffsets::default(),
    };
    let fired = ekf.handle(
        0,
        EkfResetMethod::MoveTarget,
        &mut targets,
        Vector2f::zero(),
        Vector2f::zero(),
        NeEstimates {
            pos_m: Vector2::new(1.0, 0.0),
            vel_ms: Vector2f::zero(),
        },
    );
    assert!(fired);
    assert_eq!(ekf.last_reset_ms, 0);
}

/// When already active, init keeps the previous accel target (then
/// limits it) and seeds desired from estimate minus offset.
#[test]
fn ne_init_keeps_accel_when_active_and_subtracts_offsets() {
    let mut ne = PosControlNe::new();
    let mut offsets = NeOffsetState {
        current: NeOffsets::default(),
        target: NeOffsets {
            pos_m: Vector2::new(2.0, 1.0),
            vel_ms: Vector2f::new(0.25, 0.0),
            accel_mss: Vector2f::zero(),
        },
        target_ms: 1_000,
    };
    let mut vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let inp = ne_init_inputs();
    let out = ne.init_controller(&mut offsets, &mut vel_pid, &inp);

    assert!((out.pos_target_m.x - 10.0).abs() < 1e-12);
    assert!((ne.pos_desired_m.x - 8.0).abs() < 1e-12);
    assert!((ne.vel_desired_ms.x - 0.75).abs() < 1e-6);
    assert_eq!(ne.accel_desired_mss.x, 0.0);
    assert!((out.accel_target_mss.x - 0.3).abs() < 1e-6);
    assert_eq!(out.last_update_ticks, 50);
    assert_eq!(out.ekf.last_reset_ms, 7);
    assert_eq!(out.yaw_rate_target_rads, 0.0);
    assert_eq!(out.angle_max_override_rad, 0.0);
    assert!((out.roll_target_rad - 0.1).abs() < 1e-12);
}

/// When inactive, the accel target is the lean-to-accel map of the
/// attitude target with AHRS yaw swapped in — not the attitude yaw.
/// Heading east, a pitch-down demand is a westward acceleration.
#[test]
fn ne_init_seeds_accel_from_lean_when_inactive_using_ahrs_yaw() {
    let mut ne = PosControlNe::new();
    let mut offsets = NeOffsetState::default();
    let mut vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.25, 10.0, 0.0, 0.0);
    let mut inp = ne_init_inputs();
    inp.last_update_ticks = 0;
    inp.ticks = 10;
    inp.att_target_euler_rad = Vector3f::new(0.0, -0.2, 1.2);
    inp.ahrs_yaw = core::f32::consts::FRAC_PI_2;
    inp.accel_target_mss = Vector2f::new(9.0, 9.0);
    inp.lean_angle_max_rad = 1.0;

    let out = ne.init_controller(&mut offsets, &mut vel_pid, &inp);
    let mut att = inp.att_target_euler_rad;
    att.z = inp.ahrs_yaw;
    let expect = lean_angles_to_accel_ned(att);
    assert!(
        (out.accel_target_mss.x - expect.x).abs() < 1e-5,
        "got {} want {}",
        out.accel_target_mss.x,
        expect.x
    );
    assert!((out.accel_target_mss.y - expect.y).abs() < 1e-5);
    // Integrator is accel_target - vel_target * ff
    let expect_i = out.accel_target_mss - inp.estimates.vel_ms * vel_pid.ff();
    assert!((vel_pid.integrator().x - expect_i.x).abs() < 1e-5);
}

/// The lean-angle budget shortens an over-long accel target. A port
/// that skipped the limit would keep a command the attitude loop
/// cannot fly.
#[test]
fn ne_init_limits_accel_to_the_lean_angle_budget() {
    let mut ne = PosControlNe::new();
    let mut offsets = NeOffsetState::default();
    let mut vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut inp = ne_init_inputs();
    inp.accel_target_mss = Vector2f::new(20.0, 0.0);
    inp.lean_angle_max_rad = 0.2;
    let out = ne.init_controller(&mut offsets, &mut vel_pid, &inp);
    let max = ap_math::control::angle_rad_to_accel_mss(0.2);
    assert!((out.accel_target_mss.length() - max).abs() < 1e-4);
}

/// Stopping-point init parks the trajectory at the kinematic stop and
/// zeros desired velocity, leaving the velocity target as the estimate.
#[test]
fn ne_init_stopping_point_parks_desired() {
    let mut ne = PosControlNe::new();
    let mut offsets = NeOffsetState::default();
    let mut vel_pid = AcPid2d::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let inp = ne_init_inputs();
    let limits = ne_limits();
    let expect = stopping_point_ne(
        inp.estimates.pos_m,
        Vector2::new(0.0, 0.0),
        inp.estimates.vel_ms,
        Vector2f::zero(),
        1.0,
        &limits,
    );
    let out = ne.init_controller_stopping_point(&mut offsets, &mut vel_pid, &inp, 1.0, &limits);
    assert!((ne.pos_desired_m.x - expect.x).abs() < 1e-6);
    assert!((out.pos_target_m.x - expect.x).abs() < 1e-6);
    assert_eq!(ne.vel_desired_ms.x, 0.0);
    assert!((out.vel_target_ms.x - inp.estimates.vel_ms.x).abs() < 1e-12);
}

#[test]
fn d_init_offsets_snaps_or_zeros() {
    let mut state = DOffsetState {
        current: DOffsets {
            pos_m: 1.0,
            vel_ms: 0.1,
            accel_mss: 0.0,
        },
        target: DOffsets {
            pos_m: 2.5,
            vel_ms: -0.2,
            accel_mss: 0.05,
        },
        target_ms: 1_000,
    };
    state.init(1_200);
    assert!((state.current.pos_m - 2.5).abs() < 1e-12);

    state.target_ms = 0;
    state.init(5_000);
    assert_eq!(state.current.pos_m, 0.0);
    assert_eq!(state.target.pos_m, 0.0);
}

/// D update integrates current first, then shapes, then the target.
/// A matching at-rest pair must stay put; a timeout must pull home.
#[test]
fn d_update_offsets_is_idle_then_times_out_home() {
    let mut state = DOffsetState {
        current: DOffsets {
            pos_m: -3.0,
            ..DOffsets::default()
        },
        target: DOffsets {
            pos_m: -3.0,
            ..DOffsets::default()
        },
        target_ms: 1_000,
    };
    state.update(&d_limits(), 0.02, 1_000, 0.0, 0.0, 0.0);
    assert!((state.current.pos_m + 3.0).abs() < 1e-6);
    assert!(state.current.accel_mss.abs() < 1e-6);

    state.target_ms = 0;
    state.update(&d_limits(), 0.02, 5_000, 0.0, 0.0, 0.0);
    assert_eq!(state.target.pos_m, 0.0);
    assert!(
        state.current.accel_mss > 0.0,
        "offset at -3 must accelerate down toward zero, got {}",
        state.current.accel_mss
    );
}

/// A positive (downward) limit is ignored when advancing the current
/// offset: only `min(limit, 0)` is passed. An upward saturation must
/// still let a descent offset move.
#[test]
fn d_update_offsets_ignores_a_positive_limit() {
    let mut frozen = DOffsetState {
        current: DOffsets {
            pos_m: 0.0,
            vel_ms: 1.0,
            accel_mss: 0.0,
        },
        target: DOffsets {
            pos_m: 0.0,
            vel_ms: 1.0,
            accel_mss: 0.0,
        },
        target_ms: 1_000,
    };
    let mut unclipped = frozen;
    // limit = +1 with pos_error = +1 would freeze a 1-D advance if the
    // raw limit were used (delta_pos and pos_error both share the sign).
    frozen.update(&d_limits(), 0.02, 1_000, 1.0, 1.0, 1.0);
    unclipped.update(&d_limits(), 0.02, 1_000, 0.0, 1.0, 1.0);
    assert!(
        (frozen.current.pos_m - unclipped.current.pos_m).abs() < 1e-6,
        "positive limit must be clipped to zero before the advance"
    );
    assert!(
        frozen.current.pos_m > 0.0,
        "descent offset must still advance, got {}",
        frozen.current.pos_m
    );
}

#[test]
fn d_ekf_reset_ignores_a_zero_timestamp() {
    let mut ekf = DEkfReset::init(5);
    let mut targets = DEkfTargets {
        pos_target_m: 1.0,
        vel_target_ms: 0.0,
        pos_desired_m: 1.0,
        vel_desired_ms: 0.0,
        offsets: DOffsets::default(),
    };
    let fired = ekf.handle(
        0,
        EkfResetMethod::MoveTarget,
        &mut targets,
        0.0,
        0.0,
        DEstimates {
            pos_m: 4.0,
            vel_ms: 0.0,
        },
    );
    assert!(!fired);
    assert!((targets.pos_target_m - 1.0).abs() < 1e-12);
    assert_eq!(ekf.last_reset_ms, 5);
}

#[test]
fn d_ekf_move_target_preserves_the_stored_error() {
    let mut ekf = DEkfReset::init(1);
    let mut targets = DEkfTargets {
        pos_target_m: 5.0,
        vel_target_ms: 1.0,
        pos_desired_m: 4.0,
        vel_desired_ms: 0.5,
        offsets: DOffsets {
            pos_m: 1.0,
            vel_ms: 0.5,
            accel_mss: 0.0,
        },
    };
    // error = 2, new estimate = 8, delta = 2 - (5-8) = 5
    let fired = ekf.handle(
        9,
        EkfResetMethod::MoveTarget,
        &mut targets,
        2.0,
        0.5,
        DEstimates {
            pos_m: 8.0,
            vel_ms: 2.0,
        },
    );
    assert!(fired);
    assert!((targets.pos_target_m - 10.0).abs() < 1e-6);
    assert!((targets.pos_desired_m - 9.0).abs() < 1e-6);
    assert!((targets.offsets.pos_m - 1.0).abs() < 1e-12);
}

#[test]
fn d_ekf_move_vehicle_shifts_the_offset() {
    let mut ekf = DEkfReset::init(1);
    let mut targets = DEkfTargets {
        pos_target_m: 5.0,
        vel_target_ms: 0.0,
        pos_desired_m: 5.0,
        vel_desired_ms: 0.0,
        offsets: DOffsets::default(),
    };
    let fired = ekf.handle(
        4,
        EkfResetMethod::MoveVehicle,
        &mut targets,
        1.0,
        0.0,
        DEstimates {
            pos_m: 7.0,
            vel_ms: 0.0,
        },
    );
    assert!(fired);
    assert!((targets.pos_desired_m - 5.0).abs() < 1e-12);
    assert!((targets.offsets.pos_m - 3.0).abs() < 1e-6);
}

/// Vertical init zeros terrain, subtracts offsets, seeds the accel PID
/// I term so the first throttle equals the throttle already being sent,
/// and constrains the accel target.
#[test]
fn d_init_subtracts_offsets_and_seeds_the_accel_integrator() {
    let mut d = PosControlD::new();
    let mut offsets = DOffsetState {
        current: DOffsets::default(),
        target: DOffsets {
            pos_m: 1.5,
            vel_ms: 0.2,
            accel_mss: 0.1,
        },
        target_ms: 1_000,
    };
    let mut vel_pid = AcPidBasic::new(1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = AcPid::new(PidGains {
        p: 0.5,
        i: 0.0,
        d: 0.0,
        ff: 0.25,
        dff: 0.0,
        imax: 1.0,
        pdmax: 0.0,
        filt_t_hz: 0.0,
        filt_e_hz: 0.0,
        filt_d_hz: 0.0,
        srmax: 0.0,
        srtau: 1.0,
    });
    let inp = d_init_inputs();
    let out = d.init_controller(&mut offsets, &mut vel_pid, &mut accel_pid, &inp);

    assert!((out.pos_target_m - 8.0).abs() < 1e-12);
    assert!((d.pos_desired_m - 6.5).abs() < 1e-12);
    assert!((d.vel_desired_ms - 0.2).abs() < 1e-6);
    assert!((out.accel_target_mss - 0.2).abs() < 1e-6);
    assert!((d.accel_desired_mss - 0.1).abs() < 1e-6);
    assert_eq!(out.terrain.pos_m, 0.0);
    assert_eq!(vel_pid.integrator(), 0.0);
    // I = -(0.45-0.35) - 0.5*(0.2-0.2) - 0.25*0.2 = -0.10 - 0.05 = -0.15
    assert!((accel_pid.integrator() + 0.15).abs() < 1e-5);
    assert_eq!(out.ekf.last_reset_ms, 7);
}

#[test]
fn d_init_constrains_accel_to_the_configured_limit() {
    let mut d = PosControlD::new();
    let mut offsets = DOffsetState::default();
    let mut vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(0.0);
    let mut inp = d_init_inputs();
    inp.estimated_accel_d_mss = 9.0;
    inp.accel_max_d_mss = 2.5;
    let out = d.init_controller(&mut offsets, &mut vel_pid, &mut accel_pid, &inp);
    assert!((out.accel_target_mss - 2.5).abs() < 1e-6);

    inp.estimated_accel_d_mss = -9.0;
    let out = d.init_controller(&mut offsets, &mut vel_pid, &mut accel_pid, &inp);
    assert!((out.accel_target_mss + 2.5).abs() < 1e-6);
}

/// no_descent clips every downward velocity and acceleration after init.
#[test]
fn d_init_no_descent_clips_positive_rates() {
    let mut d = PosControlD::new();
    let mut offsets = DOffsetState {
        current: DOffsets::default(),
        target: DOffsets {
            pos_m: 0.0,
            vel_ms: 1.0,
            accel_mss: 0.5,
        },
        target_ms: 1_000,
    };
    let mut vel_pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0);
    let mut accel_pid = accel_p(0.0);
    let mut inp = d_init_inputs();
    inp.estimates.vel_ms = 0.8;
    inp.estimated_accel_d_mss = 1.0;
    let mut terrain = DTerrain {
        pos_m: 0.0,
        vel_ms: 0.3,
        accel_mss: 0.2,
    };
    let out = d.init_controller_no_descent(
        &mut offsets,
        &mut vel_pid,
        &mut accel_pid,
        &inp,
        &mut terrain,
    );
    // vel_target 0.8 clips to 0; desired is estimate-offset = -0.2 (climb)
    // and MIN keeps negatives. Offset vel 1.0 clips to 0.
    assert_eq!(out.vel_target_ms, 0.0);
    assert!((d.vel_desired_ms + 0.2).abs() < 1e-6);
    assert_eq!(offsets.current.vel_ms, 0.0);
    assert_eq!(out.accel_target_mss, 0.0);
    assert_eq!(d.accel_desired_mss, 0.0);
    assert_eq!(offsets.current.accel_mss, 0.0);
    // init_terrain zeros first; the clip then has nothing downward left.
    assert_eq!(terrain.vel_ms, 0.0);
    assert_eq!(terrain.accel_mss, 0.0);
}

#[test]
fn init_terrain_is_all_zeros() {
    let t = init_terrain();
    assert_eq!(t.pos_m, 0.0);
    assert_eq!(t.vel_ms, 0.0);
    assert_eq!(t.accel_mss, 0.0);
}

#[test]
fn update_terrain_is_idle_when_current_equals_target_at_rest() {
    let mut terrain = init_terrain();
    terrain.pos_m = -3.0;
    update_terrain(&mut terrain, -3.0, &d_limits(), 0.02, 0.0, 0.0, 0.0);
    assert!((terrain.pos_m + 3.0).abs() < 1e-6);
    assert!(terrain.accel_mss.abs() < 1e-6);
}

#[test]
fn update_terrain_shapes_toward_the_target() {
    let mut terrain = init_terrain();
    terrain.pos_m = -3.0;
    update_terrain(&mut terrain, 0.0, &d_limits(), 0.02, 0.0, 0.0, 0.0);
    assert!(
        terrain.accel_mss > 0.0,
        "terrain at -3 must accelerate down toward zero, got {}",
        terrain.accel_mss
    );
}

#[test]
fn update_terrain_ignores_a_positive_limit() {
    let mut frozen = init_terrain();
    frozen.vel_ms = 1.0;
    let mut unclipped = frozen;
    update_terrain(&mut frozen, 0.0, &d_limits(), 0.02, 1.0, 1.0, 1.0);
    update_terrain(&mut unclipped, 0.0, &d_limits(), 0.02, 0.0, 1.0, 1.0);
    assert!(
        (frozen.pos_m - unclipped.pos_m).abs() < 1e-6,
        "positive limit must be clipped to zero before the advance"
    );
    assert!(
        frozen.pos_m > 0.0,
        "descent terrain must still advance, got {}",
        frozen.pos_m
    );
}

#[test]
fn update_terrain_does_not_move_the_target() {
    let mut terrain = init_terrain();
    let target = -4.0;
    update_terrain(&mut terrain, target, &d_limits(), 0.02, 0.0, 0.0, 0.0);
    let first = terrain.accel_mss;
    update_terrain(&mut terrain, target, &d_limits(), 0.02, 0.0, 0.0, 0.0);
    assert!(
        first < 0.0,
        "zero current toward a negative target must accelerate up, got {first}"
    );
}
