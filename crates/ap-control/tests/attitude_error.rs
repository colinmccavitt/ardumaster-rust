//! The attitude error decomposition.
//!
//! The point of the split is that a thrust-vector error and a heading error
//! are different kinds of problem, so the tests are mostly about keeping them
//! *separate*: a pure heading command must produce no roll or pitch error, and
//! a pure lean command must produce no yaw error. A port that got the frame
//! conversion wrong, or decomposed one quaternion instead of two, leaks one
//! into the other — and that leak is invisible on a level, north-facing
//! aircraft, which is why every case here is neither.

use ap_control::attitude_error::thrust_vector_rotation_angles;
use ap_math::quaternion::Quaternion;

fn att(roll: f32, pitch: f32, yaw: f32) -> Quaternion {
    Quaternion::from_euler(roll, pitch, yaw)
}

fn close(a: f32, b: f32, tol: f32) -> bool {
    libm::fabsf(a - b) < tol
}

/// No error at all when the two attitudes agree.
#[test]
fn matching_attitudes_give_no_error() {
    for &(r, p, y) in &[(0.0_f32, 0.0_f32, 0.0_f32), (0.3, -0.4, 1.1)] {
        let a = att(r, p, y);
        let e = thrust_vector_rotation_angles(a, a);

        assert!(close(e.error_rad.x, 0.0, 1e-5), "roll {}", e.error_rad.x);
        assert!(close(e.error_rad.y, 0.0, 1e-5), "pitch {}", e.error_rad.y);
        assert!(close(e.error_rad.z, 0.0, 1e-5), "yaw {}", e.error_rad.z);
        assert!(close(e.thrust_error_angle_rad, 0.0, 1e-5));
    }
}

/// A pure heading difference produces yaw error and nothing else.
///
/// Built as a rotation about the body's own thrust axis, which is what
/// "heading only" means. Note it is *not* the same as holding Euler roll and
/// pitch and changing Euler yaw: those angles are applied relative to the
/// yawed frame, so on a leaning aircraft that moves the thrust vector as well
/// — by 0.19 rad in the attitude used below, which is how this test found its
/// own first version wrong.
///
/// Deliberately leaning, because on a level aircraft the distinction vanishes
/// and the test would pass with the frame conversion omitted.
#[test]
fn a_pure_heading_error_stays_in_yaw() {
    use ap_math::vector3::Vector3f;

    let body = att(0.35, -0.2, 0.4);
    let heading_change = 0.5_f32;
    // About the body thrust axis: heading, by construction.
    let target = body * Quaternion::from_axis_angle(Vector3f::new(0.0, 0.0, -1.0), heading_change);

    let e = thrust_vector_rotation_angles(target, body);

    assert!(
        close(e.thrust_error_angle_rad, 0.0, 1e-4),
        "thrust vectors should already agree, got {}",
        e.thrust_error_angle_rad
    );
    assert!(
        close(e.error_rad.x, 0.0, 1e-4),
        "roll leaked: {}",
        e.error_rad.x
    );
    assert!(
        close(e.error_rad.y, 0.0, 1e-4),
        "pitch leaked: {}",
        e.error_rad.y
    );
    assert!(
        close(libm::fabsf(e.error_rad.z), heading_change, 1e-3),
        "yaw error should be the {heading_change} rad rotation, got {}",
        e.error_rad.z
    );
}

/// A pure lean difference produces no yaw error.
///
/// The mirror of the test above, and the one that catches a missing
/// inertial-to-body conversion of the rotation axis: with the same heading,
/// the leftover rotation after the thrust correction must be nothing.
#[test]
fn a_pure_lean_error_stays_out_of_yaw() {
    let body = att(0.0, 0.0, 1.2);
    let target = att(0.3, 0.0, 1.2);

    let e = thrust_vector_rotation_angles(target, body);

    assert!(
        close(e.thrust_error_angle_rad, 0.3, 1e-3),
        "thrust error should be the 0.3 rad lean, got {}",
        e.thrust_error_angle_rad
    );
    assert!(
        close(e.error_rad.z, 0.0, 1e-3),
        "yaw should be untouched, got {}",
        e.error_rad.z
    );
    // And the correction lands in roll, the axis actually leaned about.
    assert!(
        close(e.error_rad.x, 0.3, 1e-3),
        "roll error should carry it, got {}",
        e.error_rad.x
    );
}

/// The reported lean angle is the *current* attitude's, not the error.
///
/// Easy to conflate: both are angles about the thrust axis. This one is used
/// for limiting against the maximum lean, so reporting the error instead would
/// let a badly-leaning aircraft look upright as soon as its target caught up.
#[test]
fn the_lean_angle_describes_the_body_not_the_error() {
    let body = att(0.0, 0.4, 0.0);

    // Target matches: the error is zero but the aircraft is still leaning.
    let matched = thrust_vector_rotation_angles(body, body);
    assert!(
        close(matched.thrust_error_angle_rad, 0.0, 1e-5),
        "no error expected"
    );
    assert!(
        close(matched.thrust_angle_rad, 0.4, 1e-3),
        "lean angle should still be 0.4, got {}",
        matched.thrust_angle_rad
    );

    // Level body, leaning target: error but no lean.
    let level = att(0.0, 0.0, 0.0);
    let e = thrust_vector_rotation_angles(body, level);
    assert!(close(e.thrust_angle_rad, 0.0, 1e-3), "body is level");
    assert!(
        close(e.thrust_error_angle_rad, 0.4, 1e-3),
        "but the error is 0.4"
    );
}

/// Inverted flight is handled rather than producing a NaN.
///
/// The cross product of two antiparallel thrust vectors has no direction, and
/// upstream substitutes the thrust axis. A multirotor does not fly there, but
/// the decomposition runs before anything decides that.
#[test]
fn antiparallel_thrust_does_not_produce_a_nan() {
    let level = att(0.0, 0.0, 0.0);
    let inverted = att(core::f32::consts::PI, 0.0, 0.0);

    let e = thrust_vector_rotation_angles(inverted, level);

    assert!(e.error_rad.x.is_finite(), "roll error is NaN");
    assert!(e.error_rad.y.is_finite(), "pitch error is NaN");
    assert!(e.error_rad.z.is_finite(), "yaw error is NaN");
    assert!(e.thrust_error_angle_rad.is_finite());
    assert!(
        close(e.thrust_error_angle_rad, core::f32::consts::PI, 1e-3),
        "the thrust vectors are opposed, got {}",
        e.thrust_error_angle_rad
    );
}

/// Applying both corrections in order takes the body attitude to the target.
///
/// The strongest property available without a fixture: the decomposition is
/// only meaningful if the two rotations actually compose back to the whole.
#[test]
fn the_two_corrections_compose_back_to_the_target() {
    for &(br, bp, by, tr, tp, ty) in &[
        (0.0_f32, 0.0_f32, 0.0_f32, 0.2_f32, -0.3_f32, 0.7_f32),
        (0.35, -0.2, 0.4, -0.1, 0.5, 1.9),
        (-0.5, 0.6, -1.2, 0.4, -0.4, 0.2),
    ] {
        let body = att(br, bp, by);
        let target = att(tr, tp, ty);
        let e = thrust_vector_rotation_angles(target, body);

        // body * thrust_correction * heading_correction == target
        let heading = e.thrust_vector_correction.inverse() * body.inverse() * target;
        let rebuilt = body * e.thrust_vector_correction * heading;

        let (rr, rp, ry) = rebuilt.to_euler();
        assert!(close(rr, tr, 1e-3), "roll {rr} != {tr}");
        assert!(close(rp, tp, 1e-3), "pitch {rp} != {tp}");
        assert!(close(ry, ty, 1e-3), "yaw {ry} != {ty}");
    }
}

mod command_model {
    use ap_control::attitude_error::{attitude_command_model, CommandModel};

    fn close(a: f32, b: f32, tol: f32) -> bool {
        libm::fabsf(a - b) < tol
    }

    const DT: f32 = 0.0025;

    /// A non-positive dt leaves the state alone.
    ///
    /// A paused controller must not integrate. Returning a zeroed state
    /// instead would drop the rate target to nothing on the first stalled
    /// iteration, which is a step input to everything downstream.
    #[test]
    fn a_stopped_clock_does_not_advance_the_model() {
        let state = CommandModel {
            target_ang_vel: 0.7,
            target_ang_accel: 3.0,
        };

        for dt in [0.0_f32, -0.001] {
            let out = attitude_command_model(state, 0.5, 0.0, 2.0, 10.0, 0.15, dt);
            assert_eq!(out, state, "dt {dt} should change nothing");
        }
    }

    /// A held error produces a steady rate, not a diverging one.
    ///
    /// Note the rate does *not* go to zero. A held error is not physical --
    /// upstream integrates the attitude between calls, so the error shrinks as
    /// the aircraft turns -- and asking the model to chase a target that never
    /// gets closer gets a constant rate back, which is the right answer to the
    /// wrong question. What matters is that it converges rather than winding
    /// up.
    #[test]
    fn a_held_error_produces_a_steady_rate() {
        let mut state = CommandModel::default();
        let error = 0.4_f32;

        for _ in 0..4000 {
            state = attitude_command_model(state, error, 0.0, 5.0, 20.0, 0.15, DT);
        }
        let settled = state.target_ang_vel;

        // Another thousand iterations must not move it.
        for _ in 0..1000 {
            state = attitude_command_model(state, error, 0.0, 5.0, 20.0, 0.15, DT);
        }

        assert!(
            close(state.target_ang_vel, settled, 1e-3),
            "rate drifted from {settled} to {} -- it should have converged",
            state.target_ang_vel
        );
        assert!(
            state.target_ang_vel.is_finite() && libm::fabsf(state.target_ang_vel) > 0.0,
            "and should be a real, non-zero rate: {}",
            state.target_ang_vel
        );
    }

    /// A larger error asks for a larger rate.
    ///
    /// The direction of the relationship, which a sign or scaling error in the
    /// shaping would invert without making anything non-finite.
    #[test]
    fn a_larger_error_asks_for_a_larger_rate() {
        let settle = |error: f32| {
            let mut state = CommandModel::default();
            for _ in 0..4000 {
                state = attitude_command_model(state, error, 0.0, 5.0, 20.0, 0.15, DT);
            }
            state.target_ang_vel
        };

        let small = settle(0.1);
        let large = settle(0.8);

        assert!(
            large > small && small > 0.0,
            "0.8 rad should out-command 0.1 rad: {large} vs {small}"
        );
    }

    /// The rate target respects the maximum.
    #[test]
    fn the_rate_target_is_limited() {
        let max = 1.0_f32;
        let mut state = CommandModel::default();

        let mut peak = 0.0_f32;
        for _ in 0..2000 {
            // A large error, so the limit is what binds rather than the error.
            state = attitude_command_model(state, 3.0, 0.0, max, 20.0, 0.15, DT);
            peak = peak.max(libm::fabsf(state.target_ang_vel));
        }

        assert!(
            peak <= max + 1e-3,
            "rate reached {peak}, above the {max} limit"
        );
        assert!(
            peak > max * 0.9,
            "the limit should actually be reached, got {peak}"
        );
    }

    /// A zero acceleration limit falls back rather than dividing by zero.
    ///
    /// Upstream substitutes 1800 deg/s², which is effectively no limit on any
    /// real airframe — the fallback exists to keep the jerk limit
    /// (`accel_max / input_tc`) finite, not to describe a vehicle.
    #[test]
    fn a_missing_acceleration_limit_falls_back() {
        let mut state = CommandModel::default();

        for _ in 0..200 {
            state = attitude_command_model(state, 0.5, 0.0, 5.0, 0.0, 0.15, DT);
            assert!(
                state.target_ang_vel.is_finite() && state.target_ang_accel.is_finite(),
                "the fallback should keep everything finite"
            );
        }

        assert!(
            libm::fabsf(state.target_ang_vel) > 0.0,
            "and the model should still be moving"
        );
    }

    /// A zero input time constant falls back to ten loop cycles.
    #[test]
    fn a_missing_time_constant_falls_back() {
        let mut state = CommandModel::default();

        for _ in 0..200 {
            state = attitude_command_model(state, 0.5, 0.0, 5.0, 20.0, 0.0, DT);
            assert!(state.target_ang_vel.is_finite());
        }

        assert!(libm::fabsf(state.target_ang_vel) > 0.0);
    }

    /// A smaller time constant gives a sharper response.
    ///
    /// `input_tc` sets the jerk limit as `accel_max / input_tc`, so halving it
    /// doubles how fast the acceleration may change. If this comes out
    /// backwards, the parameter is inverted somewhere.
    #[test]
    fn a_smaller_time_constant_responds_faster() {
        let run = |tc: f32| {
            let mut state = CommandModel::default();
            let mut peak = 0.0_f32;
            for _ in 0..40 {
                state = attitude_command_model(state, 1.0, 0.0, 5.0, 20.0, tc, DT);
                peak = peak.max(libm::fabsf(state.target_ang_vel));
            }
            peak
        };

        let sharp = run(0.05);
        let soft = run(0.30);

        assert!(
            sharp > soft,
            "a 0.05 s constant should outrun a 0.30 s one: {sharp} vs {soft}"
        );
    }
}

mod thrust_vector_attitude {
    use ap_control::attitude_error::{attitude_from_thrust_vector, thrust_vector_rotation_angles};
    use ap_math::vector3::Vector3f;

    fn close(a: f32, b: f32, tol: f32) -> bool {
        libm::fabsf(a - b) < tol
    }

    /// Straight up with no heading is level and north-facing.
    #[test]
    fn straight_up_is_level() {
        let q = attitude_from_thrust_vector(Vector3f::new(0.0, 0.0, -1.0), 0.0);
        let (roll, pitch, yaw) = q.to_euler();

        assert!(close(roll, 0.0, 1e-5), "roll {roll}");
        assert!(close(pitch, 0.0, 1e-5), "pitch {pitch}");
        assert!(close(yaw, 0.0, 1e-5), "yaw {yaw}");
    }

    /// A zero-length thrust vector is read as straight up, not rejected.
    ///
    /// A caller that has not computed a demand yet gets a usable attitude
    /// rather than a NaN one, which would poison everything downstream.
    #[test]
    fn a_zero_thrust_vector_is_read_as_up() {
        let q = attitude_from_thrust_vector(Vector3f::new(0.0, 0.0, 0.0), 0.0);
        let (roll, pitch, _) = q.to_euler();

        assert!(roll.is_finite() && pitch.is_finite(), "should not be NaN");
        assert!(close(roll, 0.0, 1e-5) && close(pitch, 0.0, 1e-5));
    }

    /// The magnitude of the thrust vector does not matter, only its direction.
    #[test]
    fn only_the_direction_of_thrust_matters() {
        let unit = attitude_from_thrust_vector(Vector3f::new(0.3, -0.2, -0.9), 0.7);
        let scaled = attitude_from_thrust_vector(Vector3f::new(3.0, -2.0, -9.0), 0.7);

        let (r1, p1, y1) = unit.to_euler();
        let (r2, p2, y2) = scaled.to_euler();

        assert!(close(r1, r2, 1e-4) && close(p1, p2, 1e-4) && close(y1, y2, 1e-4));
    }

    /// The attitude it builds points its thrust where it was asked to.
    ///
    /// Checked by running the result back through the decomposition against a
    /// level reference: the lean angle should match the tilt requested. This
    /// ties the two directions of the same idea together, which neither on its
    /// own would.
    #[test]
    fn the_built_attitude_points_its_thrust_as_asked() {
        // A thrust vector tilted 0.3 rad from vertical.
        let tilt = 0.3_f32;
        let thrust = Vector3f::new(libm::sinf(tilt), 0.0, -libm::cosf(tilt));

        let q = attitude_from_thrust_vector(thrust, 0.0);
        let level = ap_math::quaternion::Quaternion::identity();
        let e = thrust_vector_rotation_angles(q, level);

        assert!(
            close(e.thrust_error_angle_rad, tilt, 1e-3),
            "asked for {tilt} rad of tilt, decomposition sees {}",
            e.thrust_error_angle_rad
        );
    }
}

mod rate_target {
    use ap_control::attitude_error::{update_ang_vel_target_from_att_error, AngleGains};
    use ap_math::vector3::Vector3f;

    fn gains(use_sqrt: bool, accel: f32) -> AngleGains {
        AngleGains {
            angle_p_roll: 6.0,
            angle_p_pitch: 6.0,
            angle_p_yaw: 4.0,
            accel_roll_max_radss: accel,
            accel_pitch_max_radss: accel,
            accel_yaw_max_radss: accel,
            use_sqrt_controller: use_sqrt,
        }
    }

    /// Without the square-root controller it is a plain proportional gain.
    #[test]
    fn the_proportional_path_is_gain_times_error() {
        let g = gains(false, 10.0);
        let out = update_ang_vel_target_from_att_error(Vector3f::new(0.1, 0.2, 0.3), &g, 0.0025);

        assert!(libm::fabsf(out.x - 6.0 * 0.1) < 1e-5, "roll {}", out.x);
        assert!(libm::fabsf(out.y - 6.0 * 0.2) < 1e-5, "pitch {}", out.y);
        assert!(libm::fabsf(out.z - 4.0 * 0.3) < 1e-5, "yaw {}", out.z);
    }

    /// A zero acceleration limit falls back to the proportional path.
    ///
    /// Per axis, not per vehicle: this is what lets a vehicle run the
    /// square-root controller on roll and pitch and proportional on yaw.
    #[test]
    fn a_zero_acceleration_limit_falls_back_per_axis() {
        let mut g = gains(true, 10.0);
        g.accel_yaw_max_radss = 0.0;

        let error = Vector3f::new(0.5, 0.5, 0.5);
        let out = update_ang_vel_target_from_att_error(error, &g, 0.0025);

        // Yaw took the proportional path exactly.
        assert!(libm::fabsf(out.z - 4.0 * 0.5) < 1e-5, "yaw {}", out.z);
        // Roll did not: at this error the sqrt controller is below the gain.
        assert!(
            out.x < 6.0 * 0.5,
            "roll should be shaped, got {} vs proportional {}",
            out.x,
            6.0 * 0.5
        );
    }

    /// The square-root controller holds back on large errors.
    ///
    /// That is its purpose: a proportional gain on a big error commands a rate
    /// the aircraft cannot stop from before overshooting. Small errors are
    /// where the two agree.
    #[test]
    fn the_sqrt_controller_limits_large_errors() {
        let g = gains(true, 10.0);
        let p = gains(false, 10.0);

        let small = Vector3f::new(0.01, 0.0, 0.0);
        let large = Vector3f::new(1.5, 0.0, 0.0);

        let sqrt_small = update_ang_vel_target_from_att_error(small, &g, 0.0025).x;
        let prop_small = update_ang_vel_target_from_att_error(small, &p, 0.0025).x;
        assert!(
            libm::fabsf(sqrt_small - prop_small) < 0.01,
            "small errors should agree: {sqrt_small} vs {prop_small}"
        );

        let sqrt_large = update_ang_vel_target_from_att_error(large, &g, 0.0025).x;
        let prop_large = update_ang_vel_target_from_att_error(large, &p, 0.0025).x;
        assert!(
            sqrt_large < prop_large,
            "large errors should be held back: {sqrt_large} vs {prop_large}"
        );
    }

    /// Sign is preserved on every axis.
    #[test]
    fn the_rate_target_follows_the_sign_of_the_error() {
        let g = gains(true, 10.0);
        let out = update_ang_vel_target_from_att_error(Vector3f::new(-0.4, 0.4, -0.4), &g, 0.0025);

        assert!(out.x < 0.0, "roll {}", out.x);
        assert!(out.y > 0.0, "pitch {}", out.y);
        assert!(out.z < 0.0, "yaw {}", out.z);
    }
}

mod yaw_limiting {
    use ap_control::attitude_error::{thrust_heading_rotation_angles, YawLimitGains};
    use ap_math::quaternion::Quaternion;
    use ap_math::vector3::Vector3f;

    fn att(roll: f32, pitch: f32, yaw: f32) -> Quaternion {
        Quaternion::from_euler(roll, pitch, yaw)
    }

    fn gains() -> YawLimitGains {
        YawLimitGains {
            accel_yaw_max_radss: 2.0,
            rate_yaw_kp: 0.2,
            angle_yaw_kp: 4.0,
            ..YawLimitGains::default()
        }
    }

    /// Without a yaw rate gain there is nothing to derive a limit from.
    ///
    /// Upstream skips the whole block, leaving the error as the decomposition
    /// produced it. A port that fell back to the 45-degree cap instead would
    /// start limiting a vehicle upstream leaves alone.
    #[test]
    fn no_rate_gain_means_no_limiting() {
        let mut g = gains();
        g.rate_yaw_kp = 0.0;

        let body = att(0.0, 0.0, 0.0);
        let target = att(0.0, 0.0, 2.5); // a large heading error

        let (out_target, e) = thrust_heading_rotation_angles(target, body, &g);

        assert!(
            libm::fabsf(e.error_rad.z) > 1.0,
            "the error should be untouched, got {}",
            e.error_rad.z
        );
        assert_eq!(out_target, target, "and the target should not be rebuilt");
    }

    /// Without an angle gain the limit is computed but not applied.
    #[test]
    fn no_angle_gain_means_no_limiting() {
        let mut g = gains();
        g.angle_yaw_kp = 0.0;

        let body = att(0.0, 0.0, 0.0);
        let target = att(0.0, 0.0, 2.5);

        let (out_target, e) = thrust_heading_rotation_angles(target, body, &g);

        assert!(libm::fabsf(e.error_rad.z) > 1.0, "got {}", e.error_rad.z);
        assert_eq!(out_target, target);
    }

    /// A small heading error passes through untouched.
    #[test]
    fn a_small_heading_error_is_not_limited() {
        let g = gains();
        let body = att(0.0, 0.0, 0.0);
        let target = att(0.0, 0.0, 0.05);

        let (out_target, e) = thrust_heading_rotation_angles(target, body, &g);

        assert!(
            libm::fabsf(libm::fabsf(e.error_rad.z) - 0.05) < 1e-3,
            "got {}",
            e.error_rad.z
        );
        assert_eq!(out_target, target, "no rebuild for an unlimited error");
    }

    /// A large heading error is capped, and the target is rebuilt to match.
    ///
    /// Rebuilding is the part worth pinning. Clamping only the error would
    /// leave the target unreachable, so the same oversized error would
    /// reappear on the next iteration and the aircraft would sit against the
    /// limit indefinitely.
    #[test]
    fn a_large_heading_error_is_capped_and_the_target_rebuilt() {
        let g = gains();
        let body = att(0.0, 0.0, 0.0);
        let target = att(0.0, 0.0, 2.5);

        let (out_target, e) = thrust_heading_rotation_angles(target, body, &g);

        let capped = libm::fabsf(e.error_rad.z);
        assert!(
            capped < 1.0,
            "a 2.5 rad error should be capped well below it, got {capped}"
        );
        assert_ne!(out_target, target, "the target must be rebuilt");

        // And the rebuilt target must actually produce the capped error, or
        // the next iteration would see something different again.
        let (_, again) = thrust_heading_rotation_angles(out_target, body, &g);
        assert!(
            libm::fabsf(libm::fabsf(again.error_rad.z) - capped) < 1e-3,
            "the rebuilt target should reproduce the capped error: {} vs {capped}",
            again.error_rad.z
        );
    }

    /// The cap never exceeds 45 degrees however generous the gains.
    #[test]
    fn the_cap_is_never_looser_than_forty_five_degrees() {
        let mut g = gains();
        // Absurdly capable yaw axis.
        g.rate_yaw_kp = 100.0;
        g.angle_yaw_kp = 100.0;
        g.accel_yaw_max_radss = 1000.0;

        let body = att(0.0, 0.0, 0.0);
        let target = att(0.0, 0.0, 3.0);

        let (_, e) = thrust_heading_rotation_angles(target, body, &g);

        let forty_five = 45.0_f32.to_radians();
        assert!(
            libm::fabsf(e.error_rad.z) <= forty_five + 1e-3,
            "cap should hold at 45 degrees, got {} rad",
            e.error_rad.z
        );
    }

    /// The thrust error is untouched by yaw limiting.
    ///
    /// The point of the cap is to protect thrust authority, so it would be
    /// self-defeating if capping yaw disturbed the thrust correction.
    #[test]
    fn limiting_yaw_leaves_the_thrust_correction_alone() {
        let g = gains();
        let body = att(0.2, -0.1, 0.0);
        let target = att(0.5, 0.3, 2.5);

        let unlimited = {
            let mut g = g;
            g.rate_yaw_kp = 0.0; // disables limiting
            thrust_heading_rotation_angles(target, body, &g).1
        };
        let limited = thrust_heading_rotation_angles(target, body, &g).1;

        assert!(
            libm::fabsf(limited.thrust_error_angle_rad - unlimited.thrust_error_angle_rad) < 1e-5,
            "thrust error changed: {} vs {}",
            limited.thrust_error_angle_rad,
            unlimited.thrust_error_angle_rad
        );
        assert!(
            libm::fabsf(limited.error_rad.x - unlimited.error_rad.x) < 1e-5
                && libm::fabsf(limited.error_rad.y - unlimited.error_rad.y) < 1e-5,
            "roll or pitch error moved"
        );
    }

    /// A more capable yaw axis gets a *tighter* cap, not a looser one.
    ///
    /// This reads backwards at first. The cap is the heading error that would
    /// just saturate the yaw output, and a more aggressive controller reaches
    /// that output at a smaller error — so more authority means a smaller
    /// cap. Which is right, once you notice what the cap is for: carrying more
    /// error than saturates the output buys nothing.
    ///
    /// The rate gain here is 3.0 rather than the 0.2 used elsewhere in this
    /// module, because at 0.2 both derived caps (71.6 and 6.3 rad) sit far
    /// above the 45-degree ceiling, the ceiling binds for both, and the test
    /// would compare two identical numbers and pass on any implementation.
    #[test]
    fn a_more_capable_yaw_axis_is_capped_tighter() {
        let body = att(0.0, 0.0, 0.0);
        let target = att(0.0, 0.0, 2.5);

        let cap_for = |accel: f32| {
            let mut g = gains();
            g.rate_yaw_kp = 3.0;
            g.accel_yaw_max_radss = accel;
            libm::fabsf(
                thrust_heading_rotation_angles(target, body, &g)
                    .1
                    .error_rad
                    .z,
            )
        };

        let weak = cap_for(0.3);
        let strong = cap_for(4.0);

        let forty_five = 45.0_f32.to_radians();
        assert!(
            weak < forty_five && strong < forty_five,
            "both caps must be below the ceiling or this compares nothing: \
             {weak} and {strong}"
        );
        assert!(
            strong < weak,
            "more authority should cap tighter: {strong} vs {weak}"
        );
    }

    /// The rebuilt target is reachable: applying the capped error to the body
    /// attitude gives it back.
    #[test]
    fn the_rebuilt_target_is_consistent_with_the_body() {
        let g = gains();
        let body = att(0.3, -0.2, 0.4);
        let target = att(0.3, -0.2, 3.0);

        let (out_target, e) = thrust_heading_rotation_angles(target, body, &g);

        let heading = Quaternion::from_rotation_vector(Vector3f::new(0.0, 0.0, e.error_rad.z));
        let rebuilt = body * e.thrust_vector_correction * heading;

        let (rr, rp, ry) = rebuilt.to_euler();
        let (tr, tp, ty) = out_target.to_euler();
        assert!(
            libm::fabsf(rr - tr) < 1e-3
                && libm::fabsf(rp - tp) < 1e-3
                && libm::fabsf(ry - ty) < 1e-3,
            "rebuilt {rr},{rp},{ry} != target {tr},{tp},{ty}"
        );
    }
}
