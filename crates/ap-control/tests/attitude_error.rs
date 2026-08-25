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
