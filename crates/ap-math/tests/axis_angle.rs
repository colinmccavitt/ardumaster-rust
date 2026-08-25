//! Axis-angle conversions and vector rotation.
//!
//! These are what the attitude controller's error decomposition is written in,
//! so their edge behaviour matters more than their happy path: a rotation of
//! nearly a full turn has to come back as a small negative angle, not a large
//! positive one, or the controller commands the long way round.

use ap_math::quaternion::Quaternion;
use ap_math::vector3::Vector3f;

fn close(a: f32, b: f32, tol: f32) -> bool {
    libm::fabsf(a - b) < tol
}

/// An axis and angle round-trip through the quaternion.
#[test]
fn axis_angle_round_trips() {
    let axis = Vector3f::new(0.0, 0.0, 1.0);

    for angle in [0.1_f32, 0.7, 1.5, 3.0] {
        let q = Quaternion::from_axis_angle(axis, angle);
        let back = q.to_axis_angle();

        assert!(
            close(back.z, angle, 1e-5),
            "{angle} came back as {}",
            back.z
        );
        assert!(close(back.x, 0.0, 1e-6) && close(back.y, 0.0, 1e-6));
    }
}

/// A rotation vector's length is its angle.
#[test]
fn a_rotation_vector_carries_its_angle_as_length() {
    let v = Vector3f::new(0.0, 0.3, 0.4); // length 0.5
    let q = Quaternion::from_rotation_vector(v);
    let back = q.to_axis_angle();

    assert!(close(back.y, 0.3, 1e-5), "got {}", back.y);
    assert!(close(back.z, 0.4, 1e-5), "got {}", back.z);
}

/// Zero rotations give the identity, not a degenerate quaternion.
#[test]
fn a_zero_rotation_is_the_identity() {
    let from_axis = Quaternion::from_axis_angle(Vector3f::new(0.0, 0.0, 1.0), 0.0);
    let from_vector = Quaternion::from_rotation_vector(Vector3f::new(0.0, 0.0, 0.0));

    for q in [from_axis, from_vector] {
        assert!(close(q.q1, 1.0, 1e-6), "scalar part should be one");
        assert!(close(q.q2, 0.0, 1e-6) && close(q.q3, 0.0, 1e-6) && close(q.q4, 0.0, 1e-6));

        let v = q.to_axis_angle();
        assert!(close(v.x, 0.0, 1e-6) && close(v.y, 0.0, 1e-6) && close(v.z, 0.0, 1e-6));
    }
}

/// A rotation the long way round comes back the short way.
///
/// `to_axis_angle` wraps to ±pi, so 350 degrees returns as −10. Wherever the
/// result is an error to be driven to zero — which is every use in the
/// attitude controller — the unwrapped form would command a nearly full turn
/// instead of a small correction.
#[test]
fn a_large_rotation_comes_back_wrapped() {
    let axis = Vector3f::new(0.0, 0.0, 1.0);
    let long_way = 350.0_f32.to_radians();

    let back = Quaternion::from_axis_angle(axis, long_way).to_axis_angle();

    let expected = -10.0_f32.to_radians();
    assert!(
        close(back.z, expected, 1e-4),
        "350 degrees should come back as -10, got {} degrees",
        back.z.to_degrees()
    );
}

/// Rotating a vector agrees with the rotation matrix.
///
/// Two independent formulations of the same rotation. They are algebraically
/// equal and not bit-equal, so this is a correctness check rather than a
/// parity one — but a sign error in the inlined cross products shows up at
/// once.
#[test]
fn rotating_a_vector_agrees_with_the_matrix() {
    let q = Quaternion::from_euler(0.3, -0.5, 1.2);
    let m = q.rotation_matrix();

    for v in [
        Vector3f::new(1.0, 0.0, 0.0),
        Vector3f::new(0.0, 1.0, 0.0),
        Vector3f::new(0.0, 0.0, -1.0),
        Vector3f::new(0.3, -0.7, 0.2),
    ] {
        let by_quat = q.rotate(v);
        let by_matrix = m * v;

        assert!(
            close(by_quat.x, by_matrix.x, 1e-5)
                && close(by_quat.y, by_matrix.y, 1e-5)
                && close(by_quat.z, by_matrix.z, 1e-5),
            "{v:?}: quaternion gave {by_quat:?}, matrix gave {by_matrix:?}"
        );
    }
}

/// Rotating by the identity leaves a vector alone.
#[test]
fn the_identity_rotation_is_a_no_op() {
    let v = Vector3f::new(0.3, -0.7, 0.2);
    let out = Quaternion::identity().rotate(v);

    assert!(close(out.x, v.x, 1e-6) && close(out.y, v.y, 1e-6) && close(out.z, v.z, 1e-6));
}
