//! The attitude error decomposition, upstream
//! `AC_AttitudeControl::thrust_vector_rotation_angles`. COP-007.
//!
//! A multirotor's attitude error is not one rotation but two, taken in order,
//! and the split is the whole idea. The first rotation points the thrust
//! vector where it should go; the second turns the aircraft about that thrust
//! vector to the commanded heading.
//!
//! # Why split it at all
//!
//! Because the two are not equally urgent. Getting the thrust vector wrong
//! means the aircraft accelerates the wrong way — it is a position error in
//! the making. Getting the heading wrong means it is pointing the wrong way
//! while going exactly where it should. So the controller runs them on
//! different gains and different limits, and can sacrifice heading to keep
//! thrust when it runs out of authority. A single combined error would make
//! that impossible to express: there would be nothing to give up.
//!
//! That is also why the yaw error comes out of a *second* quaternion rather
//! than from the Euler decomposition of the first. After the thrust correction
//! has been applied, whatever rotation remains is a pure rotation about the
//! body's own thrust axis — heading, by construction and not by approximation.

use ap_math::quaternion::Quaternion;
use ap_math::scalar::is_zero;
use ap_math::vector3::Vector3f;

/// The thrust direction in any body-fixed frame.
///
/// Down is positive in NED, so up — the way a multirotor's thrust points — is
/// negative Z.
const THRUST_VECTOR_UP: Vector3f = Vector3f {
    x: 0.0,
    y: 0.0,
    z: -1.0,
};

/// What the decomposition produces.
#[derive(Debug, Clone, Copy)]
pub struct AttitudeError {
    /// Roll, pitch and yaw error in radians. The first two come from the
    /// thrust correction, the third from the heading correction that follows
    /// it.
    pub error_rad: Vector3f,
    /// How far the *current* thrust vector is from vertical.
    ///
    /// Not an error: it is the lean angle, reported for callers that limit
    /// against it. It says nothing about where the aircraft is being asked to
    /// point.
    pub thrust_angle_rad: f32,
    /// The angle between the current and target thrust vectors — this one is
    /// the error.
    pub thrust_error_angle_rad: f32,
    /// The first of the two rotations, in the body frame. The caller needs it
    /// to rebuild a limited target.
    pub thrust_vector_correction: Quaternion,
}

/// Split the rotation from body to target into a thrust correction and a
/// heading correction, upstream `thrust_vector_rotation_angles`.
///
/// Both quaternions are passive rotations from their frame to NED.
pub fn thrust_vector_rotation_angles(
    attitude_target: Quaternion,
    attitude_body: Quaternion,
) -> AttitudeError {
    // Where each frame's thrust points, seen from the inertial frame.
    let att_target_thrust_vec = attitude_target.rotate(THRUST_VECTOR_UP);
    let att_body_thrust_vec = attitude_body.rotate(THRUST_VECTOR_UP);

    // The lean angle: how far the current thrust is from straight up.
    let thrust_angle_rad = libm::acosf(THRUST_VECTOR_UP.dot(att_body_thrust_vec).clamp(-1.0, 1.0));

    // The cross product gives the axis to rotate about; the dot product gives
    // how far.
    let mut thrust_vec_cross = att_body_thrust_vec.cross(att_target_thrust_vec);
    let thrust_error_angle_rad = libm::acosf(
        att_body_thrust_vec
            .dot(att_target_thrust_vec)
            .clamp(-1.0, 1.0),
    );

    // Degenerate when the two thrust vectors are parallel or antiparallel: the
    // cross product has no direction to offer. Upstream substitutes the thrust
    // axis itself, which makes the correction a rotation about the axis it
    // cannot move — a no-op for the parallel case, and an arbitrary but
    // harmless choice for the antiparallel one, which a flying aircraft does
    // not reach.
    let thrust_vector_length = thrust_vec_cross.length();
    if is_zero(thrust_vector_length) || is_zero(thrust_error_angle_rad) {
        thrust_vec_cross = THRUST_VECTOR_UP;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    // The axis was computed in the inertial frame, but the correction is
    // defined relative to the body. Rotate it back before building the
    // quaternion — skipping this gives a correction about the wrong axis
    // whenever the aircraft is not level, which is exactly when it matters.
    thrust_vec_cross = attitude_body.inverse().rotate(thrust_vec_cross);
    let thrust_vector_correction =
        Quaternion::from_axis_angle(thrust_vec_cross, thrust_error_angle_rad);

    let rotation_rad = thrust_vector_correction.to_axis_angle();
    let mut error_rad = Vector3f::new(rotation_rad.x, rotation_rad.y, 0.0);

    // Whatever rotation is left after the thrust correction is pure heading.
    let heading_vec_correction_quat =
        thrust_vector_correction.inverse() * attitude_body.inverse() * attitude_target;
    let rotation_rad = heading_vec_correction_quat.to_axis_angle();
    error_rad.z = rotation_rad.z;

    AttitudeError {
        error_rad,
        thrust_angle_rad,
        thrust_error_angle_rad,
        thrust_vector_correction,
    }
}
