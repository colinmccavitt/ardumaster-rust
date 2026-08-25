//! Euler and body-frame kinematics for the attitude controller, upstream
//! `AC_AttitudeControl`'s `euler_derivative_to_body`, `body_to_euler_derivative`,
//! `ang_vel_limit` and `body_to_euler_limit`. COP-007.
//!
//! A multirotor's controller thinks in two frames at once. Pilot input and
//! attitude limits are natural in Euler angles — "don't lean more than 45
//! degrees" — while the rate loop and the gyros work in the body frame. These
//! are the conversions between them, for the 321 (yaw-pitch-roll) sequence.
//!
//! # They are not rotations
//!
//! Converting an angular *rate* between the two is not the same as rotating a
//! vector. Euler rates are measured about three different axes — yaw about the
//! earth vertical, pitch about an intermediate axis, roll about the body axis
//! — which is why the transform below is not orthonormal and why its inverse
//! has `tan` and `sec` terms rather than a transpose.
//!
//! That asymmetry is the reason for the gimbal-lock failure: at 90 degrees of
//! pitch the yaw and roll axes coincide, the Euler description stops being
//! invertible, and [`body_to_euler_derivative`] refuses rather than dividing
//! by zero.

use ap_math::quaternion::Quaternion;
use ap_math::scalar::is_zero;
use ap_math::vector3::Vector3f;

/// Convert an Euler-frame derivative to the body frame, upstream
/// `euler_derivative_to_body`.
///
/// Works for any order of derivative — rate, acceleration — because the
/// relationship depends only on the current attitude, not on what is being
/// differentiated.
///
/// Always succeeds. This direction has no singularity: it is the inverse that
/// cannot be taken at gimbal lock, not this one.
pub fn euler_derivative_to_body(att: Quaternion, euler: Vector3f) -> Vector3f {
    let theta = att.get_euler_pitch();
    let phi = att.get_euler_roll();

    let (sin_theta, cos_theta) = (libm::sinf(theta), libm::cosf(theta));
    let (sin_phi, cos_phi) = (libm::sinf(phi), libm::cosf(phi));

    Vector3f::new(
        euler.x - sin_theta * euler.z,
        cos_phi * euler.y + sin_phi * cos_theta * euler.z,
        -sin_phi * euler.y + cos_theta * cos_phi * euler.z,
    )
}

/// Convert a body-frame derivative to the Euler frame, upstream
/// `body_to_euler_derivative`.
///
/// `None` when the vehicle is pitched 90 degrees up or down, where the Euler
/// description is not invertible. Upstream returns false and leaves the output
/// untouched; returning `None` says the same thing without the caller having
/// to remember that the out-parameter is now stale.
pub fn body_to_euler_derivative(att: Quaternion, body: Vector3f) -> Option<Vector3f> {
    let theta = att.get_euler_pitch();
    let phi = att.get_euler_roll();

    let (sin_theta, cos_theta) = (libm::sinf(theta), libm::cosf(theta));
    let (sin_phi, cos_phi) = (libm::sinf(phi), libm::cosf(phi));

    if is_zero(cos_theta) {
        return None;
    }

    // Written as `sin/cos` rather than `tan`, because that is how upstream
    // writes it and the two are not bit-identical.
    Some(Vector3f::new(
        body.x
            + sin_phi * (sin_theta / cos_theta) * body.y
            + cos_phi * (sin_theta / cos_theta) * body.z,
        cos_phi * body.y - sin_phi * body.z,
        (sin_phi / cos_theta) * body.y + (cos_phi / cos_theta) * body.z,
    ))
}

/// The smallest magnitude the sine and cosine terms are allowed to take when
/// converting a body limit to an Euler one, upstream's `constrain_float(...,
/// 0.1f, 1.0f)`.
///
/// The conversion divides by them, so near zero the Euler limit would run away
/// to something meaninglessly large. Clamping at a tenth caps the inflation at
/// ten times rather than letting an attitude near a singularity produce an
/// effectively unlimited rate.
const LIMIT_TRIG_MIN: f32 = 0.1;

/// Limit an Euler-frame angular velocity, upstream `ang_vel_limit`.
///
/// Roll and pitch are limited *together*, as an ellipse rather than a box: the
/// pair is scaled back along its own direction when it falls outside. A
/// per-axis clamp would let a diagonal command through at up to root-two of
/// the intended magnitude, which is a real difference in how hard the aircraft
/// can be commanded to move.
///
/// The elliptical path only applies when both limits are non-zero. A zero
/// limit means "unlimited on this axis" here, not "hold at zero", so the
/// combined form has nothing to normalise against and each axis falls back to
/// an independent clamp.
pub fn ang_vel_limit(
    euler: &mut Vector3f,
    roll_max_rads: f32,
    pitch_max_rads: f32,
    yaw_max_rads: f32,
) {
    if is_zero(roll_max_rads) || is_zero(pitch_max_rads) {
        if !is_zero(roll_max_rads) {
            euler.x = euler.x.clamp(-roll_max_rads, roll_max_rads);
        }
        if !is_zero(pitch_max_rads) {
            euler.y = euler.y.clamp(-pitch_max_rads, pitch_max_rads);
        }
    } else {
        let normalised =
            ap_math::vector2::Vector2::new(euler.x / roll_max_rads, euler.y / pitch_max_rads);
        let length = normalised.length();
        if length > 1.0 {
            euler.x = normalised.x * roll_max_rads / length;
            euler.y = normalised.y * pitch_max_rads / length;
        }
    }

    if !is_zero(yaw_max_rads) {
        euler.z = euler.z.clamp(-yaw_max_rads, yaw_max_rads);
    }
}

/// Convert body-frame rate or acceleration limits to Euler-frame ones,
/// upstream `body_to_euler_limit`.
///
/// Each Euler axis is limited by whichever body axis binds first, which is why
/// the pitch and yaw results are minimums over several terms: a single Euler
/// rotation can demand motion about more than one body axis at once, and the
/// tightest of those is the real constraint.
///
/// Returns the input unchanged if any component is not positive. Upstream
/// treats a non-positive limit as "no limit configured" and declines to
/// transform it rather than producing a negative or infinite Euler limit from
/// it.
#[must_use]
pub fn body_to_euler_limit(att: Quaternion, body_limit: Vector3f) -> Vector3f {
    if !ap_math::scalar::is_positive(body_limit.x)
        || !ap_math::scalar::is_positive(body_limit.y)
        || !ap_math::scalar::is_positive(body_limit.z)
    {
        return body_limit;
    }

    let phi = att.get_euler_roll();
    let theta = att.get_euler_pitch();

    // Magnitudes, then clamped. Taking the absolute value first is what makes
    // the clamp symmetric -- the limit is a magnitude and does not care which
    // way the aircraft is leaning.
    let sin_phi = libm::fabsf(libm::sinf(phi)).clamp(LIMIT_TRIG_MIN, 1.0);
    let cos_phi = libm::fabsf(libm::cosf(phi)).clamp(LIMIT_TRIG_MIN, 1.0);
    let sin_theta = libm::fabsf(libm::sinf(theta)).clamp(LIMIT_TRIG_MIN, 1.0);
    let cos_theta = libm::fabsf(libm::cosf(theta)).clamp(LIMIT_TRIG_MIN, 1.0);

    Vector3f::new(
        body_limit.x,
        (body_limit.y / cos_phi).min(body_limit.z / sin_phi),
        (body_limit.x / sin_theta)
            .min(body_limit.y / (sin_phi * cos_theta))
            .min(body_limit.z / (cos_phi * cos_theta)),
    )
}
