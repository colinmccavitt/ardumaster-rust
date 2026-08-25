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

/// The acceleration limit assumed when none is configured, upstream's
/// `radians(1800)`.
///
/// 1800 degrees per second squared — fast enough to be effectively no limit on
/// any real airframe. It exists to keep the shaping from dividing by zero, not
/// to describe a vehicle.
///
/// Written in degrees and converted, not as a radian literal: upstream
/// computes it through `radians()`, and a hand-converted constant is one
/// ulp away from that as often as not.
const DEFAULT_ACCEL_MAX_DEGSS: f32 = 1800.0;

/// How many loop iterations the default input time constant spreads the
/// acceleration over, upstream's `dt * 10.0`.
const DEFAULT_INPUT_TC_CYCLES: f32 = 10.0;

/// What the command model produced.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CommandModel {
    /// The angular velocity to command.
    pub target_ang_vel: f32,
    /// The angular acceleration used to get there.
    pub target_ang_accel: f32,
}

/// Turn an angle error into a rate and acceleration target, upstream
/// `attitude_command_model`.
///
/// The shaping is jerk limited as well as acceleration limited, which is what
/// `input_tc` sets: the jerk limit is `accel_max / input_tc`, so a smaller
/// time constant means the aircraft is allowed to change its acceleration
/// faster and the response feels sharper.
///
/// `state` carries the current rate and acceleration in and the new ones out,
/// because the shaping is a filter over time rather than a function of the
/// error alone.
///
/// Returns the state unchanged for a non-positive `dt`, as upstream does — a
/// paused controller should not integrate.
///
/// # The final `+= accel * dt`
///
/// The shaping produces a velocity for the *start* of the step and an
/// acceleration across it. Upstream advances the velocity by one step of that
/// acceleration before returning, so the caller gets the value for the end of
/// the step. Dropping it leaves the rate target one iteration behind, which at
/// 400 Hz is small and constant — exactly the kind of error that looks like a
/// slightly sluggish airframe rather than like a bug.
pub fn attitude_command_model(
    state: CommandModel,
    error_angle: f32,
    desired_ang_vel: f32,
    max_ang_vel: f32,
    accel_max: f32,
    input_tc: f32,
    dt: f32,
) -> CommandModel {
    use ap_math::scalar::is_positive;

    if !is_positive(dt) {
        return state;
    }

    let accel_max = if is_positive(accel_max) {
        accel_max
    } else {
        ap_math::scalar::radians(DEFAULT_ACCEL_MAX_DEGSS)
    };
    let input_tc = if is_positive(input_tc) {
        input_tc
    } else {
        dt * DEFAULT_INPUT_TC_CYCLES
    };

    let mut target_ang_accel = state.target_ang_accel;
    let mut target_ang_vel = state.target_ang_vel;

    // The shaping reports an error for a degenerate configuration; upstream
    // ignores its return and carries on with whatever it wrote, so this does
    // too rather than inventing a failure path the caller cannot have.
    let _ = ap_math::control::shape_angle_vel_accel(
        error_angle,
        desired_ang_vel,
        0.0,
        0.0,
        target_ang_vel,
        &mut target_ang_accel,
        -max_ang_vel,
        max_ang_vel,
        accel_max,
        accel_max / input_tc,
        dt,
        true,
    );

    target_ang_vel += target_ang_accel * dt;

    CommandModel {
        target_ang_vel,
        target_ang_accel,
    }
}

/// Bounds on the acceleration the stability controller assumes, upstream's
/// `AC_ATTITUDE_ACCEL_*_CONTROLLER_{MIN,MAX}_RADSS`.
///
/// In degrees, converted at use, for the reason given on
/// [`DEFAULT_ACCEL_MAX_DEGSS`].
///
/// Roll and pitch get a much wider range than yaw because they are what a
/// multirotor is good at: thrust vectoring gives large roll and pitch
/// authority, while yaw comes only from rotor drag differences.
const ACCEL_RP_MIN_DEGSS: f32 = 40.0;
const ACCEL_RP_MAX_DEGSS: f32 = 720.0;
const ACCEL_Y_MIN_DEGSS: f32 = 10.0;
const ACCEL_Y_MAX_DEGSS: f32 = 120.0;

/// Build an attitude from a thrust direction and a heading, upstream
/// `attitude_from_thrust_vector`.
///
/// The order matters: the thrust rotation is applied first and the heading
/// second, as `thrust * yaw`. Reversing it would yaw in the earth frame before
/// leaning, which puts the lean on the wrong axis for any non-zero heading.
///
/// A zero-length thrust vector is read as straight up rather than rejected —
/// upstream substitutes the up vector, which keeps a caller that has not yet
/// computed a demand from producing a NaN attitude.
#[must_use]
pub fn attitude_from_thrust_vector(thrust_vector: Vector3f, heading_angle_rad: f32) -> Quaternion {
    let thrust_vector = if is_zero(thrust_vector.length_squared()) {
        THRUST_VECTOR_UP
    } else {
        let length = thrust_vector.length();
        thrust_vector / length
    };

    let mut thrust_vec_cross = THRUST_VECTOR_UP.cross(thrust_vector);
    let thrust_vector_angle = libm::acosf(THRUST_VECTOR_UP.dot(thrust_vector).clamp(-1.0, 1.0));

    let thrust_vector_length = thrust_vec_cross.length();
    if is_zero(thrust_vector_length) || is_zero(thrust_vector_angle) {
        thrust_vec_cross = THRUST_VECTOR_UP;
    } else {
        thrust_vec_cross /= thrust_vector_length;
    }

    let thrust_vec_quat = Quaternion::from_axis_angle(thrust_vec_cross, thrust_vector_angle);
    // Heading is about earth-frame down, which is +Z in NED -- the opposite
    // sign to the thrust axis above.
    let yaw_quat = Quaternion::from_axis_angle(Vector3f::new(0.0, 0.0, 1.0), heading_angle_rad);

    thrust_vec_quat * yaw_quat
}

/// The gains and limits the rate target is computed from.
#[derive(Debug, Clone, Copy)]
pub struct AngleGains {
    /// `ATC_ANG_RLL_P`, already multiplied by its runtime scale.
    pub angle_p_roll: f32,
    /// `ATC_ANG_PIT_P`, scaled.
    pub angle_p_pitch: f32,
    /// `ATC_ANG_YAW_P`, scaled.
    pub angle_p_yaw: f32,
    /// Maximum roll acceleration, rad/s².
    pub accel_roll_max_radss: f32,
    /// Maximum pitch acceleration, rad/s².
    pub accel_pitch_max_radss: f32,
    /// Maximum yaw acceleration, rad/s².
    pub accel_yaw_max_radss: f32,
    /// Whether to shape the response with the square-root controller.
    pub use_sqrt_controller: bool,
}

/// Turn an attitude error into a body-frame rate target, upstream
/// `update_ang_vel_target_from_att_error`.
///
/// Each axis is either a plain proportional gain or a square-root controller,
/// per axis rather than per vehicle: the choice depends on whether that axis
/// has an acceleration limit configured, so a vehicle can legitimately run
/// sqrt on roll and pitch and proportional on yaw.
///
/// The acceleration handed to the square-root controller is *half* the axis
/// maximum, clamped. Halving leaves headroom for the rate controller
/// underneath, which needs authority of its own to track the target this
/// produces — giving the full limit here would let the attitude loop consume
/// all of it and leave the rate loop nothing.
#[must_use]
pub fn update_ang_vel_target_from_att_error(
    attitude_error_rot_vec_rad: Vector3f,
    gains: &AngleGains,
    dt: f32,
) -> Vector3f {
    use ap_math::control::sqrt_controller;
    use ap_math::scalar::radians;

    let axis = |error: f32, angle_p: f32, accel_max: f32, min_deg: f32, max_deg: f32| {
        if gains.use_sqrt_controller && !is_zero(accel_max) {
            sqrt_controller(
                error,
                angle_p,
                (accel_max / 2.0).clamp(radians(min_deg), radians(max_deg)),
                dt,
            )
        } else {
            angle_p * error
        }
    };

    Vector3f::new(
        axis(
            attitude_error_rot_vec_rad.x,
            gains.angle_p_roll,
            gains.accel_roll_max_radss,
            ACCEL_RP_MIN_DEGSS,
            ACCEL_RP_MAX_DEGSS,
        ),
        axis(
            attitude_error_rot_vec_rad.y,
            gains.angle_p_pitch,
            gains.accel_pitch_max_radss,
            ACCEL_RP_MIN_DEGSS,
            ACCEL_RP_MAX_DEGSS,
        ),
        axis(
            attitude_error_rot_vec_rad.z,
            gains.angle_p_yaw,
            gains.accel_yaw_max_radss,
            ACCEL_Y_MIN_DEGSS,
            ACCEL_Y_MAX_DEGSS,
        ),
    )
}

/// The largest heading error the controller will act on, upstream
/// `AC_ATTITUDE_YAW_MAX_ERROR_ANGLE_RAD`.
///
/// 45 degrees. Beyond that the yaw correction is capped, because a large
/// heading error asks for a yaw rate that would consume the authority the
/// aircraft needs to hold its thrust vector — and holding thrust matters more
/// than facing the right way.
const YAW_MAX_ERROR_ANGLE_DEG: f32 = 45.0;

/// What the yaw limiting needs to know.
#[derive(Debug, Clone, Copy)]
pub struct YawLimitGains {
    /// Maximum yaw acceleration, rad/s².
    pub accel_yaw_max_radss: f32,
    /// The yaw *rate* controller's proportional gain, `ATC_RAT_YAW_P`.
    pub rate_yaw_kp: f32,
    /// The yaw *angle* controller's proportional gain, `ATC_ANG_YAW_P`.
    pub angle_yaw_kp: f32,
    /// Lower clamp on the acceleration used for the limit, in degrees.
    pub accel_y_min_degss: f32,
    /// Upper clamp, in degrees.
    pub accel_y_max_degss: f32,
}

impl Default for YawLimitGains {
    fn default() -> Self {
        Self {
            accel_yaw_max_radss: 0.0,
            rate_yaw_kp: 0.0,
            angle_yaw_kp: 0.0,
            accel_y_min_degss: ACCEL_Y_MIN_DEGSS,
            accel_y_max_degss: ACCEL_Y_MAX_DEGSS,
        }
    }
}

/// Decompose the attitude error and cap the heading part, upstream
/// `thrust_heading_rotation_angles`.
///
/// The cap is the whole addition over
/// [`thrust_vector_rotation_angles`]. It is derived rather than fixed: the
/// limit is whatever heading error would just saturate the yaw output with the
/// yaw rate at zero, found by running the rate gain backwards through
/// `inv_sqrt_controller`, and then capped at 45 degrees regardless.
///
/// Deriving it from the gains rather than fixing it means a vehicle with a
/// weak yaw authority gets a tighter cap automatically, which is the point: the
/// limit exists to stop the yaw loop asking for more than the aircraft has.
///
/// When the cap binds, the *target attitude itself* is rebuilt from the capped
/// error, not merely the error clamped. That matters because the target is
/// what the next iteration compares against — clamping only the error would
/// leave the target unreachable and the error re-appearing every iteration.
///
/// Returns the possibly-updated target alongside the error.
pub fn thrust_heading_rotation_angles(
    attitude_target: Quaternion,
    attitude_body: Quaternion,
    gains: &YawLimitGains,
) -> (Quaternion, AttitudeError) {
    use ap_math::control::inv_sqrt_controller;
    use ap_math::scalar::{radians, wrap_pi};

    let mut error = thrust_vector_rotation_angles(attitude_target, attitude_body);
    let mut target = attitude_target;

    // Half the axis maximum, for the same reason as the rate target: leave
    // headroom for the loop underneath.
    let heading_accel_max = (gains.accel_yaw_max_radss / 2.0).clamp(
        radians(gains.accel_y_min_degss),
        radians(gains.accel_y_max_degss),
    );

    if is_zero(gains.rate_yaw_kp) {
        return (target, error);
    }

    let heading_error_max = inv_sqrt_controller(
        1.0 / gains.rate_yaw_kp,
        gains.angle_yaw_kp,
        heading_accel_max,
    )
    .min(radians(YAW_MAX_ERROR_ANGLE_DEG));

    if !is_zero(gains.angle_yaw_kp) && libm::fabsf(error.error_rad.z) > heading_error_max {
        error.error_rad.z = wrap_pi(error.error_rad.z).clamp(-heading_error_max, heading_error_max);

        let heading_correction =
            Quaternion::from_rotation_vector(Vector3f::new(0.0, 0.0, error.error_rad.z));
        target = attitude_body * error.thrust_vector_correction * heading_correction;
    }

    (target, error)
}

/// The thrust error above which yaw corrections start being given up,
/// upstream `AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD`. 30 degrees.
///
/// Twice this and yaw is abandoned entirely. See [`attitude_controller_run`].
pub(crate) const THRUST_ERROR_ANGLE_DEG: f32 = 30.0;

/// Advance the attitude target by one step of the target angular velocity,
/// upstream `update_attitude_target`.
///
/// Normalised afterwards, because composing a small rotation onto a quaternion
/// every iteration at 400 Hz accumulates enough error to matter within
/// seconds.
#[must_use]
pub fn update_attitude_target(
    attitude_target: Quaternion,
    ang_vel_target_rads: Vector3f,
    dt: f32,
) -> Quaternion {
    let update = Quaternion::from_rotation_vector(ang_vel_target_rads * dt);
    let mut target = attitude_target * update;
    target.normalize();
    target
}

/// Everything the controller step needs beyond the attitudes.
#[derive(Debug, Clone, Copy)]
pub struct ControllerInputs {
    /// The target angular velocity, in the *target* frame.
    pub ang_vel_target_rads: Vector3f,
    /// The most recent gyro reading, body frame.
    pub gyro_rads: Vector3f,
    /// Maximum roll rate, degrees per second.
    pub ang_vel_roll_max_degs: f32,
    /// Maximum pitch rate, degrees per second.
    pub ang_vel_pitch_max_degs: f32,
    /// Maximum yaw rate, degrees per second.
    pub ang_vel_yaw_max_degs: f32,
}

/// What one controller step produced.
#[derive(Debug, Clone, Copy)]
pub struct ControllerOutput {
    /// The body-frame angular velocity to hand the rate controller.
    pub ang_vel_body_rads: Vector3f,
    /// How much of the feedforward survived the thrust-error blending, 0 to 1.
    pub feedforward_scalar: f32,
    /// The attitude error, for callers that log or reset on it.
    pub attitude_error: AttitudeError,
    /// The target, possibly rebuilt by the yaw cap.
    pub attitude_target: Quaternion,
    /// The rotation from body to target, kept so an EKF reset can shift the
    /// target and preserve the error the controller was working on.
    ///
    /// Upstream computes this twice under two names — once as
    /// `rotation_target_to_body`, to rotate the feedforward into the body
    /// frame, and again at the end of the function as `_attitude_ang_error`
    /// from the identical expression. They cannot disagree, so this returns
    /// the one value.
    pub attitude_ang_error: Quaternion,
}

/// One step of the attitude controller, upstream
/// `attitude_controller_run_quat`.
///
/// # Giving up heading to keep thrust
///
/// The three-way branch on thrust error is the part that matters. When the
/// thrust vector is badly wrong, yaw is progressively abandoned:
///
/// - Under 30 degrees of thrust error, full feedforward on all three axes.
/// - Between 30 and 60, the roll and pitch feedforward is faded out linearly
///   and the yaw *command itself* is blended toward the measured gyro rate —
///   the controller stops trying to turn and merely stops fighting whatever
///   turn is happening.
/// - Over 60, yaw is replaced by the gyro outright. The aircraft holds
///   whatever heading rate it has and spends everything on thrust.
///
/// The reason is authority. A multirotor yaws by unbalancing rotor drag, which
/// costs thrust margin — exactly what an aircraft with a large thrust error
/// has none of. Fighting for heading there trades the thing that keeps it
/// flying for the thing that decides which way it faces.
pub fn attitude_controller_run(
    attitude_target: Quaternion,
    attitude_body: Quaternion,
    yaw_gains: &YawLimitGains,
    angle_gains: &AngleGains,
    inputs: &ControllerInputs,
    dt: f32,
) -> ControllerOutput {
    use ap_math::scalar::radians;

    let (attitude_target, attitude_error) =
        thrust_heading_rotation_angles(attitude_target, attitude_body, yaw_gains);

    let mut ang_vel_body_rads =
        update_ang_vel_target_from_att_error(attitude_error.error_rad, angle_gains, dt);

    crate::attitude_kinematics::ang_vel_limit(
        &mut ang_vel_body_rads,
        radians(inputs.ang_vel_roll_max_degs),
        radians(inputs.ang_vel_pitch_max_degs),
        radians(inputs.ang_vel_yaw_max_degs),
    );

    // The target rate is expressed in the target frame; the rate controller
    // wants it in the body frame.
    let rotation_target_to_body = attitude_body.inverse() * attitude_target;
    let ang_vel_body_feedforward = rotation_target_to_body.rotate(inputs.ang_vel_target_rads);

    let threshold = radians(THRUST_ERROR_ANGLE_DEG);
    let thrust_error = attitude_error.thrust_error_angle_rad;
    let mut feedforward_scalar = 1.0;

    if thrust_error > threshold * 2.0 {
        // Yaw abandoned: hold whatever rate the aircraft already has.
        ang_vel_body_rads.z = inputs.gyro_rads.z;
    } else if thrust_error > threshold {
        feedforward_scalar = 1.0 - (thrust_error - threshold) / threshold;
        ang_vel_body_rads.x += ang_vel_body_feedforward.x * feedforward_scalar;
        ang_vel_body_rads.y += ang_vel_body_feedforward.y * feedforward_scalar;
        // Note yaw takes the FULL feedforward here and is then blended as a
        // whole toward the gyro -- it is not scaled twice.
        ang_vel_body_rads.z += ang_vel_body_feedforward.z;
        ang_vel_body_rads.z = inputs.gyro_rads.z * (1.0 - feedforward_scalar)
            + ang_vel_body_rads.z * feedforward_scalar;
    } else {
        ang_vel_body_rads += ang_vel_body_feedforward;
    }

    ControllerOutput {
        ang_vel_body_rads,
        feedforward_scalar,
        attitude_error,
        attitude_target,
        attitude_ang_error: rotation_target_to_body,
    }
}
