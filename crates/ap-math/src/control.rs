//! Kinematic shaping for position controllers, upstream `AP_Math/control.cpp`.
//! COP-001.
//!
//! A position controller that simply drove the error to zero would command a
//! step in acceleration, which a multirotor cannot deliver and a payload would
//! not appreciate. These functions shape a demand so that position, velocity,
//! acceleration and *jerk* all stay inside limits, which is what makes
//! automatic flight feel deliberate rather than abrupt.
//!
//! Copter's position controller is the main user, which is why this is tracked
//! under the multirotor effort — but the code lives in `AP_Math` and is
//! vehicle-agnostic, so the fixed-wing port gets it too.
//!
//! # The square-root controller
//!
//! [`sqrt_controller`] is the piece everything else is built on. Close to the
//! setpoint it is a plain proportional controller. Far away it follows
//! `sqrt(2·a·Δx)` — the speed from which a constant deceleration `a` would
//! stop exactly at the target. So the vehicle approaches at the fastest speed
//! it could still stop from, and the two regimes are joined so the response is
//! continuous.
//!
//! # This slice is the scalar half
//!
//! The `_xy` vector forms, `limit_accel_xy`, `limit_accel_corner_xy` and
//! `kinematic_limit` are not here. They are the same ideas applied to a
//! direction rather than a sign, and they are their own slice.

use crate::scalar::{
    constrain_value, degrees, is_negative, is_positive, is_zero, radians, safe_sqrt, sq, wrap_pi,
};
use crate::vector2::{Vector2, Vector2f};
use crate::vector3::Vector3f;

/// Position type, upstream `postype_t`.
///
/// `double` where `HAL_WITH_POSTYPE_DOUBLE` is set, which SITL and every
/// board with the room for it does. Positions are held in centimetres from
/// an origin that may be kilometres away, and single precision runs out of
/// resolution before the range does.
pub type Postype = f64;

/// Why a shaping call did nothing.
///
/// Upstream raises `INTERNAL_ERROR(invalid_arg_or_result)` and returns,
/// leaving the output untouched. The error is recorded and flight continues,
/// so a caller passing nonsense gets silence — which is why this is a value
/// here rather than a log line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShapeError {
    /// `jerk_max` was not positive. Nothing can be shaped without a jerk
    /// limit, since it is the only thing bounding the rate of change.
    JerkNotPositive,
    /// `accel_min` was not negative or `accel_max` not positive. The two are
    /// signed bounds on the same quantity and the sign convention is not
    /// optional.
    AccelLimitsMalformed,
    /// `vel_min` was positive or `vel_max` negative, same convention.
    VelLimitsMalformed,
}

/// Move a velocity forward by one step of acceleration, upstream
/// `update_vel_accel`.
///
/// `limit` names a direction in which acceleration is constrained; `vel_error`
/// gives the sign of the velocity error. When both point the same way as the
/// step, the step is refused — unless the velocity is currently opposing the
/// limit, in which case it is clipped so it cannot cross zero.
///
/// The point is that a controller which has hit a limit should not keep
/// pushing into it, but should still be allowed to unwind.
pub fn update_vel_accel(vel: &mut f32, accel: f32, dt: f32, limit: f32, vel_error: f32) {
    // The three products below are sign tests, and multiplying by `limit` or
    // dividing by it cannot be told apart -- mutation testing keeps offering
    // the swap, so the reasoning is recorded here rather than rediscovered.
    //
    // For any non-zero `limit` the two agree in sign, which is all
    // `is_positive` reads. At `limit == 0` they genuinely differ, zero against
    // an infinity, but never observably: whichever product is left unmutated
    // still evaluates `is_positive(0.0)`, which is false, and the `&&`
    // short-circuits before the mutated half can matter. The third product
    // sits inside that guard and so is only reached when `limit` is non-zero.
    let mut delta_vel = accel * dt;
    if is_positive(delta_vel * limit) && is_positive(vel_error * limit) {
        if is_negative(*vel * limit) {
            delta_vel = constrain_value(delta_vel, -vel.abs(), vel.abs());
        } else {
            delta_vel = 0.0;
        }
    }
    *vel += delta_vel;
}

/// Move a position and velocity forward by one step, upstream
/// `update_pos_vel_accel`.
///
/// The position step is skipped entirely when it would worsen the position
/// error in the limited direction; the velocity then updates under the same
/// rule as [`update_vel_accel`].
pub fn update_pos_vel_accel(
    pos: &mut Postype,
    vel: &mut f32,
    accel: f32,
    dt: f32,
    limit: f32,
    pos_error: f32,
    vel_error: f32,
) {
    let mut delta_pos = *vel * dt + accel * 0.5 * sq(dt);
    if is_positive(delta_pos * limit) && is_positive(pos_error * limit) {
        delta_pos = 0.0;
    }
    *pos += Postype::from(delta_pos);

    update_vel_accel(vel, accel, dt, limit, vel_error);
}

/// Move an acceleration toward a target without exceeding a jerk limit,
/// upstream `shape_accel`.
///
/// # Errors
///
/// [`ShapeError::JerkNotPositive`] if `jerk_max` is not positive. Upstream
/// raises an internal error and returns, leaving `accel` untouched; so does
/// this, and says so.
pub fn shape_accel(
    accel_desired: f32,
    accel: &mut f32,
    jerk_max: f32,
    dt: f32,
) -> Result<(), ShapeError> {
    if !is_positive(jerk_max) {
        return Err(ShapeError::JerkNotPositive);
    }
    if is_positive(dt) {
        let accel_delta = constrain_value(accel_desired - *accel, -jerk_max * dt, jerk_max * dt);
        *accel += accel_delta;
    }
    Ok(())
}

/// Shape an acceleration to track a velocity demand, upstream
/// `shape_vel_accel`.
///
/// The correction gain is derived from the limits rather than tuned:
/// `jerk_max / accel_max` is the reciprocal of the time the acceleration takes
/// to reach its limit, which is the fastest the velocity loop can usefully
/// respond. The direction matters — closing a positive position error means a
/// *negative* velocity error while slowing down — so the gain is picked from
/// the sign of the error.
///
/// # Errors
///
/// [`ShapeError::AccelLimitsMalformed`] or [`ShapeError::JerkNotPositive`].
#[allow(clippy::too_many_arguments, reason = "upstream's signature")]
pub fn shape_vel_accel(
    vel_desired: f32,
    accel_desired: f32,
    vel: f32,
    accel: &mut f32,
    accel_min: f32,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
    limit_total_accel: bool,
) -> Result<(), ShapeError> {
    if !is_negative(accel_min) || !is_positive(accel_max) {
        return Err(ShapeError::AccelLimitsMalformed);
    }
    if !is_positive(jerk_max) {
        return Err(ShapeError::JerkNotPositive);
    }

    let vel_error = vel_desired - vel;

    let kpa = if is_positive(vel_error) {
        jerk_max / accel_max
    } else {
        jerk_max / -accel_min
    };

    let mut accel_target = sqrt_controller(vel_error, kpa, jerk_max, dt);
    accel_target = constrain_value(accel_target, accel_min, accel_max);
    accel_target += accel_desired;

    if limit_total_accel {
        accel_target = constrain_value(accel_target, accel_min, accel_max);
    }

    shape_accel(accel_target, accel, jerk_max, dt)
}

/// Shape an acceleration to track a position demand, upstream
/// `shape_pos_vel_accel`.
///
/// Works in the *correction frame*: the feedforward velocity is subtracted out
/// first, so the square-root controller sees only the part of the motion it is
/// responsible for. That is what lets a moving target be tracked without the
/// controller fighting its own feedforward.
///
/// # Errors
///
/// [`ShapeError::VelLimitsMalformed`], [`ShapeError::AccelLimitsMalformed`] or
/// [`ShapeError::JerkNotPositive`].
#[allow(clippy::too_many_arguments, reason = "upstream's signature")]
pub fn shape_pos_vel_accel(
    pos_desired: Postype,
    vel_desired: f32,
    accel_desired: f32,
    pos: Postype,
    vel: f32,
    accel: &mut f32,
    vel_min: f32,
    vel_max: f32,
    accel_min: f32,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
    limit_total: bool,
) -> Result<(), ShapeError> {
    if is_positive(vel_min) || is_negative(vel_max) {
        return Err(ShapeError::VelLimitsMalformed);
    }
    if !is_negative(accel_min) || !is_positive(accel_max) {
        return Err(ShapeError::AccelLimitsMalformed);
    }
    if !is_positive(jerk_max) {
        return Err(ShapeError::JerkNotPositive);
    }

    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream assigns the postype_t difference to a float; positions \
are far apart in absolute terms and close in relative ones, which is the whole \
reason the subtraction happens at the wider width first"
    )]
    let pos_error = (pos_desired - pos) as f32;

    // The acceleration allowance is taken from the direction of travel: a
    // positive position error is closed by decelerating, which uses accel_min.
    let (accel_lim, k_v) = if is_positive(pos_error) {
        let lim = -accel_min;
        (lim, jerk_max / lim)
    } else {
        let lim = accel_max;
        (lim, jerk_max / lim)
    };

    // Correction frame: remove the feedforward velocity.
    let vel_corr = vel - vel_desired;

    let mut vel_corr_cmd = sqrt_controller(pos_error, k_v, accel_lim, dt);
    let accel_corr_cmd = sqrt_controller_accel(pos_error, vel_corr_cmd, vel_corr, k_v, accel_lim);
    vel_corr_cmd += accel_corr_cmd / k_v;

    if is_negative(vel_min) {
        vel_corr_cmd = vel_corr_cmd.max(vel_min);
    }
    if is_positive(vel_max) {
        vel_corr_cmd = vel_corr_cmd.min(vel_max);
    }

    let mut vel_target = vel_desired + vel_corr_cmd;

    if limit_total {
        if is_negative(vel_min) {
            vel_target = vel_target.max(vel_min);
        }
        if is_positive(vel_max) {
            vel_target = vel_target.min(vel_max);
        }
    }

    let mut accel_target = (vel_target - vel) * k_v;
    accel_target = constrain_value(accel_target, accel_min, accel_max);
    accel_target += accel_desired;

    if limit_total {
        accel_target = constrain_value(accel_target, accel_min, accel_max);
    }

    shape_accel(accel_target, accel, jerk_max, dt)
}

/// The angular form of [`shape_pos_vel_accel`], upstream
/// `shape_angle_vel_accel`.
///
/// The demand is wrapped to the nearest equivalent angle first, so a vehicle
/// at 179 degrees asked for -179 turns two degrees rather than 358.
///
/// # Errors
///
/// As [`shape_pos_vel_accel`].
#[allow(clippy::too_many_arguments, reason = "upstream's signature")]
pub fn shape_angle_vel_accel(
    angle_desired: f32,
    angle_vel_desired: f32,
    angle_accel_desired: f32,
    angle: f32,
    angle_vel: f32,
    angle_accel: &mut f32,
    angle_vel_min: f32,
    angle_vel_max: f32,
    angle_accel_max: f32,
    angle_jerk_max: f32,
    dt: f32,
    limit_total: bool,
) -> Result<(), ShapeError> {
    let angle_desired_wrapped = angle + wrap_pi(angle_desired - angle);
    shape_pos_vel_accel(
        Postype::from(angle_desired_wrapped),
        angle_vel_desired,
        angle_accel_desired,
        Postype::from(angle),
        angle_vel,
        angle_accel,
        angle_vel_min,
        angle_vel_max,
        -angle_accel_max,
        angle_accel_max,
        angle_jerk_max,
        dt,
        limit_total,
    )
}

/// Proportional near the target, square-root far from it. Upstream
/// `sqrt_controller`.
///
/// The square-root branch returns the speed from which `second_ord_lim` of
/// deceleration would stop exactly at the target. The two branches meet at
/// `linear_dist = second_ord_lim / p²`, and the `linear_dist / 2` term inside
/// the square root is what makes the join continuous rather than merely close.
///
/// With a positive `dt` the result is clamped so the correction cannot
/// overshoot the whole error in one step.
#[must_use]
pub fn sqrt_controller(error: f32, p: f32, second_ord_lim: f32, dt: f32) -> f32 {
    let correction_rate = if is_negative(second_ord_lim) || is_zero(second_ord_lim) {
        // No acceleration limit: purely proportional.
        error * p
    } else if is_zero(p) {
        // No proportional gain: purely square-root.
        if is_positive(error) {
            safe_sqrt(2.0 * second_ord_lim * error)
        } else if is_negative(error) {
            -safe_sqrt(2.0 * second_ord_lim * -error)
        } else {
            0.0
        }
    } else {
        let linear_dist = second_ord_lim / sq(p);
        if error > linear_dist {
            safe_sqrt(2.0 * second_ord_lim * (error - linear_dist / 2.0))
        } else if error < -linear_dist {
            -safe_sqrt(2.0 * second_ord_lim * (-error - linear_dist / 2.0))
        } else {
            error * p
        }
    };

    if is_positive(dt) {
        constrain_value(correction_rate, -error.abs() / dt, error.abs() / dt)
    } else {
        correction_rate
    }
}

/// The error that would produce a given output, upstream
/// `inv_sqrt_controller`.
///
/// Used to work out how far ahead of a stop the braking must begin — see
/// [`stopping_distance`], which is this function under another name.
#[must_use]
pub fn inv_sqrt_controller(output: f32, p: f32, d_max: f32) -> f32 {
    if is_positive(d_max) && is_zero(p) {
        return (output * output) / (2.0 * d_max);
    }
    if (is_negative(d_max) || is_zero(d_max)) && !is_zero(p) {
        return output / p;
    }
    if (is_negative(d_max) || is_zero(d_max)) && is_zero(p) {
        return 0.0;
    }

    let linear_velocity = d_max / p;
    if output.abs() < linear_velocity {
        return output / p;
    }

    let linear_dist = d_max / sq(p);
    let stopping_dist = (linear_dist * 0.5) + sq(output) / (2.0 * d_max);
    if is_positive(output) {
        stopping_dist
    } else {
        -stopping_dist
    }
}

/// The rate of change [`sqrt_controller`] implies, upstream
/// `sqrt_controller_accel`.
///
/// Differentiating the controller's own response with respect to time, via the
/// chain rule and the actual closing rate. In the linear region the slope is
/// `p`; in the square-root region it is `second_ord_lim / |rate_cmd|`.
///
/// Returns zero when the vehicle is moving away from the target — there is no
/// braking profile to differentiate.
#[must_use]
pub fn sqrt_controller_accel(
    error: f32,
    rate_cmd: f32,
    rate_state: f32,
    p: f32,
    second_ord_lim: f32,
) -> f32 {
    if !is_positive(rate_cmd * rate_state) {
        return 0.0;
    }
    if !is_positive(second_ord_lim) {
        return -p * rate_state;
    }
    if !is_positive(p) {
        if is_zero(rate_cmd) {
            return 0.0;
        }
        return -(second_ord_lim / rate_cmd.abs()) * rate_state;
    }

    let linear_dist = second_ord_lim / sq(p);
    if error.abs() <= linear_dist {
        return -p * rate_state;
    }
    if is_zero(rate_cmd) {
        return 0.0;
    }
    -(second_ord_lim / rate_cmd.abs()) * rate_state
}

/// How far it takes to stop from a velocity, upstream `stopping_distance`.
///
/// Literally [`inv_sqrt_controller`]: the distance at which the controller
/// would command exactly this speed is the distance it needs to shed it.
#[must_use]
pub fn stopping_distance(velocity: f32, p: f32, accel_max: f32) -> f32 {
    inv_sqrt_controller(velocity, p, accel_max)
}

/// The fastest travel possible in a direction, given separate horizontal and
/// vertical limits. Upstream `kinematic_limit`.
///
/// A multirotor can usually move faster sideways than it can climb, and faster
/// down than up. So the speed available along a diagonal is not either limit
/// but whichever binds first — and which one that is depends on the slope of
/// the direction.
///
/// Returns zero for a direction of no length, or if any limit is zero: there
/// is no meaningful answer, and upstream returns zero rather than dividing.
#[must_use]
pub fn kinematic_limit_xyz(
    segment_length_xy: f32,
    segment_length_z: f32,
    max_xy: f32,
    max_z_neg: f32,
    max_z_pos: f32,
) -> f32 {
    if is_zero(max_xy) || is_zero(max_z_pos) || is_zero(max_z_neg) {
        return 0.0;
    }

    let max_xy = max_xy.abs();
    let max_z_pos = max_z_pos.abs();
    let max_z_neg = max_z_neg.abs();

    let length = safe_sqrt(sq(segment_length_xy) + sq(segment_length_z));
    if !is_positive(length) {
        return 0.0;
    }
    let segment_length_xy = segment_length_xy / length;
    let segment_length_z = segment_length_z / length;

    if is_zero(segment_length_xy) {
        // Straight up or straight down: the vertical limit is the answer, and
        // which one depends on the direction.
        return if is_positive(segment_length_z) {
            max_z_pos
        } else {
            max_z_neg
        };
    }
    if is_zero(segment_length_z) {
        return max_xy;
    }

    // Which limit binds is decided by comparing the direction's slope against
    // the slope the two limits themselves describe.
    let slope = segment_length_z / segment_length_xy;
    if is_positive(slope) {
        if slope.abs() < max_z_pos / max_xy {
            return max_xy / segment_length_xy;
        }
        return (max_z_pos / segment_length_z).abs();
    }
    if slope.abs() < max_z_neg / max_xy {
        return max_xy / segment_length_xy;
    }
    (max_z_neg / segment_length_z).abs()
}

/// [`kinematic_limit_xyz`] for a direction vector, upstream's `Vector3f`
/// overload of `kinematic_limit`.
#[must_use]
pub fn kinematic_limit(direction: Vector3f, max_xy: f32, max_z_neg: f32, max_z_pos: f32) -> f32 {
    if is_zero(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z) {
        return 0.0;
    }
    let segment_length_xy = safe_sqrt(sq(direction.x) + sq(direction.y));
    kinematic_limit_xyz(segment_length_xy, direction.z, max_xy, max_z_neg, max_z_pos)
}

/// Gravity, upstream `GRAVITY_MSS`.
pub const GRAVITY_MSS: f32 = 9.80665;

/// Shape a normalised stick input, upstream `input_expo`.
///
/// Softens the response near centre and steepens it toward the stops, so a
/// pilot gets fine authority around neutral without losing the extremes.
///
/// The `expo < 0.95` guard is not arbitrary caution. As `expo` approaches one
/// the denominator `1 - expo·|input|` goes to zero at full deflection, so the
/// curve becomes infinitely steep at the stops. Above the threshold the
/// input is returned untouched rather than being run through an expression
/// about to divide by nothing.
///
/// Note the guard is on `expo` alone, so a *negative* expo is allowed and
/// inverts the shaping — coarse near centre, fine at the stops. Unusual, but
/// well defined, and the denominator only grows.
#[must_use]
pub fn input_expo(input: f32, expo: f32) -> f32 {
    let input = input.clamp(-1.0, 1.0);
    if expo < 0.95 {
        return (1.0 - expo) * input / (1.0 - expo * input.abs());
    }
    input
}

/// Horizontal acceleration from a lean angle, upstream
/// `angle_rad_to_accel_mss`.
///
/// `g·tan θ`, which is exact rather than a small-angle approximation: the
/// thrust vector's horizontal component divided by its vertical one is
/// precisely the tangent, and holding altitude fixes the vertical at `g`.
///
/// It diverges at ninety degrees, correctly — an aircraft on its side
/// produces no vertical thrust and cannot hold altitude at any horizontal
/// acceleration. Callers bound the angle before converting rather than
/// bounding the result after.
#[must_use]
pub fn angle_rad_to_accel_mss(angle_rad: f32) -> f32 {
    GRAVITY_MSS * libm::tanf(angle_rad)
}

/// Horizontal acceleration from a lean angle in degrees, upstream
/// `angle_deg_to_accel_mss`.
#[must_use]
pub fn angle_deg_to_accel_mss(angle_deg: f32) -> f32 {
    angle_rad_to_accel_mss(radians(angle_deg))
}

/// Lean angle from a horizontal acceleration, upstream
/// `accel_mss_to_angle_rad`.
///
/// `atan(a/g)`, the exact inverse of [`angle_rad_to_accel_mss`], and total:
/// every finite acceleration maps to an angle below ninety degrees, so unlike
/// the forward map this one needs no bounding.
#[must_use]
pub fn accel_mss_to_angle_rad(accel_mss: f32) -> f32 {
    libm::atanf(accel_mss / GRAVITY_MSS)
}

/// Lean angle in degrees from a horizontal acceleration, upstream
/// `accel_mss_to_angle_deg`.
#[must_use]
pub fn accel_mss_to_angle_deg(accel_mss: f32) -> f32 {
    degrees(accel_mss_to_angle_rad(accel_mss))
}

/// Turn pilot stick input into a lean attitude, upstream
/// `rc_input_to_roll_pitch_rad`.
///
/// Every manual multirotor mode goes through here, and it does considerably
/// more than scale two numbers.
///
/// # It works in thrust, not in angle
///
/// The sticks map to `tan(angle_max · stick)` — a horizontal *thrust*
/// component — and only come back to angles at the end. Scaling the angles
/// directly would mean full diagonal stick asked for `angle_max` on each axis
/// and therefore more than `angle_max` of total lean. In thrust space the
/// limit is a single vector length, so the diagonal is bounded like every
/// other direction.
///
/// # Two limits that do different jobs
///
/// `angle_max` sets the scale — how much lean full stick asks for. It is
/// capped at 85 degrees because `tan` runs away toward 90 and the thrust
/// components would stop being meaningful.
///
/// `angle_limit` bounds the *result* without changing that scale, and is
/// clamped to at least 10 degrees so a caller cannot pin the aircraft level.
/// Because the limit is applied to the thrust vector's length, the stick's
/// *direction* survives it: a pilot at a limit still steers, they just cannot
/// lean further. Clamping the two angles separately would swing the commanded
/// direction toward the diagonal as one axis saturated first.
///
/// # The roll term carries a `cos(pitch)`
///
/// Euler angles apply in sequence, so by the time roll acts the aircraft is
/// already pitched and its roll axis is no longer horizontal. The same
/// correction appears in the position controller's lean-angle conversion, for
/// the same reason.
pub fn rc_input_to_roll_pitch_rad(
    roll_in_norm: f32,
    pitch_in_norm: f32,
    angle_max_rad: f32,
    angle_limit_rad: f32,
    roll_out_rad: &mut f32,
    pitch_out_rad: &mut f32,
) {
    let angle_max_rad = angle_max_rad.min(radians(85.0));

    let mut thrust = Vector2f::new(
        -libm::tanf(angle_max_rad * pitch_in_norm),
        libm::tanf(angle_max_rad * roll_in_norm),
    );

    let angle_limit_rad = constrain_value(angle_limit_rad, radians(10.0), angle_max_rad);
    let thrust_limit = libm::tanf(angle_limit_rad);

    thrust.limit_length(thrust_limit);

    *pitch_out_rad = -libm::atanf(thrust.x);
    *roll_out_rad = libm::atanf(libm::cosf(*pitch_out_rad) * thrust.y);
}

/// Project velocity forward, suppressing motion that would worsen a limited
/// error — upstream `update_vel_accel_xy`.
///
/// `limit` is a direction, not a magnitude: a non-zero vector says "the
/// vehicle cannot go further this way". The step is dropped only when all
/// three of the following point the same way as the limit — the step itself,
/// the existing error, and the current velocity. Any one of them pointing
/// back means the step is helping, and it is kept.
///
/// The third test is the subtle one and it is `!is_negative`, not
/// `is_positive`: a velocity of exactly zero still counts as "not moving
/// away", so a stationary vehicle at a limit stays suppressed rather than
/// being allowed one free step.
pub fn update_vel_accel_xy(
    vel: &mut Vector2f,
    accel: Vector2f,
    dt: f32,
    limit: Vector2f,
    vel_error: Vector2f,
) {
    let mut delta_vel = accel * dt;
    if !limit.is_zero()
        && !delta_vel.is_zero()
        && is_positive(delta_vel.dot(limit))
        && is_positive(vel_error.dot(limit))
        && !is_negative(vel.dot(limit))
    {
        delta_vel = Vector2f::new(0.0, 0.0);
    }
    *vel += delta_vel;
}

/// Project position and velocity forward — upstream `update_pos_vel_accel_xy`.
///
/// Position and velocity are suppressed independently, against their own
/// errors. A vehicle can be held in position while still being allowed to
/// slow down, which is what happens when it arrives at a boundary with speed
/// still on.
///
/// Note the position test omits the velocity check that its counterpart
/// applies — two conditions here, three there. Reproduced: position has no
/// equivalent of "already moving away", because position *is* the thing being
/// compared.
pub fn update_pos_vel_accel_xy(
    pos: &mut Vector2<Postype>,
    vel: &mut Vector2f,
    accel: Vector2f,
    dt: f32,
    limit: Vector2f,
    pos_error: Vector2f,
    vel_error: Vector2f,
) {
    let mut delta_pos = *vel * dt + accel * (0.5 * dt * dt);

    if !is_zero(limit.length_squared())
        && is_positive(delta_pos.dot(limit))
        && is_positive(pos_error.dot(limit))
    {
        delta_pos = Vector2f::new(0.0, 0.0);
    }

    // Widened per component, as the one-dimensional form does: the step is
    // computed in single precision because velocity and acceleration are, and
    // only the accumulation needs the extra bits.
    pos.x += Postype::from(delta_pos.x);
    pos.y += Postype::from(delta_pos.y);

    update_vel_accel_xy(vel, accel, dt, limit, vel_error);
}

/// Jerk-limit a two-dimensional acceleration command — upstream
/// `shape_accel_xy`.
///
/// The limit is on the *length* of the change, not on each axis. Limiting per
/// axis would let a diagonal change move by `sqrt(2)` times the intended jerk,
/// and would bend the commanded direction as one axis saturated before the
/// other. Limiting the vector keeps the direction and caps the rate.
pub fn shape_accel_xy(accel_desired: Vector2f, accel: &mut Vector2f, jerk_max: f32, dt: f32) {
    if !is_positive(jerk_max) {
        // Upstream reports an internal error and returns, leaving the
        // acceleration untouched. Reproduced: a caller that has passed a
        // nonsensical jerk limit is better served by nothing happening than
        // by an unbounded step.
        return;
    }
    if is_positive(dt) {
        let mut accel_delta = accel_desired - *accel;
        accel_delta.limit_length(jerk_max * dt);
        *accel += accel_delta;
    }
}

/// The three-dimensional spelling, which shapes only the horizontal pair —
/// upstream's `Vector3f` overload of `shape_accel_xy`.
///
/// The vertical component is left exactly as it was. That is the point of the
/// name: a multirotor's vertical axis has a different jerk budget and is
/// shaped separately.
pub fn shape_accel_xy_3d(accel_desired: Vector3f, accel: &mut Vector3f, jerk_max: f32, dt: f32) {
    let mut planar = Vector2f::new(accel.x, accel.y);
    shape_accel_xy(
        Vector2f::new(accel_desired.x, accel_desired.y),
        &mut planar,
        jerk_max,
        dt,
    );
    accel.x = planar.x;
    accel.y = planar.y;
}

/// Shape a velocity command into a jerk-limited acceleration — upstream
/// `shape_vel_accel_xy`.
///
/// The correction is computed by the vector square-root controller, then
/// passed through [`limit_accel_corner_xy`] *before* the feedforward is added.
/// Order matters: the cornering limit is about what the airframe can do to
/// change its own velocity, so applying it to the correction alone leaves the
/// feedforward — which the caller has already reasoned about — intact.
///
/// `limit_total_accel` then optionally caps the sum. A caller that has a
/// trajectory it trusts leaves it false and keeps its feedforward whole.
#[expect(
    clippy::too_many_arguments,
    reason = "upstream's signature; splitting it into a struct would make the call sites disagree with the code they are being checked against"
)]
pub fn shape_vel_accel_xy(
    vel_desired: Vector2f,
    accel_desired: Vector2f,
    vel: Vector2f,
    accel: &mut Vector2f,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
    limit_total_accel: bool,
) {
    if !is_positive(accel_max) || !is_positive(jerk_max) {
        return;
    }

    // The gain that makes the sqrt controller's linear region hand over to
    // its square-root region exactly at the acceleration limit.
    let kpa = jerk_max / accel_max;

    let vel_error = vel_desired - vel;
    let mut accel_target = sqrt_controller_xy(vel_error, kpa, jerk_max, dt);

    limit_accel_corner_xy(vel, &mut accel_target, accel_max);

    accel_target += accel_desired;

    if limit_total_accel {
        accel_target.limit_length(accel_max);
    }

    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

/// Shape a position command into a jerk-limited acceleration — upstream
/// `shape_pos_vel_accel_xy`.
///
/// The full outer loop: position error becomes a velocity correction, which
/// joins the feedforward velocity, which becomes an acceleration demand, which
/// is jerk-limited. Everything the vehicle is asked to do in the horizontal
/// plane comes through here.
///
/// # The correction is a scalar on the error's direction
///
/// The position error is reduced to a length, shaped as a scalar, and mapped
/// back onto its own direction. So the correction never points anywhere except
/// straight at the target. Shaping each axis separately would let a diagonal
/// error produce a curved approach, because the axis with less error would
/// finish first.
///
/// # The closing-rate bias
///
/// `sqrt_controller_accel` is given the *correction-frame* closing rate —
/// `(vel - vel_desired)` projected onto the error — rather than the raw
/// velocity. For a moving setpoint those differ: chasing a target that is
/// itself moving away means the position error shrinks more slowly than the
/// ground speed suggests, and biasing on ground speed would brake too early.
///
/// The result is divided by `k_v` before being added, which converts an
/// acceleration into the velocity correction that would have produced it —
/// the two terms have to be in the same units to be summed.
///
/// # Two different velocity limits
///
/// `vel_max` bounds the *correction* unconditionally, and additionally bounds
/// the *total* only when `limit_total` is set. A caller with a trajectory it
/// trusts still wants its correction bounded, but not its feedforward.
///
/// Note the correction is constrained symmetrically to `±vel_max` even though
/// the length it constrains is a magnitude — the negative half is reachable,
/// because the closing-rate bias above can drive it negative when the vehicle
/// is already overtaking the target.
#[expect(
    clippy::too_many_arguments,
    reason = "upstream's signature; splitting it into a struct would make the \
call sites disagree with the code they are being checked against"
)]
pub fn shape_pos_vel_accel_xy(
    pos_desired: Vector2<Postype>,
    vel_desired: Vector2f,
    accel_desired: Vector2f,
    pos: Vector2<Postype>,
    vel: Vector2f,
    accel: &mut Vector2f,
    vel_max: f32,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
    limit_total: bool,
) {
    if is_negative(vel_max) || !is_positive(accel_max) || !is_positive(jerk_max) {
        return;
    }

    // Inner velocity-loop gain, set so the sqrt controller's linear region
    // hands over to its square-root region exactly at the acceleration limit.
    let k_v = jerk_max / accel_max;

    let mut vel_corr_cmd = Vector2f::new(0.0, 0.0);

    let pos_error = Vector2f::new(
        (pos_desired.x - pos.x) as f32,
        (pos_desired.y - pos.y) as f32,
    );
    let pos_error_length = pos_error.length();

    if is_positive(pos_error_length) {
        let vel_corr_proj = (vel - vel_desired).dot(pos_error) / pos_error_length;

        let mut vel_corr_cmd_length = sqrt_controller(pos_error_length, k_v, accel_max, dt);

        let accel_corr_cmd_length = sqrt_controller_accel(
            pos_error_length,
            vel_corr_cmd_length,
            vel_corr_proj,
            k_v,
            accel_max,
        );

        vel_corr_cmd_length += accel_corr_cmd_length / k_v;

        if is_positive(vel_max) {
            vel_corr_cmd_length = vel_corr_cmd_length.clamp(-vel_max, vel_max);
        }

        vel_corr_cmd = pos_error * (vel_corr_cmd_length / pos_error_length);
    }

    let mut vel_target = vel_desired + vel_corr_cmd;
    if limit_total && is_positive(vel_max) {
        vel_target.limit_length(vel_max);
    }

    let mut accel_target = (vel_target - vel) * k_v;

    limit_accel_corner_xy(vel, &mut accel_target, accel_max);

    accel_target += accel_desired;

    if limit_total {
        accel_target.limit_length(accel_max);
    }

    shape_accel_xy(accel_target, accel, jerk_max, dt);
}

/// The vector square-root controller — upstream's `Vector2f` overload of
/// `sqrt_controller`.
///
/// Scalar controller on the error's *length*, with the result pointed back
/// along the error. So the correction never changes direction, only magnitude
/// — a per-axis version would corner differently depending on which way the
/// error happened to lie.
#[must_use]
pub fn sqrt_controller_xy(error: Vector2f, p: f32, second_ord_lim: f32, dt: f32) -> Vector2f {
    let error_length = error.length();
    if !is_positive(error_length) {
        return Vector2f::new(0.0, 0.0);
    }
    let correction_length = sqrt_controller(error_length, p, second_ord_lim, dt);
    error * (correction_length / error_length)
}

/// Limit acceleration, prioritising cross-track — upstream `limit_accel_xy`.
///
/// When the demand exceeds what the airframe can deliver, something has to be
/// given up, and this gives up along-track first. Cross-track acceleration is
/// what holds the vehicle on its path; along-track only changes how fast it
/// gets there. Sacrificing the path to keep the schedule is the wrong trade,
/// so the schedule goes.
///
/// With no velocity there is no track to be cross to, and it falls back to a
/// plain magnitude limit.
///
/// Returns whether any limiting happened.
pub fn limit_accel_xy(vel: Vector2f, accel: &mut Vector2f, accel_max: f32) -> bool {
    if !is_positive(accel_max) {
        return false;
    }
    if accel.length_squared() <= accel_max * accel_max {
        return false;
    }

    if vel.is_zero() {
        accel.limit_length(accel_max);
        return true;
    }

    let vel_unit = vel.normalized_or_zero();
    let mut accel_dir = vel_unit.dot(*accel);
    let mut accel_cross = *accel - vel_unit * accel_dir;

    if accel_cross.limit_length(accel_max) {
        // The cross-track component alone used the entire budget, so there is
        // nothing left for along-track at all.
        accel_dir = 0.0;
    } else {
        // limit_length cannot absolutely guarantee this difference is
        // non-negative, hence the guarded square root — upstream's comment.
        let accel_max_dir = safe_sqrt(accel_max * accel_max - accel_cross.length_squared());
        accel_dir = accel_dir.clamp(-accel_max_dir, accel_max_dir);
    }

    *accel = accel_cross + vel_unit * accel_dir;
    true
}

/// Limit acceleration with direction-dependent priority — upstream
/// `limit_accel_corner_xy`.
///
/// The same budget problem as [`limit_accel_xy`], answered differently
/// depending on whether the vehicle is trying to slow down.
///
/// **Not braking:** cross-track wins, as before. Path over schedule.
///
/// **Braking:** along-track wins. A vehicle that has asked to decelerate
/// usually has a reason — an obstacle, a boundary, an arrival — and cutting
/// the deceleration to hold a curve is the wrong trade in exactly the case
/// where holding the curve matters least.
///
/// The pre-limit to twice `accel_max` before decomposing is upstream's, and
/// its comment explains it: the demand is often proportional to velocity
/// error and can be enormous, which makes the direction ill-conditioned. Two
/// times leaves room for the decomposition to be meaningful while bounding
/// the input.
pub fn limit_accel_corner_xy(vel: Vector2f, accel: &mut Vector2f, accel_max: f32) -> bool {
    if !is_positive(accel_max) {
        return false;
    }

    if vel.is_zero() {
        return accel.limit_length(accel_max);
    }

    accel.limit_length(2.0 * accel_max);

    let vel_unit = vel.normalized_or_zero();
    let mut accel_dir_scalar = accel.dot(vel_unit);
    let mut accel_dir = vel_unit * accel_dir_scalar;
    let mut accel_cross = *accel - accel_dir;

    if is_positive(accel_dir_scalar) {
        let accel_cross_mag = accel_cross.length().min(accel_max);
        let accel_along_max = safe_sqrt(accel_max * accel_max - accel_cross_mag * accel_cross_mag);

        accel_cross.limit_length(accel_max);
        accel_dir.limit_length(accel_along_max);

        *accel = accel_cross + accel_dir;
        return true;
    }

    accel_dir_scalar = accel_dir_scalar.max(-accel_max);
    accel_dir = vel_unit * accel_dir_scalar;

    let accel_cross_max = safe_sqrt(accel_max * accel_max - accel_dir_scalar * accel_dir_scalar);
    accel_cross.limit_length(accel_cross_max);

    *accel = accel_cross + accel_dir;
    true
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "several tests assert a value was left exactly alone, which is the \
claim; an epsilon would accept a small change and a small change is the failure"
    )]

    use super::*;

    /// Close to the target the controller is proportional, and the gain is
    /// exactly p.
    #[test]
    fn near_the_target_it_is_a_p_controller() {
        // linear_dist = 10 / 4 = 2.5, so an error of 1 is inside it.
        assert_eq!(sqrt_controller(1.0, 2.0, 10.0, 0.0), 2.0);
        assert_eq!(sqrt_controller(-1.0, 2.0, 10.0, 0.0), -2.0);
        assert_eq!(sqrt_controller(0.0, 2.0, 10.0, 0.0), 0.0);
    }

    /// Far from it the answer is the speed a constant deceleration could stop
    /// from — which is what makes the approach as fast as it safely can be.
    #[test]
    fn far_from_the_target_it_is_a_braking_profile() {
        // With p = 0 the controller is pure sqrt: v = sqrt(2*a*d).
        let v = sqrt_controller(100.0, 0.0, 2.0, 0.0);
        assert!((v - (2.0_f32 * 2.0 * 100.0).sqrt()).abs() < 1e-3, "{v}");
    }

    /// The two branches meet. A discontinuity here would be a step in
    /// commanded velocity every time the vehicle crossed the threshold.
    #[test]
    fn the_two_regimes_join_continuously() {
        let (p, lim) = (2.0_f32, 10.0_f32);
        let linear_dist = lim / sq(p);
        let below = sqrt_controller(linear_dist - 1e-4, p, lim, 0.0);
        let above = sqrt_controller(linear_dist + 1e-4, p, lim, 0.0);
        assert!(
            (above - below).abs() < 1e-2,
            "a step at the join: {below} then {above}"
        );
    }

    /// It is odd about zero: reversing the error reverses the answer.
    #[test]
    fn the_response_is_symmetric() {
        for e in [0.1_f32, 1.0, 2.5, 10.0, 100.0] {
            let pos = sqrt_controller(e, 2.0, 10.0, 0.0);
            let neg = sqrt_controller(-e, 2.0, 10.0, 0.0);
            assert!((pos + neg).abs() < 1e-4, "at {e}: {pos} and {neg}");
        }
    }

    /// With a timestep the correction cannot exceed the whole error in one
    /// step, which is what stops it overshooting at low rates.
    #[test]
    fn the_correction_cannot_overshoot_in_one_step() {
        let dt = 0.1_f32;
        let error = 0.5_f32;
        let v = sqrt_controller(error, 100.0, 1000.0, dt);
        assert!(
            v <= error / dt + 1e-4,
            "{v} would overshoot an error of {error} in {dt} s"
        );
    }

    /// The inverse recovers the error the controller was given.
    #[test]
    fn the_inverse_round_trips() {
        for error in [0.5_f32, 2.0, 10.0, 100.0] {
            let out = sqrt_controller(error, 2.0, 10.0, 0.0);
            let back = inv_sqrt_controller(out, 2.0, 10.0);
            assert!(
                (back - error).abs() < 0.01 * error,
                "{error} became {out} and came back as {back}"
            );
        }
    }

    /// Stopping distance grows with the square of speed once braking limits
    /// bite — doubling the speed roughly quadruples the distance.
    #[test]
    fn stopping_distance_grows_with_the_square_of_speed() {
        let d1 = stopping_distance(10.0, 0.5, 2.0);
        let d2 = stopping_distance(20.0, 0.5, 2.0);
        let ratio = d2 / d1;
        assert!(
            (3.5..4.5).contains(&ratio),
            "expected about four times, got {ratio}"
        );
    }

    /// Jerk limiting means acceleration moves toward its target at a bounded
    /// rate, not in a step.
    #[test]
    fn acceleration_approaches_its_target_at_the_jerk_limit() {
        let mut accel = 0.0_f32;
        shape_accel(10.0, &mut accel, 5.0, 0.1).expect("valid");
        assert_eq!(accel, 0.5, "5 m/s^3 for 0.1 s is 0.5 m/s^2");

        // And it arrives eventually.
        for _ in 0..100 {
            shape_accel(10.0, &mut accel, 5.0, 0.1).expect("valid");
        }
        assert!((accel - 10.0).abs() < 1e-4, "{accel}");
    }

    /// A non-positive jerk limit is refused, and the acceleration is left
    /// alone. Upstream raises an internal error and returns; this reports it.
    #[test]
    fn a_bad_jerk_limit_is_refused_and_changes_nothing() {
        let mut accel = 3.0_f32;
        assert_eq!(
            shape_accel(10.0, &mut accel, 0.0, 0.1),
            Err(ShapeError::JerkNotPositive)
        );
        assert_eq!(accel, 3.0, "untouched");
    }

    /// Malformed limits are refused too, with the reason.
    #[test]
    fn malformed_limits_are_refused() {
        let mut accel = 0.0_f32;
        assert_eq!(
            shape_vel_accel(1.0, 0.0, 0.0, &mut accel, 1.0, 2.0, 5.0, 0.1, false),
            Err(ShapeError::AccelLimitsMalformed),
            "accel_min must be negative"
        );
        assert_eq!(
            shape_vel_accel(1.0, 0.0, 0.0, &mut accel, -1.0, 2.0, 0.0, 0.1, false),
            Err(ShapeError::JerkNotPositive)
        );
        assert_eq!(accel, 0.0);
    }

    /// A velocity demand is tracked, and the acceleration stays inside its
    /// limits the whole way.
    #[test]
    fn a_velocity_demand_is_tracked_within_the_limits() {
        let mut vel = 0.0_f32;
        let mut accel = 0.0_f32;
        let dt = 0.01_f32;

        for _ in 0..1000 {
            shape_vel_accel(5.0, 0.0, vel, &mut accel, -3.0, 3.0, 10.0, dt, true).expect("valid");
            assert!(
                (-3.0..=3.0).contains(&accel),
                "acceleration {accel} left its limits"
            );
            update_vel_accel(&mut vel, accel, dt, 0.0, 0.0);
        }
        assert!((vel - 5.0).abs() < 0.05, "should have arrived, got {vel}");
    }

    /// A position demand is tracked, and the vehicle stops there rather than
    /// oscillating around it.
    #[test]
    fn a_position_demand_is_tracked_and_settles() {
        let mut pos = 0.0_f64;
        let mut vel = 0.0_f32;
        let mut accel = 0.0_f32;
        let dt = 0.01_f32;

        for _ in 0..3000 {
            shape_pos_vel_accel(
                100.0, 0.0, 0.0, pos, vel, &mut accel, -10.0, 10.0, -3.0, 3.0, 10.0, dt, true,
            )
            .expect("valid");
            update_pos_vel_accel(&mut pos, &mut vel, accel, dt, 0.0, 0.0, 0.0);
        }

        assert!((pos - 100.0).abs() < 0.5, "settled at {pos}, wanted 100");
        assert!(
            vel.abs() < 0.1,
            "should have stopped, still moving at {vel}"
        );
    }

    /// The limit direction stops a controller pushing further into a
    /// constraint, but still lets it unwind.
    #[test]
    fn a_limit_blocks_pushing_but_not_unwinding() {
        // Pushing further positive while limited positive and error positive.
        let mut vel = 1.0_f32;
        update_vel_accel(&mut vel, 10.0, 0.1, 1.0, 1.0);
        assert_eq!(vel, 1.0, "the step should have been refused");

        // Same limit, but the velocity is negative: the step is allowed,
        // clipped so it cannot cross zero.
        let mut vel = -0.5_f32;
        update_vel_accel(&mut vel, 10.0, 0.1, 1.0, 1.0);
        assert!(
            vel > -0.5 && vel <= 0.0,
            "should unwind toward zero, got {vel}"
        );

        // No limit at all: the step applies in full.
        let mut vel = 1.0_f32;
        update_vel_accel(&mut vel, 10.0, 0.1, 0.0, 0.0);
        assert_eq!(vel, 2.0);
    }

    /// An angle demand takes the short way round.
    #[test]
    fn an_angle_demand_turns_the_short_way() {
        let mut accel = 0.0_f32;
        let angle = 3.0_f32; // just under pi
        let desired = -3.0_f32; // just over -pi: two tenths away, not six

        shape_angle_vel_accel(
            desired, 0.0, 0.0, angle, 0.0, &mut accel, -1.0, 1.0, 2.0, 5.0, 0.01, true,
        )
        .expect("valid");

        assert!(
            accel > 0.0,
            "the short way from 3.0 to -3.0 is forwards, got {accel}"
        );
    }

    /// Moving away from the target has no braking profile to differentiate.
    #[test]
    fn the_implied_rate_is_zero_when_moving_away() {
        assert_eq!(sqrt_controller_accel(10.0, 5.0, -5.0, 2.0, 10.0), 0.0);
        assert_eq!(sqrt_controller_accel(10.0, -5.0, 5.0, 2.0, 10.0), 0.0);
    }

    /// Pure horizontal motion gets the horizontal limit; pure vertical gets
    /// the vertical one, and which vertical depends on the direction.
    #[test]
    fn the_pure_axes_return_their_own_limits() {
        assert_eq!(
            kinematic_limit(Vector3f::new(1.0, 0.0, 0.0), 5.0, 2.0, 3.0),
            5.0
        );
        assert_eq!(
            kinematic_limit(Vector3f::new(0.0, 0.0, 1.0), 5.0, 2.0, 3.0),
            3.0,
            "upward"
        );
        assert_eq!(
            kinematic_limit(Vector3f::new(0.0, 0.0, -1.0), 5.0, 2.0, 3.0),
            2.0,
            "downward"
        );
    }

    /// A direction of no length has no answer, and neither does a zero limit.
    #[test]
    fn a_degenerate_direction_or_limit_gives_zero() {
        assert_eq!(kinematic_limit(Vector3f::zero(), 5.0, 2.0, 3.0), 0.0);
        assert_eq!(
            kinematic_limit(Vector3f::new(1.0, 0.0, 1.0), 0.0, 2.0, 3.0),
            0.0
        );
        assert_eq!(
            kinematic_limit(Vector3f::new(1.0, 0.0, 1.0), 5.0, 0.0, 3.0),
            0.0
        );
    }

    /// On a diagonal, whichever limit binds first is the one that decides —
    /// and the result is always at least as large as the speed either limit
    /// alone would allow along that direction.
    #[test]
    fn the_binding_limit_decides_on_a_diagonal() {
        // Shallow climb: horizontal binds.
        let shallow = kinematic_limit(Vector3f::new(10.0, 0.0, 1.0), 5.0, 2.0, 3.0);
        assert!(
            shallow > 5.0,
            "along a shallow slope, faster than 5: {shallow}"
        );

        // Steep climb: the vertical limit binds instead.
        let steep = kinematic_limit(Vector3f::new(1.0, 0.0, 10.0), 5.0, 2.0, 3.0);
        assert!(steep < shallow, "steeper should be slower: {steep}");
        assert!(
            steep > 3.0,
            "but still faster than the vertical limit alone"
        );
    }

    /// Up and down differ when the limits do — which is the reason the
    /// function takes two of them.
    #[test]
    fn climbing_and_descending_differ() {
        let up = kinematic_limit(Vector3f::new(1.0, 0.0, 5.0), 5.0, 1.0, 4.0);
        let down = kinematic_limit(Vector3f::new(1.0, 0.0, -5.0), 5.0, 1.0, 4.0);
        assert!(
            up > down,
            "a higher climb limit should allow more: {up} vs {down}"
        );
    }

    /// Scaling a direction does not change the answer: only its slope
    /// matters.
    #[test]
    fn only_the_direction_matters_not_its_length() {
        let a = kinematic_limit(Vector3f::new(3.0, 4.0, 5.0), 5.0, 2.0, 3.0);
        let b = kinematic_limit(Vector3f::new(30.0, 40.0, 50.0), 5.0, 2.0, 3.0);
        assert!((a - b).abs() < 1e-4, "{a} and {b}");
    }
}
