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

use crate::scalar::{constrain_value, is_negative, is_positive, is_zero, safe_sqrt, sq, wrap_pi};

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
}
