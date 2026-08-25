//! Hermite spline waypoint traversal, upstream `AP_Math/SplineCurve`. COP-003.
//!
//! Copter's spline waypoints curve through a turn rather than stopping at it.
//! The path is a cubic Hermite spline — fixed by the two endpoints and the
//! velocity at each — and the vehicle is walked along it at whatever speed the
//! cornering limits allow.
//!
//! # Why the velocities get scaled down on short legs
//!
//! A Hermite spline honours the end velocities exactly, so a fast entry and
//! exit across a short leg produces a curve that bulges well outside the two
//! waypoints. Upstream guards that with [`SPLINE_FACTOR`]: if the two end
//! speeds together exceed four times the straight-line distance, *both* are
//! scaled down before the spline is solved. The path stays inside something
//! reasonable at the cost of not matching the requested end velocities.
//!
//! # Speed comes from curvature, not from the schedule
//!
//! [`SplineCurve::advance_target_along_track`] does not follow a precomputed
//! timeline. At each step it evaluates the spline's own acceleration, splits
//! it into the part along the path and the part across it, and caps the speed
//! so the across-path component stays inside the lateral limit. A tight part
//! of the curve slows the vehicle down because the geometry says so.

use crate::control::kinematic_limit;
use crate::scalar::{constrain_value, is_positive, is_zero, safe_sqrt, sq, Real};
use crate::vector3::{Vector3, Vector3f};

/// Position vector, upstream `Vector3p`.
pub type Vector3p = Vector3<f64>;

/// Curve shape control, upstream `SPLINE_FACTOR`.
///
/// Larger values allow longer, more curved paths; smaller ones pull the path
/// toward a straight line. Four is upstream's.
pub const SPLINE_FACTOR: f32 = 4.0;

/// Share of the acceleration budget available along the path, upstream
/// `TANGENTIAL_ACCEL_SCALER`.
pub const TANGENTIAL_ACCEL_SCALER: f32 = 0.5;

/// Share available across it, upstream `LATERAL_ACCEL_SCALER`.
///
/// The two are each a half rather than summing to one: they bound different
/// directions and both can be at their limit at once, which is a total of
/// about 0.7 of the budget rather than 1.0.
pub const LATERAL_ACCEL_SCALER: f32 = 0.5;

/// A spline segment between two waypoints.
#[derive(Debug, Clone, Copy)]
pub struct SplineCurve {
    origin: Vector3p,
    destination: Vector3p,
    origin_vel: Vector3f,
    destination_vel: Vector3f,
    hermite_solution: [Vector3p; 4],

    time: f32,
    speed_xy: f32,
    speed_up: f32,
    speed_down: f32,
    accel_xy: f32,
    accel_z: f32,
    origin_speed_max: f32,
    destination_speed_max: f32,

    zero_length: bool,
    reached_destination: bool,
}

impl Default for SplineCurve {
    fn default() -> Self {
        Self {
            origin: Vector3p::zero(),
            destination: Vector3p::zero(),
            origin_vel: Vector3f::zero(),
            destination_vel: Vector3f::zero(),
            hermite_solution: [Vector3p::zero(); 4],
            time: 0.0,
            speed_xy: 0.0,
            speed_up: 0.0,
            speed_down: 0.0,
            accel_xy: 0.0,
            accel_z: 0.0,
            origin_speed_max: 0.0,
            destination_speed_max: 0.0,
            zero_length: false,
            reached_destination: false,
        }
    }
}

fn to_postype(v: Vector3f) -> Vector3p {
    Vector3p::new(f64::from(v.x), f64::from(v.y), f64::from(v.z))
}

#[allow(
    clippy::cast_possible_truncation,
    reason = "upstream's tofloat() narrows a postype vector the same way; the \
values are velocities and accelerations, which are small"
)]
fn to_float(v: Vector3p) -> Vector3f {
    Vector3f::new(v.x as f32, v.y as f32, v.z as f32)
}

/// What one evaluation of the spline produced.
#[derive(Debug, Clone, Copy, Default)]
pub struct SplineState {
    /// Position on the curve.
    pub position: Vector3p,
    /// First derivative with respect to spline time — not metres per second.
    pub velocity: Vector3f,
    /// Second derivative.
    pub acceleration: Vector3f,
    /// Third derivative, constant across the segment for a cubic.
    pub jerk: Vector3f,
}

/// What [`SplineCurve::advance_target_along_track`] produced.
#[derive(Debug, Clone, Copy, Default)]
pub struct SplineTarget {
    /// Where the vehicle should be.
    pub position: Vector3p,
    /// How fast, in real units.
    pub velocity: Vector3f,
}

impl SplineCurve {
    /// A curve with no segment set.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Set the kinematic limits, upstream `set_speed_accel`.
    ///
    /// Magnitudes: upstream takes the absolute value of each, so a caller
    /// passing a signed descent rate gets the same answer either way.
    pub fn set_speed_accel(
        &mut self,
        speed_xy: f32,
        speed_up: f32,
        speed_down: f32,
        accel_xy: f32,
        accel_z: f32,
    ) {
        self.speed_xy = speed_xy.abs();
        self.speed_up = speed_up.abs();
        self.speed_down = speed_down.abs();
        self.accel_xy = accel_xy.abs();
        self.accel_z = accel_z.abs();
    }

    /// Define the segment, upstream `set_origin_and_destination`.
    ///
    /// A zero-length segment is reported finished immediately rather than
    /// producing a curve with no direction.
    ///
    /// # The braking clamp is inert for the destination, and only there
    ///
    /// `calc_dt_speed_max` ends by capping its answer at the speed from which
    /// the vehicle could still slow to `destination_speed_max` before
    /// arriving, and it reads that member directly. Upstream passes the member
    /// being computed as the *out-parameter*:
    ///
    /// - for the **destination**, out-parameter and the member read are the
    ///   same storage, so the cap reads back the value it has just written —
    ///   and `sqrt(2·a·dist + v²) ≥ v` always, so it cannot reduce anything;
    /// - for the **origin**, the out-parameter is a different member, so the
    ///   cap reads the real destination speed and binds normally.
    ///
    /// The origin call also runs *before* the destination speed is assigned
    /// for this segment, so it reads whatever the previous segment left there.
    ///
    /// The port passes the reference explicitly rather than relying on
    /// aliasing, which is the only reason any of this is visible.
    pub fn set_origin_and_destination(
        &mut self,
        origin: Vector3p,
        destination: Vector3p,
        origin_vel: Vector3f,
        destination_vel: Vector3f,
    ) {
        self.origin = origin;
        self.destination = destination;

        let delta = destination - origin;
        self.zero_length =
            is_zero((delta.x * delta.x + delta.y * delta.y + delta.z * delta.z) as f32);
        if self.zero_length {
            self.time = 1.0;
            self.origin_vel = Vector3f::zero();
            self.destination_vel = Vector3f::zero();
            self.reached_destination = true;
            self.origin_speed_max = 0.0;
            self.destination_speed_max = 0.0;
            return;
        }

        self.origin_vel = origin_vel;
        self.destination_vel = destination_vel;
        self.reached_destination = false;

        // Note upstream's comment: `time` may hold a leftover from the
        // previous waypoint, so it is reset here rather than assumed zero.
        self.time = 0.0;

        // Keep a short segment from bulging: if the two end speeds together
        // exceed four times the straight-line distance, scale both down.
        let vel_len = self.origin_vel.length() + self.destination_vel.length();
        let pos_len = to_float(delta).length() * SPLINE_FACTOR;
        if vel_len > pos_len {
            let vel_scaling = pos_len / vel_len;
            self.update_solution(
                origin,
                destination,
                self.origin_vel * vel_scaling,
                self.destination_vel * vel_scaling,
            );
        } else {
            self.update_solution(origin, destination, self.origin_vel, self.destination_vel);
        }

        // The clamp binds here. Upstream's out-parameter is
        // `_origin_speed_max`, but the clamp reads `_destination_speed_max` --
        // a different member -- so there is no aliasing and the cap applies.
        //
        // Note this runs BEFORE `destination_speed_max` is assigned for this
        // segment, so it reads whatever the previous segment left there, or
        // zero on a fresh curve. Reproduced rather than tidied.
        let at_origin = self.calc_dt_speed_max(0.0, 0.0, Some(self.destination_speed_max));
        self.origin_speed_max = at_origin.speed_max;

        if self.destination_vel.is_zero() {
            self.destination_speed_max = 0.0;
        } else {
            // Likewise, aliased to `_destination_speed_max`.
            let at_dest = self.calc_dt_speed_max(1.0, 0.0, None);
            self.destination_speed_max = at_dest.speed_max;
        }
    }

    /// Build the four Hermite coefficients, upstream `update_solution`.
    ///
    /// The standard cubic Hermite basis rearranged into a plain polynomial in
    /// spline time, so evaluating it later is three multiply-adds rather than
    /// four basis functions.
    fn update_solution(
        &mut self,
        origin: Vector3p,
        dest: Vector3p,
        origin_vel: Vector3f,
        dest_vel: Vector3f,
    ) {
        let ov = to_postype(origin_vel);
        let dv = to_postype(dest_vel);
        self.hermite_solution = [
            origin,
            ov,
            origin * -3.0 - ov * 2.0 + dest * 3.0 - dv,
            origin * 2.0 + ov - dest * 2.0 + dv,
        ];
    }

    /// Evaluate the spline at a time in `[0, 1]`, upstream
    /// `calc_target_pos_vel`.
    ///
    /// The derivatives are with respect to *spline time*, not to real time —
    /// they give the shape of the curve, and the traversal speed is decided
    /// separately by [`SplineCurve::calc_dt_speed_max`].
    #[must_use]
    pub fn calc_target_pos_vel(&self, time: f32) -> SplineState {
        let time_sq = sq(time);
        let time_cubed = time_sq * time;
        let h = &self.hermite_solution;

        SplineState {
            position: h[0]
                + h[1] * f64::from(time)
                + h[2] * f64::from(time_sq)
                + h[3] * f64::from(time_cubed),
            velocity: to_float(h[1])
                + to_float(h[2]) * (2.0 * time)
                + to_float(h[3]) * (3.0 * time_sq),
            acceleration: to_float(h[2]) * 2.0 + to_float(h[3]) * (6.0 * time),
            jerk: to_float(h[3]) * 6.0,
        }
    }

    /// What one step along the curve is allowed to be.
    #[must_use]
    ///
    /// `braking_ref` is the destination speed the final clamp measures
    /// against. `None` reproduces upstream's two setup call sites, where the
    /// out-parameter aliases the member being computed and the clamp is
    /// therefore inert — see the note on [`SplineCurve::set_origin_and_destination`].
    fn calc_dt_speed_max(
        &self,
        time: f32,
        distance_delta: f32,
        braking_ref: Option<f32>,
    ) -> DtSpeedMax {
        let mut out = DtSpeedMax::default();

        let s = self.calc_target_pos_vel(time);
        out.target_pos = s.position;

        // Velocity, acceleration and jerk cannot all be zero on a real curve.
        // Upstream raises an internal error, marks the segment finished and
        // returns; the port reports it instead — see `DtSpeedMax::degenerate`.
        if s.velocity.is_zero() && s.acceleration.is_zero() && s.jerk.is_zero() {
            out.degenerate = true;
            return out;
        }

        let spline_vel_length = s.velocity.length();
        if is_zero(spline_vel_length) {
            // Standing still on the curve: the direction has to come from a
            // higher derivative, and the step size from integrating it.
            if is_zero(sq(s.acceleration.x) + sq(s.acceleration.y) + sq(s.acceleration.z)) {
                let Some(unit) = s.jerk.normalized() else {
                    out.degenerate = true;
                    return out;
                };
                out.spline_vel_unit = unit;
                out.spline_dt = Real::powf(6.0 * distance_delta / s.jerk.length(), 1.0 / 3.0);
            } else {
                let Some(unit) = s.acceleration.normalized() else {
                    out.degenerate = true;
                    return out;
                };
                out.spline_vel_unit = unit;
                out.spline_dt = safe_sqrt(2.0 * distance_delta / s.acceleration.length());
            }
        } else {
            let Some(unit) = s.velocity.normalized() else {
                out.degenerate = true;
                return out;
            };
            out.spline_vel_unit = unit;
            out.spline_dt = distance_delta / spline_vel_length;
        }

        // Split the curve's acceleration into along-path and across-path. The
        // across-path part is what a turn costs, and it is what caps the
        // speed.
        let tangent_len = s.acceleration.dot(out.spline_vel_unit);
        let accel_norm = s.acceleration - out.spline_vel_unit * tangent_len;
        let accel_norm_length = accel_norm.length();

        let tangential_speed_max = kinematic_limit(
            out.spline_vel_unit,
            self.speed_xy,
            self.speed_up,
            self.speed_down,
        );
        let accel_norm_max = LATERAL_ACCEL_SCALER
            * kinematic_limit(accel_norm, self.accel_xy, self.accel_z, self.accel_z);

        if is_zero(tangential_speed_max) {
            out.degenerate = true;
            return out;
        }

        out.speed_max = if is_positive(accel_norm_max)
            && is_positive(accel_norm_length)
            && is_positive(spline_vel_length)
            && ((accel_norm_length / accel_norm_max) > sq(spline_vel_length / tangential_speed_max))
        {
            // The turn is tight enough that lateral acceleration binds first.
            spline_vel_length / safe_sqrt(accel_norm_length / accel_norm_max)
        } else {
            tangential_speed_max
        };

        out.accel_max = TANGENTIAL_ACCEL_SCALER
            * kinematic_limit(
                out.spline_vel_unit,
                self.accel_xy,
                self.accel_z,
                self.accel_z,
            );
        if is_zero(out.accel_max) {
            out.degenerate = true;
            return out;
        }

        // And never faster than something that can still slow to the
        // destination speed by the time it arrives.
        //
        // Skipped when `braking_ref` is None. That is not an optimisation: it
        // is what upstream does, by aliasing. See the doc comment above.
        if let Some(dest_speed) = braking_ref {
            let delta = self.destination - out.target_pos;
            let dist = to_float(delta).length();
            let braking =
                safe_sqrt(2.0 * out.accel_max * (dist + sq(dest_speed) / (2.0 * out.accel_max)));
            out.speed_max = out.speed_max.min(braking);
        }

        out
    }

    /// Step the target along the curve, upstream
    /// `advance_target_along_track`.
    ///
    /// `current_vel` is the target velocity from the previous step; the new
    /// speed is that one moved toward the allowed maximum by no more than
    /// `accel_max * dt`, so the commanded speed cannot jump.
    pub fn advance_target_along_track(&mut self, dt: f32, current_vel: Vector3f) -> SplineTarget {
        if self.zero_length {
            return SplineTarget {
                position: self.destination,
                velocity: Vector3f::zero(),
            };
        }

        let speed = current_vel.length();
        let distance_delta = speed * dt;

        // Here upstream's `speed_max` is a local, so the clamp binds
        // against the real destination speed.
        let step =
            self.calc_dt_speed_max(self.time, distance_delta, Some(self.destination_speed_max));
        if step.degenerate {
            self.reached_destination = true;
            return SplineTarget {
                position: step.target_pos,
                velocity: Vector3f::zero(),
            };
        }

        let new_speed = constrain_value(
            step.speed_max,
            speed - step.accel_max * dt,
            speed + step.accel_max * dt,
        );

        self.time += step.spline_dt;
        if self.time >= 1.0 {
            self.time = 1.0;
            self.reached_destination = true;
        }

        SplineTarget {
            position: step.target_pos,
            velocity: step.spline_vel_unit * new_speed,
        }
    }

    /// Whether the segment is finished, upstream `reached_destination`.
    #[must_use]
    pub const fn reached_destination(&self) -> bool {
        self.reached_destination
    }

    /// The unscaled destination velocity, upstream `get_destination_vel`.
    #[must_use]
    pub const fn destination_vel(&self) -> Vector3f {
        self.destination_vel
    }

    /// Fastest allowed at the origin, upstream `get_origin_speed_max`.
    #[must_use]
    pub const fn origin_speed_max(&self) -> f32 {
        self.origin_speed_max
    }

    /// Fastest allowed at the destination, upstream
    /// `get_destination_speed_max`.
    #[must_use]
    pub const fn destination_speed_max(&self) -> f32 {
        self.destination_speed_max
    }

    /// Lower the destination speed, upstream `set_destination_speed_max`.
    ///
    /// Only ever lowers it: upstream takes the minimum of the current and the
    /// new value, so a later leg can slow the join but not speed it up.
    pub fn set_destination_speed_max(&mut self, destination_speed_max: f32) {
        self.destination_speed_max = self.destination_speed_max.min(destination_speed_max);
    }

    /// The current spline time, 0 at the origin and 1 at the destination.
    #[must_use]
    pub const fn spline_time(&self) -> f32 {
        self.time
    }
}

/// One step's allowances, upstream's out-parameters from `calc_dt_speed_max`.
#[derive(Debug, Clone, Copy, Default)]
struct DtSpeedMax {
    spline_dt: f32,
    target_pos: Vector3p,
    spline_vel_unit: Vector3f,
    speed_max: f32,
    accel_max: f32,
    /// Set where upstream raises an internal error and marks the segment
    /// finished: a curve with no velocity, acceleration or jerk, or a
    /// kinematic limit of zero.
    degenerate: bool,
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "a zero-length segment and a zero destination velocity must give \nEXACTLY zero; an epsilon would accept a small residual speed, which is the thing being \nruled out"
    )]

    use super::*;

    fn curve() -> SplineCurve {
        let mut c = SplineCurve::new();
        c.set_speed_accel(500.0, 250.0, 150.0, 250.0, 100.0);
        c
    }

    /// The spline passes through both waypoints exactly: that is what fixes
    /// the first coefficient and the sum of all four.
    #[test]
    fn the_curve_starts_and_ends_at_its_waypoints() {
        let mut c = curve();
        let origin = Vector3p::new(0.0, 0.0, 0.0);
        let dest = Vector3p::new(1000.0, 0.0, 0.0);
        c.set_origin_and_destination(
            origin,
            dest,
            Vector3f::new(100.0, 0.0, 0.0),
            Vector3f::new(0.0, 100.0, 0.0),
        );

        let at_start = c.calc_target_pos_vel(0.0).position;
        let at_end = c.calc_target_pos_vel(1.0).position;
        assert!((at_start - origin).length() < 1e-6, "{at_start:?}");
        assert!((at_end - dest).length() < 1e-3, "{at_end:?}");
    }

    /// A cubic has constant jerk, which is what makes the third coefficient
    /// the whole of it.
    #[test]
    fn jerk_is_constant_across_the_segment() {
        let mut c = curve();
        c.set_origin_and_destination(
            Vector3p::new(0.0, 0.0, 0.0),
            Vector3p::new(1000.0, 500.0, 0.0),
            Vector3f::new(100.0, 0.0, 0.0),
            Vector3f::new(0.0, 100.0, 0.0),
        );
        let a = c.calc_target_pos_vel(0.1).jerk;
        let b = c.calc_target_pos_vel(0.9).jerk;
        assert!((a - b).length() < 1e-3, "{a:?} and {b:?}");
    }

    /// A zero-length segment is finished the moment it is set, rather than
    /// producing a curve with no direction.
    #[test]
    fn a_zero_length_segment_is_finished_immediately() {
        let mut c = curve();
        let p = Vector3p::new(100.0, 200.0, -50.0);
        c.set_origin_and_destination(p, p, Vector3f::new(50.0, 0.0, 0.0), Vector3f::zero());

        assert!(c.reached_destination());
        assert_eq!(c.origin_speed_max(), 0.0);
        assert_eq!(c.destination_speed_max(), 0.0);

        let t = c.advance_target_along_track(0.1, Vector3f::new(10.0, 0.0, 0.0));
        assert!((t.position - p).length() < 1e-9);
        assert!(t.velocity.is_zero());
    }

    /// The short-segment guard scales both end velocities when they are large
    /// against the distance — otherwise the curve bulges outside its own
    /// waypoints.
    #[test]
    fn a_short_segment_scales_its_end_velocities_down() {
        let far = {
            let mut c = curve();
            c.set_origin_and_destination(
                Vector3p::new(0.0, 0.0, 0.0),
                Vector3p::new(10_000.0, 0.0, 0.0),
                Vector3f::new(0.0, 500.0, 0.0),
                Vector3f::new(0.0, 500.0, 0.0),
            );
            // How far the curve strays from the straight line, sampled along
            // its whole length: at the midpoint a symmetric curve's deviation
            // is exactly zero, which is where the first version of this test
            // looked.
            let mut worst = 0.0_f64;
            for i in 0..=100 {
                let p = c.calc_target_pos_vel(i as f32 / 100.0).position;
                worst = worst.max(p.y.abs());
            }
            worst
        };

        let near = {
            let mut c = curve();
            c.set_origin_and_destination(
                Vector3p::new(0.0, 0.0, 0.0),
                Vector3p::new(100.0, 0.0, 0.0),
                Vector3f::new(0.0, 500.0, 0.0),
                Vector3f::new(0.0, 500.0, 0.0),
            );
            let mut worst = 0.0_f64;
            for i in 0..=100 {
                let p = c.calc_target_pos_vel(i as f32 / 100.0).position;
                worst = worst.max(p.y.abs());
            }
            worst
        };

        // Both runs use the same end velocities, so without the guard they
        // would deviate by the same absolute amount. The guard scales both
        // velocities by pos_len/vel_len, and the deviation is linear in
        // velocity, so the short leg's should be exactly that fraction of the
        // long one's.
        //
        // 100-unit leg, 500 either end: vel_len = 1000, pos_len = 4*100 = 400,
        // so the scaling is 0.4.
        let expected_ratio: f64 = (4.0 * 100.0) / (500.0 + 500.0);
        let ratio = near / far;
        assert!(
            (ratio - expected_ratio).abs() < 0.02,
            "the guard should scale the deviation by {expected_ratio}: got {ratio} \
             ({near} against {far})"
        );

        // And the long leg is not scaled at all, because its velocities are
        // well inside the bound.
        assert!(far > 40.0, "the long leg should be left alone: {far}");
    }

    /// Walking the curve reaches the destination and reports it.
    #[test]
    fn the_target_walks_the_curve_to_the_end() {
        let mut c = curve();
        c.set_origin_and_destination(
            Vector3p::new(0.0, 0.0, 0.0),
            Vector3p::new(2000.0, 0.0, 0.0),
            Vector3f::new(200.0, 0.0, 0.0),
            Vector3f::new(200.0, 0.0, 0.0),
        );

        let mut vel = Vector3f::new(200.0, 0.0, 0.0);
        let mut steps = 0;
        while !c.reached_destination() && steps < 10_000 {
            let t = c.advance_target_along_track(0.01, vel);
            vel = t.velocity;
            assert!(
                t.position.x.is_finite() && vel.x.is_finite(),
                "step {steps} produced a non-finite state"
            );
            steps += 1;
        }
        assert!(c.reached_destination(), "never arrived after {steps} steps");
        assert!(steps > 10, "arrived implausibly fast: {steps} steps");
    }

    /// Speed never exceeds the configured horizontal limit along a flat
    /// curve.
    #[test]
    fn the_traversal_respects_the_speed_limit() {
        let mut c = curve();
        c.set_origin_and_destination(
            Vector3p::new(0.0, 0.0, 0.0),
            Vector3p::new(3000.0, 1000.0, 0.0),
            Vector3f::new(300.0, 0.0, 0.0),
            Vector3f::new(0.0, 300.0, 0.0),
        );

        let mut vel = Vector3f::new(300.0, 0.0, 0.0);
        let mut peak = 0.0_f32;
        let mut steps = 0;
        while !c.reached_destination() && steps < 10_000 {
            let t = c.advance_target_along_track(0.01, vel);
            vel = t.velocity;
            peak = peak.max(vel.length());
            steps += 1;
        }
        assert!(
            peak <= 500.0 + 1.0,
            "peak speed {peak} exceeded the 500 limit"
        );
    }

    /// The destination speed can be lowered by a later leg but never raised.
    #[test]
    fn the_destination_speed_can_only_be_lowered() {
        let mut c = curve();
        c.set_origin_and_destination(
            Vector3p::new(0.0, 0.0, 0.0),
            Vector3p::new(1000.0, 0.0, 0.0),
            Vector3f::new(100.0, 0.0, 0.0),
            Vector3f::new(100.0, 0.0, 0.0),
        );
        let before = c.destination_speed_max();
        assert!(before > 0.0);

        c.set_destination_speed_max(before * 0.5);
        assert!((c.destination_speed_max() - before * 0.5).abs() < 1e-3);

        c.set_destination_speed_max(before * 10.0);
        assert!(
            (c.destination_speed_max() - before * 0.5).abs() < 1e-3,
            "a larger value must not raise it"
        );
    }

    /// A destination velocity of zero means the vehicle must stop there, and
    /// the reported maximum says so.
    #[test]
    fn stopping_at_the_destination_gives_a_zero_speed_there() {
        let mut c = curve();
        c.set_origin_and_destination(
            Vector3p::new(0.0, 0.0, 0.0),
            Vector3p::new(1000.0, 0.0, 0.0),
            Vector3f::new(100.0, 0.0, 0.0),
            Vector3f::zero(),
        );
        assert_eq!(c.destination_speed_max(), 0.0);
    }

    /// A tight curve is traversed more slowly than a straight one, because
    /// the lateral acceleration limit binds. That is the whole point of
    /// deriving speed from curvature.
    #[test]
    fn a_tighter_curve_is_flown_more_slowly() {
        let peak_speed = |lateral: f64| -> f32 {
            let mut c = curve();
            c.set_origin_and_destination(
                Vector3p::new(0.0, 0.0, 0.0),
                Vector3p::new(1000.0, lateral, 0.0),
                Vector3f::new(400.0, 0.0, 0.0),
                Vector3f::new(-400.0, 0.0, 0.0),
            );
            let mut vel = Vector3f::new(400.0, 0.0, 0.0);
            let mut peak = 0.0_f32;
            let mut steps = 0;
            while !c.reached_destination() && steps < 5000 {
                let t = c.advance_target_along_track(0.01, vel);
                vel = t.velocity;
                peak = peak.max(vel.length());
                steps += 1;
            }
            peak
        };

        let gentle = peak_speed(2000.0);
        let tight = peak_speed(50.0);
        assert!(
            tight <= gentle,
            "a tighter turn should not be faster: {tight} against {gentle}"
        );
    }
}
