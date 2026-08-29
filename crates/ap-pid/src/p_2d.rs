//! 2-axis P controller, upstream `AC_P_2D`.
//!
//! The position loop of the NE controller: a position error becomes a
//! velocity demand. The error is clamped so the implied velocity cannot
//! exceed the configured limit, and the output itself goes through
//! `sqrt_controller_xy` so the closing rate tapers as the vehicle arrives.
//!
//! Gains are plain fields, not `AP_Float`. The parameter system is not
//! this ticket; the arithmetic is unaffected.

use ap_math::control::{inv_sqrt_controller, sqrt_controller_xy, Postype};
use ap_math::scalar::{is_positive, is_zero};
use ap_math::vector2::{Vector2, Vector2f};

/// 2-axis P controller, upstream `AC_P_2D`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcP2d {
    /// Proportional gain, upstream `_kp`.
    pub kp: f32,
    error: Vector2f,
    error_max: f32,
    d1_max: f32,
}

impl AcP2d {
    /// A controller with the given P gain and no limits yet.
    #[must_use]
    pub fn new(initial_p: f32) -> Self {
        Self {
            kp: initial_p,
            error: Vector2f::zero(),
            error_max: 0.0,
            d1_max: 0.0,
        }
    }

    /// Last position error, upstream `get_error`.
    #[must_use]
    pub fn error(&self) -> Vector2f {
        self.error
    }

    /// Current error clamp, upstream `get_error_max`.
    #[must_use]
    pub fn error_max(&self) -> f32 {
        self.error_max
    }

    /// First-derivative (acceleration) limit used by the sqrt controller.
    #[must_use]
    pub fn d1_max(&self) -> f32 {
        self.d1_max
    }

    /// Configure output, first-derivative, and second-derivative limits,
    /// upstream `set_limits`.
    ///
    /// For a position controller the output is a velocity, D1 is an
    /// acceleration, and D2 is a jerk. A non-positive argument is ignored
    /// rather than treated as "no limit from a negative number".
    ///
    /// The jerk bound, when present, also caps D1 at `D2 / kp`: the
    /// sqrt-controller's first derivative of output is `kp` times the input
    /// rate, so a jerk ceiling implies an acceleration ceiling.
    pub fn set_limits(&mut self, output_max: f32, d_out_max: f32, d2_out_max: f32) {
        self.d1_max = 0.0;
        self.error_max = 0.0;

        if is_positive(d_out_max) {
            self.d1_max = d_out_max;
        }

        if is_positive(d2_out_max) && is_positive(self.kp) {
            self.d1_max = self.d1_max.min(d2_out_max / self.kp);
        }

        if is_positive(output_max) && is_positive(self.kp) {
            self.error_max = inv_sqrt_controller(output_max, self.kp, self.d1_max);
        }
    }

    /// Tighten the error clamp after [`Self::set_limits`], upstream
    /// `set_error_max`.
    ///
    /// A non-positive argument is ignored. A zero existing clamp is replaced;
    /// a non-zero one is only reduced.
    pub fn set_error_max(&mut self, error_max: f32) {
        if is_positive(error_max) {
            if !is_zero(self.error_max) {
                self.error_max = self.error_max.min(error_max);
            } else {
                self.error_max = error_max;
            }
        }
    }

    /// Velocity demand from a position target and a measurement, upstream
    /// `update_all`.
    ///
    /// The target is borrowed mutably because an error that hits the clamp
    /// is written back: the caller asked for a point the controller will
    /// not chase, so the target is moved to the nearest point it will.
    /// `NE_update_controller` then rebuilds the desired position from that
    /// possibly-moved target minus the offset, which is how a clamp on the
    /// absolute target becomes a clamp on the trajectory.
    pub fn update_all(
        &mut self,
        target: &mut Vector2<Postype>,
        measurement: Vector2<Postype>,
    ) -> Vector2f {
        self.error = Vector2f::new(
            (target.x - measurement.x) as f32,
            (target.y - measurement.y) as f32,
        );

        if is_positive(self.error_max) && self.error.limit_length(self.error_max) {
            target.x = measurement.x + Postype::from(self.error.x);
            target.y = measurement.y + Postype::from(self.error.y);
        }

        sqrt_controller_xy(self.error, self.kp, self.d1_max, 0.0)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

    use super::*;

    #[test]
    fn an_unlimited_controller_is_error_times_kp() {
        let mut p = AcP2d::new(2.0);
        let mut target = Vector2::new(3.0, -1.0);
        let out = p.update_all(&mut target, Vector2::new(1.0, 1.0));
        assert_eq!(out, Vector2f::new(4.0, -4.0));
        assert_eq!(target, Vector2::new(3.0, -1.0), "no clamp, target stays");
    }

    #[test]
    fn a_clamped_error_rewrites_the_target() {
        let mut p = AcP2d::new(1.0);
        p.set_limits(2.0, 0.0, 0.0);
        assert!(p.error_max() > 0.0);

        let mut target = Vector2::new(100.0, 0.0);
        let measurement = Vector2::new(0.0, 0.0);
        let _ = p.update_all(&mut target, measurement);

        let err = p.error();
        assert!(
            err.length() <= p.error_max() + 1e-5,
            "error longer than clamp {}",
            p.error_max()
        );
        assert!(
            (target.x - Postype::from(err.x)).abs() < 1e-5 && target.y.abs() < 1e-5,
            "clamped target should be measurement plus the clamped error"
        );
    }

    #[test]
    fn a_jerk_limit_caps_the_acceleration_limit() {
        let mut p = AcP2d::new(2.0);
        p.set_limits(10.0, 8.0, 4.0);
        // D1 starts at 8, then min(8, 4/2) = 2.
        assert_eq!(p.d1_max(), 2.0);
    }

    #[test]
    fn set_error_max_only_tightens() {
        let mut p = AcP2d::new(1.0);
        p.set_error_max(5.0);
        assert_eq!(p.error_max(), 5.0);
        p.set_error_max(3.0);
        assert_eq!(p.error_max(), 3.0);
        p.set_error_max(9.0);
        assert_eq!(p.error_max(), 3.0, "a looser clamp is ignored");
        p.set_error_max(-1.0);
        assert_eq!(p.error_max(), 3.0, "non-positive is ignored");
    }

    #[test]
    fn zero_and_negative_limits_are_ignored() {
        let mut p = AcP2d::new(1.0);
        p.set_limits(0.0, -4.0, -1.0);
        assert_eq!(p.error_max(), 0.0);
        assert_eq!(p.d1_max(), 0.0);
    }
}
