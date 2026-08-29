//! 1-axis P controller, upstream `AC_P_1D`.
//!
//! The position loop of the vertical controller: a position error becomes a
//! velocity demand. The error is clamped independently in each direction so
//! the climb and descent limits can differ, and the output goes through
//! `sqrt_controller` so the closing rate tapers as the vehicle arrives.
//!
//! Gains are plain fields, not `AP_Float`. The parameter system is not
//! this ticket; the arithmetic is unaffected.

use ap_math::control::{inv_sqrt_controller, sqrt_controller, Postype};
use ap_math::scalar::{is_negative, is_positive, is_zero};

/// 1-axis P controller, upstream `AC_P_1D`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcP1d {
    /// Proportional gain, upstream `_kp`.
    pub kp: f32,
    error: f32,
    error_min: f32,
    error_max: f32,
    d1_max: f32,
}

impl AcP1d {
    /// A controller with the given P gain and no limits yet.
    #[must_use]
    pub fn new(initial_p: f32) -> Self {
        Self {
            kp: initial_p,
            error: 0.0,
            error_min: 0.0,
            error_max: 0.0,
            d1_max: 0.0,
        }
    }

    /// Last position error, upstream `get_error`.
    #[must_use]
    pub fn error(&self) -> f32 {
        self.error
    }

    /// Current negative-direction error clamp, upstream `get_error_min`.
    #[must_use]
    pub fn error_min(&self) -> f32 {
        self.error_min
    }

    /// Current positive-direction error clamp, upstream `get_error_max`.
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
    /// acceleration, and D2 is a jerk. A non-positive D argument is ignored
    /// rather than treated as "no limit from a negative number". Output
    /// limits are signed: `output_min` must be negative, `output_max`
    /// positive, or they are ignored.
    ///
    /// The jerk bound, when present, also caps D1 at `D2 / kp`: the
    /// sqrt-controller's first derivative of output is `kp` times the input
    /// rate, so a jerk ceiling implies an acceleration ceiling.
    pub fn set_limits(
        &mut self,
        output_min: f32,
        output_max: f32,
        d_out_max: f32,
        d2_out_max: f32,
    ) {
        self.d1_max = 0.0;
        self.error_min = 0.0;
        self.error_max = 0.0;

        if is_positive(d_out_max) {
            self.d1_max = d_out_max;
        }

        if is_positive(d2_out_max) && is_positive(self.kp) {
            self.d1_max = self.d1_max.min(d2_out_max / self.kp);
        }

        if is_negative(output_min) && is_positive(self.kp) {
            self.error_min = inv_sqrt_controller(output_min, self.kp, self.d1_max);
        }
        if is_positive(output_max) && is_positive(self.kp) {
            self.error_max = inv_sqrt_controller(output_max, self.kp, self.d1_max);
        }
    }

    /// Tighten the error clamps after [`Self::set_limits`], upstream
    /// `set_error_limits`.
    ///
    /// A non-negative `error_min` or non-positive `error_max` is ignored.
    /// A zero existing clamp is replaced; a non-zero one is only tightened.
    pub fn set_error_limits(&mut self, error_min: f32, error_max: f32) {
        if is_negative(error_min) {
            if !is_zero(self.error_min) {
                self.error_min = self.error_min.max(error_min);
            } else {
                self.error_min = error_min;
            }
        }
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
    /// The target is borrowed mutably because an error that hits a clamp
    /// is written back: the caller asked for a point the controller will
    /// not chase, so the target is moved to the nearest point it will.
    /// `D_update_controller` then rebuilds the desired position from that
    /// possibly-moved target minus the offset and terrain, which is how a
    /// clamp on the absolute target becomes a clamp on the trajectory.
    pub fn update_all(&mut self, target: &mut Postype, measurement: Postype) -> f32 {
        self.error = (*target - measurement) as f32;

        if is_negative(self.error_min) && self.error < self.error_min {
            self.error = self.error_min;
            *target = measurement + Postype::from(self.error);
        } else if is_positive(self.error_max) && self.error > self.error_max {
            self.error = self.error_max;
            *target = measurement + Postype::from(self.error);
        }

        sqrt_controller(self.error, self.kp, self.d1_max, 0.0)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

    use super::*;

    #[test]
    fn an_unlimited_controller_is_error_times_kp() {
        let mut p = AcP1d::new(2.0);
        let mut target = 3.0;
        let out = p.update_all(&mut target, 1.0);
        assert_eq!(out, 4.0);
        assert_eq!(target, 3.0, "no clamp, target stays");
    }

    #[test]
    fn a_clamped_error_rewrites_the_target() {
        let mut p = AcP1d::new(1.0);
        p.set_limits(-2.0, 2.0, 0.0, 0.0);
        assert!(p.error_max() > 0.0);
        assert!(p.error_min() < 0.0);

        let mut target = 100.0;
        let _ = p.update_all(&mut target, 0.0);
        assert!(p.error() <= p.error_max() + 1e-5);
        assert!((target - Postype::from(p.error())).abs() < 1e-5);

        let mut target = -100.0;
        let _ = p.update_all(&mut target, 0.0);
        assert!(p.error() >= p.error_min() - 1e-5);
        assert!((target - Postype::from(p.error())).abs() < 1e-5);
    }

    #[test]
    fn a_jerk_limit_caps_the_acceleration_limit() {
        let mut p = AcP1d::new(2.0);
        p.set_limits(-10.0, 10.0, 8.0, 4.0);
        // D1 starts at 8, then min(8, 4/2) = 2.
        assert_eq!(p.d1_max(), 2.0);
    }

    #[test]
    fn set_error_limits_only_tightens() {
        let mut p = AcP1d::new(1.0);
        p.set_error_limits(-5.0, 5.0);
        assert_eq!(p.error_min(), -5.0);
        assert_eq!(p.error_max(), 5.0);
        p.set_error_limits(-3.0, 3.0);
        assert_eq!(p.error_min(), -3.0);
        assert_eq!(p.error_max(), 3.0);
        p.set_error_limits(-9.0, 9.0);
        assert_eq!(p.error_min(), -3.0, "a looser clamp is ignored");
        assert_eq!(p.error_max(), 3.0);
        p.set_error_limits(1.0, -1.0);
        assert_eq!(p.error_min(), -3.0, "wrong-sign arguments are ignored");
        assert_eq!(p.error_max(), 3.0);
    }

    #[test]
    fn zero_and_wrong_sign_limits_are_ignored() {
        let mut p = AcP1d::new(1.0);
        p.set_limits(2.0, -4.0, -1.0, -1.0);
        assert_eq!(p.error_min(), 0.0);
        assert_eq!(p.error_max(), 0.0);
        assert_eq!(p.d1_max(), 0.0);
    }
}
