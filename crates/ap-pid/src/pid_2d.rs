//! 2-axis PID controller, upstream `AC_PID_2D`.
//!
//! The velocity loop of the NE controller: a velocity error becomes an
//! acceleration demand. The error and its derivative are low-passed, and
//! the integrator is allowed to grow only when it is not pushing further
//! into a limit vector — that is the anti-windup for a 2-D output that
//! has already been clipped by the lean-angle budget.
//!
//! Gains are plain fields, not `AP_Float`. The parameter system is not
//! this ticket; the arithmetic is unaffected.

use ap_math::scalar::{calc_lowpass_alpha_dt, is_positive};
use ap_math::vector2::Vector2f;

/// Default gains for the NE velocity PID on Plane, upstream
/// `POSCONTROL_NE_VEL_*` under `APM_BUILD_ArduPlane`.
pub const NE_VEL_P: f32 = 1.0;
/// Integral gain default.
pub const NE_VEL_I: f32 = 0.5;
/// Derivative gain default. Zero on Plane: the velocity measurement is
/// already filtered by the EKF and a D term on top of that is mostly noise.
pub const NE_VEL_D: f32 = 0.0;
/// Integrator length limit, metres per second squared.
pub const NE_VEL_IMAX: f32 = 10.0;
/// Error-filter cutoff, hertz.
pub const NE_VEL_FILT_HZ: f32 = 5.0;
/// Derivative-filter cutoff, hertz.
pub const NE_VEL_FILT_D_HZ: f32 = 5.0;

/// 2-axis PID controller, upstream `AC_PID_2D`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcPid2d {
    /// Proportional gain, upstream `_kp`.
    pub kp: f32,
    /// Integral gain, upstream `_ki`.
    pub ki: f32,
    /// Derivative gain, upstream `_kd`.
    pub kd: f32,
    /// Feed-forward gain, upstream `_kff`.
    pub kff: f32,
    /// Integrator length limit, upstream `_kimax`.
    pub imax: f32,
    /// Error-filter cutoff, hertz, upstream `_filt_E_hz`.
    pub filt_e_hz: f32,
    /// Derivative-filter cutoff, hertz, upstream `_filt_D_hz`.
    pub filt_d_hz: f32,
    target: Vector2f,
    error: Vector2f,
    derivative: Vector2f,
    integrator: Vector2f,
    reset_filter: bool,
}

impl AcPid2d {
    /// A controller with the given gains. The filter is flagged for reset
    /// so the first `update_all` seeds rather than blends.
    #[must_use]
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        kff: f32,
        imax: f32,
        filt_e_hz: f32,
        filt_d_hz: f32,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            kff,
            imax: imax.abs(),
            filt_e_hz,
            filt_d_hz,
            target: Vector2f::zero(),
            error: Vector2f::zero(),
            derivative: Vector2f::zero(),
            integrator: Vector2f::zero(),
            reset_filter: true,
        }
    }

    /// Plane NE-velocity defaults, the constructor `AC_PosControl` uses.
    #[must_use]
    pub fn ne_velocity() -> Self {
        Self::new(
            NE_VEL_P,
            NE_VEL_I,
            NE_VEL_D,
            0.0,
            NE_VEL_IMAX,
            NE_VEL_FILT_HZ,
            NE_VEL_FILT_D_HZ,
        )
    }

    /// Last filtered error, upstream `get_error`.
    #[must_use]
    pub fn error(&self) -> Vector2f {
        self.error
    }

    /// Integrator, upstream `get_i`.
    #[must_use]
    pub fn integrator(&self) -> Vector2f {
        self.integrator
    }

    /// Feed-forward gain, upstream `ff()`.
    #[must_use]
    pub fn ff(&self) -> f32 {
        self.kff
    }

    /// Flag the filters for reset on the next `update_all`, upstream
    /// `reset_filter`.
    pub fn reset_filter(&mut self) {
        self.reset_filter = true;
    }

    /// Zero the integrator, upstream `reset_I`.
    pub fn reset_i(&mut self) {
        self.integrator = Vector2f::zero();
    }

    /// Set the integrator and clamp it to IMAX, upstream `set_integrator`.
    pub fn set_integrator(&mut self, i: Vector2f) {
        self.integrator = i;
        self.integrator.limit_length(self.imax);
    }

    /// One control step, upstream `update_all`.
    ///
    /// Non-finite inputs return zero without touching state — a NaN in
    /// the EKF would otherwise become a NaN lean command.
    ///
    /// The `limit` vector is the previous cycle's unsaturated acceleration.
    /// The integrator is allowed to grow only when that growth is not
    /// further into the same direction: if the output was already clipped
    /// by the lean-angle budget, winding up against the clip would just
    /// delay the recovery when the budget returns.
    pub fn update_all(
        &mut self,
        target: Vector2f,
        measurement: Vector2f,
        dt: f32,
        limit: Vector2f,
    ) -> Vector2f {
        if target.is_nan() || target.is_inf() || measurement.is_nan() || measurement.is_inf() {
            return Vector2f::zero();
        }

        self.target = target;

        if self.reset_filter {
            self.reset_filter = false;
            self.error = self.target - measurement;
            self.derivative = Vector2f::zero();
        } else {
            let error_last = self.error;
            self.error += ((self.target - measurement) - self.error) * self.filt_e_alpha(dt);
            if is_positive(dt) {
                let derivative = (self.error - error_last) / dt;
                self.derivative += (derivative - self.derivative) * self.filt_d_alpha(dt);
            }
        }

        self.update_i(dt, limit);

        self.error * self.kp + self.integrator + self.derivative * self.kd + self.target * self.kff
    }

    /// Integrator step, upstream `update_i`.
    ///
    /// Anti-windup is a length freeze, not a hold of the vector: if the
    /// increment is into the limit, the integrator is scaled back to the
    /// length it had before the increment. A turn that is already at the
    /// lean budget must not grow the I term along that heading, but it
    /// may still rotate — the length is what was spent.
    fn update_i(&mut self, dt: f32, limit: Vector2f) {
        let delta_integrator = (self.error * self.ki) * dt;
        let integrator_length = self.integrator.length();
        self.integrator += delta_integrator;
        if is_positive(delta_integrator.dot(limit)) {
            let _ = self.integrator.limit_length(integrator_length);
        }
        self.integrator.limit_length(self.imax);
    }

    fn filt_e_alpha(&self, dt: f32) -> f32 {
        calc_lowpass_alpha_dt(dt, self.filt_e_hz)
    }

    fn filt_d_alpha(&self, dt: f32) -> f32 {
        calc_lowpass_alpha_dt(dt, self.filt_d_hz)
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

    use super::*;

    fn pid() -> AcPid2d {
        AcPid2d::new(2.0, 1.0, 0.0, 0.0, 5.0, 0.0, 0.0)
    }

    #[test]
    fn the_first_call_seeds_the_error_and_zeroes_d() {
        let mut p = AcPid2d::new(1.0, 0.0, 1.0, 0.0, 10.0, 5.0, 5.0);
        let out = p.update_all(
            Vector2f::new(4.0, -2.0),
            Vector2f::new(1.0, 1.0),
            0.02,
            Vector2f::zero(),
        );
        assert_eq!(p.error(), Vector2f::new(3.0, -3.0));
        assert_eq!(
            out,
            Vector2f::new(3.0, -3.0),
            "P only: D is zeroed on reset"
        );
    }

    #[test]
    fn non_finite_inputs_return_zero_without_touching_state() {
        let mut p = pid();
        let _ = p.update_all(
            Vector2f::new(1.0, 0.0),
            Vector2f::zero(),
            0.02,
            Vector2f::zero(),
        );
        let error = p.error();
        let out = p.update_all(
            Vector2f::new(f32::NAN, 0.0),
            Vector2f::zero(),
            0.02,
            Vector2f::zero(),
        );
        assert_eq!(out, Vector2f::zero());
        assert_eq!(p.error(), error);
    }

    #[test]
    fn a_limit_aligned_with_the_error_freezes_integrator_length() {
        let mut p = pid();
        let _ = p.update_all(
            Vector2f::new(2.0, 0.0),
            Vector2f::zero(),
            0.02,
            Vector2f::zero(),
        );
        let grown = p.integrator().length();
        assert!(grown > 0.0, "the integrator should have moved");

        // Same error, now with a limit in the same direction: the increment
        // is into the limit, so the length is frozen at whatever it was.
        let _ = p.update_all(
            Vector2f::new(2.0, 0.0),
            Vector2f::zero(),
            0.02,
            Vector2f::new(1.0, 0.0),
        );
        assert_eq!(
            p.integrator().length(),
            grown,
            "anti-windup must freeze length when growing into the limit"
        );
    }

    #[test]
    fn a_limit_against_the_error_does_not_freeze() {
        let mut p = pid();
        let _ = p.update_all(
            Vector2f::new(2.0, 0.0),
            Vector2f::zero(),
            0.02,
            Vector2f::zero(),
        );
        let grown = p.integrator().length();
        let _ = p.update_all(
            Vector2f::new(2.0, 0.0),
            Vector2f::zero(),
            0.02,
            Vector2f::new(-1.0, 0.0),
        );
        assert!(
            p.integrator().length() > grown,
            "unwinding against the limit must still be allowed"
        );
    }

    #[test]
    fn the_integrator_is_clamped_to_imax() {
        let mut p = AcPid2d::new(0.0, 100.0, 0.0, 0.0, 2.0, 0.0, 0.0);
        for _ in 0..50 {
            let _ = p.update_all(
                Vector2f::new(10.0, 0.0),
                Vector2f::zero(),
                0.02,
                Vector2f::zero(),
            );
        }
        assert!(
            p.integrator().length() <= 2.0 + 1e-5,
            "integrator {} past IMAX",
            p.integrator().length()
        );
    }

    #[test]
    fn feed_forward_is_target_times_kff() {
        let mut p = AcPid2d::new(0.0, 0.0, 0.0, 0.5, 10.0, 0.0, 0.0);
        let out = p.update_all(
            Vector2f::new(4.0, -2.0),
            Vector2f::new(4.0, -2.0),
            0.02,
            Vector2f::zero(),
        );
        assert_eq!(out, Vector2f::new(2.0, -1.0));
    }

    #[test]
    fn set_integrator_is_clamped() {
        let mut p = AcPid2d::new(0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0);
        p.set_integrator(Vector2f::new(10.0, 0.0));
        assert!((p.integrator().length() - 3.0).abs() < 1e-5);
    }
}
