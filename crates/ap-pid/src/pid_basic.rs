//! Lightweight 1-axis PID, upstream `AC_PID_Basic`.
//!
//! The vertical velocity loop of the position controller. Lighter than
//! [`crate::AcPid`]: no target filter, no slew limiter, no DFF, no PD-max.
//! The integrator has independent positive and negative freeze flags so
//! a throttle-lower limit can stop wind-up in one direction without
//! freezing the other.
//!
//! Gains are plain fields, not `AP_Float`. The parameter system is not
//! this ticket; the arithmetic is unaffected.

use crate::PidInfo;
use ap_math::scalar::{calc_lowpass_alpha_dt, constrain_value, is_negative, is_positive, is_zero};

/// Lightweight 1-axis PID, upstream `AC_PID_Basic`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AcPidBasic {
    /// Proportional gain, upstream `_kp`.
    pub kp: f32,
    /// Integral gain, upstream `_ki`.
    pub ki: f32,
    /// Derivative gain, upstream `_kd`.
    pub kd: f32,
    /// Feed-forward gain, upstream `_kff`.
    pub kff: f32,
    /// Integrator magnitude limit, upstream `_kimax`.
    pub imax: f32,
    /// Error low-pass cutoff, Hz, upstream `_filt_E_hz`.
    pub filt_e_hz: f32,
    /// Derivative low-pass cutoff, Hz, upstream `_filt_D_hz`.
    pub filt_d_hz: f32,

    target: f32,
    error: f32,
    derivative: f32,
    integrator: f32,
    reset_filter: bool,
    info: PidInfo,
}

impl AcPidBasic {
    /// A controller with the given gains, filters reset.
    #[must_use]
    pub fn new(
        initial_p: f32,
        initial_i: f32,
        initial_d: f32,
        initial_ff: f32,
        initial_imax: f32,
        initial_filt_e_hz: f32,
        initial_filt_d_hz: f32,
    ) -> Self {
        Self {
            kp: initial_p,
            ki: initial_i,
            kd: initial_d,
            kff: initial_ff,
            imax: initial_imax,
            filt_e_hz: initial_filt_e_hz,
            filt_d_hz: initial_filt_d_hz,
            target: 0.0,
            error: 0.0,
            derivative: 0.0,
            integrator: 0.0,
            reset_filter: true,
            info: PidInfo::default(),
        }
    }

    /// What the last call did, upstream `get_pid_info`.
    #[must_use]
    pub fn info(&self) -> PidInfo {
        self.info
    }

    /// Last filtered error, upstream `get_error`.
    #[must_use]
    pub fn error(&self) -> f32 {
        self.error
    }

    /// Integrator, upstream `get_i`.
    #[must_use]
    pub fn integrator(&self) -> f32 {
        self.integrator
    }

    /// Feed-forward term from the last target, upstream `get_ff`.
    #[must_use]
    pub fn ff(&self) -> f32 {
        self.target * self.kff
    }

    /// Zero the integrator, upstream `reset_I`.
    pub fn reset_i(&mut self) {
        self.integrator = 0.0;
    }

    /// Reset the filters on the next call, upstream `reset_filter`.
    pub fn reset_filter(&mut self) {
        self.reset_filter = true;
    }

    /// Set the integrator directly, upstream `set_integrator`.
    pub fn set_integrator(&mut self, value: f32) {
        self.integrator = constrain_value(value, -self.imax, self.imax);
    }

    /// One control step, upstream `update_all` with a single saturation flag.
    ///
    /// Converted to the two-flag form the way upstream does: a saturated
    /// output freezes growth in the direction the integrator already has.
    pub fn update_all_limited(
        &mut self,
        target: f32,
        measurement: f32,
        dt: f32,
        limit: bool,
    ) -> f32 {
        self.update_all(
            target,
            measurement,
            dt,
            limit && is_negative(self.integrator),
            limit && is_positive(self.integrator),
        )
    }

    /// One control step, upstream `update_all(target, meas, dt, limit_neg, limit_pos)`.
    ///
    /// `limit_neg` freezes the integrator while the error is negative
    /// (it may only increase). `limit_pos` freezes it while the error is
    /// positive (it may only decrease). The vertical velocity loop passes
    /// the motor throttle-lower / throttle-upper flags here.
    ///
    /// Returns `P + I + D + FF`. Unlike [`crate::AcPid`], feed-forward is
    /// included in the return — upstream's `AC_PID_Basic` adds it inside
    /// `update_all`.
    ///
    /// A non-finite target or measurement returns zero and leaves all
    /// state untouched, as upstream does.
    pub fn update_all(
        &mut self,
        target: f32,
        measurement: f32,
        dt: f32,
        limit_neg: bool,
        limit_pos: bool,
    ) -> f32 {
        if !target.is_finite() || !measurement.is_finite() {
            return 0.0;
        }

        self.target = target;

        if self.reset_filter {
            self.reset_filter = false;
            self.error = self.target - measurement;
            self.derivative = 0.0;
        } else {
            let error_last = self.error;
            self.error += calc_lowpass_alpha_dt(dt, self.filt_e_hz)
                * ((self.target - measurement) - self.error);

            if is_positive(dt) {
                let derivative = (self.error - error_last) / dt;
                self.derivative +=
                    calc_lowpass_alpha_dt(dt, self.filt_d_hz) * (derivative - self.derivative);
            }
        }

        self.update_i(dt, limit_neg, limit_pos);

        let p_out = self.error * self.kp;
        let d_out = self.derivative * self.kd;
        let ff_out = self.target * self.kff;

        self.info.target = self.target;
        self.info.actual = measurement;
        self.info.error = self.error;
        self.info.p = p_out;
        self.info.i = self.integrator;
        self.info.d = d_out;
        self.info.ff = ff_out;

        p_out + self.integrator + d_out + ff_out
    }

    /// Integrate the error, upstream `update_i`.
    fn update_i(&mut self, dt: f32, limit_neg: bool, limit_pos: bool) {
        if !is_zero(self.ki) {
            let freeze =
                (limit_neg && is_negative(self.error)) || (limit_pos && is_positive(self.error));
            if !freeze {
                self.integrator += self.error * self.ki * dt;
                self.integrator = constrain_value(self.integrator, -self.imax, self.imax);
            }
        } else {
            self.integrator = 0.0;
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp, reason = "exact values on exact inputs")]

    use super::*;

    fn p_only(kp: f32) -> AcPidBasic {
        AcPidBasic::new(kp, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0)
    }

    #[test]
    fn an_unlimited_p_controller_is_error_times_kp() {
        let mut pid = p_only(2.0);
        let out = pid.update_all(5.0, 1.0, 0.02, false, false);
        assert_eq!(out, 8.0);
        assert_eq!(pid.error(), 4.0);
    }

    #[test]
    fn feed_forward_is_included_in_the_return() {
        let mut pid = AcPidBasic::new(0.0, 0.0, 0.0, 0.5, 10.0, 0.0, 0.0);
        let out = pid.update_all(4.0, 0.0, 0.02, false, false);
        assert_eq!(out, 2.0);
        assert_eq!(pid.ff(), 2.0);
    }

    #[test]
    fn the_integrator_winds_and_clamps() {
        let mut pid = AcPidBasic::new(0.0, 10.0, 0.0, 0.0, 0.5, 0.0, 0.0);
        let mut last = 0.0;
        for _ in 0..50 {
            last = pid.update_all(1.0, 0.0, 0.02, false, false);
        }
        assert_eq!(last, 0.5);
        assert_eq!(pid.integrator(), 0.5);
    }

    #[test]
    fn throttle_upper_freezes_positive_error_windup() {
        let mut pid = AcPidBasic::new(0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let first = pid.update_all(1.0, 0.0, 0.02, false, false);
        assert!(first > 0.0);
        let frozen = pid.integrator();
        let second = pid.update_all(1.0, 0.0, 0.02, false, true);
        assert_eq!(
            second, frozen,
            "limit_pos with a positive error must freeze I"
        );
        assert_eq!(pid.integrator(), frozen);
    }

    #[test]
    fn throttle_lower_freezes_negative_error_windup() {
        let mut pid = AcPidBasic::new(0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let first = pid.update_all(-1.0, 0.0, 0.02, false, false);
        assert!(first < 0.0);
        let frozen = pid.integrator();
        let second = pid.update_all(-1.0, 0.0, 0.02, true, false);
        assert_eq!(second, frozen);
        assert_eq!(pid.integrator(), frozen);
    }

    #[test]
    fn a_limit_still_allows_the_integrator_to_unwind() {
        let mut pid = AcPidBasic::new(0.0, 10.0, 0.0, 0.0, 10.0, 0.0, 0.0);
        let _ = pid.update_all(1.0, 0.0, 0.02, false, false);
        let wound = pid.integrator();
        assert!(wound > 0.0);
        // limit_pos, but the error is now negative: I may decrease.
        let _ = pid.update_all(-1.0, 0.0, 0.02, false, true);
        assert!(
            pid.integrator() < wound,
            "the integrator must be allowed to shrink toward the error"
        );
    }

    #[test]
    fn non_finite_inputs_return_zero_and_leave_state() {
        let mut pid = p_only(1.0);
        let _ = pid.update_all(3.0, 0.0, 0.02, false, false);
        let err = pid.error();
        assert_eq!(pid.update_all(f32::NAN, 0.0, 0.02, false, false), 0.0);
        assert_eq!(pid.error(), err);
        assert_eq!(pid.update_all(0.0, f32::INFINITY, 0.02, false, false), 0.0);
        assert_eq!(pid.error(), err);
    }

    #[test]
    fn set_integrator_clamps_to_imax() {
        let mut pid = p_only(1.0);
        pid.imax = 0.25;
        pid.set_integrator(1.0);
        assert_eq!(pid.integrator(), 0.25);
        pid.set_integrator(-1.0);
        assert_eq!(pid.integrator(), -0.25);
    }
}
