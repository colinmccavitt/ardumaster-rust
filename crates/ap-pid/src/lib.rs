//! Port of `AC_PID/AC_PID.cpp` and `AC_PID.h`, pinned to `Plane-4.7.0`.
//!
//! The rate PID every fixed-wing attitude controller is built on
//! (`AP_FW_Controller` holds one), and the same class ArduCopter uses for its
//! rate loops. Tracked as FW-039.
//!
//! # Differences from upstream's shape, and why
//!
//! * **Gains are plain fields, not `AP_Float`.** The parameter system is
//!   FW-004 and is not ported yet. `ap-tecs` took the same approach and the log
//!   replay verified it, so the arithmetic is unaffected — only the binding to
//!   storage and MAVLink is deferred.
//! * **`update_all` takes `now_ms`.** Upstream's `SlewLimiter` reads
//!   `AP_HAL::millis()` internally; ADR-0004 rules out singletons, so the port
//!   passes time in, exactly as `ap-tecs` does.
//! * **Notch filters are optional.** Upstream attaches target and error notch
//!   filters through `AP_Filter` when `AP_FILTER_ENABLED`.
//!   [`AcPid::set_notch_sample_rate`] looks up the NTF/NEF index in a
//!   caller-supplied [`NotchFilterSource`] -- ADR-0004 rules out
//!   `AP::filters()`. Both notches stay absent unless a vehicle sets a
//!   non-zero index, which is every stock configuration and the path every
//!   existing test exercises.

#![no_std]

use ap_filter::notch::NotchFilter;
use ap_filter::slew::{SlewLimiter, SlewParams};
use ap_math::scalar::{calc_lowpass_alpha_dt, constrain_value, is_negative, is_positive, is_zero};

pub use ap_filter::ap_filter::{Filters, NotchFilterParams, NotchFilterSource};

/// The tunable gains, upstream's `AC_PID::Defaults` plus the members set
/// through `set_*`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidGains {
    /// Proportional gain, upstream `_kp`.
    pub p: f32,
    /// Integral gain, upstream `_ki`.
    pub i: f32,
    /// Derivative gain, upstream `_kd`.
    pub d: f32,
    /// Feed-forward gain on the filtered target, upstream `_kff`.
    pub ff: f32,
    /// Feed-forward gain on the target's derivative, upstream `_kdff`.
    pub dff: f32,
    /// Integrator magnitude limit, upstream `_kimax`.
    pub imax: f32,
    /// Limit on |P + D|. Non-positive disables it. Upstream `_kpdmax`.
    pub pdmax: f32,
    /// Target low-pass cutoff, Hz. Upstream `_filt_T_hz`.
    pub filt_t_hz: f32,
    /// Error low-pass cutoff, Hz. Upstream `_filt_E_hz`.
    pub filt_e_hz: f32,
    /// Derivative low-pass cutoff, Hz. Upstream `_filt_D_hz`.
    pub filt_d_hz: f32,
    /// Slew rate limit. Non-positive disables it. Upstream `_slew_rate_max`.
    pub srmax: f32,
    /// Slew limiter decay time constant. Upstream `_slew_rate_tau`.
    pub srtau: f32,
}

impl Default for PidGains {
    fn default() -> Self {
        Self {
            p: 0.0,
            i: 0.0,
            d: 0.0,
            ff: 0.0,
            dff: 0.0,
            imax: 0.0,
            pdmax: 0.0,
            filt_t_hz: 0.0,
            filt_e_hz: 0.0,
            filt_d_hz: 0.0,
            srmax: 0.0,
            srtau: 1.0,
        }
    }
}

/// What the controller did on the last call, upstream `AP_PIDInfo`.
///
/// Upstream logs this as the `PIDR`/`PIDP`/`PIDY` messages, which is what makes
/// a real-flight replay possible as a cross-check.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PidInfo {
    /// Filtered target, upstream `target`.
    pub target: f32,
    /// Measurement as supplied, upstream `actual`.
    pub actual: f32,
    /// Filtered error, upstream `error`.
    pub error: f32,
    /// Proportional contribution after every scaling, upstream `P`.
    pub p: f32,
    /// Integral contribution, upstream `I`.
    pub i: f32,
    /// Derivative contribution after every scaling, upstream `D`.
    pub d: f32,
    /// Feed-forward on the filtered target, upstream `FF`.
    pub ff: f32,
    /// Feed-forward on the target derivative, upstream `DFF`.
    pub dff: f32,
    /// Slew-limiter modifier applied to P and D, upstream `Dmod`.
    pub dmod: f32,
    /// Measured slew rate, upstream `slew_rate`.
    pub slew_rate: f32,
    /// Whether the caller reported the output as limited, upstream `limit`.
    pub limit: bool,
    /// Whether the P+D sum limit bound this call, upstream `PD_limit`.
    pub pd_limit: bool,
    /// Whether the filters were reset on this call, upstream `reset`.
    pub reset: bool,
    /// Whether the integrator was set externally, upstream `I_term_set`.
    pub i_term_set: bool,
}

/// Per-axis output scalings, upstream's `pd_scale` and `i_scale` arguments.
///
/// Always passed together and almost always 1.0. Grouping them keeps call
/// sites readable — upstream's bare `1.0f, 1.0f` gives no clue which is which.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Scaling {
    /// Multiplies the P and D contributions, upstream `pd_scale`.
    pub pd: f32,
    /// Multiplies the integrator's growth, upstream `i_scale`.
    pub i: f32,
}

impl Default for Scaling {
    /// Unscaled, which is what every caller that does not care should pass.
    fn default() -> Self {
        Self { pd: 1.0, i: 1.0 }
    }
}

/// Rate PID controller, upstream `AC_PID`.
#[derive(Debug, Clone, Copy)]
pub struct AcPid {
    /// Tunable gains. Public because upstream exposes each through an
    /// accessor and callers adjust them at runtime.
    pub gains: PidGains,

    target: f32,
    error: f32,
    derivative: f32,
    target_derivative: f32,
    integrator: f32,

    reset_filter: bool,
    i_set: bool,

    /// Upstream `_slew_limit_scale`, set to 1 in the constructor and changed
    /// only by `set_slew_limit_scale`.
    slew_limit_scale: i8,

    slew_limiter: SlewLimiter,
    info: PidInfo,

    /// Target-notch index, upstream `_notch_T_filter` (`ATC_RAT_*_NTF`).
    pub notch_t_filter: i8,
    /// Error-notch index, upstream `_notch_E_filter` (`ATC_RAT_*_NEF`).
    pub notch_e_filter: i8,
    target_notch: Option<NotchFilter<f32>>,
    error_notch: Option<NotchFilter<f32>>,
}

impl AcPid {
    /// A controller with the given gains, filters reset.
    #[must_use]
    pub fn new(gains: PidGains) -> Self {
        Self {
            gains,
            target: 0.0,
            error: 0.0,
            derivative: 0.0,
            target_derivative: 0.0,
            integrator: 0.0,
            // upstream constructs with `_flags._reset_filter = true` so the
            // first call seeds from its inputs rather than stepping from zero
            reset_filter: true,
            i_set: false,
            slew_limit_scale: 1,
            slew_limiter: SlewLimiter::new(),
            info: PidInfo::default(),
            notch_t_filter: 0,
            notch_e_filter: 0,
            target_notch: None,
            error_notch: None,
        }
    }

    /// The target notch, if `set_notch_sample_rate` allocated one.
    #[must_use]
    pub fn target_notch(&self) -> Option<&NotchFilter<f32>> {
        self.target_notch.as_ref()
    }

    /// The error notch, if `set_notch_sample_rate` allocated one.
    #[must_use]
    pub fn error_notch(&self) -> Option<&NotchFilter<f32>> {
        self.error_notch.as_ref()
    }

    /// Configure optional target/error notches, upstream `set_notch_sample_rate`.
    ///
    /// Both indices zero is a no-op. A non-zero index allocates, then looks up.
    /// A null lookup keeps an uninitialised notch; a failed setup drops it and clears the index.
    pub fn set_notch_sample_rate(&mut self, sample_rate: f32, filters: &impl NotchFilterSource) {
        if self.notch_t_filter == 0 && self.notch_e_filter == 0 {
            return;
        }
        if self.notch_t_filter != 0 {
            if self.target_notch.is_none() {
                self.target_notch = Some(NotchFilter::new());
            }
            if let Some(params) = filters.get_filter(self.notch_t_filter as u8) {
                let notch = self.target_notch.as_mut().expect("just allocated");
                if !params.setup_notch_filter(notch, sample_rate) {
                    self.target_notch = None;
                    self.notch_t_filter = 0;
                }
            }
        }
        if self.notch_e_filter != 0 {
            if self.error_notch.is_none() {
                self.error_notch = Some(NotchFilter::new());
            }
            if let Some(params) = filters.get_filter(self.notch_e_filter as u8) {
                let notch = self.error_notch.as_mut().expect("just allocated");
                if !params.setup_notch_filter(notch, sample_rate) {
                    self.error_notch = None;
                    self.notch_e_filter = 0;
                }
            }
        }
    }

    /// What the last call did, upstream `get_pid_info`.
    #[must_use]
    pub fn info(&self) -> PidInfo {
        self.info
    }

    /// The integrator, upstream `get_i`.
    #[must_use]
    pub fn integrator(&self) -> f32 {
        self.integrator
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
    ///
    /// Flags the change so the next `PidInfo` reports it, as upstream does.
    pub fn set_integrator(&mut self, value: f32) {
        self.i_set = true;
        self.integrator = constrain_value(value, -self.gains.imax, self.gains.imax);
    }

    /// Upstream `set_slew_limit_scale`.
    pub fn set_slew_limit_scale(&mut self, scale: i8) {
        self.slew_limit_scale = scale;
    }

    /// One control step, upstream `update_all`.
    ///
    /// `limit` tells the controller the output is saturated, which stops the
    /// integrator growing in the direction that would make it worse. `scaling`
    /// carries the per-axis scalings the attitude controllers apply.
    ///
    /// Returns `P + D + I`. Feed-forward is reported in [`PidInfo`] but not
    /// added here, matching upstream: the caller adds it.
    ///
    /// A non-finite target or measurement returns zero and leaves all state
    /// untouched, as upstream does.
    pub fn update_all(
        &mut self,
        target: f32,
        measurement: f32,
        dt: f32,
        limit: bool,
        scaling: Scaling,
        now_ms: u32,
    ) -> f32 {
        if !target.is_finite() || !measurement.is_finite() {
            return 0.0;
        }

        self.info.reset = self.reset_filter;
        if self.reset_filter {
            // First sample, or an explicit reset: seed the filters from the
            // inputs rather than stepping toward them from stale state.
            self.reset_filter = false;
            self.target = target;
            if let Some(n) = self.target_notch.as_mut() {
                n.reset();
                self.target = n.apply(self.target);
            }
            self.error = self.target - measurement;
            if let Some(n) = self.error_notch.as_mut() {
                n.reset();
                self.error = n.apply(self.error);
            }
            // clear the derivative history so the reset does not show up as a
            // spike on the next call
            self.derivative = 0.0;
            self.target_derivative = 0.0;
        } else {
            let target_last = self.target;
            let mut target = target;
            if let Some(n) = self.target_notch.as_mut() {
                target = n.apply(target);
            }
            self.target += calc_lowpass_alpha_dt(dt, self.gains.filt_t_hz) * (target - self.target);

            let error_last = self.error;
            let mut error = self.target - measurement;
            if let Some(n) = self.error_notch.as_mut() {
                error = n.apply(error);
            }
            self.error += calc_lowpass_alpha_dt(dt, self.gains.filt_e_hz) * (error - self.error);

            if is_positive(dt) {
                let derivative = (self.error - error_last) / dt;
                self.derivative += calc_lowpass_alpha_dt(dt, self.gains.filt_d_hz)
                    * (derivative - self.derivative);
                self.target_derivative = (self.target - target_last) / dt;
            }
        }

        self.update_i(dt, limit, scaling.i);

        let mut p_out = self.error * self.gains.p;
        let mut d_out = self.derivative * self.gains.d;
        let i_out = self.integrator;

        // The slew limiter is fed the PREVIOUS call's P and D: upstream reads
        // `_pid_info.P` and `_pid_info.D` here, and only assigns them further
        // down. Reproduced deliberately -- using this call's values would
        // change the modifier by one cycle.
        self.info.dmod = self.slew_limiter.modifier(
            (self.info.p + self.info.d) * f32::from(self.slew_limit_scale),
            dt,
            now_ms,
            SlewParams {
                slew_rate_max: self.gains.srmax,
                slew_rate_tau: self.gains.srtau,
            },
        );
        self.info.slew_rate = self.slew_limiter.get_slew_rate();

        p_out *= self.info.dmod;
        d_out *= self.info.dmod;

        p_out *= scaling.pd;
        d_out *= scaling.pd;

        self.info.pd_limit = false;
        if is_positive(self.gains.pdmax) {
            let pd_sum_abs = (p_out + d_out).abs();
            if pd_sum_abs > self.gains.pdmax {
                let scale = self.gains.pdmax / pd_sum_abs;
                p_out *= scale;
                d_out *= scale;
                self.info.pd_limit = true;
            }
        }

        self.info.target = self.target;
        self.info.actual = measurement;
        self.info.error = self.error;
        self.info.p = p_out;
        self.info.d = d_out;
        self.info.i = i_out;
        self.info.limit = limit;
        self.info.i_term_set = self.i_set;
        self.i_set = false;
        self.info.ff = self.target * self.gains.ff;
        self.info.dff = self.target_derivative * self.gains.dff;

        p_out + d_out + i_out
    }

    /// One control step from a pre-computed error, upstream `update_error`.
    ///
    /// Routes through [`Self::update_all`] with a zero target and the error
    /// negated as the measurement, so `target - measurement` recovers the
    /// error. That bypasses the target filter, which upstream keeps for
    /// backward compatibility, and it then forces the logged target and actual
    /// back to zero.
    pub fn update_error(&mut self, error: f32, dt: f32, limit: bool, now_ms: u32) -> f32 {
        if !error.is_finite() {
            return 0.0;
        }
        self.target = 0.0;
        let out = self.update_all(0.0, -error, dt, limit, Scaling::default(), now_ms);
        self.info.target = 0.0;
        self.info.actual = 0.0;
        out
    }

    /// Integrate the error, upstream `update_i`.
    fn update_i(&mut self, dt: f32, limit: bool, i_scale: f32) {
        if !is_zero(self.gains.i) && is_positive(dt) {
            // While the output is limited the integrator may shrink but not
            // grow: it may only move when the error opposes it.
            let opposes = (is_positive(self.integrator) && is_negative(self.error))
                || (is_negative(self.integrator) && is_positive(self.error));
            if !limit || opposes {
                self.integrator += self.error * self.gains.i * i_scale * dt;
                self.integrator =
                    constrain_value(self.integrator, -self.gains.imax, self.gains.imax);
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

    fn gains() -> PidGains {
        PidGains {
            p: 2.0,
            i: 0.5,
            d: 0.1,
            ff: 0.25,
            imax: 10.0,
            filt_t_hz: 20.0,
            filt_e_hz: 20.0,
            filt_d_hz: 10.0,
            ..PidGains::default()
        }
    }

    /// The first call must seed from its inputs rather than stepping toward
    /// them, or every controller would start with a transient.
    #[test]
    fn first_call_seeds_the_filters() {
        let mut pid = AcPid::new(gains());
        pid.update_all(10.0, 4.0, 0.02, false, Scaling::default(), 0);
        let info = pid.info();
        assert!(info.reset, "the first call must report a reset");
        assert_eq!(info.target, 10.0, "target is seeded, not filtered toward");
        assert_eq!(info.error, 6.0, "error is seeded from target - measurement");
        assert_eq!(info.d, 0.0, "no derivative history exists yet");
    }

    /// While limited, the integrator may shrink but not grow. This is the
    /// wind-up protection every saturating controller depends on.
    #[test]
    fn a_limited_output_stops_the_integrator_growing() {
        let mut pid = AcPid::new(gains());
        pid.update_all(10.0, 0.0, 0.02, false, Scaling::default(), 0);
        for step in 1..50 {
            pid.update_all(10.0, 0.0, 0.02, false, Scaling::default(), step * 20);
        }
        let grown = pid.integrator();
        assert!(grown > 0.0, "the integrator should have wound up: {grown}");

        // now report the output as limited, with the error still positive
        let before = pid.integrator();
        pid.update_all(10.0, 0.0, 0.02, true, Scaling::default(), 1000);
        assert_eq!(
            pid.integrator(),
            before,
            "a positive error must not grow a positive integrator while limited"
        );

        // reverse the error and it may move again, back toward zero
        pid.update_all(-10.0, 0.0, 0.02, true, Scaling::default(), 1020);
        assert!(
            pid.integrator() < before,
            "an opposing error must be allowed to unwind the integrator"
        );
    }

    #[test]
    fn the_integrator_is_clamped_to_imax() {
        let mut pid = AcPid::new(gains());
        for step in 0..2000 {
            pid.update_all(100.0, 0.0, 0.02, false, Scaling::default(), step * 20);
        }
        assert!(
            pid.integrator() <= gains().imax,
            "integrator {} exceeded imax",
            pid.integrator()
        );
        assert!(pid.integrator() > 0.0);
    }

    /// A zero integral gain must not merely stop integrating -- upstream
    /// clears the integrator outright.
    #[test]
    fn zero_i_gain_clears_the_integrator() {
        let mut g = gains();
        let mut pid = AcPid::new(g);
        for step in 0..20 {
            pid.update_all(10.0, 0.0, 0.02, false, Scaling::default(), step * 20);
        }
        assert!(pid.integrator() > 0.0);

        g.i = 0.0;
        pid.gains = g;
        pid.update_all(10.0, 0.0, 0.02, false, Scaling::default(), 1000);
        assert_eq!(pid.integrator(), 0.0, "a zero I gain clears the integrator");
    }

    #[test]
    fn non_finite_inputs_are_rejected_without_touching_state() {
        let mut pid = AcPid::new(gains());
        pid.update_all(10.0, 0.0, 0.02, false, Scaling::default(), 0);
        let before = pid.info();
        let integ = pid.integrator();

        assert_eq!(
            pid.update_all(f32::NAN, 0.0, 0.02, false, Scaling::default(), 20),
            0.0
        );
        assert_eq!(
            pid.update_all(0.0, f32::INFINITY, 0.02, false, Scaling::default(), 20),
            0.0
        );
        assert_eq!(pid.info(), before, "state must be untouched");
        assert_eq!(pid.integrator(), integ);
    }

    /// The P+D limit scales both terms together, preserving their ratio.
    #[test]
    fn pd_limit_scales_p_and_d_together() {
        let mut g = gains();
        g.pdmax = 1.0;
        let mut pid = AcPid::new(g);
        pid.update_all(100.0, 0.0, 0.02, false, Scaling::default(), 0);
        pid.update_all(100.0, 0.0, 0.02, false, Scaling::default(), 20);
        let info = pid.info();
        assert!(info.pd_limit, "the limit should have bound");
        assert!(
            (info.p + info.d).abs() <= g.pdmax + 1e-6,
            "P+D = {} exceeds pdmax",
            info.p + info.d
        );
    }

    /// `update_error` must report zero target and actual, whatever it did
    /// internally -- callers log those fields.
    #[test]
    fn update_error_reports_zero_target_and_actual() {
        let mut pid = AcPid::new(gains());
        pid.update_error(5.0, 0.02, false, 0);
        pid.update_error(5.0, 0.02, false, 20);
        let info = pid.info();
        assert_eq!(info.target, 0.0);
        assert_eq!(info.actual, 0.0);
        assert_eq!(info.error, 5.0, "the error itself is still reported");
    }

    /// Feed-forward is reported but not included in the return value; the
    /// caller adds it. Getting this wrong would double-count it.
    #[test]
    fn feed_forward_is_reported_but_not_returned() {
        let mut pid = AcPid::new(gains());
        pid.update_all(10.0, 10.0, 0.02, false, Scaling::default(), 0);
        let out = pid.update_all(10.0, 10.0, 0.02, false, Scaling::default(), 20);
        let info = pid.info();
        assert!(info.ff != 0.0, "feed-forward should be reported");
        assert_eq!(
            out,
            info.p + info.i + info.d,
            "the return value is P + I + D only"
        );
    }
}
