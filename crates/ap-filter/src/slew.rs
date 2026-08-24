//! Port of `Filter/SlewLimiter.{h,cpp}`, pinned to `Plane-4.7.0`.
//!
//! Detects controller oscillation by tracking the peak positive and negative
//! slew rates of a signal, and returns a multiplier in `(0, 1]` used to reduce
//! PID gains when the output is slewing faster than configured.
//!
//! # DIVERGENCE D-006 — uninitialised state
//!
//! Upstream's constructor initialises only the two parameter references and the
//! internal low-pass filter. **Thirteen** further members are left
//! indeterminate. See DIVERGENCES.md. This port zeroes all of them.
//!
//! # Shape changes (ADR-0004, not behavior changes)
//!
//! - **Time is a parameter.** Upstream calls `AP_HAL::millis()` internally.
//!   That is the global-singleton pattern ADR-0004 forbids, so `now_ms` is
//!   passed in. It also makes the oscillation logic unit-testable without
//!   mocking a clock.
//! - **Parameters are passed per call.** Upstream stores `const float&`
//!   references to live `AP_Float` parameters, so a parameter change takes
//!   effect on the next call. Passing [`SlewParams`] each call reproduces that
//!   exactly, without storing references.
//!
//! Millisecond arithmetic uses `wrapping_sub`, matching C++ unsigned overflow
//! semantics. The upstream counter wraps roughly every 49 days and the
//! comparisons rely on that.

use ap_math::scalar::is_positive;

use crate::lowpass::LowPassFilterFloat;

/// Number of consecutive slew-rate exceedance events recorded per direction.
/// Upstream `SLEWLIMITER_N_EVENTS`; 2 corresponds to a complete cycle.
pub const N_EVENTS: usize = 2;

/// Time in ms for a half cycle of the slowest oscillation expected.
/// Upstream `WINDOW_MS`.
const WINDOW_MS: u32 = 300;

/// Ratio of modifier reduction to slew rate exceedance ratio.
/// Upstream `MODIFIER_GAIN`.
const MODIFIER_GAIN: f32 = 1.5;

/// Cutoff for the internal slew-rate derivative filter, in Hz.
/// Upstream `DERIVATIVE_CUTOFF_FREQ`.
const DERIVATIVE_CUTOFF_FREQ: f32 = 25.0;

/// The tuning parameters, read fresh on every call.
///
/// Upstream holds `const float&` references to live parameters; passing this
/// per call reproduces that without storing references.
#[derive(Debug, Clone, Copy)]
pub struct SlewParams {
    /// Maximum permitted slew rate. Non-positive disables limiting.
    pub slew_rate_max: f32,
    /// Decay time constant, in seconds.
    pub slew_rate_tau: f32,
}

/// Slew-rate limiting filter. Upstream `SlewLimiter`.
#[derive(Debug, Clone, Copy)]
pub struct SlewLimiter {
    slew_filter: LowPassFilterFloat,
    output_slew_rate: f32,
    modifier_slew_rate: f32,
    last_sample: f32,
    max_pos_slew_rate: f32,
    max_neg_slew_rate: f32,
    max_pos_slew_event_ms: u32,
    max_neg_slew_event_ms: u32,
    pos_event_index: usize,
    neg_event_index: usize,
    pos_event_ms: [u32; N_EVENTS],
    neg_event_ms: [u32; N_EVENTS],
    pos_event_stored: bool,
    neg_event_stored: bool,
}

impl Default for SlewLimiter {
    /// DIVERGENCE D-006: every field is initialised. Upstream leaves thirteen
    /// of them indeterminate.
    #[inline]
    fn default() -> Self {
        let mut slew_filter = LowPassFilterFloat::new(DERIVATIVE_CUTOFF_FREQ);
        slew_filter.reset_to(0.0);
        Self {
            slew_filter,
            output_slew_rate: 0.0,
            modifier_slew_rate: 0.0,
            last_sample: 0.0,
            max_pos_slew_rate: 0.0,
            max_neg_slew_rate: 0.0,
            max_pos_slew_event_ms: 0,
            max_neg_slew_event_ms: 0,
            pos_event_index: 0,
            neg_event_index: 0,
            pos_event_ms: [0; N_EVENTS],
            neg_event_ms: [0; N_EVENTS],
            pos_event_stored: false,
            neg_event_stored: false,
        }
    }
}

impl SlewLimiter {
    /// A limiter in its rest state.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// The last computed oscillation slew rate. Upstream `get_slew_rate()`.
    #[inline]
    pub fn get_slew_rate(&self) -> f32 {
        self.output_slew_rate
    }

    /// Gain multiplier in `(0, 1]` keeping the output within the slew rate.
    ///
    /// Upstream `modifier(sample, dt)`, with `now_ms` and `params` passed in
    /// rather than read from globals.
    pub fn modifier(&mut self, sample: f32, dt: f32, now_ms: u32, params: SlewParams) -> f32 {
        if !is_positive(dt) {
            return 1.0;
        }
        let SlewParams {
            slew_rate_max,
            slew_rate_tau,
        } = params;

        // low pass filtered slew rate
        let slew_rate = self.slew_filter.apply((sample - self.last_sample) / dt, dt);
        self.last_sample = sample;

        // decay the peak slew rates once they leave the window period
        let decay_alpha = if slew_rate_tau > 0.0 {
            dt.min(slew_rate_tau) / slew_rate_tau
        } else {
            0.0
        };
        // increases are attacked twice as fast, to blunt gusts and setpoint steps
        let attack_alpha = (2.0 * decay_alpha).min(1.0);

        if slew_rate > self.max_pos_slew_rate {
            self.max_pos_slew_rate = slew_rate;
            self.max_pos_slew_event_ms = now_ms;
        } else if now_ms.wrapping_sub(self.max_pos_slew_event_ms) > WINDOW_MS {
            self.max_pos_slew_rate *= 1.0 - decay_alpha;
        }

        if -slew_rate > self.max_neg_slew_rate {
            self.max_neg_slew_rate = -slew_rate;
            self.max_neg_slew_event_ms = now_ms;
        } else if now_ms.wrapping_sub(self.max_neg_slew_event_ms) > WINDOW_MS {
            self.max_neg_slew_rate *= 1.0 - decay_alpha;
        }

        let raw_slew_rate = 0.5 * (self.max_pos_slew_rate + self.max_neg_slew_rate);
        self.output_slew_rate =
            (1.0 - attack_alpha) * self.output_slew_rate + attack_alpha * raw_slew_rate;
        self.output_slew_rate = self.output_slew_rate.min(raw_slew_rate);

        if slew_rate_max <= 0.0 {
            return 1.0;
        }

        // constrain the slew rate used for the calculation
        let limited_raw_slew_rate = 0.5
            * (self.max_pos_slew_rate.min(10.0 * slew_rate_max)
                + self.max_neg_slew_rate.min(10.0 * slew_rate_max));

        // record a series of positive exceedance events
        if !self.pos_event_stored && slew_rate > slew_rate_max {
            if self.pos_event_index >= N_EVENTS {
                self.pos_event_index = 0;
            }
            if let Some(slot) = self.pos_event_ms.get_mut(self.pos_event_index) {
                *slot = now_ms;
            }
            self.pos_event_index += 1;
            self.pos_event_stored = true;
            self.neg_event_stored = false;
        }

        // and negative
        if !self.neg_event_stored && -slew_rate > slew_rate_max {
            if self.neg_event_index >= N_EVENTS {
                self.neg_event_index = 0;
            }
            if let Some(slot) = self.neg_event_ms.get_mut(self.neg_event_index) {
                *slot = now_ms;
            }
            self.neg_event_index += 1;
            self.neg_event_stored = true;
            self.pos_event_stored = false;
        }

        // oldest recorded event across both directions
        let mut oldest_ms = now_ms;
        for i in 0..N_EVENTS {
            if let Some(v) = self.pos_event_ms.get(i) {
                oldest_ms = oldest_ms.min(*v);
            }
            if let Some(v) = self.neg_event_ms.get(i) {
                oldest_ms = oldest_ms.min(*v);
            }
        }

        // Reduce further when the oldest exceedance falls outside the window
        // needed for the required number of events. Stops mode changes and
        // similar one-off spikes from causing unwanted gain reduction.
        let mut modifier_input = limited_raw_slew_rate;
        let span = now_ms.wrapping_sub(oldest_ms);
        let threshold = (N_EVENTS as u32 + 1) * WINDOW_MS;
        if span > threshold && slew_rate_tau > 0.0 {
            let oldest_time_from_window = 0.001 * (span.wrapping_sub(threshold)) as f32;
            modifier_input *= libm::expf(-oldest_time_from_window / slew_rate_tau);
        }

        self.modifier_slew_rate =
            (1.0 - attack_alpha) * self.modifier_slew_rate + attack_alpha * modifier_input;
        self.modifier_slew_rate = self.modifier_slew_rate.min(modifier_input);

        // gain adjustment
        if self.modifier_slew_rate > slew_rate_max {
            slew_rate_max
                / (slew_rate_max + MODIFIER_GAIN * (self.modifier_slew_rate - slew_rate_max))
        } else {
            1.0
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // Upstream ships no unit test for SlewLimiter, so these are PORT-DERIVED
    // from reading SlewLimiter.cpp.

    fn params() -> SlewParams {
        SlewParams {
            slew_rate_max: 10.0,
            slew_rate_tau: 1.0,
        }
    }

    /// DIVERGENCE D-006, pinned.
    ///
    /// UPSTREAM: the constructor initialises only the two parameter references
    /// and the internal filter, leaving thirteen members indeterminate
    /// (SlewLimiter.h:36-48). Reading them is undefined behavior.
    /// PORTED: every field starts at zero.
    ///
    /// Do not "restore parity"; there is no defined behavior to restore.
    #[test]
    fn d006_state_is_deterministically_zeroed() {
        let s = SlewLimiter::new();
        assert_eq!(s.get_slew_rate(), 0.0);
        assert_eq!(s.last_sample, 0.0);
        assert_eq!(s.max_pos_slew_rate, 0.0);
        assert_eq!(s.max_neg_slew_rate, 0.0);
        assert_eq!(s.pos_event_ms, [0; N_EVENTS]);
        assert_eq!(s.neg_event_ms, [0; N_EVENTS]);
        assert!(!s.pos_event_stored);
        assert!(!s.neg_event_stored);

        // A steady signal from rest must not reduce gain. With garbage
        // last_sample this would produce a huge first derivative.
        let mut s = SlewLimiter::new();
        let m = s.modifier(0.0, 0.02, 1000, params());
        assert_eq!(m, 1.0, "a signal at rest must not crush gains");
    }

    /// Non-positive dt short-circuits to no modification.
    #[test]
    fn non_positive_dt_returns_unity() {
        let mut s = SlewLimiter::new();
        assert_eq!(s.modifier(5.0, 0.0, 1000, params()), 1.0);
        assert_eq!(s.modifier(5.0, -0.01, 1000, params()), 1.0);
    }

    /// A non-positive slew_rate_max disables limiting.
    #[test]
    fn non_positive_max_disables_limiting() {
        let mut s = SlewLimiter::new();
        let p = SlewParams {
            slew_rate_max: 0.0,
            slew_rate_tau: 1.0,
        };
        for i in 0..20 {
            let t = 1000 + i * 20;
            // large alternating input that would otherwise trigger limiting
            let sample = if i % 2 == 0 { 100.0 } else { -100.0 };
            assert_eq!(s.modifier(sample, 0.02, t, p), 1.0);
        }
    }

    /// A slow signal well inside the limit leaves gain untouched.
    #[test]
    fn slow_signal_leaves_gain_unity() {
        let mut s = SlewLimiter::new();
        let mut m = 1.0;
        for i in 0..50 {
            let t = 1000 + i * 20;
            m = s.modifier(i as f32 * 0.01, 0.02, t, params());
        }
        assert_eq!(m, 1.0, "slow signal should not reduce gain, got {m}");
    }

    /// A fast oscillation drives the modifier below 1 and never out of range.
    #[test]
    fn oscillation_reduces_gain() {
        let mut s = SlewLimiter::new();
        let mut m = 1.0;
        for i in 0..100 {
            let t = 1000 + i * 20;
            let sample = if i % 2 == 0 { 5.0 } else { -5.0 };
            m = s.modifier(sample, 0.02, t, params());
            assert!(m > 0.0 && m <= 1.0, "modifier out of range: {m}");
        }
        assert!(m < 1.0, "sustained oscillation should reduce gain, got {m}");
        assert!(s.get_slew_rate() > 0.0);
    }

    /// Millisecond arithmetic wraps like the C++ uint32 counter rather than
    /// panicking, so behavior is unchanged across the ~49 day rollover.
    #[test]
    fn millisecond_counter_wraps_without_panicking() {
        let mut s = SlewLimiter::new();
        // start just before the u32 rollover and step across it
        for i in 0..20u32 {
            // build the timestamp with wrapping too, or the test itself
            // overflows before the code under test is even reached
            let t = (u32::MAX - 200).wrapping_add(i * 20);
            let m = s.modifier(i as f32 * 0.1, 0.02, t, params());
            assert!(
                m > 0.0 && m <= 1.0,
                "modifier out of range across rollover: {m}"
            );
        }
    }
}
