//! Actuator / rate / target filter cutoffs on `AP_AutoTune::start`.
//!
//! Upstream writes 0.75 Hz on `actuator_filter` and `rate_filter`, and
//! 4 Hz on `target_filter`, all sampled at the scheduler loop rate:
//!
//! ```text
//! actuator_filter.set_cutoff_frequency(loop_rate_hz, 0.75);
//! rate_filter.set_cutoff_frequency(loop_rate_hz, 0.75);
//! target_filter.set_cutoff_frequency(loop_rate_hz, 4);
//! actuator_filter.reset();
//! rate_filter.reset();
//! ```
//!
//! `target_filter` is not reset in `start`. The zero-FF floor stays in
//! [`crate::start`]; `ff_filter` stays in [`crate::ff_estimate`].

use ap_filter::lowpass::LowPassFilterConstDtFloat;

/// Actuator LPF cutoff, upstream `0.75` Hz.
pub const ACTUATOR_FILTER_HZ: f32 = 0.75;

/// Rate LPF cutoff, upstream `0.75` Hz.
pub const RATE_FILTER_HZ: f32 = 0.75;

/// Target LPF cutoff, upstream `4` Hz.
pub const TARGET_FILTER_HZ: f32 = 4.0;

/// The three start-time LPFs, upstream `LowPassFilterConstDtFloat`
/// `actuator_filter` / `rate_filter` / `target_filter`.
#[derive(Debug, Clone, Copy)]
pub struct StartFilters {
    /// Upstream `AP_AutoTune::actuator_filter`.
    pub actuator: LowPassFilterConstDtFloat,
    /// Upstream `AP_AutoTune::rate_filter`.
    pub rate: LowPassFilterConstDtFloat,
    /// Upstream `AP_AutoTune::target_filter`.
    pub target: LowPassFilterConstDtFloat,
}

impl Default for StartFilters {
    fn default() -> Self {
        Self::new()
    }
}

impl StartFilters {
    /// Unconfigured filters, matching default-constructed C++ members.
    #[must_use]
    pub fn new() -> Self {
        Self {
            actuator: LowPassFilterConstDtFloat::default(),
            rate: LowPassFilterConstDtFloat::default(),
            target: LowPassFilterConstDtFloat::default(),
        }
    }

    /// Apply the `start` cutoffs at `loop_rate_hz` and reset actuator/rate.
    ///
    /// Upstream `AP_AutoTune::start` after `dt` / IMAX, before the zero-FF
    /// floor. `target_filter` keeps its history; only actuator and rate
    /// are `reset()`.
    pub fn configure(&mut self, loop_rate_hz: f32) {
        set_start_filter_cutoffs(
            &mut self.actuator,
            &mut self.rate,
            &mut self.target,
            loop_rate_hz,
        );
    }
}

/// Write the three `start` cutoffs, then reset actuator and rate.
///
/// `loop_rate_hz` is `AP::scheduler().get_loop_rate_hz()`.
pub fn set_start_filter_cutoffs(
    actuator: &mut LowPassFilterConstDtFloat,
    rate: &mut LowPassFilterConstDtFloat,
    target: &mut LowPassFilterConstDtFloat,
    loop_rate_hz: f32,
) {
    actuator.set_cutoff_frequency(loop_rate_hz, ACTUATOR_FILTER_HZ);
    rate.set_cutoff_frequency(loop_rate_hz, RATE_FILTER_HZ);
    target.set_cutoff_frequency(loop_rate_hz, TARGET_FILTER_HZ);
    actuator.reset();
    rate.reset();
}

/// Fresh filters already configured for `loop_rate_hz`.
#[must_use]
pub fn start_filters(loop_rate_hz: f32) -> StartFilters {
    let mut filters = StartFilters::new();
    filters.configure(loop_rate_hz);
    filters
}
