//! Single-event FF estimate and `ff_filter`, upstream `AP_AutoTune::update`.
//!
//! After a demand event ends, `update` writes
//! `FF_single = max_actuator / (max_rate * scaler)` on
//! [`AtState::DemandPos`], or `min_actuator / (min_rate * scaler)`
//! otherwise, then `ff_filter.apply(FF_single)` and increments
//! `ff_count`. The filtered value is not the accepted estimate until
//! `ff_count == 4`; `ff_count == 1` only floors P/D. I-term / FF
//! coupling stays in [`crate::ff`]; P raise/lower stays in [`crate::update`].

use ap_filter::mode::ModeFilterFloatSize5;

use crate::gains::AtGains;
use crate::state::{AtState, AutoTune};

/// Upstream `ModeFilterFloat_Size5` return rank (`ff_filter(2)`).
pub const FF_FILTER_RETURN_ELEMENT: usize = 2;

/// First-event count, upstream `if (ff_count == 1)`.
pub const FF_COUNT_FIRST: u16 = 1;

/// Accepted-estimate count, upstream `else if (ff_count == 4)`.
pub const FF_COUNT_READY: u16 = 4;

/// Minimum D on the first event, upstream `MAX(D, 0.0005)`.
pub const AUTOTUNE_MIN_D: f32 = 0.0005;

/// Minimum P on the first event, upstream `MAX(P, 0.01)`.
pub const AUTOTUNE_MIN_P: f32 = 0.01;

/// P scale when the estimate is accepted, upstream `P *= 0.5`.
pub const FF_READY_P_SCALE: f32 = 0.5;

/// Single-event FF from a finished demand, upstream `FF_single`.
///
/// Positive demand uses the max actuator/rate pair; every other state
/// (negative demand, and the C++ `else`) uses the min pair.
#[must_use]
pub fn ff_single(
    state: AtState,
    max_actuator: f32,
    min_actuator: f32,
    max_rate: f32,
    min_rate: f32,
    scaler: f32,
) -> f32 {
    if state == AtState::DemandPos {
        max_actuator / (max_rate * scaler)
    } else {
        min_actuator / (min_rate * scaler)
    }
}

/// Whether `ff_count` is still below the accept gate.
///
/// Upstream `else if (ff_count < 4)` — keep going, no P/D raise yet.
#[must_use]
pub const fn ff_estimate_pending(ff_count: u16) -> bool {
    ff_count < FF_COUNT_READY
}

/// Whether the filtered FF is the accepted estimate.
///
/// True from the `ff_count == 4` event onward.
#[must_use]
pub const fn ff_estimate_ready(ff_count: u16) -> bool {
    ff_count >= FF_COUNT_READY
}

/// P/D rewrite from the `ff_count == 1` / `== 4` gates.
///
/// Count 1 floors D at [`AUTOTUNE_MIN_D`] and P at [`AUTOTUNE_MIN_P`].
/// Count 4 halves P (`P *= 0.5`). Other counts leave both unchanged.
#[must_use]
pub fn apply_ff_count_gate(p: f32, d: f32, ff_count: u16) -> (f32, f32) {
    if ff_count == FF_COUNT_FIRST {
        let d = if d < AUTOTUNE_MIN_D {
            AUTOTUNE_MIN_D
        } else {
            d
        };
        let p = if p < AUTOTUNE_MIN_P {
            AUTOTUNE_MIN_P
        } else {
            p
        };
        (p, d)
    } else if ff_count == FF_COUNT_READY {
        (p * FF_READY_P_SCALE, d)
    } else {
        (p, d)
    }
}

/// Apply [`apply_ff_count_gate`] to an [`AtGains`] snapshot.
#[must_use]
pub fn apply_ff_count_gains(gains: AtGains, ff_count: u16) -> AtGains {
    let (p, d) = apply_ff_count_gate(gains.p, gains.d, ff_count);
    AtGains { p, d, ..gains }
}

/// Running `ff_filter` / `ff_count` / `FF_single` for one axis.
///
/// Upstream `ModeFilterFloat_Size5 ff_filter` constructed as `ff_filter(2)`,
/// plus the `FF_single` / `ff_count` fields on `AP_AutoTune`.
#[derive(Debug, Clone, Copy)]
pub struct FfEstimate {
    /// Last single-event ratio, upstream `AP_AutoTune::FF_single`.
    pub ff_single: f32,
    /// Finished-event count, upstream `AP_AutoTune::ff_count`.
    pub ff_count: u16,
    filter: ModeFilterFloatSize5,
}

impl Default for FfEstimate {
    fn default() -> Self {
        Self::new()
    }
}

impl FfEstimate {
    /// Empty filter and zero count, matching `start` after `ff_filter.reset()`.
    #[must_use]
    pub fn new() -> Self {
        Self {
            ff_single: 0.0,
            ff_count: 0,
            filter: ModeFilterFloatSize5::new(FF_FILTER_RETURN_ELEMENT),
        }
    }

    /// Clear the median filter and counters, upstream `ff_filter.reset()` /
    /// `ff_count = 0` in `start`.
    pub fn reset(&mut self) {
        self.ff_single = 0.0;
        self.ff_count = 0;
        self.filter.reset();
    }

    /// Push one sample through `ff_filter.apply`.
    pub fn apply(&mut self, sample: f32) -> f32 {
        self.filter.apply(sample)
    }

    /// Last value returned by [`Self::apply`] / [`Self::apply_event`].
    #[must_use]
    pub fn filtered(&self) -> f32 {
        self.filter.get()
    }

    /// One finished demand: compute `FF_single`, apply the filter, bump count.
    ///
    /// Returns the filtered FF (`float FF = ff_filter.apply(FF_single)`).
    pub fn apply_event(
        &mut self,
        state: AtState,
        max_actuator: f32,
        min_actuator: f32,
        max_rate: f32,
        min_rate: f32,
        scaler: f32,
    ) -> f32 {
        self.ff_single = ff_single(
            state,
            max_actuator,
            min_actuator,
            max_rate,
            min_rate,
            scaler,
        );
        let ff = self.filter.apply(self.ff_single);
        self.ff_count = self.ff_count.saturating_add(1);
        ff
    }

    /// [`ff_estimate_ready`] for this session's `ff_count`.
    #[must_use]
    pub const fn estimate_ready(&self) -> bool {
        ff_estimate_ready(self.ff_count)
    }

    /// [`ff_estimate_pending`] for this session's `ff_count`.
    #[must_use]
    pub const fn estimate_pending(&self) -> bool {
        ff_estimate_pending(self.ff_count)
    }
}

impl AutoTune {
    /// Apply the `ff_count == 1` / `== 4` P/D gates to the live axis.
    ///
    /// No-op when not running, matching the early return at the top of
    /// `AP_AutoTune::update`.
    pub fn apply_ff_count_gate(&mut self, ff_count: u16) {
        if !self.running {
            return;
        }
        self.current = apply_ff_count_gains(self.current, ff_count);
    }
}
