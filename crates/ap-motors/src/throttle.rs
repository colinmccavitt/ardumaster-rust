//! The throttle input path, upstream `AP_MotorsMulticopter`'s
//! `update_throttle_filter` and `update_throttle_hover`. COP-004.
//!
//! Between the pilot's stick and anything that reads "the throttle" there is a
//! low-pass filter, and beside it a slew-rate estimate that exists so the
//! vehicle can notice the pilot moving the stick violently. Separately, and on
//! a much longer time constant, the vehicle learns what throttle it actually
//! hovers at.
//!
//! Nothing here reads a clock of its own. Upstream calls `AP_HAL::micros()`
//! inside the update; per ADR-0004 the timestamp arrives as an argument, which
//! is also what makes the slew estimate testable.

use ap_filter::derivative::DerivativeFilter;
use ap_filter::lowpass::LowPassFilter;
use ap_math::scalar::is_equal;

/// Time constant for learning the hover throttle, upstream
/// `AP_MOTORS_THST_HOVER_TC`.
///
/// Ten seconds. The estimate is meant to track where the aircraft settles over
/// a flight, not to follow the stick.
const HOVER_TC: f32 = 10.0;

/// Bounds on the learned hover throttle, upstream `AP_MOTORS_THST_HOVER_MIN`
/// and `_MAX`.
///
/// These are the range the third-order expo polynomial can actually reach, so
/// a value outside them is not merely unlikely but unusable downstream.
const HOVER_MIN: f32 = 0.125;
const HOVER_MAX: f32 = 0.6875;

/// How the vehicle treats the learned hover throttle, upstream `MOT_HOVER_LEARN`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HoverLearn {
    /// Do not learn.
    Disabled = 0,
    /// Learn in memory, but do not persist it.
    #[default]
    LearnOnly = 1,
    /// Learn and save on disarm.
    LearnAndSave = 2,
}

/// The filtered pilot throttle and its slew-rate estimate.
#[derive(Debug, Clone)]
pub struct ThrottleInput {
    filter: LowPassFilter<f32>,
    slew: DerivativeFilter<7>,
    slew_filter: LowPassFilter<f32>,
    slew_rate: f32,
}

impl Default for ThrottleInput {
    fn default() -> Self {
        Self::new()
    }
}

impl ThrottleInput {
    /// An input path with both filters unconfigured and unseeded.
    ///
    /// Both cutoffs start at zero, which is what upstream default-constructs
    /// them to: no filtering until the vehicle sets a frequency. A vehicle
    /// that forgets to is passing the throttle through unfiltered, not
    /// blocking it.
    ///
    /// The slew detector is seven samples wide, upstream
    /// `DerivativeFilterFloat_Size7`.
    pub fn new() -> Self {
        Self {
            filter: LowPassFilter::new(0.0),
            slew: DerivativeFilter::new(),
            slew_filter: LowPassFilter::new(0.0),
            slew_rate: 0.0,
        }
    }

    /// Upstream `set_throttle_filter_cutoff`.
    pub fn set_filter_cutoff(&mut self, filt_hz: f32) {
        self.filter.set_cutoff_frequency(filt_hz);
    }

    /// Upstream `set_slew_filter_cutoff`.
    pub fn set_slew_filter_cutoff(&mut self, filt_hz: f32) {
        self.slew_filter.set_cutoff_frequency(filt_hz);
    }

    /// The filtered throttle, upstream `get_throttle`.
    ///
    /// Note this is the *filtered* value, not the last one handed to
    /// [`Self::update`]. Code that mixes the two up gets a throttle that reads
    /// as zero until the filter has been run, which is a quiet way to make a
    /// spool-up finish instantly.
    pub fn throttle(&self) -> f32 {
        self.filter.get().clamp(0.0, 1.0)
    }

    /// The filter value with no clamp, which is what upstream stores.
    ///
    /// [`Self::throttle`] clamps on the way out rather than on the way in, so
    /// the two differ whenever the filter has been driven past its range.
    pub fn throttle_raw(&self) -> f32 {
        self.filter.get()
    }

    /// The bidirectional form, upstream `get_throttle_bidirectional`.
    pub fn throttle_bidirectional(&self) -> f32 {
        (2.0_f32 * (self.filter.get() - 0.5)).clamp(-1.0, 1.0)
    }

    /// The filtered slew rate, upstream `get_throttle_slew_rate`.
    pub fn slew_rate(&self) -> f32 {
        self.slew_rate
    }

    /// One iteration, upstream `update_throttle_filter`.
    ///
    /// While disarmed the filter is reset rather than run, so a throttle slew
    /// after arming starts from zero instead of resuming from wherever the
    /// stick was left.
    ///
    /// The slew detector is only given a sample when the filtered value
    /// actually changed. That is what keeps a stationary stick from feeding
    /// the derivative filter a run of identical samples, which would drag its
    /// estimate toward zero at whatever rate the loop happens to run.
    pub fn update(&mut self, throttle_in: f32, armed: bool, dt_s: f32, now_us: u32) {
        let last_thr = self.filter.get();

        if armed {
            self.filter.apply(throttle_in, dt_s);
            // Clamped by reset, not by a plain assignment: resetting also
            // clears the filter's history, so a saturated input does not leave
            // the filter holding a value it can drift back down from.
            if self.filter.get() < 0.0 {
                self.filter.reset_to(0.0);
            }
            if self.filter.get() > 1.0 {
                self.filter.reset_to(1.0);
            }
        } else {
            self.filter.reset_to(0.0);
        }

        let new_thr = self.filter.get();

        if !is_equal(last_thr, new_thr) {
            self.slew.update(new_thr, now_us);
        }

        // The derivative filter works in per-microsecond units because that is
        // what its timestamps are in; the scaling makes it per-second.
        let rate = (self.slew.slope() * 1e6).abs();
        self.slew_rate = self.slew_filter.apply(rate, dt_s);
    }
}

/// The learned hover throttle, upstream `_throttle_hover`.
#[derive(Debug, Clone, Copy)]
pub struct HoverThrottle {
    value: f32,
}

impl HoverThrottle {
    /// Start from a configured value, upstream's `MOT_THST_HOVER` parameter.
    pub fn new(value: f32) -> Self {
        Self { value }
    }

    /// The stored value clamped to the reachable range, upstream
    /// `get_throttle_hover`.
    pub fn get(&self) -> f32 {
        self.value.clamp(HOVER_MIN, HOVER_MAX)
    }

    /// The raw stored value, which is what upstream writes to the parameter.
    pub fn raw(&self) -> f32 {
        self.value
    }

    /// One iteration of learning, upstream `update_throttle_hover`.
    ///
    /// A first-order lag toward the current throttle with a ten-second time
    /// constant, clamped on write. `dt` is passed separately from the main
    /// loop's because upstream calls this at 100 Hz from the vehicle rather
    /// than from the motors update.
    pub fn update(&mut self, throttle: f32, dt: f32, learn: HoverLearn) {
        if learn == HoverLearn::Disabled {
            return;
        }
        self.value = (self.value + (dt / (dt + HOVER_TC)) * (throttle - self.value))
            .clamp(HOVER_MIN, HOVER_MAX);
    }
}
