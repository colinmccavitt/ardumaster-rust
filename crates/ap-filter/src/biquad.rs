//! Second-order low pass, upstream `Filter/LowPassFilter2p`.
//!
//! A biquad in direct form II, with the coefficients of a two-pole Butterworth
//! derived from a sample rate and a cutoff. This is the filter
//! `AP_InertialSensor` runs every gyro and accelerometer sample through, which
//! is why it is here: the single-pole [`crate::lowpass`] filters roll off at
//! 20 dB/decade, and vibration rejection on an airframe needs 40.
//!
//! # Why the first sample passes through unchanged
//!
//! A filter that started from zero would ramp toward the signal over its time
//! constant, injecting a startup transient into a control loop. So the first
//! sample seeds both delay elements at the value that makes the filter already
//! settled: `sample / (1 + a1 + a2)`.
//!
//! That expression is exactly right rather than approximately right. Summing
//! the coefficients gives `b0 + b1 + b2 = 4*ohm^2/c` and `1 + a1 + a2 =
//! 4*ohm^2/c` -- the same quantity, so the filter's DC gain is exactly one and
//! seeding at `sample/(1+a1+a2)` makes the first output exactly `sample`. It
//! also means the divisor can never be zero for a positive cutoff, so the
//! seeding cannot produce an infinity.
//!
//! # The Nyquist guard is load-bearing
//!
//! `compute_params` clamps the cutoff to `0.4 * sample_freq` before anything
//! else. That is not only about aliasing: the coefficients come from
//! `tan(pi/fr)` where `fr = sample_freq/cutoff_freq`, and `tan` goes to
//! infinity at `fr = 2`. Clamping at 0.4 keeps `fr >= 2.5`, which keeps the
//! tangent finite with room to spare.

use ap_math::scalar::{is_positive, Real};

use crate::lowpass::Filterable;

/// Biquad coefficients, upstream `DigitalBiquadFilter::biquad_params`.
///
/// A default-constructed set is all zeros, which reads as "no cutoff" and
/// makes [`DigitalBiquadFilter::apply`] pass samples through untouched.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct BiquadParams {
    /// The cutoff actually in use, after the Nyquist clamp. Hz.
    pub cutoff_freq: f32,
    /// The sample rate the coefficients were computed for. Hz.
    pub sample_freq: f32,
    /// First feedback coefficient.
    pub a1: f32,
    /// Second feedback coefficient.
    pub a2: f32,
    /// Feed-forward coefficients.
    pub b0: f32,
    /// Feed-forward coefficients.
    pub b1: f32,
    /// Feed-forward coefficients.
    pub b2: f32,
}

impl BiquadParams {
    /// Two-pole Butterworth coefficients for a sample rate and cutoff,
    /// upstream `DigitalBiquadFilter::compute_params`.
    #[must_use]
    pub fn compute(sample_freq: f32, cutoff_freq: f32) -> Self {
        let mut p = Self::default();
        p.update(sample_freq, cutoff_freq);
        p
    }

    /// Recompute in place, which is the shape upstream's `compute_params`
    /// actually has -- it writes into a `biquad_params` the caller owns.
    ///
    /// The distinction matters on one path. A cutoff that is not positive,
    /// including one clamped to nothing by a zero or negative sample rate,
    /// records the two frequencies and returns **without touching
    /// `a1`..`b2`**. So retuning a live filter to zero leaves the previous
    /// tuning's coefficients in place. They are dead while the cutoff is zero,
    /// since [`DigitalBiquadFilter::apply`] passes samples straight through --
    /// but [`DigitalBiquadFilter::reset_to`] still reads them, through
    /// `1 / (1 + a1 + a2)`. Zeroing them here would seed the delay elements
    /// differently on that path, so the leftovers are reproduced.
    ///
    /// A never-tuned set is all zeros, which is what upstream's
    /// statically-stored filters get and what D-005 is about.
    pub fn update(&mut self, sample_freq: f32, cutoff_freq: f32) {
        let p = self;

        // Upstream's MIN macro is a ternary, so it yields the second operand
        // when either is NaN. Written out rather than using f32::min, which
        // has the opposite NaN behaviour.
        let nyquist_guard = sample_freq * 0.4;
        p.cutoff_freq = if cutoff_freq < nyquist_guard {
            cutoff_freq
        } else {
            nyquist_guard
        };
        p.sample_freq = sample_freq;

        if !is_positive(p.cutoff_freq) {
            return;
        }

        let fr = p.sample_freq / p.cutoff_freq;
        // `Real::tan` rather than `fr.tan()`: under `cfg(test)` std's inherent
        // f32 methods win name resolution and would silently route around
        // libm. See D-017.
        let ohm = Real::tan(core::f32::consts::PI / fr);
        let cos_pi_4 = Real::cos(core::f32::consts::PI / 4.0);
        let c = 1.0 + 2.0 * cos_pi_4 * ohm + ohm * ohm;

        p.b0 = ohm * ohm / c;
        p.b1 = 2.0 * p.b0;
        p.b2 = p.b0;
        p.a1 = 2.0 * (ohm * ohm - 1.0) / c;
        p.a2 = (1.0 - 2.0 * cos_pi_4 * ohm + ohm * ohm) / c;
    }
}

/// The biquad itself, upstream `DigitalBiquadFilter<T>`.
///
/// Holds only the two delay elements: the coefficients live with the caller,
/// which is how upstream shares one parameter set across several filters.
#[derive(Debug, Clone, Copy)]
pub struct DigitalBiquadFilter<T> {
    delay_element_1: T,
    delay_element_2: T,
    initialised: bool,
}

impl<T: Filterable> Default for DigitalBiquadFilter<T> {
    /// DIVERGENCE D-005: deterministic. Upstream's constructor sets the two
    /// delay elements and leaves `initialised` indeterminate, which is
    /// undefined behavior on first use -- the same defect as `DigitalLPF`, in
    /// a second class.
    fn default() -> Self {
        Self {
            delay_element_1: T::zero(),
            delay_element_2: T::zero(),
            initialised: false,
        }
    }
}

impl<T: Filterable> DigitalBiquadFilter<T> {
    /// A fresh, unseeded filter.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Filter one sample, upstream `apply`.
    ///
    /// Passes the sample through untouched when either frequency is not
    /// positive, which is how a filter with no configured cutoff behaves.
    pub fn apply(&mut self, sample: T, params: &BiquadParams) -> T {
        if !is_positive(params.cutoff_freq) || !is_positive(params.sample_freq) {
            return sample;
        }

        if !self.initialised {
            self.reset_to(sample, params);
        }

        let delay_element_0 = sample
            .sub(self.delay_element_1.scale(params.a1))
            .sub(self.delay_element_2.scale(params.a2));
        let output = delay_element_0
            .scale(params.b0)
            .add(self.delay_element_1.scale(params.b1))
            .add(self.delay_element_2.scale(params.b2));

        self.delay_element_2 = self.delay_element_1;
        self.delay_element_1 = delay_element_0;

        output
    }

    /// Forget the state, so the next sample re-seeds. Upstream `reset()`.
    ///
    /// Note this does *not* zero the delay elements -- it only clears the
    /// flag. They are overwritten on the next sample, so their old values are
    /// unreachable.
    pub fn reset(&mut self) {
        self.initialised = false;
    }

    /// Seed the filter settled at `value`, upstream `reset(value, params)`.
    pub fn reset_to(&mut self, value: T, params: &BiquadParams) {
        let settled = value.scale(1.0 / (1.0 + params.a1 + params.a2));
        self.delay_element_1 = settled;
        self.delay_element_2 = settled;
        self.initialised = true;
    }

    /// Whether the filter has been seeded.
    ///
    /// Not an upstream interface, and the reason it exists is D-005: the flag
    /// is the field upstream fails to initialise, so the port states its value
    /// rather than leaving it implicit.
    #[must_use]
    pub const fn is_initialised(&self) -> bool {
        self.initialised
    }
}

/// A biquad bound to its own coefficients, upstream `LowPassFilter2p<T>`.
#[derive(Debug, Clone, Copy)]
pub struct LowPassFilter2p<T> {
    params: BiquadParams,
    filter: DigitalBiquadFilter<T>,
}

impl<T: Filterable> Default for LowPassFilter2p<T> {
    /// Pass-through until a cutoff is set, upstream's default constructor
    /// (which memsets its parameters to zero).
    fn default() -> Self {
        Self {
            params: BiquadParams::default(),
            filter: DigitalBiquadFilter::default(),
        }
    }
}

impl<T: Filterable> LowPassFilter2p<T> {
    /// A pass-through filter with no cutoff configured.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// A filter for a sample rate and cutoff, both Hz.
    #[must_use]
    pub fn with_cutoff(sample_freq: f32, cutoff_freq: f32) -> Self {
        Self {
            params: BiquadParams::compute(sample_freq, cutoff_freq),
            filter: DigitalBiquadFilter::default(),
        }
    }

    /// Recompute the coefficients, upstream `set_cutoff_frequency`.
    ///
    /// Deliberately does **not** reset the filter: upstream retunes in flight
    /// as the notch tracks engine RPM, and clearing the delay elements would
    /// put a step into the gyro signal every time it did.
    pub fn set_cutoff_frequency(&mut self, sample_freq: f32, cutoff_freq: f32) {
        self.params.update(sample_freq, cutoff_freq);
    }

    /// The cutoff in use, after the Nyquist clamp. Upstream
    /// `get_cutoff_freq`.
    #[must_use]
    pub const fn cutoff_freq(&self) -> f32 {
        self.params.cutoff_freq
    }

    /// The sample rate the coefficients were computed for. Upstream
    /// `get_sample_freq`.
    #[must_use]
    pub const fn sample_freq(&self) -> f32 {
        self.params.sample_freq
    }

    /// The coefficients themselves.
    #[must_use]
    pub const fn params(&self) -> BiquadParams {
        self.params
    }

    /// Filter one sample.
    pub fn apply(&mut self, sample: T) -> T {
        self.filter.apply(sample, &self.params)
    }

    /// Forget the state, so the next sample re-seeds.
    pub fn reset(&mut self) {
        self.filter.reset();
    }

    /// Seed the filter settled at `value`.
    pub fn reset_to(&mut self, value: T) {
        self.filter.reset_to(value, &self.params);
    }

    /// Whether the filter has been seeded. See [`DigitalBiquadFilter::is_initialised`].
    #[must_use]
    pub const fn is_initialised(&self) -> bool {
        self.filter.is_initialised()
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "these compare for exact equality on purpose: a sample passed \nthrough untouched, or a coefficient left exactly as it was by an early return. An \nepsilon would admit precisely the changes these tests exist to rule out"
    )]

    use super::*;
    use ap_math::vector3::Vector3f;

    /// D-005, second class. A fresh biquad is deterministically unseeded;
    /// upstream leaves the flag indeterminate.
    #[test]
    fn d005_a_fresh_biquad_is_deterministically_unseeded() {
        let f: DigitalBiquadFilter<f32> = DigitalBiquadFilter::new();
        assert!(!f.is_initialised());
        let g: LowPassFilter2p<Vector3f> = LowPassFilter2p::new();
        assert!(!g.is_initialised());
    }

    /// The seeding is exact, not approximate: the filter's DC gain is one, so
    /// the first sample comes back unchanged. A filter seeded from zero would
    /// return roughly a tenth of it.
    #[test]
    fn the_first_sample_passes_through_unchanged() {
        let mut f = LowPassFilter2p::<f32>::with_cutoff(1000.0, 20.0);
        let out = f.apply(7.5);
        assert!(
            (out - 7.5).abs() < 1e-5,
            "the first sample should pass through, got {out}"
        );
    }

    /// And it is the *seeding* doing that, not a pass-through branch: after
    /// the first sample the filter really does filter.
    #[test]
    fn a_step_is_smoothed_after_the_first_sample() {
        let mut f = LowPassFilter2p::<f32>::with_cutoff(1000.0, 20.0);
        assert!((f.apply(0.0)).abs() < 1e-6);
        let first = f.apply(1.0);
        assert!(
            first < 0.05,
            "a step should not appear at the output immediately, got {first}"
        );
        for _ in 0..500 {
            f.apply(1.0);
        }
        let settled = f.apply(1.0);
        assert!(
            (settled - 1.0).abs() < 1e-3,
            "and it should settle at unity gain, got {settled}"
        );
    }

    /// The Nyquist guard clamps the cutoff and keeps the coefficients finite.
    /// Without it `tan(pi/fr)` diverges as the cutoff approaches half the
    /// sample rate.
    #[test]
    fn the_cutoff_is_clamped_to_forty_percent_of_the_sample_rate() {
        let f = LowPassFilter2p::<f32>::with_cutoff(1000.0, 900.0);
        assert!((f.cutoff_freq() - 400.0).abs() < 1e-3);
        let p = f.params();
        assert!(p.a1.is_finite() && p.a2.is_finite() && p.b0.is_finite());
    }

    /// The claim the seeding rests on: DC gain is exactly one, because the
    /// numerator and denominator coefficient sums are the same quantity.
    #[test]
    fn the_dc_gain_is_one() {
        for (fs, fc) in [(1000.0, 20.0), (400.0, 10.0), (8000.0, 188.0), (50.0, 5.0)] {
            let p = BiquadParams::compute(fs, fc);
            let gain = (p.b0 + p.b1 + p.b2) / (1.0 + p.a1 + p.a2);
            assert!((gain - 1.0).abs() < 1e-4, "fs {fs} fc {fc}: DC gain {gain}");
        }
    }

    /// No cutoff means pass-through, and the coefficients are determined
    /// rather than left over -- the D-005 second path.
    #[test]
    fn no_cutoff_is_pass_through_with_determined_coefficients() {
        // Fresh, so there are no previous coefficients to leave behind.
        let mut f = LowPassFilter2p::<f32>::with_cutoff(1000.0, 0.0);
        assert_eq!(
            f.params(),
            BiquadParams {
                sample_freq: 1000.0,
                ..BiquadParams::default()
            }
        );
        assert_eq!(f.apply(3.0), 3.0);
        assert_eq!(f.apply(-9.0), -9.0);
        assert!(
            !f.is_initialised(),
            "pass-through returns before seeding, so the filter stays unseeded"
        );
    }

    /// Retuning to zero leaves the previous coefficients in place, which is
    /// upstream's behaviour and is observable: `reset_to` divides by
    /// `1 + a1 + a2`, so zeroing them would seed the delay elements
    /// differently.
    #[test]
    fn retuning_to_zero_keeps_the_old_coefficients() {
        let mut f = LowPassFilter2p::<f32>::with_cutoff(1000.0, 20.0);
        let tuned = f.params();
        f.set_cutoff_frequency(1000.0, 0.0);
        let after = f.params();

        assert_eq!(after.cutoff_freq, 0.0);
        assert_eq!(after.a1, tuned.a1, "coefficients should survive the retune");
        assert_eq!(after.a2, tuned.a2);
        assert_eq!(after.b0, tuned.b0);
    }

    /// A zero sample rate clamps the cutoff to zero, which is pass-through --
    /// the guard covers a bad sample rate as well as a bad cutoff.
    #[test]
    fn a_zero_sample_rate_is_pass_through() {
        let mut f = LowPassFilter2p::<f32>::with_cutoff(0.0, 20.0);
        assert_eq!(f.apply(4.0), 4.0);
    }

    /// Retuning does not disturb the state. Upstream retunes in flight as the
    /// notch tracks engine RPM; resetting would step the gyro signal each time.
    #[test]
    fn retuning_does_not_reset_the_state() {
        let mut f = LowPassFilter2p::<f32>::with_cutoff(1000.0, 20.0);
        f.apply(1.0);
        assert!(f.is_initialised());
        f.set_cutoff_frequency(1000.0, 40.0);
        assert!(f.is_initialised(), "retuning should not unseed the filter");
    }

    /// Reset clears the flag so the next sample re-seeds, which means that
    /// sample passes through unchanged again.
    #[test]
    fn reset_makes_the_next_sample_reseed() {
        let mut f = LowPassFilter2p::<f32>::with_cutoff(1000.0, 20.0);
        for _ in 0..100 {
            f.apply(1.0);
        }
        f.reset();
        assert!(!f.is_initialised());
        let out = f.apply(5.0);
        assert!(
            (out - 5.0).abs() < 1e-5,
            "re-seeded, so pass-through: {out}"
        );
    }

    /// The vector form filters each axis independently with shared
    /// coefficients.
    #[test]
    fn the_vector_form_filters_each_axis() {
        let mut v = LowPassFilter2p::<Vector3f>::with_cutoff(1000.0, 20.0);
        let mut x = LowPassFilter2p::<f32>::with_cutoff(1000.0, 20.0);

        v.apply(Vector3f::new(0.0, 0.0, 0.0));
        x.apply(0.0);
        for i in 0..50 {
            let s = if i < 25 { 1.0 } else { -1.0 };
            let got = v.apply(Vector3f::new(s, 2.0 * s, -s));
            let want = x.apply(s);
            assert!((got.x - want).abs() < 1e-6);
            assert!((got.y - 2.0 * want).abs() < 1e-5);
            assert!((got.z + want).abs() < 1e-6);
        }
    }
}
