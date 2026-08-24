//! Port of `Filter/LowPassFilter.{h,cpp}`, pinned to `Plane-4.7.0`.
//!
//! Two shapes, matching upstream:
//!
//! - [`LowPassFilter`] takes `dt` on every sample and recomputes alpha each time.
//! - [`LowPassFilterConstDt`] takes a sample rate once and caches alpha, which is
//!   cheaper but assumes a fixed step.
//!
//! # DIVERGENCE D-005
//!
//! Upstream's `DigitalLPF` declares `bool initialised;` (`LowPassFilter.h:72`) with no
//! initializer, and its constructor sets only `output`. Reading it at
//! `LowPassFilter.cpp:26` is **undefined behavior**. See DIVERGENCES.md.
//!
//! The intent is unambiguous — `reset()` sets it false precisely to trigger re-seeding —
//! so this port implements the intended behavior deterministically: a fresh filter is
//! uninitialised, and the first sample passes through unchanged.

use ap_math::scalar::calc_lowpass_alpha_dt;
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

/// Values a low-pass filter can carry. Upstream instantiates its templates for
/// `float`, `Vector2f` and `Vector3f`; this mirrors exactly those.
pub trait Filterable: Copy {
    /// The additive identity.
    fn zero() -> Self;
    /// Componentwise addition.
    fn add(self, rhs: Self) -> Self;
    /// Componentwise subtraction.
    fn sub(self, rhs: Self) -> Self;
    /// Scale by a scalar factor.
    fn scale(self, k: f32) -> Self;
}

impl Filterable for f32 {
    #[inline]
    fn zero() -> Self {
        0.0
    }
    #[inline]
    fn add(self, rhs: Self) -> Self {
        self + rhs
    }
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        self - rhs
    }
    #[inline]
    fn scale(self, k: f32) -> Self {
        self * k
    }
}

impl Filterable for Vector2f {
    #[inline]
    fn zero() -> Self {
        Vector2f::zero()
    }
    #[inline]
    fn add(self, rhs: Self) -> Self {
        self + rhs
    }
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        self - rhs
    }
    #[inline]
    fn scale(self, k: f32) -> Self {
        self * k
    }
}

impl Filterable for Vector3f {
    #[inline]
    fn zero() -> Self {
        Vector3f::zero()
    }
    #[inline]
    fn add(self, rhs: Self) -> Self {
        self + rhs
    }
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        self - rhs
    }
    #[inline]
    fn scale(self, k: f32) -> Self {
        self * k
    }
}

/// The filter arithmetic shared by both shapes. Upstream `DigitalLPF<T>`.
#[derive(Debug, Clone, Copy)]
pub struct DigitalLpf<T> {
    output: T,
    initialised: bool,
}

impl<T: Filterable> Default for DigitalLpf<T> {
    /// DIVERGENCE D-005: deterministic. Upstream leaves `initialised`
    /// indeterminate, which is undefined behavior on first use.
    #[inline]
    fn default() -> Self {
        Self {
            output: T::zero(),
            initialised: false,
        }
    }
}

impl<T: Filterable> DigitalLpf<T> {
    /// A fresh, unseeded filter.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// The latest filtered value. Upstream `get()`.
    #[inline]
    pub fn get(&self) -> T {
        self.output
    }

    /// Seed the filter to `value`, marking it initialised. Upstream `reset(value)`.
    #[inline]
    pub fn reset_to(&mut self, value: T) {
        self.output = value;
        self.initialised = true;
    }

    /// Mark the filter unseeded, so the next sample passes through unchanged.
    /// Upstream `reset()`.
    #[inline]
    pub fn reset(&mut self) {
        self.initialised = false;
    }

    /// Whether the filter has seen a sample since the last reset.
    #[inline]
    pub fn is_initialised(&self) -> bool {
        self.initialised
    }

    /// Apply one sample with a precomputed alpha. Upstream `_apply()`.
    ///
    /// Upstream computes the filtered value first and *then* overwrites it with
    /// the raw sample when uninitialised, so the first sample always passes
    /// through unchanged. The ordering is preserved, minus the wasted work.
    #[inline]
    fn apply_alpha(&mut self, sample: T, alpha: f32) -> T {
        if !self.initialised {
            self.initialised = true;
            self.output = sample;
            return self.output;
        }
        self.output = self.output.add(sample.sub(self.output).scale(alpha));
        self.output
    }
}

/// Low-pass filter with a variable time step. Upstream `LowPassFilter<T>`.
#[derive(Debug, Clone, Copy)]
pub struct LowPassFilter<T> {
    lpf: DigitalLpf<T>,
    cutoff_freq: f32,
}

impl<T: Filterable> Default for LowPassFilter<T> {
    #[inline]
    fn default() -> Self {
        Self {
            lpf: DigitalLpf::default(),
            cutoff_freq: 0.0,
        }
    }
}

impl<T: Filterable> LowPassFilter<T> {
    /// A filter with the given cutoff in Hz.
    #[inline]
    pub fn new(cutoff_freq: f32) -> Self {
        Self {
            lpf: DigitalLpf::default(),
            cutoff_freq,
        }
    }

    /// Change the cutoff frequency, in Hz. Upstream `set_cutoff_frequency()`.
    #[inline]
    pub fn set_cutoff_frequency(&mut self, cutoff_freq: f32) {
        self.cutoff_freq = cutoff_freq;
    }

    /// The cutoff frequency, in Hz. Upstream `get_cutoff_freq()`.
    #[inline]
    pub fn get_cutoff_freq(&self) -> f32 {
        self.cutoff_freq
    }

    /// Apply one sample over `dt` seconds. Upstream `apply(sample, dt)`.
    #[inline]
    pub fn apply(&mut self, sample: T, dt: f32) -> T {
        let alpha = calc_lowpass_alpha_dt(dt, self.cutoff_freq);
        self.lpf.apply_alpha(sample, alpha)
    }

    /// The latest filtered value.
    #[inline]
    pub fn get(&self) -> T {
        self.lpf.get()
    }

    /// Seed the filter to `value`.
    #[inline]
    pub fn reset_to(&mut self, value: T) {
        self.lpf.reset_to(value);
    }

    /// Mark the filter unseeded.
    #[inline]
    pub fn reset(&mut self) {
        self.lpf.reset();
    }
}

/// Low-pass filter with a fixed time step and cached alpha.
/// Upstream `LowPassFilterConstDt<T>`.
#[derive(Debug, Clone, Copy)]
pub struct LowPassFilterConstDt<T> {
    lpf: DigitalLpf<T>,
    cutoff_freq: f32,
    alpha: f32,
}

impl<T: Filterable> Default for LowPassFilterConstDt<T> {
    #[inline]
    fn default() -> Self {
        Self {
            lpf: DigitalLpf::default(),
            cutoff_freq: 0.0,
            alpha: 0.0,
        }
    }
}

impl<T: Filterable> LowPassFilterConstDt<T> {
    /// A filter for the given sample and cutoff frequencies, in Hz.
    #[inline]
    pub fn new(sample_freq: f32, cutoff_freq: f32) -> Self {
        let mut f = Self::default();
        f.set_cutoff_frequency(sample_freq, cutoff_freq);
        f
    }

    /// Recompute alpha for the given sample and cutoff frequencies, in Hz.
    ///
    /// Upstream sets `alpha = 1` for a non-positive sample rate, which makes the
    /// filter pass input straight through rather than dividing by zero.
    #[inline]
    pub fn set_cutoff_frequency(&mut self, sample_freq: f32, cutoff_freq: f32) {
        self.cutoff_freq = cutoff_freq;
        if sample_freq <= 0.0 {
            self.alpha = 1.0;
        } else {
            self.alpha = calc_lowpass_alpha_dt(1.0 / sample_freq, self.cutoff_freq);
        }
    }

    /// The cutoff frequency, in Hz.
    #[inline]
    pub fn get_cutoff_freq(&self) -> f32 {
        self.cutoff_freq
    }

    /// Apply one sample. Upstream `apply(sample)`.
    #[inline]
    pub fn apply(&mut self, sample: T) -> T {
        self.lpf.apply_alpha(sample, self.alpha)
    }

    /// The latest filtered value.
    #[inline]
    pub fn get(&self) -> T {
        self.lpf.get()
    }

    /// Seed the filter to `value`.
    #[inline]
    pub fn reset_to(&mut self, value: T) {
        self.lpf.reset_to(value);
    }

    /// Mark the filter unseeded.
    #[inline]
    pub fn reset(&mut self) {
        self.lpf.reset();
    }
}

/// Upstream `LowPassFilterFloat`.
pub type LowPassFilterFloat = LowPassFilter<f32>;
/// Upstream `LowPassFilterVector2f`.
pub type LowPassFilterVector2f = LowPassFilter<Vector2f>;
/// Upstream `LowPassFilterVector3f`.
pub type LowPassFilterVector3f = LowPassFilter<Vector3f>;
/// Upstream `LowPassFilterConstDtFloat`.
pub type LowPassFilterConstDtFloat = LowPassFilterConstDt<f32>;
/// Upstream `LowPassFilterConstDtVector2f`.
pub type LowPassFilterConstDtVector2f = LowPassFilterConstDt<Vector2f>;
/// Upstream `LowPassFilterConstDtVector3f`.
pub type LowPassFilterConstDtVector3f = LowPassFilterConstDt<Vector3f>;

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // Upstream ships no unit test for LowPassFilter (tests/ has only
    // averagefilter, modefilter and notchfilter), so these are PORT-DERIVED
    // from reading LowPassFilter.cpp.

    fn near(a: f32, b: f32) {
        assert!((a - b).abs() < 1.0e-6, "expected {b}, got {a}");
    }

    /// The first sample passes through unchanged, seeding the filter.
    #[test]
    fn first_sample_passes_through() {
        let mut f = LowPassFilterFloat::new(1.0);
        near(f.apply(10.0, 0.1), 10.0);
        near(f.get(), 10.0);
    }

    /// DIVERGENCE D-005, pinned.
    ///
    /// UPSTREAM: `DigitalLPF` leaves `bool initialised` uninitialised
    /// (LowPassFilter.h:72, constructor at LowPassFilter.cpp:17). Reading it is
    /// undefined behavior; if the garbage byte is nonzero the filter skips
    /// first-sample seeding and ramps from zero instead.
    /// PORTED: deterministic - a fresh filter is always unseeded.
    ///
    /// Do not "restore parity" here; there is no defined behavior to restore.
    #[test]
    fn d005_fresh_filter_is_deterministically_unseeded() {
        let f = DigitalLpf::<f32>::new();
        assert!(!f.is_initialised());
        assert_eq!(f.get(), 0.0);

        // Seeding is observable: the first sample is adopted exactly, never
        // blended toward from zero.
        let mut g = LowPassFilterConstDtFloat::new(100.0, 10.0);
        near(g.apply(5.0), 5.0);

        // and again after an explicit reset
        g.reset();
        assert!(!g.lpf.is_initialised());
        near(g.apply(-3.0), -3.0);
    }

    /// After seeding, output moves toward the sample by alpha each step.
    #[test]
    fn converges_toward_a_step_input() {
        let mut f = LowPassFilterFloat::new(1.0);
        f.reset_to(0.0);
        let mut last = 0.0;
        for _ in 0..200 {
            last = f.apply(1.0, 0.01);
        }
        assert!(last > 0.99, "should approach the step, got {last}");
        assert!(last <= 1.0, "must not overshoot a step, got {last}");
    }

    /// alpha semantics from calc_lowpass_alpha_dt, exercised through the filter.
    #[test]
    fn alpha_edge_cases_match_upstream() {
        // zero cutoff means alpha 1: output follows input exactly
        let mut f = LowPassFilterFloat::new(0.0);
        f.reset_to(0.0);
        near(f.apply(7.0, 0.01), 7.0);

        // zero dt means alpha 0: output holds
        let mut g = LowPassFilterFloat::new(5.0);
        g.reset_to(2.0);
        near(g.apply(100.0, 0.0), 2.0);

        // non-positive sample rate forces alpha 1 in the const-dt filter
        let mut h = LowPassFilterConstDtFloat::new(0.0, 10.0);
        h.reset_to(0.0);
        near(h.apply(4.0), 4.0);
    }

    /// reset_to seeds without passing a sample through.
    #[test]
    fn reset_to_seeds_the_filter() {
        let mut f = LowPassFilterConstDtFloat::new(100.0, 10.0);
        f.reset_to(42.0);
        near(f.get(), 42.0);
        // already seeded, so this sample is filtered rather than adopted
        let out = f.apply(0.0);
        assert!(out < 42.0 && out > 0.0, "expected a blend, got {out}");
    }

    /// The vector instantiations filter componentwise.
    #[test]
    fn vector_filters_work_componentwise() {
        let mut f = LowPassFilterVector3f::new(1.0);
        let first = f.apply(Vector3f::new(1.0, 2.0, 3.0), 0.1);
        assert_eq!(first, Vector3f::new(1.0, 2.0, 3.0));

        let mut g = LowPassFilterVector2f::new(1.0);
        let v = g.apply(Vector2f::new(4.0, 5.0), 0.1);
        assert_eq!(v, Vector2f::new(4.0, 5.0));
    }
}
