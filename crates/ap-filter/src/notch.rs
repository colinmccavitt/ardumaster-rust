//! Band-stop filter, upstream `Filter/NotchFilter`.
//!
//! A low pass cannot help with motor and propeller noise. That noise sits at a
//! frequency proportional to RPM, often well inside the band the control loops
//! need, so attenuating it with a low pass would attenuate the control response
//! with it. A notch removes a narrow band and leaves the rest alone, and it
//! tracks the RPM so the band follows the noise.
//!
//! # Attenuation and quality, from bandwidth
//!
//! [`NotchFilter::calculate_a_and_q`] converts a human-facing description --
//! centre frequency, bandwidth, attenuation in dB -- into the two numbers the
//! biquad wants. `A` is the linear amplitude ratio, `10^(-dB/40)`, a *quarter*
//! power rather than the usual half because it is applied squared in the
//! coefficients. `Q` comes from the bandwidth expressed in octaves.
//!
//! # Retuning is rate limited
//!
//! The centre frequency can move by at most five percent per update. A notch
//! whose centre jumped would step the filtered signal, and the thing driving
//! the centre is an RPM estimate that can be noisy or briefly wrong. Slewing
//! means a bad estimate perturbs the notch rather than throwing it across the
//! spectrum.

use ap_math::scalar::{constrain_value, is_equal, is_positive, is_zero, sq, Real};

use crate::lowpass::Filterable;

/// The most the centre frequency may move in one update, as a fraction.
/// Upstream `NOTCH_MAX_SLEW`.
pub const NOTCH_MAX_SLEW: f32 = 0.05;

/// Lower slew bound, upstream `NOTCH_MAX_SLEW_LOWER`.
pub const NOTCH_MAX_SLEW_LOWER: f32 = 1.0 - NOTCH_MAX_SLEW;

/// Upper slew bound, upstream `NOTCH_MAX_SLEW_UPPER`.
///
/// The reciprocal of the lower bound rather than `1 + NOTCH_MAX_SLEW`, so a
/// slew down followed by a slew up returns exactly where it started.
pub const NOTCH_MAX_SLEW_UPPER: f32 = 1.0 / NOTCH_MAX_SLEW_LOWER;

/// A single band-stop biquad, upstream `NotchFilter<T>`.
///
/// DIVERGENCE D-005, third occurrence: upstream declares this class with **no
/// constructor at all**, so every member of a non-static instance is
/// indeterminate -- `initialised` and `need_reset`, all five coefficients, all
/// four state vectors and the three cached frequencies. Rust cannot express
/// that; the port is deterministic by construction and matches the
/// zero-initialised case that statically stored instances get.
#[derive(Debug, Clone, Copy)]
pub struct NotchFilter<T> {
    initialised: bool,
    need_reset: bool,

    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,

    center_freq_hz: f32,
    sample_freq_hz: f32,
    a_gain: f32,

    ntchsig1: T,
    ntchsig2: T,
    signal1: T,
    signal2: T,
}

impl<T: Filterable> Default for NotchFilter<T> {
    /// See D-005 on the struct: every one of these is a field upstream leaves
    /// indeterminate.
    fn default() -> Self {
        Self {
            initialised: false,
            need_reset: false,
            b0: 0.0,
            b1: 0.0,
            b2: 0.0,
            a1: 0.0,
            a2: 0.0,
            center_freq_hz: 0.0,
            sample_freq_hz: 0.0,
            a_gain: 0.0,
            ntchsig1: T::zero(),
            ntchsig2: T::zero(),
            signal1: T::zero(),
            signal2: T::zero(),
        }
    }
}

impl<T: Filterable> NotchFilter<T> {
    /// A fresh, uninitialised notch. Passes samples through until
    /// [`NotchFilter::init`] succeeds.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Attenuation ratio and quality factor for a band description, upstream
    /// `calculate_A_and_Q`.
    ///
    /// Returns `Q = 0` when the centre frequency is not above half the
    /// bandwidth, which [`NotchFilter::init_with_a_and_q`] then rejects. That
    /// is the degenerate case where the requested band would extend down
    /// through zero.
    #[must_use]
    pub fn calculate_a_and_q(
        center_freq_hz: f32,
        bandwidth_hz: f32,
        attenuation_db: f32,
    ) -> (f32, f32) {
        let a = Real::powf(10.0_f32, -attenuation_db / 40.0);
        if center_freq_hz > 0.5 * bandwidth_hz {
            let octaves = Real::log2(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0)) * 2.0;
            let two_oct = Real::powf(2.0_f32, octaves);
            (a, Real::sqrt(two_oct) / (two_oct - 1.0))
        } else {
            (a, 0.0)
        }
    }

    /// Configure from a band description, upstream `init`.
    ///
    /// Note this clears `initialised` **before** testing the guard, so a
    /// rejected configuration leaves the filter passing samples through --
    /// and so the early-return in [`NotchFilter::init_with_a_and_q`] can never
    /// fire from this path.
    pub fn init(
        &mut self,
        sample_freq_hz: f32,
        center_freq_hz: f32,
        bandwidth_hz: f32,
        attenuation_db: f32,
    ) {
        self.initialised = false;
        if center_freq_hz > 0.5 * bandwidth_hz && center_freq_hz < 0.5 * sample_freq_hz {
            let (a, q) = Self::calculate_a_and_q(center_freq_hz, bandwidth_hz, attenuation_db);
            self.init_with_a_and_q(sample_freq_hz, center_freq_hz, a, q);
        }
    }

    /// Configure from attenuation and quality directly, upstream
    /// `init_with_A_and_Q`.
    ///
    /// Does nothing when the configuration is unchanged -- this runs every
    /// loop as the notch tracks RPM, and recomputing four transcendentals to
    /// arrive at the same coefficients is wasted work.
    ///
    /// A rejected configuration clears `initialised` and **leaves the cached
    /// centre frequency alone**, so the next acceptable update slews from
    /// where the notch last was rather than from nothing.
    pub fn init_with_a_and_q(&mut self, sample_freq_hz: f32, center_freq_hz: f32, a: f32, q: f32) {
        if self.initialised
            && is_equal(center_freq_hz, self.center_freq_hz)
            && is_equal(sample_freq_hz, self.sample_freq_hz)
            && is_equal(a, self.a_gain)
        {
            return;
        }

        let mut new_center_freq = center_freq_hz;

        // Rate limit, but not on the first update and not while a reset is
        // pending -- there is no previous centre to slew from in either case.
        if self.initialised && !self.need_reset && !is_zero(self.center_freq_hz) {
            new_center_freq = constrain_value(
                new_center_freq,
                self.center_freq_hz * NOTCH_MAX_SLEW_LOWER,
                self.center_freq_hz * NOTCH_MAX_SLEW_UPPER,
            );
        }

        if is_positive(new_center_freq) && new_center_freq < 0.5 * sample_freq_hz && q > 0.0 {
            let omega = 2.0 * core::f32::consts::PI * new_center_freq / sample_freq_hz;
            let alpha = Real::sin(omega) / (2.0 * q);

            self.b0 = 1.0 + alpha * sq(a);
            self.b1 = -2.0 * Real::cos(omega);
            self.b2 = 1.0 - alpha * sq(a);
            self.a1 = self.b1;
            self.a2 = 1.0 - alpha;

            // Pre-divided by a0 so `apply` is five multiplies and no division.
            let a0_inv = 1.0 / (1.0 + alpha);
            self.b0 *= a0_inv;
            self.b1 *= a0_inv;
            self.b2 *= a0_inv;
            self.a1 *= a0_inv;
            self.a2 *= a0_inv;

            self.center_freq_hz = new_center_freq;
            self.sample_freq_hz = sample_freq_hz;
            self.a_gain = a;
            self.initialised = true;
        } else {
            // Leave center_freq_hz at its last value, deliberately.
            self.initialised = false;
        }
    }

    /// Filter one sample, upstream `apply`.
    ///
    /// An unconfigured or just-reset filter returns the sample and seeds all
    /// four delay elements with it, so the first filtered output starts from
    /// the signal rather than from zero.
    pub fn apply(&mut self, sample: T) -> T {
        if !self.initialised || self.need_reset {
            self.signal1 = sample;
            self.signal2 = sample;
            self.ntchsig1 = sample;
            self.ntchsig2 = sample;
            self.need_reset = false;
            return sample;
        }

        let output = sample
            .scale(self.b0)
            .add(self.ntchsig1.scale(self.b1))
            .add(self.ntchsig2.scale(self.b2))
            .sub(self.signal1.scale(self.a1))
            .sub(self.signal2.scale(self.a2));

        self.ntchsig2 = self.ntchsig1;
        self.ntchsig1 = sample;

        self.signal2 = self.signal1;
        self.signal1 = output;
        output
    }

    /// Re-seed on the next sample, upstream `reset`.
    ///
    /// Deferred rather than immediate: the filter does not know the value to
    /// seed with until that sample arrives.
    pub fn reset(&mut self) {
        self.need_reset = true;
    }

    /// Stop filtering, upstream `disable`. Samples pass through until the
    /// next successful init.
    pub fn disable(&mut self) {
        self.initialised = false;
    }

    /// The centre frequency in use, upstream `center_freq_hz()`.
    #[must_use]
    pub const fn center_freq(&self) -> f32 {
        self.center_freq_hz
    }

    /// The sample rate the coefficients were computed for, upstream
    /// `sample_freq_hz()`.
    #[must_use]
    pub const fn sample_freq(&self) -> f32 {
        self.sample_freq_hz
    }

    /// Whether the filter is configured and filtering.
    ///
    /// Not an upstream interface. It exists because `initialised` is one of
    /// the members upstream never initialises -- see D-005.
    #[must_use]
    pub const fn is_initialised(&self) -> bool {
        self.initialised
    }

    /// The five coefficients, in upstream's order.
    #[must_use]
    pub const fn coefficients(&self) -> (f32, f32, f32, f32, f32) {
        (self.b0, self.b1, self.b2, self.a1, self.a2)
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "these compare for exact equality on purpose: a sample passed \
through untouched, or a cached frequency left exactly as it was by a rejected \
configuration. An epsilon would admit the changes these tests rule out"
    )]

    use super::*;

    /// D-005, third occurrence. Upstream declares `NotchFilter` with no
    /// constructor at all, so every member of a non-static instance is
    /// indeterminate.
    #[test]
    fn d005_a_fresh_notch_is_deterministically_unconfigured() {
        let f: NotchFilter<f32> = NotchFilter::new();
        assert!(!f.is_initialised());
        assert_eq!(f.center_freq(), 0.0);
        assert_eq!(f.coefficients(), (0.0, 0.0, 0.0, 0.0, 0.0));
    }

    /// Unconfigured, it passes everything through.
    #[test]
    fn an_unconfigured_notch_passes_through() {
        let mut f = NotchFilter::<f32>::new();
        assert_eq!(f.apply(3.0), 3.0);
        assert_eq!(f.apply(-7.5), -7.5);
    }

    /// The point of the thing: energy at the centre frequency is attenuated
    /// and energy well away from it is not.
    #[test]
    fn it_attenuates_at_the_centre_and_not_elsewhere() {
        let fs = 1000.0_f32;
        let response = |tone_hz: f32| -> f32 {
            let mut f = NotchFilter::<f32>::new();
            f.init(fs, 100.0, 40.0, 30.0);
            assert!(f.is_initialised());
            let mut peak = 0.0_f32;
            for i in 0..2000 {
                let t = i as f32 / fs;
                let out = f.apply(Real::sin(2.0 * core::f32::consts::PI * tone_hz * t));
                // ignore the settling transient
                if i > 1000 {
                    peak = peak.max(out.abs());
                }
            }
            peak
        };

        let at_centre = response(100.0);
        let well_below = response(10.0);
        let well_above = response(400.0);

        assert!(
            at_centre < 0.1,
            "a 100 Hz tone should be attenuated hard, got {at_centre}"
        );
        assert!(
            well_below > 0.9,
            "10 Hz should pass essentially untouched, got {well_below}"
        );
        assert!(
            well_above > 0.9,
            "400 Hz should pass essentially untouched, got {well_above}"
        );
    }

    /// Retuning is rate limited to five percent, so a wild RPM estimate
    /// perturbs the notch rather than throwing it across the spectrum.
    #[test]
    fn the_centre_frequency_slews_at_five_percent() {
        let mut f = NotchFilter::<f32>::new();
        f.init(1000.0, 100.0, 40.0, 30.0);
        assert_eq!(f.center_freq(), 100.0);

        let (a, q) = NotchFilter::<f32>::calculate_a_and_q(100.0, 40.0, 30.0);
        // Ask for 400 Hz; it may only reach 100/0.95.
        f.init_with_a_and_q(1000.0, 400.0, a, q);
        let expected_up = 100.0 / NOTCH_MAX_SLEW_LOWER;
        assert!(
            (f.center_freq() - expected_up).abs() < 1e-3,
            "expected a slew to {expected_up}, got {}",
            f.center_freq()
        );

        // And down.
        let before = f.center_freq();
        f.init_with_a_and_q(1000.0, 1.0, a, q);
        assert!((f.center_freq() - before * NOTCH_MAX_SLEW_LOWER).abs() < 1e-3);
    }

    /// The slew bounds are reciprocal, so down-then-up returns exactly where
    /// it started rather than drifting.
    #[test]
    fn the_slew_bounds_are_reciprocal() {
        assert!((NOTCH_MAX_SLEW_LOWER * NOTCH_MAX_SLEW_UPPER - 1.0).abs() < 1e-7);
    }

    /// A repeat of the same configuration is skipped, which is what keeps a
    /// per-loop retune from recomputing four transcendentals for nothing.
    #[test]
    fn an_unchanged_configuration_is_skipped() {
        let mut f = NotchFilter::<f32>::new();
        f.init(1000.0, 100.0, 40.0, 30.0);
        let before = f.coefficients();
        let (a, q) = NotchFilter::<f32>::calculate_a_and_q(100.0, 40.0, 30.0);
        f.init_with_a_and_q(1000.0, 100.0, a, q);
        assert_eq!(f.coefficients(), before);
    }

    /// A rejected configuration disables the filter but keeps the cached
    /// centre, so the next acceptable update slews from where the notch was.
    #[test]
    fn a_rejected_configuration_keeps_the_cached_centre() {
        let mut f = NotchFilter::<f32>::new();
        f.init(1000.0, 100.0, 40.0, 30.0);
        assert!(f.is_initialised());

        // Above half the sample rate: rejected.
        f.init(1000.0, 600.0, 40.0, 30.0);
        assert!(!f.is_initialised());
        assert_eq!(f.center_freq(), 100.0, "the cached centre should survive");
        assert_eq!(f.apply(2.5), 2.5, "and it should pass through meanwhile");
    }

    /// A bandwidth wide enough to reach zero is degenerate, and is rejected
    /// through `Q = 0`.
    #[test]
    fn a_bandwidth_reaching_zero_is_rejected() {
        let (_, q) = NotchFilter::<f32>::calculate_a_and_q(20.0, 40.0, 30.0);
        assert_eq!(q, 0.0);
        let mut f = NotchFilter::<f32>::new();
        f.init(1000.0, 20.0, 40.0, 30.0);
        assert!(!f.is_initialised());
    }

    /// Reset is deferred to the next sample, because the value to seed with
    /// is not known until then.
    ///
    /// The follow-up sample has to differ from the seed. A notch passes DC
    /// exactly, so feeding the seed value again would return it unchanged --
    /// which is the filter working, not a failed reseed.
    #[test]
    fn reset_reseeds_on_the_next_sample() {
        let mut f = NotchFilter::<f32>::new();
        f.init(1000.0, 100.0, 40.0, 30.0);
        for _ in 0..50 {
            f.apply(1.0);
        }
        f.reset();
        assert_eq!(f.apply(9.0), 9.0, "the seeding sample passes through");
        assert_ne!(
            f.apply(0.0),
            0.0,
            "and then it filters again, carrying the seeded state"
        );
    }

    /// A notch passes DC with gain exactly one. Summing the coefficients,
    /// `b0 + b1 + b2 - a1 - a2 = (1 + alpha)/a0 = 1`, so a constant signal
    /// comes through untouched however the notch is tuned.
    ///
    /// This is what makes the seeding in `apply` correct: seeding all four
    /// delay elements with the sample puts the filter at a genuine steady
    /// state rather than an approximate one.
    #[test]
    fn a_notch_passes_dc_exactly() {
        for (fs, fc, bw) in [
            (1000.0_f32, 100.0_f32, 40.0_f32),
            (8000.0, 188.0, 60.0),
            (400.0, 80.0, 20.0),
        ] {
            let mut f = NotchFilter::<f32>::new();
            f.init(fs, fc, bw, 30.0);
            assert!(f.is_initialised());
            let (b0, b1, b2, a1, a2) = f.coefficients();
            let dc = b0 + b1 + b2 - a1 - a2;
            assert!((dc - 1.0).abs() < 1e-5, "fs {fs} fc {fc}: DC gain {dc}");

            // Bring it into service the way upstream does -- see
            // `a_freshly_configured_notch_rings_until_reset` below.
            f.reset();
            let mut worst: f32 = 0.0;
            for _ in 0..2000 {
                worst = worst.max((f.apply(4.25) - 4.25).abs());
            }
            assert!(
                worst < 1.0e-4,
                "fs {fs} fc {fc}: a notch should pass DC untouched, worst {worst:e}"
            );
        }
    }

    /// A notch configured but not reset starts from zero state and rings.
    ///
    /// `apply` seeds its delay elements only when `!initialised ||
    /// need_reset`, and a successful `init` leaves `initialised` true with
    /// `need_reset` false — so the first sample is filtered against zeros.
    ///
    /// This is why `AP_InertialSensor_Backend::apply_gyro_filters` resets
    /// every inactive notch: "while inactive we reset the filter so when it
    /// activates the first output will be the first input sample". Anything
    /// bringing a notch into service without that reset gets this transient,
    /// on the gyro signal, in flight.
    #[test]
    fn a_freshly_configured_notch_rings_until_reset() {
        let settle = |reset_first: bool| -> f32 {
            let mut f = NotchFilter::<f32>::new();
            f.init(1000.0, 100.0, 40.0, 30.0);
            if reset_first {
                f.reset();
            }
            let mut worst: f32 = 0.0;
            for _ in 0..500 {
                worst = worst.max((f.apply(4.25) - 4.25).abs());
            }
            worst
        };

        let without = settle(false);
        let with = settle(true);
        assert!(
            without > 1.0,
            "an unreset notch should ring hard on a constant input, got {without}"
        );
        assert!(
            with < 1.0e-4,
            "and resetting first should remove it entirely, got {with}"
        );
    }

    /// Attenuation is a quarter-power ratio, because it is applied squared in
    /// the coefficients.
    #[test]
    fn attenuation_is_a_quarter_power_ratio() {
        let (a, _) = NotchFilter::<f32>::calculate_a_and_q(100.0, 40.0, 40.0);
        // 10^(-40/40) = 0.1
        assert!((a - 0.1).abs() < 1e-6, "got {a}");
    }
}
