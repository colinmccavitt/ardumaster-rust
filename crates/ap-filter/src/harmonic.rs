//! A bank of notches tracking a fundamental and its harmonics, upstream
//! `Filter/HarmonicNotchFilter`.
//!
//! One notch removes one frequency. Motor noise is not one frequency: it is a
//! fundamental at the rotation rate plus harmonics at multiples of it, and on a
//! multi-motor airframe several fundamentals at once. This places a notch at
//! each, tracks them all as RPM changes, and runs a sample through the lot.
//!
//! # Composite notches
//!
//! A single deep notch is narrow, so it has to track the noise accurately. Two
//! or three shallower notches spread either side of the centre cover a wider
//! band for the same total attenuation, which tolerates a worse RPM estimate.
//! The spread is `bandwidth / (32 * centre)`, applied as a multiplier so the
//! arrangement stays symmetric in the log-frequency sense.
//!
//! # No allocator
//!
//! Upstream heap-allocates the bank and grows it with `expand_filter_count`.
//! ADR-0004 rules out an allocator, so the bank is a fixed array sized by
//! `HAL_HNF_MAX_FILTERS` -- the same bound upstream clamps to anyway, so no
//! configuration that works upstream is refused here. `expand_filter_count`
//! has no analogue and needs none: the capacity is always the maximum.
//!
//! # What this does not include
//!
//! Parameter binding (the `HarmonicNotchFilterParams` object is an AP_Param
//! group -- FW-004), logging of notch centres, and the dynamic-harmonic and
//! per-motor RPM sources that decide *what* centre frequencies to pass in.
//! Those live in `AP_InertialSensor` and the ESC/FFT backends.

use ap_math::scalar::{constrain_value, is_zero, linear_interpolate};

use crate::lowpass::Filterable;
use crate::notch::NotchFilter;

/// Highest harmonic index the bitmask can address, upstream
/// `HNF_MAX_HARMONICS`.
pub const HNF_MAX_HARMONICS: usize = 16;

/// Bank capacity, upstream `HAL_HNF_MAX_FILTERS`.
///
/// 54 on SITL, H7 and Linux. F7 boards get 27 and everything else 24; this
/// port targets SITL, and the value is the *cap* upstream applies rather than
/// an allocation size, so a smaller board simply enables fewer notches.
pub const HAL_HNF_MAX_FILTERS: usize = 54;

/// Notches above this fraction of the sample rate are disabled, upstream
/// `HARMONIC_NYQUIST_CUTOFF`.
pub const HARMONIC_NYQUIST_CUTOFF: f32 = 0.48;

/// Below this fraction of the minimum frequency a notch is disabled outright,
/// upstream `NOTCHFILTER_ATTENUATION_CUTOFF`.
pub const NOTCHFILTER_ATTENUATION_CUTOFF: f32 = 0.25;

/// Where the centre frequency comes from, upstream
/// `HarmonicNotchDynamicMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TrackingMode {
    /// A static notch at the configured frequency.
    #[default]
    Fixed = 0,
    /// Scaled by throttle.
    UpdateThrottle = 1,
    /// From an RPM sensor.
    UpdateRpm = 2,
    /// From BLHeli ESC telemetry.
    UpdateBlHeli = 3,
    /// From the gyro FFT.
    UpdateGyroFft = 4,
    /// From the second RPM sensor.
    UpdateRpm2 = 5,
}

/// How many notches make up each composite, upstream
/// `HarmonicNotchFilterParams::num_composite_notches`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum CompositeNotches {
    /// One notch per harmonic.
    #[default]
    Single,
    /// Two, spread either side of the centre.
    Double,
    /// Three: one at the centre and one either side.
    Triple,
    /// Five: one at the centre, two either side at one and two spreads.
    ///
    /// DIVERGENCE D-021: upstream accepts this option and then delivers a
    /// triple notch.
    Quintuple,
}

impl CompositeNotches {
    /// The number of notches, upstream's `num_composite_notches` return.
    #[must_use]
    pub const fn count(self) -> u8 {
        match self {
            Self::Single => 1,
            Self::Double => 2,
            Self::Triple => 3,
            Self::Quintuple => 5,
        }
    }
}

/// The configuration a harmonic notch needs, upstream
/// `HarmonicNotchFilterParams`.
///
/// A plain struct here. Upstream's is an `AP_Param` group, and binding it to
/// storage is FW-004.
#[derive(Debug, Clone, Copy)]
pub struct HarmonicNotchParams {
    /// Fundamental centre frequency, Hz.
    pub center_freq_hz: f32,
    /// Width of the band each composite covers, Hz.
    pub bandwidth_hz: f32,
    /// Attenuation at the centre, dB.
    pub attenuation_db: f32,
    /// The lowest fraction of the configured centre a notch will track down
    /// to before it is faded out.
    pub freq_min_ratio: f32,
    /// Bitmask of harmonics to place notches at; bit 0 is the fundamental.
    pub harmonics: u32,
    /// How many notches make up each composite.
    pub composite_notches: CompositeNotches,
    /// Where the centre frequency comes from.
    pub tracking_mode: TrackingMode,
    /// Treat a very low frequency as the minimum rather than disabling the
    /// notch, upstream `Options::TreatLowAsMin`.
    pub treat_low_as_min: bool,
}

impl Default for HarmonicNotchParams {
    fn default() -> Self {
        Self {
            center_freq_hz: 80.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
            freq_min_ratio: 1.0,
            harmonics: 1,
            composite_notches: CompositeNotches::Single,
            tracking_mode: TrackingMode::Fixed,
            treat_low_as_min: false,
        }
    }
}

/// A bank of notch filters tracking a fundamental and its harmonics.
#[derive(Debug, Clone, Copy)]
pub struct HarmonicNotchFilter<T, const N: usize = HAL_HNF_MAX_FILTERS> {
    filters: [NotchFilter<T>; N],

    initialised: bool,
    sample_freq_hz: f32,
    notch_spread: f32,
    a_gain: f32,
    q_factor: f32,
    minimum_freq: f32,

    harmonics: u32,
    num_harmonics: u32,
    composite_notches: u8,

    /// Capacity actually in use, upstream `_num_filters`.
    num_filters: usize,
    /// How many of those are configured this update, upstream
    /// `_num_enabled_filters`.
    num_enabled_filters: usize,

    params: HarmonicNotchParams,
}

impl<T: Filterable, const N: usize> Default for HarmonicNotchFilter<T, N> {
    fn default() -> Self {
        Self {
            filters: [NotchFilter::default(); N],
            initialised: false,
            sample_freq_hz: 0.0,
            notch_spread: 0.0,
            a_gain: 0.0,
            q_factor: 0.0,
            minimum_freq: 0.0,
            harmonics: 0,
            num_harmonics: 0,
            composite_notches: 0,
            num_filters: 0,
            num_enabled_filters: 0,
            params: HarmonicNotchParams::default(),
        }
    }
}

impl<T: Filterable, const N: usize> HarmonicNotchFilter<T, N> {
    /// An empty bank.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Reserve the notches this configuration needs, upstream
    /// `allocate_filters`.
    ///
    /// `num_notches` is the number of independent centre frequencies -- one
    /// per motor when tracking ESC telemetry.
    ///
    /// DIVERGENCE D-021: upstream clamps the composite count to three here,
    /// which silently turns a requested quintuple notch into a triple and
    /// leaves the code that would place the extra pair unreachable. This
    /// clamps to five.
    pub fn allocate_filters(
        &mut self,
        num_notches: u8,
        harmonics: u32,
        composite_notches: CompositeNotches,
    ) {
        self.composite_notches = composite_notches.count().min(5);
        self.num_harmonics = harmonics.count_ones();
        self.harmonics = harmonics;
        self.num_filters = (self.num_harmonics as usize)
            .saturating_mul(num_notches as usize)
            .saturating_mul(self.composite_notches as usize)
            .min(HAL_HNF_MAX_FILTERS)
            .min(N);
    }

    /// Compute the shaping constants for a sample rate, upstream `init`.
    ///
    /// Does nothing if no notches were reserved or the sample rate is unusable,
    /// which leaves the bank passing samples through.
    pub fn init(&mut self, sample_freq_hz: f32, params: HarmonicNotchParams) {
        self.params = params;

        if self.num_filters == 0 || is_zero(sample_freq_hz) || sample_freq_hz.is_nan() {
            return;
        }

        self.sample_freq_hz = sample_freq_hz;

        let bandwidth_hz = params.bandwidth_hz;
        let mut center_freq_hz = params.center_freq_hz;

        let nyquist_limit = sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;
        let bandwidth_limit = bandwidth_hz * 0.52;

        // The lowest frequency any notch will still be enabled at.
        self.minimum_freq = center_freq_hz * params.freq_min_ratio;

        center_freq_hz = constrain_value(center_freq_hz, bandwidth_limit, nyquist_limit);

        // Spread needed for two notches of half the bandwidth to match one
        // notch of the full bandwidth.
        self.notch_spread = bandwidth_hz / (32.0 * center_freq_hz);

        let (a, q) = NotchFilter::<T>::calculate_a_and_q(
            center_freq_hz,
            bandwidth_hz / f32::from(self.composite_notches),
            params.attenuation_db,
        );
        self.a_gain = a;
        self.q_factor = q;

        self.initialised = true;

        // A static notch has nothing to track, so place it now.
        if params.tracking_mode == TrackingMode::Fixed {
            self.update(center_freq_hz);
        }
    }

    /// Retune every notch from a single fundamental, upstream `update`.
    pub fn update(&mut self, center_freq_hz: f32) {
        self.update_multi(&[center_freq_hz]);
    }

    /// Retune from several independent fundamentals -- one per motor, when
    /// tracking ESC telemetry. Upstream `update(num_centers, centers[])`.
    ///
    /// Notches are ordered centre-major within each harmonic: `f1h1, f2h1,
    /// f3h1, f1h2, f2h2, ...`.
    pub fn update_multi(&mut self, centers: &[f32]) {
        if !self.initialised || centers.is_empty() {
            return;
        }

        let nyquist_limit = self.sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;
        let num_centers = centers.len();

        self.num_enabled_filters = 0;

        for i in 0..num_centers * HNF_MAX_HARMONICS {
            if self.num_enabled_filters >= self.num_filters {
                break;
            }
            let harmonic_n = i / num_centers;
            let center_n = i % num_centers;

            if (1_u32 << harmonic_n) & self.harmonics == 0 {
                continue;
            }

            let Some(&center) = centers.get(center_n) else {
                continue;
            };
            let notch_center = constrain_value(center, 0.0, nyquist_limit);
            #[allow(
                clippy::cast_possible_truncation,
                reason = "harmonic_n is bounded by HNF_MAX_HARMONICS = 16"
            )]
            let harmonic_mul = (harmonic_n + 1) as u8;

            // Upstream's shape exactly: the centre notch is placed for every
            // composite count except two, then pairs are added outward.
            if self.composite_notches != 2 {
                self.place(notch_center, 1.0, harmonic_mul);
            }
            if self.composite_notches > 1 {
                self.place(notch_center, 1.0 - self.notch_spread, harmonic_mul);
                self.place(notch_center, 1.0 + self.notch_spread, harmonic_mul);
            }
            if self.composite_notches > 3 {
                self.place(notch_center, 1.0 - 2.0 * self.notch_spread, harmonic_mul);
                self.place(notch_center, 1.0 + 2.0 * self.notch_spread, harmonic_mul);
            }
        }
    }

    /// Configure the next notch in the bank and advance the counter.
    ///
    /// Bounds-checked, which upstream's is not: its loop guard is tested once
    /// per harmonic but the counter advances up to five times inside, so a
    /// bank whose capacity is not a multiple of the composite count can be
    /// written past the end. Unreachable on SITL, where the cap of 54 divides
    /// by every composite count -- but not on an F7 board, whose 27 does not
    /// divide by two.
    fn place(&mut self, notch_center: f32, spread_mul: f32, harmonic_mul: u8) {
        let idx = self.num_enabled_filters;
        if idx >= self.num_filters || idx >= N {
            return;
        }
        self.num_enabled_filters += 1;
        self.set_center_frequency(idx, notch_center, spread_mul, harmonic_mul);
    }

    /// Point one notch at a frequency, upstream `set_center_frequency`.
    fn set_center_frequency(
        &mut self,
        idx: usize,
        notch_center: f32,
        spread_mul: f32,
        harmonic_mul: u8,
    ) {
        if self.filters.get(idx).is_none() {
            return;
        }
        let nyquist_limit = self.sample_freq_hz * HARMONIC_NYQUIST_CUTOFF;
        let mut notch_center = notch_center * f32::from(harmonic_mul);

        // Above Nyquist there is nothing meaningful to remove.
        //
        // Upstream tests `notch_center` rather than `notch_center *
        // spread_mul`, and says so in a comment: the upper notch of a double
        // or triple can therefore sit above Nyquist. Reproduced, comment and
        // all -- it is a documented uncertainty upstream, not an oversight,
        // and changing it would move notches on a real airframe.
        if notch_center >= nyquist_limit {
            if let Some(f) = self.filters.get_mut(idx) {
                f.disable();
            }
            return;
        }

        let mut harmonic_min_freq = self.minimum_freq;
        let mut a = self.a_gain;

        if self.params.treat_low_as_min {
            // Keep the harmonics spread out rather than collapsing them all
            // onto the minimum.
            harmonic_min_freq *= f32::from(harmonic_mul);
        } else {
            let disable_freq = harmonic_min_freq * NOTCHFILTER_ATTENUATION_CUTOFF;
            if notch_center < disable_freq {
                if let Some(f) = self.filters.get_mut(idx) {
                    f.disable();
                }
                return;
            }
            if notch_center < harmonic_min_freq {
                // Fade the attenuation out toward unity as the notch
                // approaches the disable point, so switching it off is not a
                // step change in the filtered signal.
                a = linear_interpolate(a, 1.0, notch_center, harmonic_min_freq, disable_freq);
            }
        }

        notch_center = notch_center.max(harmonic_min_freq);

        // Applied last, so the composite stays symmetric about the centre.
        notch_center *= spread_mul;

        let (sample_freq, q) = (self.sample_freq_hz, self.q_factor);
        if let Some(f) = self.filters.get_mut(idx) {
            f.init_with_a_and_q(sample_freq, notch_center, a, q);
        }
    }

    /// Run a sample through every enabled notch in turn, upstream `apply`.
    pub fn apply(&mut self, sample: T) -> T {
        if !self.initialised {
            return sample;
        }
        let mut output = sample;
        for f in self.filters.iter_mut().take(self.num_enabled_filters) {
            output = f.apply(output);
        }
        output
    }

    /// Re-seed every notch on its next sample, upstream `reset`.
    ///
    /// This is how a notch is brought into service without a transient -- see
    /// `notch::tests::a_freshly_configured_notch_rings_until_reset`.
    pub fn reset(&mut self) {
        if !self.initialised {
            return;
        }
        for f in self.filters.iter_mut().take(self.num_filters) {
            f.reset();
        }
    }

    /// Whether the bank has been configured, upstream `_initialised`.
    #[must_use]
    pub const fn is_initialised(&self) -> bool {
        self.initialised
    }

    /// How many notches are reserved, upstream `_num_filters`.
    #[must_use]
    pub const fn num_filters(&self) -> usize {
        self.num_filters
    }

    /// How many notches the last update configured, upstream
    /// `_num_enabled_filters`.
    #[must_use]
    pub const fn num_enabled_filters(&self) -> usize {
        self.num_enabled_filters
    }

    /// The centre frequency of one notch, for inspection and logging.
    #[must_use]
    pub fn notch_center(&self, idx: usize) -> Option<f32> {
        if idx >= self.num_enabled_filters {
            return None;
        }
        self.filters.get(idx).map(NotchFilter::center_freq)
    }

    /// Whether one notch is actually filtering, as opposed to disabled by the
    /// Nyquist or minimum-frequency guards.
    #[must_use]
    pub fn notch_active(&self, idx: usize) -> bool {
        idx < self.num_enabled_filters
            && self
                .filters
                .get(idx)
                .is_some_and(NotchFilter::is_initialised)
    }

    /// The spread multiplier the composite notches are placed at, upstream
    /// `_notch_spread`.
    #[must_use]
    pub const fn notch_spread(&self) -> f32 {
        self.notch_spread
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "these compare for exact equality on purpose: a sample passed \
through untouched, or a notch left exactly where it was"
    )]

    use super::*;
    use ap_math::scalar::Real;

    fn params(harmonics: u32, composite: CompositeNotches) -> HarmonicNotchParams {
        HarmonicNotchParams {
            center_freq_hz: 100.0,
            bandwidth_hz: 40.0,
            attenuation_db: 30.0,
            freq_min_ratio: 1.0,
            harmonics,
            composite_notches: composite,
            tracking_mode: TrackingMode::Fixed,
            treat_low_as_min: false,
        }
    }

    fn bank(harmonics: u32, composite: CompositeNotches) -> HarmonicNotchFilter<f32> {
        let mut h = HarmonicNotchFilter::<f32>::new();
        h.allocate_filters(1, harmonics, composite);
        h.init(1000.0, params(harmonics, composite));
        h.reset();
        h
    }

    /// Nothing allocated means nothing filtered.
    #[test]
    fn an_unallocated_bank_passes_through() {
        let mut h = HarmonicNotchFilter::<f32>::new();
        h.init(1000.0, params(1, CompositeNotches::Single));
        assert!(!h.is_initialised());
        assert_eq!(h.apply(3.0), 3.0);
    }

    /// The bitmask selects which harmonics get a notch, and the count follows
    /// from how many bits are set.
    #[test]
    fn the_harmonic_mask_decides_how_many_notches() {
        // fundamental only
        assert_eq!(bank(0b1, CompositeNotches::Single).num_enabled_filters(), 1);
        // fundamental and second harmonic
        assert_eq!(
            bank(0b11, CompositeNotches::Single).num_enabled_filters(),
            2
        );
        // fundamental, second, fourth
        assert_eq!(
            bank(0b1011, CompositeNotches::Single).num_enabled_filters(),
            3
        );
    }

    /// Harmonics land at multiples of the fundamental.
    #[test]
    fn harmonics_land_at_multiples_of_the_fundamental() {
        let h = bank(0b111, CompositeNotches::Single);
        assert_eq!(h.num_enabled_filters(), 3);
        assert!((h.notch_center(0).expect("f1") - 100.0).abs() < 0.5);
        assert!((h.notch_center(1).expect("f2") - 200.0).abs() < 0.5);
        assert!((h.notch_center(2).expect("f3") - 300.0).abs() < 0.5);
    }

    /// A composite places its notches symmetrically about the centre.
    #[test]
    fn a_double_notch_straddles_the_centre() {
        let h = bank(0b1, CompositeNotches::Double);
        assert_eq!(h.num_enabled_filters(), 2);
        let lower = h.notch_center(0).expect("lower");
        let upper = h.notch_center(1).expect("upper");
        assert!(lower < 100.0 && upper > 100.0, "{lower} {upper}");
        let spread = h.notch_spread();
        assert!((lower - 100.0 * (1.0 - spread)).abs() < 0.5);
        assert!((upper - 100.0 * (1.0 + spread)).abs() < 0.5);
    }

    /// A triple is a double plus one at the centre.
    #[test]
    fn a_triple_notch_adds_the_centre() {
        let h = bank(0b1, CompositeNotches::Triple);
        assert_eq!(h.num_enabled_filters(), 3);
        assert!((h.notch_center(0).expect("centre") - 100.0).abs() < 0.5);
    }

    /// D-021. Upstream clamps the composite count to three, so a requested
    /// quintuple notch silently becomes a triple -- and the branch in `update`
    /// that would place the outer pair is unreachable. The port delivers five.
    #[test]
    fn d021_a_quintuple_notch_places_five() {
        assert_eq!(CompositeNotches::Quintuple.count(), 5);
        let h = bank(0b1, CompositeNotches::Quintuple);
        assert_eq!(h.num_enabled_filters(), 5, "upstream would give three here");

        let spread = h.notch_spread();
        // centre, then +/- one spread, then +/- two
        assert!((h.notch_center(0).expect("centre") - 100.0).abs() < 0.5);
        assert!((h.notch_center(1).expect("inner low") - 100.0 * (1.0 - spread)).abs() < 0.5);
        assert!((h.notch_center(2).expect("inner high") - 100.0 * (1.0 + spread)).abs() < 0.5);
        assert!((h.notch_center(3).expect("outer low") - 100.0 * (1.0 - 2.0 * spread)).abs() < 0.5);
        assert!(
            (h.notch_center(4).expect("outer high") - 100.0 * (1.0 + 2.0 * spread)).abs() < 0.5
        );
    }

    /// The bank actually attenuates at the fundamental and at its harmonics,
    /// which is the whole point and is not implied by the notch placement
    /// tests above.
    #[test]
    fn the_bank_attenuates_the_fundamental_and_its_harmonics() {
        let response = |tone_hz: f32| -> f32 {
            let mut h = bank(0b11, CompositeNotches::Single);
            let mut peak = 0.0_f32;
            for i in 0..3000 {
                let t = i as f32 / 1000.0;
                let out = h.apply(Real::sin(2.0 * core::f32::consts::PI * tone_hz * t));
                if i > 1500 {
                    peak = peak.max(out.abs());
                }
            }
            peak
        };

        assert!(response(100.0) < 0.15, "fundamental: {}", response(100.0));
        assert!(response(200.0) < 0.15, "harmonic: {}", response(200.0));
        assert!(response(20.0) > 0.85, "well below: {}", response(20.0));
    }

    /// Harmonics above Nyquist are disabled rather than aliased down.
    #[test]
    fn harmonics_above_nyquist_are_disabled() {
        // 1000 Hz sample rate, cutoff 0.48 -> 480 Hz. With a 200 Hz
        // fundamental the third harmonic at 600 Hz is past it.
        let mut h = HarmonicNotchFilter::<f32>::new();
        h.allocate_filters(1, 0b111, CompositeNotches::Single);
        let mut p = params(0b111, CompositeNotches::Single);
        p.center_freq_hz = 200.0;
        h.init(1000.0, p);
        h.reset();

        assert_eq!(h.num_enabled_filters(), 3, "all three are placed");
        assert!(h.notch_active(0), "200 Hz should filter");
        assert!(h.notch_active(1), "400 Hz should filter");
        assert!(!h.notch_active(2), "600 Hz is past Nyquist and must be off");
    }

    /// Well below the minimum frequency a notch is switched off entirely, and
    /// between the minimum and the cutoff its attenuation fades toward unity
    /// so switching off is not a step change.
    #[test]
    fn a_notch_fades_out_below_the_minimum_frequency() {
        let mut h = HarmonicNotchFilter::<f32>::new();
        h.allocate_filters(1, 0b1, CompositeNotches::Single);
        let mut p = params(0b1, CompositeNotches::Single);
        p.freq_min_ratio = 1.0;
        p.tracking_mode = TrackingMode::UpdateRpm;
        h.init(1000.0, p);
        h.reset();

        // Minimum is 100 Hz, disable point is 25 Hz.
        h.update(100.0);
        assert!(h.notch_active(0));

        h.update(50.0);
        assert!(h.notch_active(0), "half way down it should still filter");

        h.update(10.0);
        assert!(!h.notch_active(0), "below the cutoff it should be off");
    }

    /// Several independent fundamentals, one per motor. Ordering is
    /// centre-major within each harmonic.
    #[test]
    fn several_centres_are_tracked_independently() {
        let mut h = HarmonicNotchFilter::<f32>::new();
        h.allocate_filters(3, 0b11, CompositeNotches::Single);
        let mut p = params(0b11, CompositeNotches::Single);
        p.tracking_mode = TrackingMode::UpdateBlHeli;
        h.init(1000.0, p);
        h.reset();

        h.update_multi(&[100.0, 150.0, 200.0]);
        assert_eq!(h.num_enabled_filters(), 6);
        // f1h1, f2h1, f3h1 then f1h2, f2h2, f3h2
        assert!((h.notch_center(0).expect("f1h1") - 100.0).abs() < 0.5);
        assert!((h.notch_center(1).expect("f2h1") - 150.0).abs() < 0.5);
        assert!((h.notch_center(2).expect("f3h1") - 200.0).abs() < 0.5);
        assert!((h.notch_center(3).expect("f1h2") - 200.0).abs() < 0.5);
        assert!((h.notch_center(4).expect("f2h2") - 300.0).abs() < 0.5);
        assert!((h.notch_center(5).expect("f3h2") - 400.0).abs() < 0.5);
    }

    /// A static notch is placed by `init` itself, with nothing to track.
    #[test]
    fn a_fixed_notch_is_placed_at_init() {
        let mut h = HarmonicNotchFilter::<f32>::new();
        h.allocate_filters(1, 0b1, CompositeNotches::Single);
        h.init(1000.0, params(0b1, CompositeNotches::Single));
        assert_eq!(
            h.num_enabled_filters(),
            1,
            "a fixed notch needs no update call"
        );
    }

    /// A tracking notch is not, because there is nothing to track yet.
    #[test]
    fn a_tracking_notch_waits_for_an_update() {
        let mut h = HarmonicNotchFilter::<f32>::new();
        h.allocate_filters(1, 0b1, CompositeNotches::Single);
        let mut p = params(0b1, CompositeNotches::Single);
        p.tracking_mode = TrackingMode::UpdateRpm;
        h.init(1000.0, p);
        assert_eq!(h.num_enabled_filters(), 0);
        assert_eq!(h.apply(2.0), 2.0, "and it passes through meanwhile");
    }

    /// The bank never configures more notches than it reserved, whatever the
    /// composite count -- upstream's loop guard is tested once per harmonic
    /// while the counter advances up to five times inside it.
    #[test]
    fn the_bank_never_exceeds_its_reservation() {
        for composite in [
            CompositeNotches::Single,
            CompositeNotches::Double,
            CompositeNotches::Triple,
            CompositeNotches::Quintuple,
        ] {
            let mut h = HarmonicNotchFilter::<f32, 7>::new();
            h.allocate_filters(8, 0xFFFF, composite);
            let mut p = params(0xFFFF, composite);
            p.tracking_mode = TrackingMode::UpdateBlHeli;
            h.init(1000.0, p);
            h.update_multi(&[100.0; 8]);

            assert!(
                h.num_enabled_filters() <= h.num_filters(),
                "{composite:?}: enabled {} exceeds reserved {}",
                h.num_enabled_filters(),
                h.num_filters()
            );
            assert!(h.num_filters() <= 7, "and the reservation fits the bank");
        }
    }
}
