//! Smooth low-noise differentiator, upstream `Filter/DerivativeFilter`.
//!
//! Differentiating a noisy signal amplifies the noise: a plain
//! `(y2 - y1) / (t2 - t1)` on barometric altitude gives a climb rate that is
//! mostly sensor noise. This uses Holoborodko's smooth noise-robust
//! differentiators, which combine several central differences at different
//! spacings with weights chosen so that noise cancels while a genuine slope
//! survives.
//!
//! `AP_Baro` uses the seven-sample form for climb rate, which is the only
//! instantiation the fixed-wing path reaches.
//!
//! # Non-uniform spacing
//!
//! Each sample carries its own timestamp and every central difference is
//! divided by its own elapsed time, so samples arriving irregularly still give
//! the right slope. That is why this is not a fixed-coefficient FIR.
//!
//! # Reference
//!
//! <http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/>

/// A smooth differentiator over `N` samples.
///
/// `N` must be 5, 7, 9 or 11 — those are the sizes Holoborodko's weights are
/// given for, and the only ones upstream instantiates. Any other size yields a
/// slope of zero, which is upstream's `default:` case.
///
/// DIVERGENCE D-005, fourth occurrence: upstream's constructor body is empty.
/// It delegates to `FilterWithBuffer`, which clears the sample buffer and the
/// index, and leaves `_new_data`, `_last_slope` and the whole `_timestamps`
/// array indeterminate. The port initialises all of them.
#[derive(Debug, Clone, Copy)]
pub struct DerivativeFilter<const N: usize> {
    samples: [f32; N],
    /// Microsecond timestamps, needed because the samples are not evenly
    /// spaced.
    timestamps: [u32; N],
    sample_index: usize,
    new_data: bool,
    last_slope: f32,
}

impl<const N: usize> Default for DerivativeFilter<N> {
    fn default() -> Self {
        Self {
            samples: [0.0; N],
            timestamps: [0; N],
            sample_index: 0,
            new_data: false,
            last_slope: 0.0,
        }
    }
}

impl<const N: usize> DerivativeFilter<N> {
    /// An empty filter.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a sample, upstream `update`.
    ///
    /// A repeated timestamp is ignored rather than stored: two samples at the
    /// same instant would give a zero denominator in [`Self::slope`].
    pub fn update(&mut self, sample: f32, timestamp: u32) {
        let i = self.sample_index;
        let i1 = if i == 0 { N - 1 } else { i - 1 };

        if self.timestamps.get(i1) == Some(&timestamp) {
            return;
        }

        // The timestamp goes in before the sample, because storing the sample
        // is what advances the index.
        if let Some(t) = self.timestamps.get_mut(i) {
            *t = timestamp;
        }
        if let Some(s) = self.samples.get_mut(i) {
            *s = sample;
        }
        self.sample_index = (i + 1) % N;

        self.new_data = true;
    }

    /// Index into the ring the way upstream's `f(i)` and `x(i)` macros do.
    ///
    /// The `3 * N / 2` offset exists to keep the expression positive for
    /// negative `i` before the modulo — C's `%` on a negative operand gives a
    /// negative result, which would index out of the buffer.
    fn ring(&self, i: isize) -> usize {
        let n = N as isize;
        let s = self.sample_index as isize;
        ((((s - 1) + i + 1) + 3 * n / 2) % n) as usize
    }

    /// One weighted central difference across `±k`.
    ///
    /// The weight multiplies the difference *before* the division, which is
    /// how upstream groups it: `2*42*(f(1) - f(-1)) / (x(1) - x(-1))` parses
    /// as `((84 * diff) / dt)`, not `84 * (diff / dt)`. Those are not the same
    /// in floating point, and at size 11 the difference is visible.
    fn term(&self, k: isize, weight: f32) -> f32 {
        let (hi, lo) = (self.ring(k), self.ring(-k));
        let (Some(&f_hi), Some(&f_lo)) = (self.samples.get(hi), self.samples.get(lo)) else {
            return 0.0;
        };
        let (Some(&x_hi), Some(&x_lo)) = (self.timestamps.get(hi), self.timestamps.get(lo)) else {
            return 0.0;
        };
        // Wrapping, because the timestamps are a free-running microsecond
        // counter and upstream's uint32_t subtraction wraps too.
        weight * (f_hi - f_lo) / x_hi.wrapping_sub(x_lo) as f32
    }

    /// The slope, upstream `slope`.
    ///
    /// Returns the previous answer unchanged if no sample has arrived since
    /// the last call — the computation is not free and the answer cannot have
    /// moved.
    pub fn slope(&mut self) -> f32 {
        if !self.new_data {
            return self.last_slope;
        }

        // Upstream's "we haven't filled the buffer yet" test. It works because
        // both slots are zero until enough samples have arrived, which is
        // precisely why leaving `_timestamps` uninitialised is a defect and
        // not a cosmetic one — with garbage in them this guard does not fire
        // and the slope is computed across an unfilled buffer.
        if N >= 2 && self.timestamps.get(N - 1) == self.timestamps.get(N - 2) {
            return 0.0;
        }

        // Holoborodko's weights. The outer factor on each term is 2k; the
        // inner one is the tabulated coefficient.
        let mut result = match N {
            5 => (self.term(1, 2.0 * 2.0) + self.term(2, 4.0 * 1.0)) / 8.0,
            7 => {
                (self.term(1, 2.0 * 5.0) + self.term(2, 4.0 * 4.0) + self.term(3, 6.0 * 1.0)) / 32.0
            }
            9 => {
                (self.term(1, 2.0 * 14.0)
                    + self.term(2, 4.0 * 14.0)
                    + self.term(3, 6.0 * 6.0)
                    + self.term(4, 8.0 * 1.0))
                    / 128.0
            }
            11 => {
                (self.term(1, 2.0 * 42.0)
                    + self.term(2, 4.0 * 48.0)
                    + self.term(3, 6.0 * 27.0)
                    + self.term(4, 8.0 * 8.0)
                    + self.term(5, 10.0 * 1.0))
                    / 512.0
            }
            _ => 0.0,
        };

        // A repeated timestamp that slipped through, or a wrapped counter,
        // can still divide by zero.
        if result.is_nan() || result.is_infinite() {
            result = 0.0;
        }

        self.new_data = false;
        self.last_slope = result;
        result
    }

    /// Clear the filter, upstream `reset`.
    ///
    /// DIVERGENCE: upstream's `reset` forwards to `FilterWithBuffer::reset`,
    /// which clears the samples and the index and nothing else — the
    /// timestamps, `_new_data` and `_last_slope` all survive. So upstream's
    /// `slope()` after a `reset()` returns the *pre-reset* slope when no new
    /// sample has arrived, and once samples do arrive the stale timestamps
    /// stop the unfilled-buffer guard from firing, so the slope is computed
    /// across the boundary between new samples and cleared ones.
    ///
    /// Latent rather than reachable: nothing in the fixed-wing path calls it.
    /// `AP_Baro` holds the only instantiation and only ever calls `update` and
    /// `slope`. Fixed here because a reset that leaves the answer behind is
    /// not a reset.
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    /// How many samples the filter holds.
    #[must_use]
    pub const fn filter_size(&self) -> usize {
        N
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::float_cmp,
        reason = "exact zero is the claim in the unfilled-buffer and reset tests"
    )]

    use super::*;

    /// The size `AP_Baro` uses for climb rate.
    type Baro = DerivativeFilter<7>;

    /// D-005, fourth occurrence and the worst of them: upstream leaves
    /// `_new_data`, `_last_slope` and every entry of `_timestamps`
    /// indeterminate, and the unfilled-buffer guard is *implemented as* a test
    /// on two of those slots being equal.
    #[test]
    fn d005_a_fresh_filter_is_deterministic() {
        let mut f = Baro::new();
        assert_eq!(f.slope(), 0.0);
        assert_eq!(f.filter_size(), 7);
    }

    /// Nothing is claimed until the buffer is full.
    #[test]
    fn an_unfilled_buffer_has_no_slope() {
        let mut f = Baro::new();
        for i in 0..6_u32 {
            f.update(i as f32, i * 1000);
            assert_eq!(f.slope(), 0.0, "after {} samples", i + 1);
        }
    }

    /// A straight line has a constant slope, and the filter should find it
    /// exactly — the weights are constructed to be exact for low-order
    /// polynomials.
    #[test]
    fn a_straight_line_gives_its_gradient() {
        let mut f = Baro::new();
        // 2 units per 1000 microseconds
        for i in 0..14_u32 {
            f.update(2.0 * i as f32, i * 1000);
        }
        let got = f.slope();
        assert!(
            (got - 0.002).abs() < 1.0e-7,
            "expected 0.002 per microsecond, got {got}"
        );
    }

    /// Irregular spacing is handled, because every difference is divided by
    /// its own elapsed time. A filter using fixed coefficients would get this
    /// wrong.
    #[test]
    fn irregular_spacing_still_gives_the_right_gradient() {
        let mut f = Baro::new();
        let gaps = [1000_u32, 2500, 700, 1800, 900, 3000, 1200];
        let mut t = 0_u32;
        for i in 0..14 {
            #[allow(clippy::indexing_slicing, reason = "modulo keeps this in range")]
            let gap = gaps[i % gaps.len()];
            t += gap;
            f.update(2.0 * t as f32, t);
        }
        let got = f.slope();
        assert!(
            (got - 2.0).abs() < 1.0e-4,
            "the signal rises 2 per microsecond, got {got}"
        );
    }

    /// The point of the thing: it rejects noise far better than a plain
    /// two-point difference on the same data.
    #[test]
    fn it_rejects_noise_better_than_a_two_point_difference() {
        // A deterministic zig-zag on top of a straight line.
        let signal = |i: u32| -> f32 {
            let base = 0.5 * i as f32;
            let noise = if i.is_multiple_of(2) { 1.0 } else { -1.0 };
            base + noise
        };

        let mut f = Baro::new();
        let mut worst_filtered = 0.0_f32;
        let mut worst_two_point = 0.0_f32;
        let truth = 0.5 / 1000.0; // per microsecond

        for i in 0..40_u32 {
            f.update(signal(i), i * 1000);
            if i >= 8 {
                worst_filtered = worst_filtered.max((f.slope() - truth).abs());
                let two_point = (signal(i) - signal(i - 1)) / 1000.0;
                worst_two_point = worst_two_point.max((two_point - truth).abs());
            }
        }

        assert!(
            worst_filtered < worst_two_point * 0.25,
            "the differentiator should reject the zig-zag: {worst_filtered:e} against \
             a two-point difference's {worst_two_point:e}"
        );
    }

    /// A repeated timestamp is dropped rather than stored, because it would
    /// divide by zero.
    #[test]
    fn a_repeated_timestamp_is_ignored() {
        let mut f = Baro::new();
        for i in 0..14_u32 {
            f.update(2.0 * i as f32, i * 1000);
        }
        let before = f.slope();
        f.update(999.0, 13 * 1000);
        assert_eq!(f.slope(), before, "the duplicate should not have landed");
    }

    /// Between samples the answer is cached, not recomputed.
    #[test]
    fn the_slope_is_cached_between_samples() {
        let mut f = Baro::new();
        for i in 0..14_u32 {
            f.update(2.0 * i as f32, i * 1000);
        }
        let first = f.slope();
        assert_eq!(f.slope(), first);
        assert_eq!(f.slope(), first);
    }

    /// A reset means no derivative is known. Upstream's forwards to the
    /// buffer's reset and leaves the timestamps, the flag and the cached
    /// slope in place, so `slope()` keeps returning the pre-reset answer.
    #[test]
    fn reset_forgets_the_slope() {
        let mut f = Baro::new();
        for i in 0..14_u32 {
            f.update(2.0 * i as f32, i * 1000);
        }
        assert!(f.slope() > 0.0);

        f.reset();
        assert_eq!(
            f.slope(),
            0.0,
            "upstream would still report the pre-reset slope here"
        );

        // And it starts genuinely empty rather than half full.
        for i in 0..5_u32 {
            f.update(3.0 * i as f32, 100_000 + i * 1000);
            assert_eq!(f.slope(), 0.0, "still filling");
        }
    }

    /// A timestamp counter that wraps past its maximum still gives the right
    /// elapsed time, because the subtraction wraps with it.
    #[test]
    fn a_wrapping_timestamp_counter_is_handled() {
        let mut f = Baro::new();
        let start = u32::MAX - 5000;
        for i in 0..14_u32 {
            f.update(2.0 * i as f32, start.wrapping_add(i * 1000));
        }
        let got = f.slope();
        assert!(
            (got - 0.002).abs() < 1.0e-7,
            "the counter wraps mid-buffer; got {got}"
        );
    }

    /// Sizes without tabulated weights give zero, upstream's `default:` case.
    #[test]
    fn an_unsupported_size_gives_zero() {
        let mut f = DerivativeFilter::<6>::new();
        for i in 0..12_u32 {
            f.update(2.0 * i as f32, i * 1000);
        }
        assert_eq!(f.slope(), 0.0);
    }
}
