//! Port of `Filter/ModeFilter.{h,cpp}`, pinned to `Plane-4.7.0`.
//!
//! A rank-order filter. The buffer is kept **sorted** by insertion; each new
//! sample displaces either the highest or the lowest existing sample, and the
//! choice alternates every call. The output is the sample at a caller-chosen
//! rank — the median by default.
//!
//! Alternating which end is dropped is what makes this behave like a mode
//! filter rather than a running median: a persistent outlier at one end is
//! evicted on the following call.
//!
//! Note `sample_index` is a **count** here, not a ring pointer — it saturates
//! at `N` and never wraps, because the buffer is sorted rather than circular.
//! Upstream overloads the same base-class field for both meanings.

use crate::buffer::FilterBuffer;

/// Rank-order filter over the last `N` samples. Upstream `ModeFilter<T,N>`.
#[derive(Debug, Clone, Copy)]
pub struct ModeFilter<T, const N: usize> {
    buf: FilterBuffer<T, N>,
    return_element: usize,
    drop_high_sample: bool,
    output: T,
}

impl<T: Copy + Default + PartialOrd, const N: usize> ModeFilter<T, N> {
    /// A filter returning the sample at rank `return_element` once full.
    ///
    /// Upstream clamps an out-of-range rank to the median (`N / 2`) rather than
    /// rejecting it, and its own test relies on that: `ModeFilterInt16_Size5{8}`
    /// behaves as `{2}`.
    #[inline]
    pub fn new(return_element: usize) -> Self {
        Self {
            buf: FilterBuffer::new(),
            return_element: if return_element >= N {
                N / 2
            } else {
                return_element
            },
            drop_high_sample: true,
            output: T::default(),
        }
    }

    /// The buffer capacity.
    #[inline]
    pub fn get_filter_size(&self) -> usize {
        N
    }

    /// The last value returned by [`Self::apply`]. Upstream `get()`.
    #[inline]
    pub fn get(&self) -> T {
        self.output
    }

    /// The sample at slot `i` of the sorted buffer, or `None` if out of range.
    #[inline]
    pub fn get_sample(&self, i: usize) -> Option<T> {
        self.buf.get_sample(i)
    }

    /// Clear the filter.
    #[inline]
    pub fn reset(&mut self) {
        self.buf.reset();
        self.drop_high_sample = true;
        self.output = T::default();
    }

    /// Add a sample and return the value at the configured rank.
    ///
    /// Upstream `apply()`. While the buffer is still filling, the middle of the
    /// samples held so far is returned instead of the configured rank.
    #[inline]
    pub fn apply(&mut self, sample: T) -> T {
        let drop_high = self.drop_high_sample;
        self.isort(sample, drop_high);
        self.drop_high_sample = !self.drop_high_sample;

        let idx = if self.buf.sample_index < N {
            self.buf.sample_index / 2
        } else {
            self.return_element
        };
        self.output = self.buf.get_sample(idx).unwrap_or_default();
        self.output
    }

    /// Insertion sort that drops one end to make room. Upstream `isort()`.
    ///
    /// While the buffer is not yet full, upstream grows it and forces the
    /// drop-high path regardless of the caller's request.
    fn isort(&mut self, new_sample: T, drop_high: bool) {
        let mut drop_high = drop_high;
        if self.buf.sample_index < N {
            self.buf.sample_index += 1;
            drop_high = true;
        }
        let count = self.buf.sample_index;

        if drop_high {
            // walk down from the top, pushing larger samples up one slot.
            // Mirrors upstream's `while (i > 0 && samples[i-1] > new_sample)`,
            // keeping the comparison positive: with a NaN sample the `>` is
            // false and the loop stops, exactly as the C++ does.
            let mut i = count.saturating_sub(1);
            while i > 0 {
                let prev = match self.buf.get_sample(i - 1) {
                    Some(v) => v,
                    None => break,
                };
                if prev > new_sample {
                    if let Some(slot) = self.buf.samples.get_mut(i) {
                        *slot = prev;
                    }
                    i -= 1;
                } else {
                    break;
                }
            }
            if let Some(slot) = self.buf.samples.get_mut(i) {
                *slot = new_sample;
            }
        } else {
            // walk up from the bottom, pushing smaller samples down one slot.
            // Mirrors upstream's
            // `while (i < sample_index-1 && samples[i+1] < new_sample)`.
            let mut i = 0usize;
            while i + 1 < count {
                let next = match self.buf.get_sample(i + 1) {
                    Some(v) => v,
                    None => break,
                };
                if next < new_sample {
                    if let Some(slot) = self.buf.samples.get_mut(i) {
                        *slot = next;
                    }
                    i += 1;
                } else {
                    break;
                }
            }
            if let Some(slot) = self.buf.samples.get_mut(i) {
                *slot = new_sample;
            }
        }
    }
}

/// Upstream `ModeFilterFloat_Size5`.
pub type ModeFilterFloatSize5 = ModeFilter<f32, 5>;
/// Upstream `ModeFilterInt16_Size5`.
pub type ModeFilterInt16Size5 = ModeFilter<i16, 5>;
/// Upstream `ModeFilterInt16_Size3`.
pub type ModeFilterInt16Size3 = ModeFilter<i16, 3>;

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // Ported from upstream libraries/Filter/tests/test_modefilter.cpp
    // at Plane-4.7.0.

    /// UPSTREAM-PARITY: TEST(ModeFilterTest, Int16_Size5), median arm.
    ///
    /// A 5-slot median filter fed an alternating 0 / large signal holds 0
    /// throughout: the alternating drop-high/drop-low eviction keeps the
    /// outliers from ever reaching the middle rank.
    #[test]
    fn int16_size5_median_rejects_alternating_outliers() {
        let mut f = ModeFilterInt16Size5::new(2);
        for _ in 0..5 {
            assert_eq!(0, f.apply(0));
        }
        assert_eq!(0, f.apply(5));
        assert_eq!(0, f.apply(0));
        for _ in 0..8 {
            assert_eq!(0, f.apply(10));
            assert_eq!(0, f.apply(0));
        }
    }

    /// UPSTREAM-PARITY: TEST(ModeFilterTest, Int16_Size5), out-of-range arm.
    ///
    /// Upstream constructs with rank 8 on a 5-slot filter; the constructor
    /// clamps it to the median. The expected sequence is upstream's verbatim.
    #[test]
    fn int16_size5_out_of_range_rank_clamps_to_median() {
        let mut f = ModeFilterInt16Size5::new(8);
        assert_eq!(1, f.apply(1));
        assert_eq!(3, f.apply(3));
        assert_eq!(2, f.apply(2));
        assert_eq!(3, f.apply(4));
        assert_eq!(3, f.apply(5));
        assert_eq!(4, f.apply(6));
        assert_eq!(4, f.apply(7));
        assert_eq!(5, f.apply(8));
        assert_eq!(5, f.get());
    }

    /// UPSTREAM-PARITY: TEST(ModeFilterTest, Float_Size5)
    #[test]
    fn float_size5_matches_upstream() {
        let mut f = ModeFilterFloatSize5::new(2);
        for _ in 0..5 {
            assert_eq!(0.0, f.apply(0.0));
        }
        assert_eq!(0.0, f.apply(5.0));
        assert_eq!(0.0, f.apply(0.0));
        for _ in 0..3 {
            assert_eq!(0.0, f.apply(10.0));
            assert_eq!(0.0, f.apply(0.0));
        }
    }

    /// PORT-DERIVED: the buffer really is kept sorted.
    #[test]
    fn buffer_stays_sorted() {
        let mut f = ModeFilterInt16Size5::new(2);
        for v in [5, 1, 4, 2, 3] {
            f.apply(v);
        }
        let mut prev = i16::MIN;
        for i in 0..5 {
            let s = f.get_sample(i).unwrap();
            assert!(s >= prev, "buffer not sorted at {i}: {s} < {prev}");
            prev = s;
        }
    }

    /// PORT-DERIVED: a steady signal converges to that value.
    #[test]
    fn steady_input_converges() {
        let mut f = ModeFilterFloatSize5::new(2);
        let mut last = 0.0;
        for _ in 0..20 {
            last = f.apply(7.0);
        }
        assert_eq!(7.0, last);
    }
}
