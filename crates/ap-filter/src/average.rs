//! Port of `Filter/AverageFilter.h`, pinned to `Plane-4.7.0`.
//!
//! A boxcar average over the last `N` samples. Before the buffer fills, the sum
//! covers the whole buffer (unfilled slots are zero) but divides by the number
//! of samples actually seen, so early output is a true average rather than one
//! biased toward zero.
//!
//! Upstream carries a second template parameter `U` — a wider accumulator type
//! chosen to avoid overflow when summing integers. That is represented here by
//! [`Averageable::Sum`].

use crate::buffer::FilterBuffer;

/// Types that can be averaged, with the wider accumulator upstream calls `U`.
pub trait Averageable: Copy + Default {
    /// The accumulator type. Upstream's second template parameter.
    type Sum: Copy;
    /// A zero accumulator.
    fn zero_sum() -> Self::Sum;
    /// Add a sample to the accumulator.
    fn accumulate(sum: Self::Sum, v: Self) -> Self::Sum;
    /// Divide the accumulator by a sample count, back to the value type.
    fn divide(sum: Self::Sum, n: usize) -> Self;
}

impl Averageable for f32 {
    type Sum = f32;
    #[inline]
    fn zero_sum() -> f32 {
        0.0
    }
    #[inline]
    fn accumulate(sum: f32, v: f32) -> f32 {
        sum + v
    }
    #[inline]
    fn divide(sum: f32, n: usize) -> f32 {
        sum / n as f32
    }
}

impl Averageable for u16 {
    type Sum = u32;
    #[inline]
    fn zero_sum() -> u32 {
        0
    }
    #[inline]
    fn accumulate(sum: u32, v: u16) -> u32 {
        sum.wrapping_add(v as u32)
    }
    #[inline]
    fn divide(sum: u32, n: usize) -> u16 {
        (sum / n as u32) as u16
    }
}

impl Averageable for i16 {
    type Sum = i32;
    #[inline]
    fn zero_sum() -> i32 {
        0
    }
    #[inline]
    fn accumulate(sum: i32, v: i16) -> i32 {
        sum.wrapping_add(v as i32)
    }
    #[inline]
    fn divide(sum: i32, n: usize) -> i16 {
        (sum / n as i32) as i16
    }
}

/// Boxcar average of the last `N` samples. Upstream `AverageFilter<T,U,N>`.
#[derive(Debug, Clone, Copy)]
pub struct AverageFilter<T, const N: usize> {
    buf: FilterBuffer<T, N>,
    num_samples: usize,
}

impl<T: Averageable, const N: usize> Default for AverageFilter<T, N> {
    #[inline]
    fn default() -> Self {
        Self {
            buf: FilterBuffer::new(),
            num_samples: 0,
        }
    }
}

impl<T: Averageable, const N: usize> AverageFilter<T, N> {
    /// An empty filter.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// The buffer capacity. Upstream `get_filter_size()`.
    #[inline]
    pub fn get_filter_size(&self) -> usize {
        N
    }

    /// The sample at slot `i`, or `None` if out of range.
    #[inline]
    pub fn get_sample(&self, i: usize) -> Option<T> {
        self.buf.get_sample(i)
    }

    /// Clear the filter. Upstream `reset()`.
    #[inline]
    pub fn reset(&mut self) {
        self.buf.reset();
        self.num_samples = 0;
    }

    /// Add a sample and return the average so far. Upstream `apply()`.
    ///
    /// Upstream notes "there is a risk of overflow here that we ignore" when
    /// summing; the integer impls use wrapping arithmetic so the port wraps the
    /// same way rather than panicking in a debug build.
    #[inline]
    pub fn apply(&mut self, sample: T) -> T {
        self.buf.push(sample);

        self.num_samples += 1;
        if self.num_samples > N {
            self.num_samples = N;
        }

        let mut sum = T::zero_sum();
        for s in self.buf.samples.iter() {
            sum = T::accumulate(sum, *s);
        }
        T::divide(sum, self.num_samples)
    }
}

/// Upstream `AverageFilterFloat_Size5`.
pub type AverageFilterFloatSize5 = AverageFilter<f32, 5>;
/// Upstream `AverageFilterUInt16_Size4`.
pub type AverageFilterUInt16Size4 = AverageFilter<u16, 4>;

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]

    use super::*;

    // Ported from upstream libraries/Filter/tests/test_averagefilter.cpp
    // at Plane-4.7.0.

    /// UPSTREAM-PARITY: TEST(AverageFilterTest, Float_Size5)
    #[test]
    fn float_size5_matches_upstream() {
        let mut f = AverageFilterFloatSize5::new();
        let size = 5usize;
        let test_value = 5.0_f32;

        assert_eq!(size, f.get_filter_size());

        // a constant input averages to itself, filled or not
        for _ in 0..(size + 2) {
            assert_eq!(test_value, f.apply(test_value));
        }

        // one sample of 6x the value across a 5-slot buffer of 5s:
        // (5*4 + 30) / 5 == 10 == test_value * 2
        assert_eq!(
            test_value * 2.0,
            f.apply(test_value * (f.get_filter_size() + 1) as f32)
        );
        assert_eq!(Some(test_value), f.get_sample(1));
        assert_eq!(
            Some(test_value * (f.get_filter_size() + 1) as f32),
            f.get_sample(2)
        );

        // after reset the first sample averages to itself again
        f.reset();
        assert_eq!(test_value, f.apply(test_value));
    }

    /// UPSTREAM-PARITY: TEST(AverageFilterTest, UInt16_Size5)
    /// (upstream's test name says Size5 but it constructs a Size4 filter)
    #[test]
    fn uint16_size4_matches_upstream() {
        let mut f = AverageFilterUInt16Size4::new();
        let size = 4usize;
        let test_value = 5u16;

        assert_eq!(size, f.get_filter_size());
        for _ in 0..(size + 2) {
            assert_eq!(test_value, f.apply(test_value));
        }
        // (5*3 + 25) / 4 == 10 == test_value * 2
        assert_eq!(
            test_value * 2,
            f.apply(test_value * (f.get_filter_size() + 1) as u16)
        );
        f.reset();
        assert_eq!(test_value, f.apply(test_value));
    }

    /// PORT-DERIVED: before the buffer fills, the divisor is the number of
    /// samples seen, not the capacity, so early output is not biased to zero.
    #[test]
    fn partial_buffer_divides_by_samples_seen() {
        let mut f = AverageFilter::<f32, 5>::new();
        assert_eq!(10.0, f.apply(10.0)); // 10/1
        assert_eq!(15.0, f.apply(20.0)); // 30/2
        assert_eq!(20.0, f.apply(30.0)); // 60/3
    }
}
