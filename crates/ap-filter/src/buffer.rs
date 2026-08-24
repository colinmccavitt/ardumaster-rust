//! Port of `Filter/FilterWithBuffer.h`, pinned to `Plane-4.7.0`.
//!
//! The fixed-size sample buffer shared by [`crate::average`] and
//! [`crate::mode`]. Upstream uses a `FILTER_SIZE` template parameter; this uses
//! a const generic, so the buffer is still stack-allocated with no allocator.
//!
//! Note the two subclasses use `sample_index` differently, and the port keeps
//! that as-is:
//!
//! - `AverageFilter` treats it as a **ring pointer** that wraps at `N`.
//! - `ModeFilter` treats it as a **count** of samples held, saturating at `N`,
//!   and never wraps because its buffer is kept sorted.
//!
//! That overloading is upstream's; it is confusing but not a defect.

/// Fixed-size sample buffer. Upstream `FilterWithBuffer<T, FILTER_SIZE>`.
#[derive(Debug, Clone, Copy)]
pub struct FilterBuffer<T, const N: usize> {
    pub(crate) samples: [T; N],
    pub(crate) sample_index: usize,
}

impl<T: Copy + Default, const N: usize> Default for FilterBuffer<T, N> {
    #[inline]
    fn default() -> Self {
        Self {
            samples: [T::default(); N],
            sample_index: 0,
        }
    }
}

impl<T: Copy + Default, const N: usize> FilterBuffer<T, N> {
    /// An empty buffer, all slots zeroed. Upstream's constructor calls `reset()`.
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
    ///
    /// Upstream `get_sample()` indexes without bounds checking; returning
    /// `Option` keeps the workspace `indexing_slicing` lint honest and cannot
    /// read past the buffer.
    #[inline]
    pub fn get_sample(&self, i: usize) -> Option<T> {
        self.samples.get(i).copied()
    }

    /// Clear all samples and rewind the index. Upstream `reset()`.
    #[inline]
    pub fn reset(&mut self) {
        self.samples = [T::default(); N];
        self.sample_index = 0;
    }

    /// Store a sample at the current index and advance, wrapping at `N`.
    ///
    /// Upstream's base `apply()` returns the raw sample unchanged; subclasses
    /// do the filtering.
    #[inline]
    pub fn push(&mut self, sample: T) -> T {
        if let Some(slot) = self.samples.get_mut(self.sample_index) {
            *slot = sample;
        }
        self.sample_index += 1;
        if self.sample_index >= N {
            self.sample_index = 0;
        }
        sample
    }
}
