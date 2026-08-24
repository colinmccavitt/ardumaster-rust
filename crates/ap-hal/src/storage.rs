//! Persistent storage, ported from `AP_HAL/Storage.h`.
//!
//! Backs the parameter system (FW-004) and mission storage. Upstream models it
//! as a flat byte-addressed region read and written in blocks.
//!
//! # Divergence from upstream's shape
//!
//! Upstream's `read_block`/`write_block` take `void*` and cannot report
//! failure:
//!
//! ```cpp
//! virtual void read_block(void *dst, uint16_t src, size_t n) = 0;
//! virtual void write_block(uint16_t dst, const void* src, size_t n) = 0;
//! ```
//!
//! An out-of-range offset is therefore silently undefined — the backend either
//! clamps, reads adjacent memory, or does nothing, depending on the board. The
//! port takes slices, which carry their own length, and returns
//! [`crate::Result`] so a bad offset is a value the caller can see.
//!
//! This is a shape change rather than a behavior change: backends that
//! currently succeed still succeed, and the length is now checked instead of
//! assumed. Registered as **D-007** in DIVERGENCES.md because it converts
//! silent undefined behavior into a reported error, which a caller could
//! observe.

use crate::Result;

/// Byte-addressed persistent storage. Upstream `AP_HAL::Storage`.
pub trait Storage {
    /// Total size of the storage region, in bytes.
    ///
    /// Not present upstream, where the size is a compile-time board constant.
    /// Making it explicit is what allows the bounds checks below.
    fn size(&self) -> usize;

    /// Read `dst.len()` bytes starting at `src`.
    ///
    /// Upstream `read_block()`, which returns void and cannot fail.
    fn read_block(&self, dst: &mut [u8], src: usize) -> Result<()>;

    /// Write `src` starting at offset `dst`.
    ///
    /// Upstream `write_block()`, which returns void and cannot fail.
    fn write_block(&mut self, dst: usize, src: &[u8]) -> Result<()>;

    /// Erase the whole region. Upstream `erase()`, which defaults to failing.
    fn erase(&mut self) -> Result<()> {
        Err(crate::Error::Unsupported)
    }

    /// Whether the backend is functioning. Upstream `healthy()`, default true.
    fn healthy(&self) -> bool {
        true
    }
}

/// A storage backend held entirely in RAM.
///
/// Not a port of anything upstream — upstream's SITL backend is file-backed.
/// This exists so the parameter system (FW-004) can be unit-tested without a
/// filesystem, in the same spirit as the mockable [`crate::time::Clock`].
#[derive(Debug)]
pub struct RamStorage<const N: usize> {
    bytes: [u8; N],
}

impl<const N: usize> Default for RamStorage<N> {
    #[inline]
    fn default() -> Self {
        Self { bytes: [0; N] }
    }
}

impl<const N: usize> RamStorage<N> {
    /// A zeroed region.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

impl<const N: usize> Storage for RamStorage<N> {
    #[inline]
    fn size(&self) -> usize {
        N
    }

    fn read_block(&self, dst: &mut [u8], src: usize) -> Result<()> {
        let end = src.checked_add(dst.len()).ok_or(crate::Error::BusError)?;
        let slice = self.bytes.get(src..end).ok_or(crate::Error::BusError)?;
        dst.copy_from_slice(slice);
        Ok(())
    }

    fn write_block(&mut self, dst: usize, src: &[u8]) -> Result<()> {
        let end = dst.checked_add(src.len()).ok_or(crate::Error::BusError)?;
        let slice = self.bytes.get_mut(dst..end).ok_or(crate::Error::BusError)?;
        slice.copy_from_slice(src);
        Ok(())
    }

    fn erase(&mut self) -> Result<()> {
        self.bytes = [0; N];
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_a_block() {
        let mut s = RamStorage::<64>::new();
        s.write_block(8, &[1, 2, 3, 4]).unwrap();

        let mut buf = [0u8; 4];
        s.read_block(&mut buf, 8).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);

        // untouched bytes stay zero
        let mut before = [0xFFu8; 4];
        s.read_block(&mut before, 4).unwrap();
        assert_eq!(before, [0, 0, 0, 0]);
    }

    /// DIVERGENCE D-007: upstream cannot report this; the offset simply runs
    /// past the region with backend-defined consequences.
    #[test]
    fn d007_out_of_range_access_is_reported_not_silent() {
        let mut s = RamStorage::<16>::new();

        // straddling the end
        assert!(s.write_block(14, &[1, 2, 3, 4]).is_err());
        let mut buf = [0u8; 4];
        assert!(s.read_block(&mut buf, 14).is_err());

        // starting past the end
        assert!(s.read_block(&mut buf, 99).is_err());

        // an offset that would overflow the addition rather than wrap
        assert!(s.read_block(&mut buf, usize::MAX).is_err());

        // a failed write leaves the region untouched
        let mut all = [0xFFu8; 16];
        s.read_block(&mut all, 0).unwrap();
        assert_eq!(all, [0u8; 16], "a rejected write must not partially apply");
    }

    #[test]
    fn exact_fit_is_allowed() {
        let mut s = RamStorage::<4>::new();
        assert!(s.write_block(0, &[9, 9, 9, 9]).is_ok());
        let mut buf = [0u8; 4];
        assert!(s.read_block(&mut buf, 0).is_ok());
        assert_eq!(buf, [9, 9, 9, 9]);
    }

    #[test]
    fn erase_zeroes_and_reports_healthy() {
        let mut s = RamStorage::<8>::new();
        s.write_block(0, &[7; 8]).unwrap();
        s.erase().unwrap();
        let mut buf = [0xFFu8; 8];
        s.read_block(&mut buf, 0).unwrap();
        assert_eq!(buf, [0u8; 8]);
        assert!(s.healthy());
        assert_eq!(s.size(), 8);
    }
}
