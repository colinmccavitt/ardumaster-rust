//! On-chip flash, ported from `AP_HAL/Flash.h`.
//!
//! Erase a page (sector), write bytes, and read them back. Page size is the
//! erase granularity (`getpagesize`) — that is the sector size. Upstream has
//! no `read`: the mapped address from `getpageaddr` is dereferenced in place.
//! The port takes a slice so a mock can serve the same bytes without an MMIO
//! window.
//!
//! `keep_unlocked` and `ispageerased` stay on the trait; bootloader and
//! crashdump paths use them after an erase.

use crate::{Error, Result};

/// Erased NOR byte. STM32 / ChibiOS flash reads as this after `erasepage`.
pub const ERASED_BYTE: u8 = 0xFF;

/// Mock page (sector) size in bytes. Upstream `getpagesize` is board-specific.
pub const PAGE_SIZE: u32 = 256;

/// Mock page count. Upstream `getnumpages` is board-specific.
pub const NUM_PAGES: u32 = 4;

const TOTAL: usize = (PAGE_SIZE as usize) * (NUM_PAGES as usize);

/// On-chip flash. Upstream `AP_HAL::Flash`.
///
/// Fallible ops return [`Result`] instead of upstream `bool` so a failed
/// erase or write is not a sentinel in the value (same shape as
/// [`crate::can_iface::CanIface::send`]).
pub trait Flash {
    /// Byte address of `page`. Upstream `getpageaddr`.
    fn page_addr(&self, page: u32) -> u32;

    /// Erase granularity of `page` in bytes. Upstream `getpagesize`.
    ///
    /// This is the sector size: erase is per-page, write is byte-addressed
    /// inside that page.
    fn page_size(&self, page: u32) -> u32;

    /// How many pages the device exposes. Upstream `getnumpages`.
    fn num_pages(&self) -> u32;

    /// Erase `page` to [`ERASED_BYTE`]. Upstream `erasepage`.
    fn erase_page(&mut self, page: u32) -> Result<()>;

    /// Program `buf` at `addr`. Upstream `write`.
    fn write(&mut self, addr: u32, buf: &[u8]) -> Result<()>;

    /// Copy `buf.len()` bytes from `addr`.
    ///
    /// Not upstream — flash is memory-mapped there. The mock needs an
    /// explicit read so tests can check erase/write without an MMIO map.
    fn read(&self, addr: u32, buf: &mut [u8]) -> Result<()>;

    /// Hold the controller unlocked across a burst of writes.
    /// Upstream `keep_unlocked`.
    fn keep_unlocked(&mut self, set: bool);

    /// True when every byte of `page` is still [`ERASED_BYTE`].
    /// Upstream `ispageerased`.
    fn is_page_erased(&self, page: u32) -> bool;
}

/// An in-memory [`Flash`] for tests and SITL bring-up.
///
/// RAM-backed: [`Flash::write`] overwrites rather than AND-programming NOR
/// cells. [`Flash::erase_page`] still fills the sector with [`ERASED_BYTE`]
/// so [`Flash::is_page_erased`] and a following read match a fresh chip.
#[derive(Debug)]
pub struct MockFlash {
    bytes: [u8; TOTAL],
    unlocked: bool,
}

impl Default for MockFlash {
    fn default() -> Self {
        Self {
            bytes: [ERASED_BYTE; TOTAL],
            unlocked: false,
        }
    }
}

impl MockFlash {
    /// A fully erased device ([`NUM_PAGES`] sectors of [`PAGE_SIZE`] bytes).
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether [`Flash::keep_unlocked`] last set the latch.
    #[must_use]
    pub const fn is_kept_unlocked(&self) -> bool {
        self.unlocked
    }

    fn page_range(&self, page: u32) -> Result<core::ops::Range<usize>> {
        if page >= NUM_PAGES {
            return Err(Error::Unsupported);
        }
        let start = (page as usize).saturating_mul(PAGE_SIZE as usize);
        let end = start.saturating_add(PAGE_SIZE as usize);
        if end > TOTAL {
            return Err(Error::BusError);
        }
        Ok(start..end)
    }

    fn span(&self, addr: u32, len: usize) -> Result<core::ops::Range<usize>> {
        let start = addr as usize;
        let end = start.checked_add(len).ok_or(Error::BusError)?;
        if end > TOTAL {
            return Err(Error::BusError);
        }
        Ok(start..end)
    }
}

impl Flash for MockFlash {
    fn page_addr(&self, page: u32) -> u32 {
        if page >= NUM_PAGES {
            return 0;
        }
        page.saturating_mul(PAGE_SIZE)
    }

    fn page_size(&self, page: u32) -> u32 {
        if page >= NUM_PAGES {
            return 0;
        }
        PAGE_SIZE
    }

    fn num_pages(&self) -> u32 {
        NUM_PAGES
    }

    fn erase_page(&mut self, page: u32) -> Result<()> {
        let range = self.page_range(page)?;
        if let Some(region) = self.bytes.get_mut(range) {
            for b in region.iter_mut() {
                *b = ERASED_BYTE;
            }
            Ok(())
        } else {
            Err(Error::BusError)
        }
    }

    fn write(&mut self, addr: u32, buf: &[u8]) -> Result<()> {
        let range = self.span(addr, buf.len())?;
        if let Some(dest) = self.bytes.get_mut(range) {
            dest.copy_from_slice(buf);
            Ok(())
        } else {
            Err(Error::BusError)
        }
    }

    fn read(&self, addr: u32, buf: &mut [u8]) -> Result<()> {
        let range = self.span(addr, buf.len())?;
        if let Some(src) = self.bytes.get(range) {
            buf.copy_from_slice(src);
            Ok(())
        } else {
            Err(Error::BusError)
        }
    }

    fn keep_unlocked(&mut self, set: bool) {
        self.unlocked = set;
    }

    fn is_page_erased(&self, page: u32) -> bool {
        let Ok(range) = self.page_range(page) else {
            return false;
        };
        self.bytes
            .get(range)
            .is_some_and(|region| region.iter().all(|b| *b == ERASED_BYTE))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sector_size_and_page_map() {
        let flash = MockFlash::new();
        assert_eq!(flash.num_pages(), NUM_PAGES);
        assert_eq!(flash.page_size(0), PAGE_SIZE);
        assert_eq!(flash.page_size(NUM_PAGES - 1), PAGE_SIZE);
        assert_eq!(flash.page_size(NUM_PAGES), 0);
        assert_eq!(flash.page_addr(0), 0);
        assert_eq!(flash.page_addr(1), PAGE_SIZE);
        assert_eq!(flash.page_addr(NUM_PAGES), 0);
        assert!(flash.is_page_erased(0));
        assert!(flash.is_page_erased(1));
    }

    #[test]
    fn erase_write_read_round_trip() {
        let mut flash = MockFlash::new();
        let addr = flash.page_addr(1);
        assert_eq!(flash.page_size(1), PAGE_SIZE);

        let mut erased = [0u8; 4];
        assert!(flash.read(addr, &mut erased).is_ok());
        assert_eq!(erased, [ERASED_BYTE; 4]);

        assert!(flash.write(addr, &[0xDE, 0xAD, 0xBE, 0xEF]).is_ok());
        let mut got = [0u8; 4];
        assert!(flash.read(addr, &mut got).is_ok());
        assert_eq!(got, [0xDE, 0xAD, 0xBE, 0xEF]);
        assert!(!flash.is_page_erased(1));
        assert!(
            flash.is_page_erased(0),
            "write must not touch other sectors"
        );

        assert!(flash.erase_page(1).is_ok());
        assert!(flash.is_page_erased(1));
        assert!(flash.read(addr, &mut got).is_ok());
        assert_eq!(got, [ERASED_BYTE; 4]);
    }

    #[test]
    fn out_of_range_is_reported() {
        let mut flash = MockFlash::new();
        assert_eq!(flash.erase_page(NUM_PAGES), Err(Error::Unsupported));
        assert!(!flash.is_page_erased(NUM_PAGES));
        assert_eq!(flash.write(TOTAL as u32, &[1]), Err(Error::BusError));
        let mut buf = [0u8; 1];
        assert_eq!(flash.read(TOTAL as u32, &mut buf), Err(Error::BusError));
        // straddling the end
        assert_eq!(
            flash.write((TOTAL as u32) - 1, &[1, 2]),
            Err(Error::BusError)
        );
    }

    #[test]
    fn keep_unlocked_latches() {
        let mut flash = MockFlash::new();
        assert!(!flash.is_kept_unlocked());
        flash.keep_unlocked(true);
        assert!(flash.is_kept_unlocked());
        flash.keep_unlocked(false);
        assert!(!flash.is_kept_unlocked());
    }

    /// The trait stays object-safe, which is what allows `&dyn` in the HAL
    /// context. If a future method breaks object safety this fails to compile
    /// here rather than at some distant call site.
    #[test]
    fn flash_trait_is_object_safe() {
        let mut flash = MockFlash::new();
        let f: &mut dyn Flash = &mut flash;
        assert_eq!(f.num_pages(), NUM_PAGES);
        assert_eq!(f.page_size(0), PAGE_SIZE);
        let addr = f.page_addr(0);
        assert!(f.write(addr, &[0x11, 0x22]).is_ok());
        let mut buf = [0u8; 2];
        assert!(f.read(addr, &mut buf).is_ok());
        assert_eq!(buf, [0x11, 0x22]);
        assert!(f.erase_page(0).is_ok());
        assert!(f.is_page_erased(0));
        f.keep_unlocked(true);
    }
}
