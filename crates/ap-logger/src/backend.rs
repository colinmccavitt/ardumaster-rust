//! Logger backend write path, upstream `AP_Logger_Backend` / `AP_Logger_Block`.

/// Write-path seam for a logger backend.
///
/// `write_block` is `AP_Logger_Backend::WriteBlock`: copy `buffer` at the
/// current offset and report whether the backend accepted it.
///
/// `start_write` / `end_write` are the page-write bookends on
/// `AP_Logger_Block` (`StartWrite` / `FinishWrite`; the ticket names the
/// closer `EndWrite`). They do not invent a filesystem — they only mark
/// where a page write begins and ends.
pub trait LogBackend {
    /// Write a block of data at the current offset.
    ///
    /// Upstream `AP_Logger_Backend::WriteBlock`. Returns `true` when the
    /// backend accepted the whole block.
    fn write_block(&mut self, buffer: &[u8]) -> bool;

    /// Begin a page write at `page_adr`.
    ///
    /// Upstream `AP_Logger_Block::StartWrite`.
    fn start_write(&mut self, page_adr: u32);

    /// Finish the current page write.
    ///
    /// Upstream `AP_Logger_Block::FinishWrite` (ticket name: `EndWrite`).
    fn end_write(&mut self);
}

/// In-memory [`LogBackend`] that records bytes for tests.
///
/// Not a DataFlash device: there is no page erase, wrap, or log index.
/// `write_block` appends into a fixed buffer; `start_write` / `end_write`
/// only record the page address and how many times a write was closed.
#[derive(Debug)]
pub struct MemoryBackend<const N: usize> {
    bytes: [u8; N],
    len: usize,
    page_adr: u32,
    writing: bool,
    ended: u32,
}

impl<const N: usize> Default for MemoryBackend<N> {
    #[inline]
    fn default() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
            page_adr: 0,
            writing: false,
            ended: 0,
        }
    }
}

impl<const N: usize> MemoryBackend<N> {
    /// An empty backend with a zeroed buffer.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Bytes accepted by [`LogBackend::write_block`], in order.
    #[must_use]
    pub fn recorded(&self) -> &[u8] {
        match self.bytes.get(..self.len) {
            Some(slice) => slice,
            None => &[],
        }
    }

    /// Page address from the last [`LogBackend::start_write`].
    #[must_use]
    pub const fn page_adr(&self) -> u32 {
        self.page_adr
    }

    /// Whether a page write is open (after `start_write`, before `end_write`).
    #[must_use]
    pub const fn is_writing(&self) -> bool {
        self.writing
    }

    /// How many times [`LogBackend::end_write`] has been called.
    #[must_use]
    pub const fn ended_writes(&self) -> u32 {
        self.ended
    }
}

impl<const N: usize> LogBackend for MemoryBackend<N> {
    fn write_block(&mut self, buffer: &[u8]) -> bool {
        let Some(end) = self.len.checked_add(buffer.len()) else {
            return false;
        };
        if end > N {
            return false;
        }
        let Some(dst) = self.bytes.get_mut(self.len..end) else {
            return false;
        };
        dst.copy_from_slice(buffer);
        self.len = end;
        true
    }

    fn start_write(&mut self, page_adr: u32) {
        self.page_adr = page_adr;
        self.writing = true;
    }

    fn end_write(&mut self) {
        self.writing = false;
        self.ended = self.ended.saturating_add(1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_block_records_bytes() {
        let mut log = MemoryBackend::<16>::new();
        assert!(log.write_block(b"DF"));
        assert!(log.write_block(&[0x80, 0x01]));
        assert_eq!(log.recorded(), &[b'D', b'F', 0x80, 0x01]);
    }

    #[test]
    fn write_block_rejects_when_full() {
        let mut log = MemoryBackend::<2>::new();
        assert!(log.write_block(&[1, 2]));
        assert!(!log.write_block(&[3]));
        assert_eq!(log.recorded(), &[1, 2]);
    }

    #[test]
    fn start_write_and_end_write_bookend_a_page() {
        let mut log = MemoryBackend::<8>::new();
        assert!(!log.is_writing());
        log.start_write(7);
        assert_eq!(log.page_adr(), 7);
        assert!(log.is_writing());
        assert!(log.write_block(b"AB"));
        log.end_write();
        assert!(!log.is_writing());
        assert_eq!(log.ended_writes(), 1);
        assert_eq!(log.recorded(), b"AB");
        assert_eq!(log.page_adr(), 7);
    }

    /// The trait stays object-safe so a logger front-end can hold `&mut dyn
    /// LogBackend`. If a later method breaks object safety this fails here.
    #[test]
    fn log_backend_trait_is_object_safe() {
        let mut log = MemoryBackend::<8>::new();
        let backend: &mut dyn LogBackend = &mut log;
        backend.start_write(1);
        assert!(backend.write_block(&[0xA5]));
        backend.end_write();
    }
}
