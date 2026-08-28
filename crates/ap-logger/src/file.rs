//! File logger backend, upstream `AP_Logger_File`.
//!
//! StartWrite / EndWrite here open and close a named log-file session
//! (`_write_filename` + `_write_fd`). That is not
//! [`crate::backend`]'s DataFlash page-address bookends
//! (`AP_Logger_Block::StartWrite` / `FinishWrite`).
//!
//! Bytes land in a fixed buffer so tests do not need a POSIX
//! filesystem. `logging_started` is `_write_fd != -1`: a session is
//! open after StartWrite and closed after EndWrite.

use crate::backend::LogBackend;

/// Longest stored log path. Enough for `/APM/LOGS/NNNNNNNN.BIN`.
pub const LOG_FILE_PATH_MAX: usize = 64;

/// In-memory [`LogBackend`] that records a named file session.
///
/// Not a POSIX open: `start_write` stores `_write_filename` and marks
/// the fd open; `write_block` appends into a fixed buffer; `end_write`
/// is `stop_logging` (`_write_fd = -1`). The path and recorded bytes
/// stay readable after close so a later download slice can replay them.
#[derive(Debug)]
pub struct FileBackend<const N: usize> {
    path: [u8; LOG_FILE_PATH_MAX],
    path_len: usize,
    bytes: [u8; N],
    len: usize,
    /// `_write_fd != -1`.
    open: bool,
    ended: u32,
}

impl<const N: usize> Default for FileBackend<N> {
    #[inline]
    fn default() -> Self {
        Self {
            path: [0; LOG_FILE_PATH_MAX],
            path_len: 0,
            bytes: [0; N],
            len: 0,
            open: false,
            ended: 0,
        }
    }
}

impl<const N: usize> FileBackend<N> {
    /// An empty backend with no file session.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Open a log at `path`.
    ///
    /// Upstream `AP_Logger_File::start_new_log`: `stop_logging`, then
    /// set `_write_filename` and open with `O_WRONLY|O_CREAT|O_TRUNC`.
    /// Ticket name: StartWrite. Returns `false` when `path` is empty
    /// or longer than [`LOG_FILE_PATH_MAX`].
    #[must_use]
    pub fn start_write(&mut self, path: &str) -> bool {
        if path.is_empty() || path.len() > LOG_FILE_PATH_MAX {
            return false;
        }
        if self.open {
            self.end_write();
        }
        let Some(dst) = self.path.get_mut(..path.len()) else {
            return false;
        };
        dst.copy_from_slice(path.as_bytes());
        self.path_len = path.len();
        self.len = 0;
        self.open = true;
        true
    }

    /// Close the current log.
    ///
    /// Upstream `AP_Logger_File::stop_logging` (`_write_fd = -1`).
    /// Ticket name: EndWrite. Path and recorded bytes stay so they can
    /// be inspected after the session ends.
    pub fn end_write(&mut self) {
        if !self.open {
            return;
        }
        self.open = false;
        self.ended = self.ended.saturating_add(1);
    }

    /// Stored `_write_filename`, or empty when no StartWrite succeeded.
    #[must_use]
    pub fn path(&self) -> &str {
        let Some(bytes) = self.path.get(..self.path_len) else {
            return "";
        };
        match core::str::from_utf8(bytes) {
            Ok(s) => s,
            Err(_) => "",
        }
    }

    /// Whether a file session is open. Upstream `logging_started`.
    #[must_use]
    pub const fn logging_started(&self) -> bool {
        self.open
    }

    /// Bytes accepted by [`LogBackend::write_block`] during the current
    /// (or last) session, in order.
    #[must_use]
    pub fn recorded(&self) -> &[u8] {
        match self.bytes.get(..self.len) {
            Some(slice) => slice,
            None => &[],
        }
    }

    /// How many times [`Self::end_write`] has closed a session.
    #[must_use]
    pub const fn ended_writes(&self) -> u32 {
        self.ended
    }
}

impl<const N: usize> LogBackend for FileBackend<N> {
    fn write_block(&mut self, buffer: &[u8]) -> bool {
        if !self.open {
            return false;
        }
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

    /// Page-address StartWrite is `AP_Logger_Block` only. A file
    /// session is opened with [`FileBackend::start_write`].
    fn start_write(&mut self, _page_adr: u32) {}

    fn end_write(&mut self) {
        FileBackend::end_write(self);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_write_opens_named_path_and_end_write_closes() {
        let mut log = FileBackend::<16>::new();
        assert!(!log.logging_started());
        assert_eq!(log.path(), "");
        assert!(log.start_write("/APM/LOGS/00000001.BIN"));
        assert!(log.logging_started());
        assert_eq!(log.path(), "/APM/LOGS/00000001.BIN");
        assert!(log.write_block(b"DF"));
        log.end_write();
        assert!(!log.logging_started());
        assert_eq!(log.ended_writes(), 1);
        assert_eq!(log.path(), "/APM/LOGS/00000001.BIN");
        assert_eq!(log.recorded(), b"DF");
        assert!(!log.write_block(b"X"));
        assert_eq!(log.recorded(), b"DF");
    }

    #[test]
    fn start_write_rejects_empty_and_too_long_path() {
        let mut log = FileBackend::<8>::new();
        assert!(!log.start_write(""));
        assert!(!log.logging_started());
        let too_long = [b'A'; LOG_FILE_PATH_MAX + 1];
        let Some(path) = core::str::from_utf8(&too_long).ok() else {
            panic!("ascii");
        };
        assert!(!log.start_write(path));
        assert!(!log.logging_started());
    }

    #[test]
    fn start_write_again_stops_and_truncates() {
        let mut log = FileBackend::<16>::new();
        assert!(log.start_write("logs/00000001.BIN"));
        assert!(log.write_block(b"OLD"));
        assert!(log.start_write("logs/00000002.BIN"));
        assert_eq!(log.ended_writes(), 1);
        assert_eq!(log.path(), "logs/00000002.BIN");
        assert!(log.logging_started());
        assert!(log.recorded().is_empty());
        assert!(log.write_block(b"NEW"));
        assert_eq!(log.recorded(), b"NEW");
    }
}
