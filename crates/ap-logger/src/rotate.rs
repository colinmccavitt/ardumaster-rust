//! Log-file rotation / max-files, upstream `AP_Logger` max log files.
//!
//! `AP_Logger_File` keeps at most `get_max_num_logs()` files (a
//! parameter, often 500). When `StartNewLog` would exceed that cap it
//! unlinks the oldest (`find_oldest_log` / `RemoveFile`). This stub is
//! that cap on the mock catalog: keep at most [`max_files`](LogRotate::max_files)
//! rows and drop the oldest when a new log would overflow. Not POSIX
//! unlink, not `Prep_MinSpace`, not the io-thread erase walk.

use crate::backend::LogBackend;
use crate::file::FileBackend;
use crate::transfer::{LogEntry, LogRequestList, LogTransfer, MAX_LOGS};

/// Default mock cap. Upstream `get_max_num_logs` is a parameter
/// (often 500); this stub stays inside the [`MAX_LOGS`] table.
pub const DEFAULT_MAX_LOG_FILES: u16 = 15;

/// Rotation front-end over the file-backend mock catalog.
///
/// Owns a [`LogTransfer`] listing. Each [`Self::start_write`] that
/// would push `num_logs` past [`max_files`](Self::max_files) drops the
/// oldest catalog row first. `last_log_id` still tracks the highest
/// number seen (upstream `find_last_log`), not the remaining min.
#[derive(Debug)]
pub struct LogRotate<const N: usize> {
    transfer: LogTransfer<N>,
    max_files: u16,
}

impl<const N: usize> Default for LogRotate<N> {
    #[inline]
    fn default() -> Self {
        Self {
            transfer: LogTransfer::new(),
            max_files: clamp_max_files(DEFAULT_MAX_LOG_FILES),
        }
    }
}

impl<const N: usize> LogRotate<N> {
    /// Empty catalog, last-log id 0, default max-files cap.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Empty catalog with an explicit max-files cap.
    ///
    /// `max_files` is clamped to `1..=MAX_LOGS`. A cap of 0 is treated
    /// as 1 so a StartWrite can still open one log.
    #[must_use]
    pub fn with_max_files(max_files: u16) -> Self {
        Self {
            transfer: LogTransfer::new(),
            max_files: clamp_max_files(max_files),
        }
    }

    /// Listing / file-backend mock this rotation front-end owns.
    #[must_use]
    pub const fn transfer(&self) -> &LogTransfer<N> {
        &self.transfer
    }

    /// The file-backend mock session.
    #[must_use]
    pub const fn file(&self) -> &FileBackend<N> {
        self.transfer.file()
    }

    /// How many logs the mock catalog holds. Upstream `get_num_logs`.
    #[must_use]
    pub const fn num_logs(&self) -> u16 {
        self.transfer.num_logs()
    }

    /// Highest log number seen on a StartWrite path. Upstream `find_last_log`.
    #[must_use]
    pub const fn last_log_id(&self) -> u16 {
        self.transfer.last_log_id()
    }

    /// Catalog log-number at a 0-based index.
    #[must_use]
    pub fn log_id_at(&self, index: u16) -> Option<u16> {
        self.transfer.log_id_at(index)
    }

    /// Current max-files cap. Upstream `get_max_num_logs`.
    #[must_use]
    pub const fn max_files(&self) -> u16 {
        self.max_files
    }

    /// Change the max-files cap and drop oldest rows until `num_logs`
    /// fits. Upstream parameter write of `get_max_num_logs`.
    pub fn set_max_files(&mut self, max_files: u16) {
        self.max_files = clamp_max_files(max_files);
        self.evict_overflow();
    }

    /// Open a named log on the file-backend mock and register it.
    ///
    /// The new row is added first, then oldest rows are dropped until
    /// `num_logs` fits [`max_files`](Self::max_files). Re-opening an
    /// existing id upserts in place and does not rotate. Returns
    /// `false` when the path is rejected (empty or longer than
    /// [`LOG_FILE_PATH_MAX`]).
    #[must_use]
    pub fn start_write(&mut self, path: &str) -> bool {
        if !self.transfer.start_write(path) {
            return false;
        }
        self.evict_overflow();
        true
    }

    /// Close the current file session.
    pub fn end_write(&mut self) {
        self.transfer.end_write();
    }

    /// Append bytes through the file-backend mock.
    #[must_use]
    pub fn write_block(&mut self, buffer: &[u8]) -> bool {
        self.transfer.write_block(buffer)
    }

    /// Handle `LOG_REQUEST_LIST` against the (possibly rotated) catalog.
    pub fn handle_log_request_list(&self, req: LogRequestList, out: &mut [LogEntry]) -> usize {
        self.transfer.handle_log_request_list(req, out)
    }

    fn evict_overflow(&mut self) {
        while self.transfer.num_logs() > self.max_files {
            if self.transfer.drop_oldest().is_none() {
                break;
            }
        }
    }
}

impl<const N: usize> LogBackend for LogRotate<N> {
    fn write_block(&mut self, buffer: &[u8]) -> bool {
        LogRotate::write_block(self, buffer)
    }

    fn start_write(&mut self, _page_adr: u32) {}

    fn end_write(&mut self) {
        LogRotate::end_write(self);
    }
}

fn clamp_max_files(max_files: u16) -> u16 {
    let cap = match u16::try_from(MAX_LOGS) {
        Ok(n) => n,
        Err(_) => u16::MAX,
    };
    if max_files < 1 {
        1
    } else if max_files > cap {
        cap
    } else {
        max_files
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn start_write_over_max_files_drops_oldest() {
        let mut rot = LogRotate::<8>::with_max_files(2);
        assert_eq!(rot.max_files(), 2);
        assert_eq!(rot.num_logs(), 0);

        assert!(rot.start_write("/APM/LOGS/00000001.BIN"));
        rot.end_write();
        assert!(rot.start_write("/APM/LOGS/00000002.BIN"));
        rot.end_write();
        assert_eq!(rot.num_logs(), 2);
        assert_eq!(rot.log_id_at(0), Some(1));
        assert_eq!(rot.log_id_at(1), Some(2));
        assert_eq!(rot.last_log_id(), 2);

        assert!(rot.start_write("/APM/LOGS/00000003.BIN"));
        rot.end_write();
        assert_eq!(rot.num_logs(), 2);
        assert_eq!(rot.log_id_at(0), Some(2));
        assert_eq!(rot.log_id_at(1), Some(3));
        assert_eq!(rot.log_id_at(2), None);
        assert_eq!(rot.last_log_id(), 3);
        assert!(!rot.transfer().contains_log(1));
        assert!(rot.transfer().contains_log(2));
        assert!(rot.transfer().contains_log(3));
    }

    #[test]
    fn last_log_id_tracks_newest_after_rotation() {
        let mut rot = LogRotate::<8>::with_max_files(1);
        assert!(rot.start_write("/APM/LOGS/00000004.BIN"));
        rot.end_write();
        assert!(rot.start_write("/APM/LOGS/00000009.BIN"));
        rot.end_write();
        assert_eq!(rot.num_logs(), 1);
        assert_eq!(rot.log_id_at(0), Some(9));
        assert_eq!(rot.last_log_id(), 9);
        assert_eq!(rot.file().path(), "/APM/LOGS/00000009.BIN");
    }

    #[test]
    fn listing_reflects_rotated_catalog() {
        let mut rot = LogRotate::<16>::with_max_files(2);
        assert!(rot.start_write("/APM/LOGS/00000001.BIN"));
        assert!(rot.write_block(b"AA"));
        rot.end_write();
        assert!(rot.start_write("/APM/LOGS/00000002.BIN"));
        assert!(rot.write_block(b"BBB"));
        rot.end_write();
        assert!(rot.start_write("/APM/LOGS/00000003.BIN"));
        assert!(rot.write_block(b"C"));
        rot.end_write();

        let mut out = [LogEntry::default(); 4];
        let n = rot.handle_log_request_list(
            LogRequestList {
                start: 0,
                end: 0xffff,
                target_system: 1,
                target_component: 1,
            },
            &mut out,
        );
        assert_eq!(n, 2);
        assert_eq!(out[0].num_logs, 2);
        assert_eq!(out[0].id, 1);
        assert_eq!(out[1].id, 2);
        assert_eq!(out[0].size, 3);
        assert_eq!(out[1].size, 1);
    }

    #[test]
    fn reopen_same_id_does_not_drop() {
        let mut rot = LogRotate::<8>::with_max_files(2);
        assert!(rot.start_write("/APM/LOGS/00000001.BIN"));
        rot.end_write();
        assert!(rot.start_write("/APM/LOGS/00000002.BIN"));
        rot.end_write();
        assert!(rot.start_write("/APM/LOGS/00000001.BIN"));
        rot.end_write();
        assert_eq!(rot.num_logs(), 2);
        assert_eq!(rot.log_id_at(0), Some(1));
        assert_eq!(rot.log_id_at(1), Some(2));
        assert_eq!(rot.last_log_id(), 1);
    }

    #[test]
    fn set_max_files_trims_oldest() {
        let mut rot = LogRotate::<8>::with_max_files(3);
        assert!(rot.start_write("/APM/LOGS/00000001.BIN"));
        rot.end_write();
        assert!(rot.start_write("/APM/LOGS/00000002.BIN"));
        rot.end_write();
        assert!(rot.start_write("/APM/LOGS/00000003.BIN"));
        rot.end_write();
        assert_eq!(rot.num_logs(), 3);

        rot.set_max_files(1);
        assert_eq!(rot.max_files(), 1);
        assert_eq!(rot.num_logs(), 1);
        assert_eq!(rot.log_id_at(0), Some(3));
        assert_eq!(rot.last_log_id(), 3);
    }

    #[test]
    fn max_files_clamped_to_table() {
        let rot = LogRotate::<8>::with_max_files(0);
        assert_eq!(rot.max_files(), 1);
        let rot = LogRotate::<8>::with_max_files(500);
        assert_eq!(
            rot.max_files(),
            u16::try_from(MAX_LOGS).expect("MAX_LOGS fits u16")
        );
        assert_eq!(DEFAULT_MAX_LOG_FILES, 15);
    }

    #[test]
    fn rejected_path_does_not_rotate() {
        let mut rot = LogRotate::<8>::with_max_files(1);
        assert!(rot.start_write("/APM/LOGS/00000001.BIN"));
        rot.end_write();
        assert!(!rot.start_write(""));
        assert_eq!(rot.num_logs(), 1);
        assert_eq!(rot.log_id_at(0), Some(1));
    }
}
