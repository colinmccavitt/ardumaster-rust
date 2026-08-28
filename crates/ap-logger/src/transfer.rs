//! MAVLink log-transfer listing, upstream `AP_Logger_MAVLinkLogTransfer`.
//!
//! `LOG_REQUEST_LIST` (msgid 117) asks the vehicle for the on-disk catalog.
//! This slice is that listing: `get_num_logs` / `find_last_log` taken from
//! the [`FileBackend`] mock, and `handle_log_request_list` emitting
//! [`LogEntry`] (`LOG_ENTRY`, msgid 118). Not `LOG_REQUEST_DATA` /
//! `LOG_DATA` replay, erase, or download lockout.

use crate::backend::LogBackend;
use crate::file::FileBackend;

/// `LOG_REQUEST_LIST` message id. Upstream `MAVLINK_MSG_ID_LOG_REQUEST_LIST`.
pub const MSG_ID_LOG_REQUEST_LIST: u32 = 117;
/// `LOG_ENTRY` message id. Upstream `MAVLINK_MSG_ID_LOG_ENTRY`.
pub const MSG_ID_LOG_ENTRY: u32 = 118;

/// Packed `LOG_REQUEST_LIST` length (`start`, `end`, `target_system`,
/// `target_component`).
pub const LOG_REQUEST_LIST_LEN: usize = 6;
/// Packed `LOG_ENTRY` length (`time_utc`, `size`, `id`, `num_logs`,
/// `last_log_num`).
pub const LOG_ENTRY_LEN: usize = 14;

/// Stub catalog cap. Upstream `get_max_num_logs` is a parameter (often 500);
/// this mock keeps a small fixed table so listing stays `no_std`.
pub const MAX_LOGS: usize = 16;

/// GCS `LOG_REQUEST_LIST` payload, upstream `mavlink_log_request_list_t`.
///
/// Wire order is size-sorted: `start` / `end` then the two target bytes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LogRequestList {
    /// First list entry. `0` means the first available log.
    pub start: u16,
    /// Last list entry. `0xffff` means the last available log.
    pub end: u16,
    /// Destination system id.
    pub target_system: u8,
    /// Destination component id.
    pub target_component: u8,
}

impl LogRequestList {
    /// Pack into 6 little-endian bytes. `None` if `buf` is shorter than 6.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..LOG_REQUEST_LIST_LEN)?;
        dest.get_mut(..2)?.copy_from_slice(&self.start.to_le_bytes());
        dest.get_mut(2..4)?.copy_from_slice(&self.end.to_le_bytes());
        *dest.get_mut(4)? = self.target_system;
        *dest.get_mut(5)? = self.target_component;
        Some(LOG_REQUEST_LIST_LEN)
    }

    /// Unpack 6 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..LOG_REQUEST_LIST_LEN)?;
        Some(Self {
            start: u16::from_le_bytes([*src.get(0)?, *src.get(1)?]),
            end: u16::from_le_bytes([*src.get(2)?, *src.get(3)?]),
            target_system: *src.get(4)?,
            target_component: *src.get(5)?,
        })
    }
}

/// Vehicle `LOG_ENTRY` payload, upstream `mavlink_log_entry_t`.
///
/// Wire order is size-sorted: `time_utc` / `size` then the three `uint16`
/// fields. `last_log_num` is the last list entry being sent (upstream
/// `_log_last_list_entry`), not [`LogTransfer::last_log_id`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LogEntry {
    /// UTC timestamp of the log, or 0 when unknown.
    pub time_utc: u32,
    /// Size of the log in bytes.
    pub size: u32,
    /// 1-based list entry (upstream `_log_next_list_entry`).
    pub id: u16,
    /// Total number of logs. Upstream `get_num_logs`.
    pub num_logs: u16,
    /// Last list entry in this reply. Upstream `_log_last_list_entry`.
    pub last_log_num: u16,
}

impl LogEntry {
    /// Pack into 14 little-endian bytes. `None` if `buf` is shorter than 14.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..LOG_ENTRY_LEN)?;
        dest.get_mut(..4)?.copy_from_slice(&self.time_utc.to_le_bytes());
        dest.get_mut(4..8)?.copy_from_slice(&self.size.to_le_bytes());
        dest.get_mut(8..10)?.copy_from_slice(&self.id.to_le_bytes());
        dest.get_mut(10..12)?.copy_from_slice(&self.num_logs.to_le_bytes());
        dest.get_mut(12..14)?
            .copy_from_slice(&self.last_log_num.to_le_bytes());
        Some(LOG_ENTRY_LEN)
    }

    /// Unpack 14 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..LOG_ENTRY_LEN)?;
        Some(Self {
            time_utc: u32::from_le_bytes([
                *src.get(0)?,
                *src.get(1)?,
                *src.get(2)?,
                *src.get(3)?,
            ]),
            size: u32::from_le_bytes([
                *src.get(4)?,
                *src.get(5)?,
                *src.get(6)?,
                *src.get(7)?,
            ]),
            id: u16::from_le_bytes([*src.get(8)?, *src.get(9)?]),
            num_logs: u16::from_le_bytes([*src.get(10)?, *src.get(11)?]),
            last_log_num: u16::from_le_bytes([*src.get(12)?, *src.get(13)?]),
        })
    }
}

/// Parse the numeric id from a file-backend path (`/APM/LOGS/00000007.BIN` -> 7).
///
/// Upstream `_log_file_name` / `dirent_to_log_num`. `None` when the
/// filename is not a decimal log number (optionally `.BIN` / `.bin`).
#[must_use]
pub fn log_id_from_path(path: &str) -> Option<u16> {
    let name = match path.rsplit('/').next() {
        Some(n) => n,
        None => path,
    };
    let stem = match name
        .strip_suffix(".BIN")
        .or_else(|| name.strip_suffix(".bin"))
    {
        Some(s) => s,
        None => name,
    };
    if stem.is_empty() {
        return None;
    }
    let mut n: u32 = 0;
    let mut digits = 0u8;
    let bytes = stem.as_bytes();
    let mut i = 0usize;
    while i < bytes.len() {
        let Some(&b) = bytes.get(i) else {
            break;
        };
        if !b.is_ascii_digit() {
            return None;
        }
        n = n.saturating_mul(10).saturating_add(u32::from(b - b'0'));
        if n > u32::from(u16::MAX) {
            return None;
        }
        digits = digits.saturating_add(1);
        i = i.saturating_add(1);
    }
    if digits == 0 {
        return None;
    }
    u16::try_from(n).ok()
}

/// MAVLink listing front-end over a [`FileBackend`] mock.
///
/// Each [`Self::start_write`] records a catalog row from the named path
/// (log id + later size). `num_logs` / `last_log_id` are
/// `AP_Logger_File::get_num_logs` / `find_last_log`. Listing does not
/// walk a POSIX directory.
#[derive(Debug)]
pub struct LogTransfer<const N: usize> {
    file: FileBackend<N>,
    ids: [u16; MAX_LOGS],
    sizes: [u32; MAX_LOGS],
    count: u16,
    last_log_id: u16,
}

impl<const N: usize> Default for LogTransfer<N> {
    #[inline]
    fn default() -> Self {
        Self {
            file: FileBackend::new(),
            ids: [0; MAX_LOGS],
            sizes: [0; MAX_LOGS],
            count: 0,
            last_log_id: 0,
        }
    }
}

impl<const N: usize> LogTransfer<N> {
    /// An empty catalog with no file session.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The file-backend mock this listing reads.
    #[must_use]
    pub const fn file(&self) -> &FileBackend<N> {
        &self.file
    }

    /// How many logs the mock has opened. Upstream `get_num_logs`.
    #[must_use]
    pub const fn num_logs(&self) -> u16 {
        self.count
    }

    /// Highest log number seen on a StartWrite path. Upstream `find_last_log`.
    #[must_use]
    pub const fn last_log_id(&self) -> u16 {
        self.last_log_id
    }

    /// Open a named log on the file-backend mock and register it.
    ///
    /// Snapshots the previous session's size first — `FileBackend::start_write`
    /// truncates the buffer. Returns `false` when the path is rejected.
    #[must_use]
    pub fn start_write(&mut self, path: &str) -> bool {
        self.snapshot_current();
        if !self.file.start_write(path) {
            return false;
        }
        let id = match log_id_from_path(path) {
            Some(n) => n,
            None => self.last_log_id.saturating_add(1),
        };
        self.last_log_id = id;
        self.upsert(id, 0);
        true
    }

    /// Close the current file session, keeping catalog size from recorded bytes.
    pub fn end_write(&mut self) {
        self.snapshot_current();
        self.file.end_write();
    }

    /// Append bytes through the file-backend mock and refresh that log's size.
    pub fn write_block(&mut self, buffer: &[u8]) -> bool {
        if !LogBackend::write_block(&mut self.file, buffer) {
            return false;
        }
        self.snapshot_current();
        true
    }

    /// Handle `LOG_REQUEST_LIST`. Writes [`LogEntry`] rows into `out`.
    ///
    /// Matches `AP_Logger::handle_log_request_list` + `handle_log_send_listing`:
    /// no logs yields one zero entry; otherwise entries `start..=end` are
    /// clamped to `1..=num_logs`. Returns how many entries were written.
    pub fn handle_log_request_list(&self, req: LogRequestList, out: &mut [LogEntry]) -> usize {
        let num = self.num_logs();
        if num == 0 {
            let Some(slot) = out.get_mut(0) else {
                return 0;
            };
            *slot = LogEntry {
                time_utc: 0,
                size: 0,
                id: 0,
                num_logs: 0,
                last_log_num: 0,
            };
            return 1;
        }

        let mut start = req.start;
        let mut end = req.end;
        if end > num {
            end = num;
        }
        if start < 1 {
            start = 1;
        }
        if start > end {
            return 0;
        }

        let mut written = 0usize;
        let mut entry = start;
        while entry <= end {
            let Some(slot) = out.get_mut(written) else {
                break;
            };
            let idx = usize::from(entry.saturating_sub(1));
            let size = match self.sizes.get(idx) {
                Some(&s) => s,
                None => 0,
            };
            *slot = LogEntry {
                time_utc: 0,
                size,
                id: entry,
                num_logs: num,
                last_log_num: end,
            };
            written = written.saturating_add(1);
            entry = match entry.checked_add(1) {
                Some(v) => v,
                None => break,
            };
        }
        written
    }

    fn snapshot_current(&mut self) {
        let path = self.file.path();
        if path.is_empty() {
            return;
        }
        let id = match log_id_from_path(path) {
            Some(n) => n,
            None => self.last_log_id,
        };
        if id == 0 {
            return;
        }
        let size = match u32::try_from(self.file.recorded().len()) {
            Ok(n) => n,
            Err(_) => u32::MAX,
        };
        self.upsert(id, size);
    }

    fn upsert(&mut self, id: u16, size: u32) {
        let mut i = 0usize;
        while i < usize::from(self.count) {
            if self.ids.get(i).copied() == Some(id) {
                if let Some(slot) = self.sizes.get_mut(i) {
                    *slot = size;
                }
                return;
            }
            i = i.saturating_add(1);
        }
        if usize::from(self.count) >= MAX_LOGS {
            return;
        }
        let idx = usize::from(self.count);
        let Some(id_slot) = self.ids.get_mut(idx) else {
            return;
        };
        let Some(sz_slot) = self.sizes.get_mut(idx) else {
            return;
        };
        *id_slot = id;
        *sz_slot = size;
        self.count = self.count.saturating_add(1);
    }
}

impl<const N: usize> LogBackend for LogTransfer<N> {
    fn write_block(&mut self, buffer: &[u8]) -> bool {
        LogTransfer::write_block(self, buffer)
    }

    fn start_write(&mut self, _page_adr: u32) {}

    fn end_write(&mut self) {
        LogTransfer::end_write(self);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_id_from_typical_file_backend_path() {
        assert_eq!(log_id_from_path("/APM/LOGS/00000001.BIN"), Some(1));
        assert_eq!(log_id_from_path("logs/00000007.bin"), Some(7));
        assert_eq!(log_id_from_path("00000012.BIN"), Some(12));
        assert_eq!(log_id_from_path("LASTLOG.TXT"), None);
        assert_eq!(log_id_from_path(""), None);
    }

    #[test]
    fn empty_catalog_lists_one_zero_entry() {
        let xfer = LogTransfer::<8>::new();
        assert_eq!(xfer.num_logs(), 0);
        assert_eq!(xfer.last_log_id(), 0);
        let mut out = [LogEntry::default(); 2];
        let n = xfer.handle_log_request_list(
            LogRequestList {
                start: 0,
                end: 0xffff,
                target_system: 1,
                target_component: 1,
            },
            &mut out,
        );
        assert_eq!(n, 1);
        assert_eq!(out[0].id, 0);
        assert_eq!(out[0].num_logs, 0);
        assert_eq!(out[0].last_log_num, 0);
        assert_eq!(out[0].size, 0);
    }

    #[test]
    fn request_list_uses_file_backend_count_and_last_id() {
        let mut xfer = LogTransfer::<32>::new();
        assert!(xfer.start_write("/APM/LOGS/00000003.BIN"));
        assert!(xfer.write_block(b"ABC"));
        xfer.end_write();
        assert!(xfer.start_write("/APM/LOGS/00000004.BIN"));
        assert!(xfer.write_block(b"WXYZ"));
        xfer.end_write();

        assert_eq!(xfer.num_logs(), 2);
        assert_eq!(xfer.last_log_id(), 4);
        assert_eq!(xfer.file().path(), "/APM/LOGS/00000004.BIN");
        assert_eq!(xfer.file().recorded(), b"WXYZ");

        let mut out = [LogEntry::default(); 4];
        let n = xfer.handle_log_request_list(
            LogRequestList {
                start: 0,
                end: 0xffff,
                ..LogRequestList::default()
            },
            &mut out,
        );
        assert_eq!(n, 2);
        assert_eq!(out[0].id, 1);
        assert_eq!(out[0].num_logs, 2);
        assert_eq!(out[0].size, 3);
        assert_eq!(out[0].last_log_num, 2);
        assert_eq!(out[1].id, 2);
        assert_eq!(out[1].size, 4);
        assert_eq!(out[1].last_log_num, 2);
    }

    #[test]
    fn request_list_clamps_range() {
        let mut xfer = LogTransfer::<16>::new();
        assert!(xfer.start_write("/APM/LOGS/00000001.BIN"));
        assert!(xfer.write_block(&[1]));
        assert!(xfer.start_write("/APM/LOGS/00000002.BIN"));
        assert!(xfer.write_block(&[2, 3]));
        xfer.end_write();

        let mut out = [LogEntry::default(); 4];
        let n = xfer.handle_log_request_list(
            LogRequestList {
                start: 2,
                end: 9,
                ..LogRequestList::default()
            },
            &mut out,
        );
        assert_eq!(n, 1);
        assert_eq!(out[0].id, 2);
        assert_eq!(out[0].num_logs, 2);
        assert_eq!(out[0].last_log_num, 2);
        assert_eq!(out[0].size, 2);
    }

    #[test]
    fn log_entry_payload_roundtrip() {
        let entry = LogEntry {
            time_utc: 0,
            size: 12,
            id: 3,
            num_logs: 5,
            last_log_num: 5,
        };
        let mut buf = [0u8; LOG_ENTRY_LEN];
        assert_eq!(entry.encode(&mut buf), Some(LOG_ENTRY_LEN));
        assert_eq!(LogEntry::decode(&buf), Some(entry));
    }
}
