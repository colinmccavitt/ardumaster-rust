//! Log erase / `LOG_ERASE`, upstream `AP_Logger::EraseAll`.
//!
//! `LOG_ERASE` (msgid 121) asks the vehicle to wipe onboard logs.
//! `handle_log_request_erase` calls `EraseAll()`, which
//! `AP_Logger_File` implements by `stop_logging` then walking the
//! catalog. This stub is that wipe on the file-backend mock: clear
//! the listing catalog, reset `find_last_log`, and zero `_dropped`.
//! Not POSIX unlink, not the armed-guard, not the io-thread
//! `erase.log_num` walk.

use crate::backend::LogBackend;
use crate::dropped::DroppedMessages;
use crate::file::FileBackend;
use crate::structure::LogStructure;
use crate::transfer::{LogEntry, LogRequestList, LogTransfer};
use crate::write::LogValue;

/// `LOG_ERASE` message id. Upstream `MAVLINK_MSG_ID_LOG_ERASE`.
pub const MSG_ID_LOG_ERASE: u32 = 121;

/// Packed `LOG_ERASE` length (`target_system`, `target_component`).
pub const LOG_ERASE_LEN: usize = 2;

/// GCS `LOG_ERASE` payload, upstream `mavlink_log_erase_t`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LogEraseRequest {
    /// Destination system id.
    pub target_system: u8,
    /// Destination component id.
    pub target_component: u8,
}

impl LogEraseRequest {
    /// Pack into 2 bytes. `None` if `buf` is shorter than 2.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..LOG_ERASE_LEN)?;
        *dest.get_mut(0)? = self.target_system;
        *dest.get_mut(1)? = self.target_component;
        Some(LOG_ERASE_LEN)
    }

    /// Unpack 2 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..LOG_ERASE_LEN)?;
        Some(Self {
            target_system: *src.get(0)?,
            target_component: *src.get(1)?,
        })
    }
}

/// Erase front-end over the file-backend mock catalog.
///
/// Owns a [`LogTransfer`] listing plus the [`DroppedMessages`] counter
/// so `EraseAll` can reset both. Writes go through the drop counter
/// (`WritePrioritisedBlock` when the mock buffer is full).
#[derive(Debug)]
pub struct LogErase<const N: usize> {
    transfer: LogTransfer<N>,
    dropped: DroppedMessages,
}

impl<const N: usize> Default for LogErase<N> {
    #[inline]
    fn default() -> Self {
        Self {
            transfer: LogTransfer::new(),
            dropped: DroppedMessages::new(),
        }
    }
}

impl<const N: usize> LogErase<N> {
    /// Empty catalog, last-log id 0, drop count 0.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Listing / file-backend mock this erase front-end owns.
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

    /// Buffer-full drops since the last erase / start. Upstream `num_dropped`.
    #[must_use]
    pub const fn num_dropped(&self) -> u32 {
        self.dropped.num_dropped()
    }

    /// Open a named log on the file-backend mock and register it.
    #[must_use]
    pub fn start_write(&mut self, path: &str) -> bool {
        self.transfer.start_write(path)
    }

    /// Close the current file session.
    pub fn end_write(&mut self) {
        self.transfer.end_write();
    }

    /// Append bytes; increment `_dropped` when the mock buffer is full.
    #[must_use]
    pub fn write_block(&mut self, buffer: &[u8]) -> bool {
        self.dropped.write_block(&mut self.transfer, buffer)
    }

    /// Pack a FMT-described message and `WriteBlock` it.
    ///
    /// Increments [`num_dropped`](Self::num_dropped) only when the
    /// backend is full. Packing failures are not drops.
    #[must_use]
    pub fn write(&mut self, structure: &LogStructure, fields: &[LogValue<'_>]) -> bool {
        self.dropped.write(&mut self.transfer, structure, fields)
    }

    /// Wipe onboard logs. Upstream `AP_Logger::EraseAll`.
    ///
    /// `stop_logging`, then clear the file-backend mock catalog,
    /// `find_last_log`, recorded bytes, and `_dropped`.
    pub fn erase_all(&mut self) {
        self.transfer.end_write();
        self.transfer = LogTransfer::new();
        self.dropped.clear();
    }

    /// Handle `LOG_ERASE`. Upstream `handle_log_request_erase`.
    ///
    /// The payload is accepted for wire symmetry; dest ids are not
    /// filtered (upstream decodes then ignores the packet).
    pub fn handle_log_erase(&mut self, _req: LogEraseRequest) {
        self.erase_all();
    }

    /// Handle `LOG_REQUEST_LIST` against the (possibly empty) catalog.
    pub fn handle_log_request_list(&self, req: LogRequestList, out: &mut [LogEntry]) -> usize {
        self.transfer.handle_log_request_list(req, out)
    }
}

impl<const N: usize> LogBackend for LogErase<N> {
    fn write_block(&mut self, buffer: &[u8]) -> bool {
        LogErase::write_block(self, buffer)
    }

    fn start_write(&mut self, _page_adr: u32) {}

    fn end_write(&mut self) {
        LogErase::end_write(self);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::write::calc_msg_len;

    fn imu_row() -> LogStructure {
        LogStructure {
            msg_type: 7,
            msg_len: calc_msg_len("BH").expect("len"),
            name: "IMU",
            format: "BH",
            labels: "I,V",
        }
    }

    #[test]
    fn erase_all_clears_catalog_last_id_and_drop_count() {
        let mut erase = LogErase::<8>::new();
        assert_eq!(erase.num_logs(), 0);
        assert_eq!(erase.last_log_id(), 0);
        assert_eq!(erase.num_dropped(), 0);

        assert!(erase.start_write("/APM/LOGS/00000003.BIN"));
        assert!(erase.write_block(b"ABCDEFGH"));
        assert!(!erase.write_block(b"X"));
        erase.end_write();
        assert!(erase.start_write("/APM/LOGS/00000007.BIN"));
        erase.end_write();

        assert_eq!(erase.num_logs(), 2);
        assert_eq!(erase.last_log_id(), 7);
        assert_eq!(erase.num_dropped(), 1);
        assert_eq!(erase.file().path(), "/APM/LOGS/00000007.BIN");

        erase.erase_all();

        assert_eq!(erase.num_logs(), 0);
        assert_eq!(erase.last_log_id(), 0);
        assert_eq!(erase.num_dropped(), 0);
        assert_eq!(erase.file().path(), "");
        assert!(!erase.file().logging_started());
        assert!(erase.file().recorded().is_empty());

        let mut out = [LogEntry::default(); 2];
        let n = erase.handle_log_request_list(
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
    }

    #[test]
    fn handle_log_erase_wipes_like_erase_all() {
        let mut erase = LogErase::<16>::new();
        assert!(erase.start_write("/APM/LOGS/00000001.BIN"));
        let row = imu_row();
        let fields = [LogValue::U8(3), LogValue::U16(9)];
        assert!(erase.write(&row, &fields));
        erase.end_write();
        assert_eq!(erase.num_logs(), 1);
        assert_eq!(erase.last_log_id(), 1);

        erase.handle_log_erase(LogEraseRequest {
            target_system: 1,
            target_component: 1,
        });
        assert_eq!(erase.num_logs(), 0);
        assert_eq!(erase.last_log_id(), 0);
        assert_eq!(erase.num_dropped(), 0);
        assert_eq!(erase.file().path(), "");
    }

    #[test]
    fn log_erase_payload_roundtrip() {
        let req = LogEraseRequest {
            target_system: 1,
            target_component: 191,
        };
        let mut buf = [0u8; LOG_ERASE_LEN];
        assert_eq!(req.encode(&mut buf), Some(LOG_ERASE_LEN));
        assert_eq!(LogEraseRequest::decode(&buf), Some(req));
        assert_eq!(MSG_ID_LOG_ERASE, 121);
        assert!(LogEraseRequest::decode(&[1]).is_none());
    }
}
