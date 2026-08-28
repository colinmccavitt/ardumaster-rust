//! MAVLink log-transfer replay, upstream `AP_Logger_MAVLinkLogTransfer`.
//!
//! `LOG_REQUEST_DATA` (msgid 119) asks the vehicle for a byte range of one
//! on-disk log. This slice is that download: `handle_log_request_data` +
//! `handle_log_send_data` serving the [`FileBackend`] mock's recorded
//! bytes as [`LogData`] (`LOG_DATA`, msgid 120) chunks of
//! [`LOG_DATA_CHUNK_LEN`] (upstream `MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN`).
//! Listing stays in [`crate::transfer`]; erase is a later slice.

use crate::backend::LogBackend;
use crate::file::FileBackend;
use crate::transfer::log_id_from_path;

/// `LOG_REQUEST_DATA` message id. Upstream `MAVLINK_MSG_ID_LOG_REQUEST_DATA`.
pub const MSG_ID_LOG_REQUEST_DATA: u32 = 119;
/// `LOG_DATA` message id. Upstream `MAVLINK_MSG_ID_LOG_DATA`.
pub const MSG_ID_LOG_DATA: u32 = 120;

/// Packed `LOG_REQUEST_DATA` length (`ofs`, `count`, `id`, targets).
pub const LOG_REQUEST_DATA_LEN: usize = 12;
/// Packed `LOG_DATA` length (`ofs`, `id`, `count`, `data[90]`).
pub const LOG_DATA_LEN: usize = 97;
/// Payload bytes per `LOG_DATA`. Upstream `MAVLINK_MSG_LOG_DATA_FIELD_DATA_LEN`.
pub const LOG_DATA_CHUNK_LEN: usize = 90;

/// GCS `LOG_REQUEST_DATA` payload, upstream `mavlink_log_request_data_t`.
///
/// Wire order is size-sorted: `ofs` / `count` / `id` then the two target
/// bytes (`mavlink_msg_log_request_data_pack`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LogRequestData {
    /// Offset into the log.
    pub ofs: u32,
    /// Number of bytes requested.
    pub count: u32,
    /// 1-based catalog id from `LOG_ENTRY` (not the filename number).
    pub id: u16,
    /// Destination system id.
    pub target_system: u8,
    /// Destination component id.
    pub target_component: u8,
}

impl LogRequestData {
    /// Pack into 12 little-endian bytes. `None` if `buf` is shorter than 12.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..LOG_REQUEST_DATA_LEN)?;
        dest.get_mut(..4)?.copy_from_slice(&self.ofs.to_le_bytes());
        dest.get_mut(4..8)?.copy_from_slice(&self.count.to_le_bytes());
        dest.get_mut(8..10)?.copy_from_slice(&self.id.to_le_bytes());
        *dest.get_mut(10)? = self.target_system;
        *dest.get_mut(11)? = self.target_component;
        Some(LOG_REQUEST_DATA_LEN)
    }

    /// Unpack 12 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..LOG_REQUEST_DATA_LEN)?;
        Some(Self {
            ofs: u32::from_le_bytes([
                *src.get(0)?,
                *src.get(1)?,
                *src.get(2)?,
                *src.get(3)?,
            ]),
            count: u32::from_le_bytes([
                *src.get(4)?,
                *src.get(5)?,
                *src.get(6)?,
                *src.get(7)?,
            ]),
            id: u16::from_le_bytes([*src.get(8)?, *src.get(9)?]),
            target_system: *src.get(10)?,
            target_component: *src.get(11)?,
        })
    }
}

/// Vehicle `LOG_DATA` payload, upstream `mavlink_log_data_t`.
///
/// Wire order is size-sorted: `ofs` / `id` / `count` / `data[90]`.
/// `count == 0` is end-of-log (upstream `handle_log_send_data`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogData {
    /// Offset into the log of this chunk.
    pub ofs: u32,
    /// 1-based catalog id being sent.
    pub id: u16,
    /// Valid bytes in `data` (zero for end of log).
    pub count: u8,
    /// Chunk payload. Bytes past `count` are zero-filled.
    pub data: [u8; LOG_DATA_CHUNK_LEN],
}

impl Default for LogData {
    fn default() -> Self {
        Self {
            ofs: 0,
            id: 0,
            count: 0,
            data: [0; LOG_DATA_CHUNK_LEN],
        }
    }
}

impl LogData {
    /// Pack into 97 little-endian bytes. `None` if `buf` is shorter than 97.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..LOG_DATA_LEN)?;
        dest.get_mut(..4)?.copy_from_slice(&self.ofs.to_le_bytes());
        dest.get_mut(4..6)?.copy_from_slice(&self.id.to_le_bytes());
        *dest.get_mut(6)? = self.count;
        dest.get_mut(7..LOG_DATA_LEN)?.copy_from_slice(&self.data);
        Some(LOG_DATA_LEN)
    }

    /// Unpack 97 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..LOG_DATA_LEN)?;
        let mut data = [0u8; LOG_DATA_CHUNK_LEN];
        data.copy_from_slice(src.get(7..LOG_DATA_LEN)?);
        Some(Self {
            ofs: u32::from_le_bytes([
                *src.get(0)?,
                *src.get(1)?,
                *src.get(2)?,
                *src.get(3)?,
            ]),
            id: u16::from_le_bytes([*src.get(4)?, *src.get(5)?]),
            count: *src.get(6)?,
            data,
        })
    }

    /// The valid slice of `data` (`data[..count]`).
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        match self.data.get(..usize::from(self.count)) {
            Some(slice) => slice,
            None => &[],
        }
    }
}

/// MAVLink replay front-end over a [`FileBackend`] mock.
///
/// The mock holds one session's recorded bytes. `num_logs` is 1 after a
/// successful StartWrite (the catalog row the GCS addresses as id 1).
/// `last_log_id` is still the filename number from
/// [`log_id_from_path`]. Replay does not walk a POSIX file.
#[derive(Debug)]
pub struct LogReplay<const N: usize> {
    file: FileBackend<N>,
    sending: bool,
    log_num: u16,
    offset: u32,
    remaining: u32,
}

impl<const N: usize> Default for LogReplay<N> {
    #[inline]
    fn default() -> Self {
        Self {
            file: FileBackend::new(),
            sending: false,
            log_num: 0,
            offset: 0,
            remaining: 0,
        }
    }
}

impl<const N: usize> LogReplay<N> {
    /// An empty replay with no file session.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The file-backend mock this replay reads.
    #[must_use]
    pub const fn file(&self) -> &FileBackend<N> {
        &self.file
    }

    /// How many logs the mock can serve. One after StartWrite, else 0.
    #[must_use]
    pub fn num_logs(&self) -> u16 {
        if self.file.path().is_empty() {
            0
        } else {
            1
        }
    }

    /// Filename log number. Upstream `find_last_log`.
    #[must_use]
    pub fn last_log_id(&self) -> u16 {
        match log_id_from_path(self.file.path()) {
            Some(n) => n,
            None => {
                if self.file.path().is_empty() {
                    0
                } else {
                    1
                }
            }
        }
    }

    /// Whether a `LOG_DATA` send is in progress.
    #[must_use]
    pub const fn is_sending(&self) -> bool {
        self.sending
    }

    /// Open a named log on the file-backend mock.
    ///
    /// Cancels an in-progress download. Returns `false` when the path is
    /// rejected.
    #[must_use]
    pub fn start_write(&mut self, path: &str) -> bool {
        self.end_transfer();
        self.file.start_write(path)
    }

    /// Close the current file session. Recorded bytes stay for replay.
    pub fn end_write(&mut self) {
        self.file.end_write();
    }

    /// Append bytes through the file-backend mock.
    pub fn write_block(&mut self, buffer: &[u8]) -> bool {
        LogBackend::write_block(&mut self.file, buffer)
    }

    /// Handle `LOG_REQUEST_DATA`. Writes [`LogData`] chunks into `out`.
    ///
    /// Matches `AP_Logger::handle_log_request_data` + `handle_log_send_data`:
    /// invalid id cancels; otherwise chunks of [`LOG_DATA_CHUNK_LEN`] are
    /// taken from the file-backend mock starting at `ofs`, up to `count`
    /// bytes. A request past EOF emits one zero-count packet. Returns how
    /// many packets were written.
    pub fn handle_log_request_data(&mut self, req: LogRequestData, out: &mut [LogData]) -> usize {
        if !self.begin_request(req) {
            return 0;
        }
        let mut written = 0usize;
        while written < out.len() {
            let Some(pkt) = self.handle_log_send_data() else {
                break;
            };
            let Some(slot) = out.get_mut(written) else {
                break;
            };
            *slot = pkt;
            written = written.saturating_add(1);
        }
        written
    }

    /// Arm a download from `req`. `false` if already sending or id is bad.
    ///
    /// Upstream drops a second `LOG_REQUEST_DATA` while a send is live, and
    /// rejects `id < 1` or `id > get_num_logs`.
    #[must_use]
    pub fn begin_request(&mut self, req: LogRequestData) -> bool {
        if self.sending {
            return false;
        }
        let num = self.num_logs();
        if req.id < 1 || req.id > num {
            self.end_transfer();
            return false;
        }
        let size = self.recorded_size();
        self.log_num = req.id;
        self.offset = req.ofs;
        if req.ofs >= size {
            self.remaining = 0;
        } else {
            self.remaining = size.saturating_sub(req.ofs);
        }
        if self.remaining > req.count {
            self.remaining = req.count;
        }
        self.sending = true;
        true
    }

    /// Emit one `LOG_DATA` chunk. `None` when idle.
    ///
    /// Upstream `handle_log_send_data`: copy up to 90 bytes, zero-fill the
    /// rest, advance `ofs`, and end the transfer on a short read or when
    /// `remaining` hits 0.
    #[must_use]
    pub fn handle_log_send_data(&mut self) -> Option<LogData> {
        if !self.sending {
            return None;
        }
        let recorded = self.file.recorded();
        let start = match usize::try_from(self.offset) {
            Ok(n) => n,
            Err(_) => {
                let pkt = self.eof_packet();
                self.end_transfer();
                return Some(pkt);
            }
        };
        let want = self.remaining;
        let chunk_max = match u32::try_from(LOG_DATA_CHUNK_LEN) {
            Ok(n) => n,
            Err(_) => {
                self.end_transfer();
                return None;
            }
        };
        let take = if want > chunk_max { chunk_max } else { want };
        let avail = recorded.len().saturating_sub(start);
        let take_usize = match usize::try_from(take) {
            Ok(n) => n,
            Err(_) => 0,
        };
        let n = if take_usize > avail {
            avail
        } else {
            take_usize
        };
        let n_u8 = match u8::try_from(n) {
            Ok(v) => v,
            Err(_) => {
                self.end_transfer();
                return None;
            }
        };
        let mut data = [0u8; LOG_DATA_CHUNK_LEN];
        if n > 0 {
            let end = start.saturating_add(n);
            if let (Some(dst), Some(src)) = (data.get_mut(..n), recorded.get(start..end)) {
                dst.copy_from_slice(src);
            }
        }
        let pkt = LogData {
            ofs: self.offset,
            id: self.log_num,
            count: n_u8,
            data,
        };
        let n_u32 = match u32::try_from(n) {
            Ok(v) => v,
            Err(_) => 0,
        };
        self.offset = self.offset.saturating_add(n_u32);
        self.remaining = self.remaining.saturating_sub(n_u32);
        if n < LOG_DATA_CHUNK_LEN || self.remaining == 0 {
            self.end_transfer();
        }
        Some(pkt)
    }

    /// Stop an in-progress send. Upstream `end_log_transfer`.
    pub fn end_transfer(&mut self) {
        self.sending = false;
        self.log_num = 0;
        self.offset = 0;
        self.remaining = 0;
    }

    fn recorded_size(&self) -> u32 {
        match u32::try_from(self.file.recorded().len()) {
            Ok(n) => n,
            Err(_) => u32::MAX,
        }
    }

    fn eof_packet(&self) -> LogData {
        LogData {
            ofs: self.offset,
            id: self.log_num,
            count: 0,
            data: [0; LOG_DATA_CHUNK_LEN],
        }
    }
}

impl<const N: usize> LogBackend for LogReplay<N> {
    fn write_block(&mut self, buffer: &[u8]) -> bool {
        LogReplay::write_block(self, buffer)
    }

    fn start_write(&mut self, _page_adr: u32) {}

    fn end_write(&mut self) {
        LogReplay::end_write(self);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_request_data_payload_roundtrip() {
        let req = LogRequestData {
            ofs: 90,
            count: 180,
            id: 1,
            target_system: 1,
            target_component: 1,
        };
        let mut buf = [0u8; LOG_REQUEST_DATA_LEN];
        assert_eq!(req.encode(&mut buf), Some(LOG_REQUEST_DATA_LEN));
        assert_eq!(LogRequestData::decode(&buf), Some(req));
    }

    #[test]
    fn log_data_payload_roundtrip() {
        let mut data = [0u8; LOG_DATA_CHUNK_LEN];
        if let Some(slot) = data.get_mut(0) {
            *slot = 0xA3;
        }
        if let Some(slot) = data.get_mut(1) {
            *slot = 0x95;
        }
        let pkt = LogData {
            ofs: 10,
            id: 1,
            count: 2,
            data,
        };
        let mut buf = [0u8; LOG_DATA_LEN];
        assert_eq!(pkt.encode(&mut buf), Some(LOG_DATA_LEN));
        assert_eq!(LogData::decode(&buf), Some(pkt));
        assert_eq!(pkt.payload(), &[0xA3, 0x95]);
    }

    #[test]
    fn request_data_replays_file_backend_bytes_in_chunks() {
        let mut replay = LogReplay::<256>::new();
        assert_eq!(replay.num_logs(), 0);
        assert_eq!(replay.last_log_id(), 0);

        assert!(replay.start_write("/APM/LOGS/00000007.BIN"));
        let mut body = [0u8; 200];
        let mut i = 0usize;
        while i < body.len() {
            if let Some(slot) = body.get_mut(i) {
                *slot = (i & 0xff) as u8;
            }
            i = i.saturating_add(1);
        }
        assert!(replay.write_block(&body));
        replay.end_write();

        assert_eq!(replay.num_logs(), 1);
        assert_eq!(replay.last_log_id(), 7);
        assert_eq!(replay.file().recorded(), &body);

        let mut out = [LogData::default(); 4];
        let n = replay.handle_log_request_data(
            LogRequestData {
                ofs: 0,
                count: u32::MAX,
                id: 1,
                target_system: 1,
                target_component: 1,
            },
            &mut out,
        );
        assert_eq!(n, 3);
        assert_eq!(out[0].ofs, 0);
        assert_eq!(out[0].id, 1);
        assert_eq!(out[0].count, 90);
        assert_eq!(out[0].payload(), body.get(..90).unwrap_or(&[]));
        assert_eq!(out[1].ofs, 90);
        assert_eq!(out[1].count, 90);
        assert_eq!(out[1].payload(), body.get(90..180).unwrap_or(&[]));
        assert_eq!(out[2].ofs, 180);
        assert_eq!(out[2].count, 20);
        assert_eq!(out[2].payload(), body.get(180..200).unwrap_or(&[]));
        assert!(!replay.is_sending());
    }

    #[test]
    fn request_data_rejects_bad_id_and_honors_ofs_count() {
        let mut replay = LogReplay::<64>::new();
        assert!(replay.start_write("/APM/LOGS/00000001.BIN"));
        assert!(replay.write_block(b"ABCDEFGHIJ"));
        replay.end_write();

        let mut out = [LogData::default(); 2];
        assert_eq!(
            replay.handle_log_request_data(
                LogRequestData {
                    ofs: 0,
                    count: 4,
                    id: 0,
                    ..LogRequestData::default()
                },
                &mut out
            ),
            0
        );
        assert_eq!(
            replay.handle_log_request_data(
                LogRequestData {
                    ofs: 0,
                    count: 4,
                    id: 2,
                    ..LogRequestData::default()
                },
                &mut out
            ),
            0
        );

        let n = replay.handle_log_request_data(
            LogRequestData {
                ofs: 3,
                count: 4,
                id: 1,
                ..LogRequestData::default()
            },
            &mut out,
        );
        assert_eq!(n, 1);
        assert_eq!(out[0].ofs, 3);
        assert_eq!(out[0].count, 4);
        assert_eq!(out[0].payload(), b"DEFG");
    }

    #[test]
    fn request_past_eof_sends_zero_count() {
        let mut replay = LogReplay::<16>::new();
        assert!(replay.start_write("logs/00000002.BIN"));
        assert!(replay.write_block(b"XY"));
        replay.end_write();

        let mut out = [LogData::default(); 2];
        let n = replay.handle_log_request_data(
            LogRequestData {
                ofs: 2,
                count: 10,
                id: 1,
                ..LogRequestData::default()
            },
            &mut out,
        );
        assert_eq!(n, 1);
        assert_eq!(out[0].ofs, 2);
        assert_eq!(out[0].count, 0);
        assert!(out[0].payload().is_empty());
    }
}
