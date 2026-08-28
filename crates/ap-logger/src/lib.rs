//! Dataflash logging, upstream `libraries/AP_Logger`. FW-030.
//!
//! The write-path seam is a [`LogBackend`] that accepts a block of
//! bytes (`WriteBlock`) and the page-write bookends (`StartWrite` /
//! `EndWrite`). Message identity lives in [`structure`]: the three-byte
//! packet header and the FMT table entry (`type`, `length`, `name`,
//! `format`, `labels`) from upstream `LogStructure.h`. Typed messages
//! go through [`write`]: `Write()` packs a FMT-described payload and
//! hands it to `WriteBlock`. [`gate`] is the front-end `LOG_BITMASK`
//! class enable plus the logging-started latch: `Write` is a no-op
//! when the class is disabled or no log is open. [`file`] is
//! `AP_Logger_File`: StartWrite / EndWrite open and close a named
//! log-file session (`_write_filename`) into a buffer mock.
//! [`transfer`] is MAVLink log-transfer listing: `LOG_REQUEST_LIST`
//! (msgid 117) answered with `LOG_ENTRY` (msgid 118), using log count
//! and last-log id from the file-backend mock. [`replay`] is MAVLink
//! log-transfer download: `LOG_REQUEST_DATA` (msgid 119) answered with
//! `LOG_DATA` (msgid 120) chunks of recorded bytes from the
//! file-backend mock. [`dropped`] is the buffer-full counter:
//! `_dropped` increments when `Write` is rejected by a full backend
//! and `num_dropped()` exposes the count. [`erase`] is MAVLink log
//! erase: `LOG_ERASE` (msgid 121) calls `EraseAll()`, which clears the
//! file-backend mock catalog, last-log id, and drop count. [`rotate`]
//! is max-files rotation: `get_max_num_logs` caps the mock catalog and
//! drops the oldest row when a new StartWrite would overflow.
//! [`streaming`] is `WriteStreaming`: a rate-limited periodic write
//! gate that only emits when `1000 / rate_hz` milliseconds have
//! elapsed since the last send of that msgid. [`registry`] is the
//! FMT / `LogStructure` table: register rows, then look up `type`
//! and `length` by message name (`msg_fmt_for_name`).
//!
//! # What this crate does not include yet
//!
//! The DataFlash page map and the `AP_Logger` front-end. Those land
//! in later FW-030 slices.

#![no_std]

pub mod backend;
pub mod dropped;
pub mod erase;
pub mod file;
pub mod gate;
pub mod registry;
pub mod replay;
pub mod rotate;
pub mod streaming;
pub mod structure;
pub mod transfer;
pub mod write;

pub use backend::{LogBackend, MemoryBackend};
pub use dropped::DroppedMessages;
pub use erase::{LogErase, LogEraseRequest, LOG_ERASE_LEN, MSG_ID_LOG_ERASE};
pub use file::{FileBackend, LOG_FILE_PATH_MAX};
pub use gate::{
    LogGate, DEFAULT_LOG_BITMASK, MASK_LOG_ATTITUDE_FAST, MASK_LOG_ATTITUDE_FULLRATE,
    MASK_LOG_ATTITUDE_MED, MASK_LOG_CAMERA, MASK_LOG_CMD, MASK_LOG_COMPASS, MASK_LOG_CTUN,
    MASK_LOG_CURRENT, MASK_LOG_GPS, MASK_LOG_IMU, MASK_LOG_IMU_RAW, MASK_LOG_NOTCH_FULLRATE,
    MASK_LOG_NTUN, MASK_LOG_PM, MASK_LOG_RC, MASK_LOG_SONAR, MASK_LOG_TECS,
    MASK_LOG_VIDEO_STABILISATION,
};
pub use registry::{FmtRegistry, MAX_FMT_ROWS};
pub use replay::{
    LogData, LogReplay, LogRequestData, LOG_DATA_CHUNK_LEN, LOG_DATA_LEN, LOG_REQUEST_DATA_LEN,
    MSG_ID_LOG_DATA, MSG_ID_LOG_REQUEST_DATA,
};
pub use rotate::{LogRotate, DEFAULT_MAX_LOG_FILES};
pub use streaming::{WriteStreaming, DEFAULT_STREAM_RATE_HZ};
pub use structure::{
    fill_format, LogFormat, LogPacketHeader, LogStructure, FMT_FORMAT_LEN, FMT_LABELS_LEN,
    FMT_NAME_LEN, HEAD_BYTE1, HEAD_BYTE2, LOG_FORMAT_LEN, LOG_FORMAT_MSG, LOG_PACKET_HEADER_LEN,
    LS_FORMAT_SIZE, LS_LABELS_SIZE, LS_NAME_SIZE,
};
pub use transfer::{
    log_id_from_path, LogEntry, LogRequestList, LogTransfer, LOG_ENTRY_LEN, LOG_REQUEST_LIST_LEN,
    MAX_LOGS, MSG_ID_LOG_ENTRY, MSG_ID_LOG_REQUEST_LIST,
};
pub use write::{
    calc_msg_len, field_size, pack_message, write_message, LogValue, LOG_PACKET_MAX_LEN,
};
