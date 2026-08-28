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
//! when the class is disabled or no log is open.
//!
//! # What this crate does not include yet
//!
//! The DataFlash page map, log rotation, rate limiting, the file
//! backend, and the `AP_Logger` front-end. Those land in later FW-030
//! slices.

#![no_std]

pub mod backend;
pub mod gate;
pub mod structure;
pub mod write;

pub use backend::{LogBackend, MemoryBackend};
pub use gate::{
    LogGate, DEFAULT_LOG_BITMASK, MASK_LOG_ATTITUDE_FAST, MASK_LOG_ATTITUDE_FULLRATE,
    MASK_LOG_ATTITUDE_MED, MASK_LOG_CAMERA, MASK_LOG_CMD, MASK_LOG_COMPASS, MASK_LOG_CTUN,
    MASK_LOG_CURRENT, MASK_LOG_GPS, MASK_LOG_IMU, MASK_LOG_IMU_RAW, MASK_LOG_NOTCH_FULLRATE,
    MASK_LOG_NTUN, MASK_LOG_PM, MASK_LOG_RC, MASK_LOG_SONAR, MASK_LOG_TECS,
    MASK_LOG_VIDEO_STABILISATION,
};
pub use structure::{
    fill_format, LogFormat, LogPacketHeader, LogStructure, FMT_FORMAT_LEN, FMT_LABELS_LEN,
    FMT_NAME_LEN, HEAD_BYTE1, HEAD_BYTE2, LOG_FORMAT_LEN, LOG_FORMAT_MSG, LOG_PACKET_HEADER_LEN,
    LS_FORMAT_SIZE, LS_LABELS_SIZE, LS_NAME_SIZE,
};
pub use write::{
    calc_msg_len, field_size, pack_message, write_message, LogValue, LOG_PACKET_MAX_LEN,
};
