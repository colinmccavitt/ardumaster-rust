//! Dataflash logging, upstream `libraries/AP_Logger`. FW-030.
//!
//! The write-path seam is a [`LogBackend`] that accepts a block of
//! bytes (`WriteBlock`) and the page-write bookends (`StartWrite` /
//! `EndWrite`). Message identity lives in [`structure`]: the three-byte
//! packet header and the FMT table entry (`type`, `length`, `name`,
//! `format`, `labels`) from upstream `LogStructure.h`.
//!
//! # What this crate does not include yet
//!
//! The DataFlash page map, log rotation, rate limiting, the Write()
//! typed-message dispatcher, and the `AP_Logger` front-end. Those land
//! in later FW-030 slices.

#![no_std]

pub mod backend;
pub mod structure;

pub use backend::{LogBackend, MemoryBackend};
pub use structure::{
    fill_format, LogFormat, LogPacketHeader, LogStructure, FMT_FORMAT_LEN, FMT_LABELS_LEN,
    FMT_NAME_LEN, HEAD_BYTE1, HEAD_BYTE2, LOG_FORMAT_LEN, LOG_FORMAT_MSG, LOG_PACKET_HEADER_LEN,
    LS_FORMAT_SIZE, LS_LABELS_SIZE, LS_NAME_SIZE,
};
