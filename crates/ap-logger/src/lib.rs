//! Dataflash logging backend, upstream `libraries/AP_Logger`. FW-030.
//!
//! This slice is the write-path seam: a [`LogBackend`] that accepts a block
//! of bytes (`WriteBlock`) and the page-write bookends (`StartWrite` /
//! `EndWrite`). The in-memory backend records those bytes so later FMT and
//! typed-message work can be tested without a DataFlash chip or filesystem.
//!
//! # What this slice does not include
//!
//! The DataFlash page map, log rotation, FMT emission, rate limiting, and
//! the `AP_Logger` front-end. Those land in later FW-030 slices. This crate
//! is the backend trait, not a logger.

#![no_std]

pub mod backend;

pub use backend::{LogBackend, MemoryBackend};
