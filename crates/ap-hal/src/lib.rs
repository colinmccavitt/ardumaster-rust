//! Port of the ArduPilot `libraries/AP_HAL` interface, pinned to `Plane-4.7.0`.
//!
//! Tracked as **FW-001**. This crate is the seam the whole port hangs from: it
//! defines the hardware boundary as traits, with no behavior of its own, which
//! is why its verification is `compile-only`.
//!
//! ADR-0004 governs three things here:
//! - Fallible operations return [`Result`], never panic and never throw;
//!   upstream builds `-fno-exceptions` so there is no unwinding to mirror.
//! - The crate is `no_std`, matching how upstream compiles for ChibiOS targets.
//! - **No singletons.** Upstream reaches subsystems through 114 `AP::foo()`
//!   global accessors. Those are deliberately not reproduced; subsystem
//!   references are grouped into an explicit context and threaded through the
//!   scheduler instead. This is the one place the port's call sites diverge
//!   textually from upstream on purpose.

#![no_std]

/// Errors surfaced across the HAL boundary.
///
/// Upstream signals failure with `bool` returns, sentinel values, and
/// out-parameters. Ported code must reproduce that behavior rather than
/// widening it: where upstream returns `false` and the caller ignores it, the
/// port ignores it too. Improving on upstream error handling is a separate
/// ticket, never a side effect of porting (ADR-0003).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// The device or peripheral is not present.
    NotPresent,
    /// The operation timed out.
    Timeout,
    /// The operation is not supported by this backend.
    Unsupported,
    /// The transfer failed at the bus level.
    BusError,
}

/// Result alias for HAL operations.
pub type Result<T> = core::result::Result<T, Error>;

pub mod analog;
pub mod context;
pub mod device;
pub mod gpio;
pub mod internal_error;
pub mod rc;
pub mod semaphore;
pub mod serial;
pub mod storage;
pub mod time;
pub mod util;
