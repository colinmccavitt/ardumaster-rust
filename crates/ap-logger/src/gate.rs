//! `LOG_BITMASK` class enable and logging-started latch.
//!
//! Upstream `AP_Logger::should_log` / `AP_Logger::logging_started`.
//! Vehicle code asks `should_log(MASK_LOG_*)` before emitting a class;
//! backends report `logging_started` once a log is open. This slice is
//! that pair — not armed / log-while-disarmed, not download lockout,
//! not the file backend.
//!
//! `Write` through [`LogGate::write`] is a no-op when the class bit is
//! off or the started latch is still clear.

use crate::backend::LogBackend;
use crate::structure::LogStructure;
use crate::write::{write_message, LogValue};

/// Attitude at the fast rate. Upstream `MASK_LOG_ATTITUDE_FAST`.
pub const MASK_LOG_ATTITUDE_FAST: u32 = 1 << 0;
/// Attitude at the medium rate. Upstream `MASK_LOG_ATTITUDE_MED`.
pub const MASK_LOG_ATTITUDE_MED: u32 = 1 << 1;
/// GPS messages. Upstream `MASK_LOG_GPS`.
pub const MASK_LOG_GPS: u32 = 1 << 2;
/// Performance monitoring. Upstream `MASK_LOG_PM`.
pub const MASK_LOG_PM: u32 = 1 << 3;
/// Control tuning. Upstream `MASK_LOG_CTUN`.
pub const MASK_LOG_CTUN: u32 = 1 << 4;
/// Navigation tuning. Upstream `MASK_LOG_NTUN`.
pub const MASK_LOG_NTUN: u32 = 1 << 5;
/// IMU messages. Upstream `MASK_LOG_IMU`.
pub const MASK_LOG_IMU: u32 = 1 << 7;
/// Mission commands. Upstream `MASK_LOG_CMD`.
pub const MASK_LOG_CMD: u32 = 1 << 8;
/// Battery / current. Upstream `MASK_LOG_CURRENT`.
pub const MASK_LOG_CURRENT: u32 = 1 << 9;
/// Compass. Upstream `MASK_LOG_COMPASS`.
pub const MASK_LOG_COMPASS: u32 = 1 << 10;
/// TECS. Upstream `MASK_LOG_TECS`.
pub const MASK_LOG_TECS: u32 = 1 << 11;
/// Camera. Upstream `MASK_LOG_CAMERA`.
pub const MASK_LOG_CAMERA: u32 = 1 << 12;
/// RC input. Upstream `MASK_LOG_RC`.
pub const MASK_LOG_RC: u32 = 1 << 13;
/// Rangefinder / sonar. Upstream `MASK_LOG_SONAR`.
pub const MASK_LOG_SONAR: u32 = 1 << 14;
/// Raw IMU. Upstream `MASK_LOG_IMU_RAW`.
pub const MASK_LOG_IMU_RAW: u32 = 1 << 19;
/// Attitude at the full rate. Upstream `MASK_LOG_ATTITUDE_FULLRATE`.
pub const MASK_LOG_ATTITUDE_FULLRATE: u32 = 1 << 20;
/// Video stabilisation. Upstream `MASK_LOG_VIDEO_STABILISATION`.
pub const MASK_LOG_VIDEO_STABILISATION: u32 = 1 << 21;
/// Notch at the full rate. Upstream `MASK_LOG_NOTCH_FULLRATE`.
pub const MASK_LOG_NOTCH_FULLRATE: u32 = 1 << 22;

/// Default Plane `LOG_BITMASK`. Upstream `DEFAULT_LOG_BITMASK` (`0xffff`).
pub const DEFAULT_LOG_BITMASK: u32 = 0xffff;

/// Front-end `LOG_BITMASK` plus the logging-started latch.
///
/// Upstream `AP_Logger` stores `_log_bitmask` at `init` and reports
/// `logging_started` from any backend that has opened a log. This stub
/// keeps that pair in one place so `Write` can refuse work before
/// packing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogGate {
    /// Stored `LOG_BITMASK`. Upstream `*_log_bitmask`.
    log_bitmask: u32,
    /// True once a backend has started a log. Upstream `logging_started`.
    started: bool,
}

impl Default for LogGate {
    fn default() -> Self {
        Self::new(DEFAULT_LOG_BITMASK)
    }
}

impl LogGate {
    /// Gate with `log_bitmask` and the started latch clear.
    ///
    /// Backends start unopened (`AP_Logger_File::logging_started` is
    /// `_write_fd != -1`).
    #[must_use]
    pub const fn new(log_bitmask: u32) -> Self {
        Self {
            log_bitmask,
            started: false,
        }
    }

    /// Stored `LOG_BITMASK`.
    #[must_use]
    pub const fn log_bitmask(&self) -> u32 {
        self.log_bitmask
    }

    /// Replace the stored `LOG_BITMASK`.
    pub fn set_log_bitmask(&mut self, log_bitmask: u32) {
        self.log_bitmask = log_bitmask;
    }

    /// Whether `mask` is enabled in `LOG_BITMASK`.
    ///
    /// Upstream `AP_Logger::should_log`: `!(mask & *_log_bitmask)` is
    /// the first reject. Armed / download / backend-count checks land
    /// later.
    #[must_use]
    pub const fn should_log(&self, mask: u32) -> bool {
        (mask & self.log_bitmask) != 0
    }

    /// Whether a log is open.
    ///
    /// Upstream `AP_Logger::logging_started`: true if any backend
    /// reports started.
    #[must_use]
    pub const fn logging_started(&self) -> bool {
        self.started
    }

    /// Latch start. Upstream `start_new_log` / File open path.
    pub fn start_logging(&mut self) {
        self.started = true;
    }

    /// Latch stop. Upstream `stop_logging` / File close (`_write_fd = -1`).
    pub fn stop_logging(&mut self) {
        self.started = false;
    }

    /// Pack and `WriteBlock` only when the class is enabled and a log
    /// is open.
    ///
    /// Returns `false` and leaves the backend untouched when
    /// [`should_log`](Self::should_log) is false or
    /// [`logging_started`](Self::logging_started) is false. That is
    /// the front-end no-op: vehicle code that still calls `Write`
    /// without checking the gate must not emit bytes.
    #[must_use]
    pub fn write<B: LogBackend + ?Sized>(
        &self,
        backend: &mut B,
        class_mask: u32,
        structure: &LogStructure,
        fields: &[LogValue<'_>],
    ) -> bool {
        if !self.should_log(class_mask) || !self.logging_started() {
            return false;
        }
        write_message(backend, structure, fields)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::MemoryBackend;
    use crate::write::calc_msg_len;

    fn test_row() -> LogStructure {
        LogStructure {
            msg_type: 1,
            msg_len: calc_msg_len("BH").expect("len"),
            name: "TEST",
            format: "BH",
            labels: "A,B",
        }
    }

    fn test_fields() -> [LogValue<'static>; 2] {
        [LogValue::U8(1), LogValue::U16(2)]
    }

    #[test]
    fn should_log_requires_class_bit() {
        let gate = LogGate::new(MASK_LOG_IMU | MASK_LOG_GPS);
        assert!(gate.should_log(MASK_LOG_IMU));
        assert!(gate.should_log(MASK_LOG_GPS));
        assert!(!gate.should_log(MASK_LOG_TECS));
        assert!(!gate.should_log(MASK_LOG_ATTITUDE_FAST));
        assert_eq!(gate.log_bitmask(), MASK_LOG_IMU | MASK_LOG_GPS);
    }

    #[test]
    fn default_bitmask_enables_low_plane_classes() {
        let gate = LogGate::default();
        assert_eq!(gate.log_bitmask(), DEFAULT_LOG_BITMASK);
        assert!(gate.should_log(MASK_LOG_ATTITUDE_FAST));
        assert!(gate.should_log(MASK_LOG_SONAR));
        assert!(!gate.should_log(MASK_LOG_IMU_RAW));
    }

    #[test]
    fn logging_started_latches_on_start_and_stop() {
        let mut gate = LogGate::new(DEFAULT_LOG_BITMASK);
        assert!(!gate.logging_started());
        gate.start_logging();
        assert!(gate.logging_started());
        gate.stop_logging();
        assert!(!gate.logging_started());
    }

    #[test]
    fn write_is_noop_when_class_disabled() {
        let mut gate = LogGate::new(MASK_LOG_GPS);
        gate.start_logging();
        let mut log = MemoryBackend::<32>::new();
        let row = test_row();
        let fields = test_fields();
        assert!(!gate.write(&mut log, MASK_LOG_IMU, &row, &fields));
        assert!(log.recorded().is_empty());
    }

    #[test]
    fn write_is_noop_when_not_started() {
        let gate = LogGate::new(MASK_LOG_IMU);
        let mut log = MemoryBackend::<32>::new();
        let row = test_row();
        let fields = test_fields();
        assert!(gate.should_log(MASK_LOG_IMU));
        assert!(!gate.logging_started());
        assert!(!gate.write(&mut log, MASK_LOG_IMU, &row, &fields));
        assert!(log.recorded().is_empty());
    }

    #[test]
    fn write_dispatches_when_enabled_and_started() {
        let mut gate = LogGate::new(MASK_LOG_IMU);
        gate.start_logging();
        let mut log = MemoryBackend::<32>::new();
        let row = test_row();
        let fields = test_fields();
        assert!(gate.write(&mut log, MASK_LOG_IMU, &row, &fields));
        assert_eq!(log.recorded().len(), usize::from(row.msg_len));
    }
}
