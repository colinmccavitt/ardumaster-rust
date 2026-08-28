//! WriteStreaming / rate-limited periodic write gate, upstream
//! `AP_Logger::WriteStreaming` + `AP_Logger_RateLimiter`.
//!
//! `WriteStreaming` is `WriteV(..., is_critical=false, is_streaming=true)`.
//! The backend rate limiter then only emits when
//! `(now_ms - last_send_ms[msgid]) >= 1000 / rate_hz`. A rate of 0
//! disables the gate (upstream `_FILE_RATEMAX` default). This stub is
//! that period check — not `_log_pause`, not the disarm rate, not
//! multi-instance scheduler-tick reuse.

use crate::backend::LogBackend;
use crate::structure::LogStructure;
use crate::write::{write_message, LogValue};

/// Default streaming rate. Upstream `_MAV_RATEMAX` default (`10` Hz).
pub const DEFAULT_STREAM_RATE_HZ: u16 = 10;

/// Message-id table size. Upstream `last_send_ms[256]`.
const MSGID_SLOTS: usize = 256;

/// Rate-limited `WriteStreaming` front-end.
///
/// Owns the last-send time for each msgid. [`Self::write`] packs and
/// `WriteBlock`s only when [`should_emit`](Self::should_emit) is true
/// for that msgid at `now_ms`.
#[derive(Clone, Copy, Debug)]
pub struct WriteStreaming {
    rate_hz: u16,
    last_send_ms: [Option<u16>; MSGID_SLOTS],
}

impl Default for WriteStreaming {
    #[inline]
    fn default() -> Self {
        Self::with_rate_hz(DEFAULT_STREAM_RATE_HZ)
    }
}

impl WriteStreaming {
    /// Gate at [`DEFAULT_STREAM_RATE_HZ`], no msgid sent yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Gate at `rate_hz`. `0` means unlimited (always emit).
    #[must_use]
    pub fn with_rate_hz(rate_hz: u16) -> Self {
        Self {
            rate_hz,
            last_send_ms: [None; MSGID_SLOTS],
        }
    }

    /// Stored streaming rate in Hz. Upstream `_FILE_RATEMAX` /
    /// `_MAV_RATEMAX` / `_BLK_RATEMAX`.
    #[must_use]
    pub const fn rate_hz(&self) -> u16 {
        self.rate_hz
    }

    /// Replace the streaming rate. Does not clear last-send times.
    pub fn set_rate_hz(&mut self, rate_hz: u16) {
        self.rate_hz = rate_hz;
    }

    /// Minimum gap between emits of one msgid, in milliseconds.
    ///
    /// `None` when the rate is 0 (unlimited). Upstream
    /// `should_log_streaming` uses `1000.0 / rate_hz`.
    #[must_use]
    pub const fn period_ms(&self) -> Option<u16> {
        period_ms_for(self.rate_hz)
    }

    /// Last emit time for `msgid`, or `None` if it has never passed
    /// the gate. Upstream `last_send_ms[msgid]`.
    #[must_use]
    pub const fn last_send_ms(&self, msgid: u8) -> Option<u16> {
        self.last_send_ms[msgid as usize]
    }

    /// Whether a streaming write of `msgid` may emit at `now_ms`.
    ///
    /// Upstream `AP_Logger_RateLimiter::should_log_streaming`: the
    /// first call for a msgid always emits; later calls emit only
    /// when the period has elapsed (`now - last >= 1000 / rate_hz`).
    /// A rate of 0 always emits. Updates [`last_send_ms`](Self::last_send_ms)
    /// when the gate opens, matching the limiter (the decision is
    /// recorded before `WriteBlock`).
    #[must_use]
    pub fn should_emit(&mut self, msgid: u8, now_ms: u16) -> bool {
        let slot = msgid as usize;
        if let Some(period) = period_ms_for(self.rate_hz) {
            if let Some(last) = self.last_send_ms[slot] {
                if now_ms.wrapping_sub(last) < period {
                    return false;
                }
            }
        }
        self.last_send_ms[slot] = Some(now_ms);
        true
    }

    /// Reset last-send times. Upstream `StartNewLog` starts a fresh
    /// rate-limit window.
    pub fn clear(&mut self) {
        self.last_send_ms = [None; MSGID_SLOTS];
    }

    /// Pack and `WriteBlock` only when the streaming period has elapsed.
    ///
    /// Returns `false` and leaves the backend untouched when
    /// [`should_emit`](Self::should_emit) is false. Packing / backend
    /// failures still consume the rate-limit slot (the limiter already
    /// recorded `last_send_ms`).
    #[must_use]
    pub fn write<B: LogBackend + ?Sized>(
        &mut self,
        backend: &mut B,
        now_ms: u16,
        structure: &LogStructure,
        fields: &[LogValue<'_>],
    ) -> bool {
        if !self.should_emit(structure.msg_type, now_ms) {
            return false;
        }
        write_message(backend, structure, fields)
    }
}

const fn period_ms_for(rate_hz: u16) -> Option<u16> {
    if rate_hz == 0 {
        None
    } else {
        Some(1000 / rate_hz)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::MemoryBackend;
    use crate::write::calc_msg_len;

    fn test_row(msg_type: u8) -> LogStructure {
        LogStructure {
            msg_type,
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
    fn default_rate_is_mav_ratemax() {
        let stream = WriteStreaming::new();
        assert_eq!(stream.rate_hz(), DEFAULT_STREAM_RATE_HZ);
        assert_eq!(stream.period_ms(), Some(100));
        assert_eq!(DEFAULT_STREAM_RATE_HZ, 10);
    }

    #[test]
    fn first_emit_then_gate_until_period_elapses() {
        let mut stream = WriteStreaming::with_rate_hz(10);
        assert!(stream.should_emit(7, 0));
        assert_eq!(stream.last_send_ms(7), Some(0));
        assert!(!stream.should_emit(7, 99));
        assert_eq!(stream.last_send_ms(7), Some(0));
        assert!(stream.should_emit(7, 100));
        assert_eq!(stream.last_send_ms(7), Some(100));
        assert!(!stream.should_emit(7, 199));
        assert!(stream.should_emit(7, 200));
    }

    #[test]
    fn msgids_are_rate_limited_independently() {
        let mut stream = WriteStreaming::with_rate_hz(10);
        assert!(stream.should_emit(1, 0));
        assert!(stream.should_emit(2, 10));
        assert!(!stream.should_emit(1, 50));
        assert!(stream.should_emit(2, 110));
        assert!(stream.should_emit(1, 100));
    }

    #[test]
    fn zero_rate_always_emits() {
        let mut stream = WriteStreaming::with_rate_hz(0);
        assert_eq!(stream.period_ms(), None);
        assert!(stream.should_emit(3, 0));
        assert!(stream.should_emit(3, 0));
        assert!(stream.should_emit(3, 1));
    }

    #[test]
    fn write_is_noop_inside_the_period() {
        let mut stream = WriteStreaming::with_rate_hz(10);
        let mut log = MemoryBackend::<32>::new();
        let row = test_row(7);
        let fields = test_fields();
        assert!(stream.write(&mut log, 0, &row, &fields));
        assert_eq!(log.recorded().len(), usize::from(row.msg_len));
        assert!(!stream.write(&mut log, 50, &row, &fields));
        assert_eq!(log.recorded().len(), usize::from(row.msg_len));
        assert!(stream.write(&mut log, 100, &row, &fields));
        assert_eq!(log.recorded().len(), usize::from(row.msg_len) * 2);
    }

    #[test]
    fn clear_opens_the_rate_window_again() {
        let mut stream = WriteStreaming::with_rate_hz(10);
        assert!(stream.should_emit(4, 0));
        assert!(!stream.should_emit(4, 10));
        stream.clear();
        assert_eq!(stream.last_send_ms(4), None);
        assert!(stream.should_emit(4, 10));
    }
}
