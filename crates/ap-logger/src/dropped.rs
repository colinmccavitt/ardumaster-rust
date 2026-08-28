//! Dropped-message / buffer-full counter, upstream `AP_Logger_Backend::_dropped`.
//!
//! `WritePrioritisedBlock` increments `_dropped` when the write buffer
//! cannot take the whole message (`space < size`) or when a
//! non-critical message would eat reserved critical space.
//! `AP_Logger::num_dropped` exposes the first backend's count. This
//! stub is that increment-and-expose — not DSF `buf_space_*` stats,
//! not critical-message reservation, not rate limiting.
//!
//! Packing failures (`Write` cannot represent the FMT row) are not
//! drops: the backend never saw the block. Only a full backend
//! (`WriteBlock` returns false) increments the counter.

use crate::backend::LogBackend;
use crate::structure::LogStructure;
use crate::write::{pack_message, LogValue, LOG_PACKET_MAX_LEN};

/// Dropped-message / buffer-full counter.
///
/// Upstream `AP_Logger_Backend::_dropped` / `num_dropped()`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DroppedMessages {
    dropped: u32,
}

impl DroppedMessages {
    /// Counter at zero. Upstream `_dropped` starts at 0 and is
    /// cleared again in `StartNewLog`.
    #[must_use]
    pub const fn new() -> Self {
        Self { dropped: 0 }
    }

    /// How many writes the backend rejected because it was full.
    ///
    /// Upstream `AP_Logger_Backend::num_dropped` /
    /// `AP_Logger::num_dropped`.
    #[must_use]
    pub const fn num_dropped(&self) -> u32 {
        self.dropped
    }

    /// Reset the counter. Upstream `StartNewLog` sets `_dropped = 0`.
    pub fn clear(&mut self) {
        self.dropped = 0;
    }

    /// `WriteBlock` a raw buffer; increment `_dropped` when the
    /// backend rejects it (buffer full).
    ///
    /// Upstream `_WritePrioritisedBlock` when `space < size`.
    #[must_use]
    pub fn write_block<B: LogBackend + ?Sized>(
        &mut self,
        backend: &mut B,
        buffer: &[u8],
    ) -> bool {
        if backend.write_block(buffer) {
            true
        } else {
            self.dropped = self.dropped.saturating_add(1);
            false
        }
    }

    /// Pack a FMT-described message and `WriteBlock` it.
    ///
    /// Upstream `Write` → `WriteBlock` / `WritePrioritisedBlock`.
    /// Returns `false` without incrementing when packing fails.
    /// Increments [`num_dropped`](Self::num_dropped) only when the
    /// backend is full.
    #[must_use]
    pub fn write<B: LogBackend + ?Sized>(
        &mut self,
        backend: &mut B,
        structure: &LogStructure,
        fields: &[LogValue<'_>],
    ) -> bool {
        let mut buf = [0u8; LOG_PACKET_MAX_LEN];
        let Some(n) = pack_message(structure, fields, &mut buf) else {
            return false;
        };
        let Some(pkt) = buf.get(..n) else {
            return false;
        };
        self.write_block(backend, pkt)
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
    fn write_does_not_count_when_backend_accepts() {
        let mut drops = DroppedMessages::new();
        let mut log = MemoryBackend::<32>::new();
        let row = test_row();
        let fields = test_fields();
        assert!(drops.write(&mut log, &row, &fields));
        assert_eq!(drops.num_dropped(), 0);
        assert_eq!(log.recorded().len(), usize::from(row.msg_len));
    }

    #[test]
    fn write_block_increments_when_backend_full() {
        let mut drops = DroppedMessages::new();
        let mut log = MemoryBackend::<2>::new();
        assert!(drops.write_block(&mut log, &[1, 2]));
        assert_eq!(drops.num_dropped(), 0);
        assert!(!drops.write_block(&mut log, &[3]));
        assert_eq!(drops.num_dropped(), 1);
        assert_eq!(log.recorded(), &[1, 2]);
        assert!(!drops.write_block(&mut log, &[4, 5]));
        assert_eq!(drops.num_dropped(), 2);
    }

    #[test]
    fn write_increments_when_typed_message_does_not_fit() {
        let mut drops = DroppedMessages::new();
        let mut log = MemoryBackend::<4>::new();
        let row = test_row();
        let fields = test_fields();
        assert_eq!(row.msg_len, 6);
        assert!(!drops.write(&mut log, &row, &fields));
        assert_eq!(drops.num_dropped(), 1);
        assert!(log.recorded().is_empty());
    }

    #[test]
    fn packing_failure_is_not_a_drop() {
        let mut drops = DroppedMessages::new();
        let mut log = MemoryBackend::<32>::new();
        let row = test_row();
        let fields = [LogValue::I16(1), LogValue::U16(2)];
        assert!(!drops.write(&mut log, &row, &fields));
        assert_eq!(drops.num_dropped(), 0);
        assert!(log.recorded().is_empty());
    }

    #[test]
    fn clear_resets_count_like_start_new_log() {
        let mut drops = DroppedMessages::new();
        let mut log = MemoryBackend::<1>::new();
        assert!(!drops.write_block(&mut log, &[1, 2]));
        assert_eq!(drops.num_dropped(), 1);
        drops.clear();
        assert_eq!(drops.num_dropped(), 0);
        assert_eq!(DroppedMessages::default().num_dropped(), 0);
    }
}
