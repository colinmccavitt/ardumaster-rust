//! Typed-message Write dispatcher, upstream `AP_Logger::Write`.
//!
//! `Write(name, labels, fmt, ...)` maps a FMT row, packs
//! `HEAD_BYTE1` / `HEAD_BYTE2` / `msgid` plus each format field, and
//! hands the bytes to `WriteBlock`. This slice is that pack-and-dispatch
//! — not FMT emission on first use, not `LOG_BITMASK`, not file IO.
//!
//! Rust has no `va_list`. [`LogValue`] is the typed stand-in for the
//! C++ varargs; [`write_message`] is `Write` / `WriteV` down to
//! `WriteBlock` on a [`LogBackend`].

use crate::backend::LogBackend;
use crate::structure::{LogPacketHeader, LogStructure, LOG_PACKET_HEADER_LEN};

/// Maximum on-wire DataFlash packet. Upstream `LOG_PACKET_MAX_LEN` (`UINT8_MAX`).
pub const LOG_PACKET_MAX_LEN: usize = 255;

/// One typed field matching a character in [`LogStructure::format`].
///
/// Upstream `AP_Logger_Backend::Write` pulls these from `va_list`.
/// String formats (`n` / `N` / `Z`) take [`LogValue::Text`] and are
/// copied with zero-fill, truncated to the field width.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LogValue<'a> {
    /// Format `b`.
    I8(i8),
    /// Format `B` or `M`.
    U8(u8),
    /// Format `h` or `c`.
    I16(i16),
    /// Format `H` or `C`.
    U16(u16),
    /// Format `i`, `L`, or `e`.
    I32(i32),
    /// Format `I` or `E`.
    U32(u32),
    /// Format `q`.
    I64(i64),
    /// Format `Q`.
    U64(u64),
    /// Format `f`.
    F32(f32),
    /// Format `d`.
    F64(f64),
    /// Format `n` (4), `N` (16), or `Z` (64).
    Text(&'a str),
}

/// On-wire size of one format character, not including the packet header.
///
/// Upstream `AP_Logger::Write_calc_msg_len` switch. `None` for an
/// unknown specifier.
#[must_use]
pub const fn field_size(fmt_char: u8) -> Option<usize> {
    match fmt_char {
        b'a' => Some(64),
        b'b' | b'B' | b'M' => Some(1),
        b'c' | b'h' | b'C' | b'H' | b'g' => Some(2),
        b'e' | b'f' | b'i' | b'E' | b'I' | b'L' => Some(4),
        b'n' => Some(4),
        b'd' | b'q' | b'Q' => Some(8),
        b'N' => Some(16),
        b'Z' => Some(64),
        _ => None,
    }
}

/// Bytes taken by a packed message for `fmt`, including the 3-byte header.
///
/// Upstream `AP_Logger::Write_calc_msg_len`. `None` when a specifier is
/// unknown or the total would exceed [`LOG_PACKET_MAX_LEN`].
#[must_use]
pub fn calc_msg_len(fmt: &str) -> Option<u8> {
    let mut len = LOG_PACKET_HEADER_LEN;
    let bytes = fmt.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        let Some(ch) = bytes.get(i) else {
            return None;
        };
        let Some(size) = field_size(*ch) else {
            return None;
        };
        let Some(next) = len.checked_add(size) else {
            return None;
        };
        if next > LOG_PACKET_MAX_LEN {
            return None;
        }
        len = next;
        i = match i.checked_add(1) {
            Some(n) => n,
            None => return None,
        };
    }
    u8::try_from(len).ok()
}

/// Pack `structure` + `fields` as a DataFlash packet.
///
/// Layout matches `AP_Logger_Backend::Write`: header, then each format
/// field in little-endian C storage order. `structure.msg_len` must
/// equal [`calc_msg_len`] of `structure.format`, and `fields` must
/// line up 1:1 with the format characters.
///
/// Returns the number of bytes written into `out`, or `None` when the
/// format, values, or buffer cannot represent the message.
#[must_use]
pub fn pack_message(
    structure: &LogStructure,
    fields: &[LogValue<'_>],
    out: &mut [u8],
) -> Option<usize> {
    let fmt = structure.format.as_bytes();
    if fields.len() != fmt.len() {
        return None;
    }
    let msg_len = calc_msg_len(structure.format)?;
    if msg_len != structure.msg_len {
        return None;
    }
    let n = usize::from(msg_len);
    if out.len() < n {
        return None;
    }

    let header = LogPacketHeader::new(structure.msg_type).pack();
    let mut off = write_bytes(out, 0, &header)?;
    let mut i = 0;
    while i < fmt.len() {
        let ch = *fmt.get(i)?;
        let value = fields.get(i)?;
        off = pack_field(ch, value, out, off)?;
        i = i.checked_add(1)?;
    }
    if off != n {
        return None;
    }
    Some(off)
}

/// Pack a FMT-described message and `WriteBlock` it.
///
/// Upstream `AP_Logger::Write` / `WriteV` → `AP_Logger_Backend::Write`
/// → `WriteBlock`. Returns `false` when packing fails or the backend
/// rejects the block.
#[must_use]
pub fn write_message<B: LogBackend + ?Sized>(
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
    backend.write_block(pkt)
}

fn pack_field(fmt_char: u8, value: &LogValue<'_>, out: &mut [u8], off: usize) -> Option<usize> {
    match (fmt_char, value) {
        (b'b', LogValue::I8(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'B' | b'M', LogValue::U8(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'h' | b'c', LogValue::I16(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'H' | b'C', LogValue::U16(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'i' | b'L' | b'e', LogValue::I32(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'I' | b'E', LogValue::U32(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'q', LogValue::I64(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'Q', LogValue::U64(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'f', LogValue::F32(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'd', LogValue::F64(v)) => write_bytes(out, off, &v.to_le_bytes()),
        (b'n', LogValue::Text(s)) => pack_text(out, off, s, 4),
        (b'N', LogValue::Text(s)) => pack_text(out, off, s, 16),
        (b'Z', LogValue::Text(s)) => pack_text(out, off, s, 64),
        _ => None,
    }
}

fn pack_text(out: &mut [u8], off: usize, text: &str, width: usize) -> Option<usize> {
    let end = off.checked_add(width)?;
    let dst = out.get_mut(off..end)?;
    dst.fill(0);
    let src = text.as_bytes();
    let n = core::cmp::min(src.len(), width);
    let (Some(dst_n), Some(src_n)) = (dst.get_mut(..n), src.get(..n)) else {
        return None;
    };
    dst_n.copy_from_slice(src_n);
    Some(end)
}

fn write_bytes(buf: &mut [u8], off: usize, src: &[u8]) -> Option<usize> {
    let end = off.checked_add(src.len())?;
    let dst = buf.get_mut(off..end)?;
    dst.copy_from_slice(src);
    Some(end)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::MemoryBackend;
    use crate::structure::{HEAD_BYTE1, HEAD_BYTE2, LOG_FORMAT_LEN};

    fn test_row() -> LogStructure {
        LogStructure {
            msg_type: 1,
            msg_len: 14,
            name: "TEST",
            format: "BHIf",
            labels: "A,B,C,D",
        }
    }

    #[test]
    fn calc_msg_len_matches_write_calc_msg_len() {
        assert_eq!(calc_msg_len("BBnNZ"), Some(LOG_FORMAT_LEN as u8));
        assert_eq!(calc_msg_len("BHIf"), Some(14));
        assert_eq!(calc_msg_len("Q"), Some(11));
        assert_eq!(calc_msg_len(""), Some(LOG_PACKET_HEADER_LEN as u8));
        assert_eq!(calc_msg_len("x"), None);
    }

    #[test]
    fn pack_message_writes_header_and_le_fields() {
        let row = test_row();
        let fields = [
            LogValue::U8(0x11),
            LogValue::U16(0x2233),
            LogValue::U32(0x4455_6677),
            LogValue::F32(1.0),
        ];
        let mut buf = [0u8; LOG_PACKET_MAX_LEN];
        let n = pack_message(&row, &fields, &mut buf).expect("pack");
        assert_eq!(n, 14);
        assert_eq!(buf.get(..3), Some([HEAD_BYTE1, HEAD_BYTE2, 1].as_slice()));
        assert_eq!(buf.get(3), Some(&0x11));
        assert_eq!(buf.get(4..6), Some([0x33, 0x22].as_slice()));
        assert_eq!(buf.get(6..10), Some([0x77, 0x66, 0x55, 0x44].as_slice()));
        assert_eq!(buf.get(10..14), Some(1.0f32.to_le_bytes().as_slice()));
    }

    #[test]
    fn write_message_dispatches_through_memory_backend() {
        let row = test_row();
        let fields = [
            LogValue::U8(7),
            LogValue::U16(1000),
            LogValue::U32(0x0102_0304),
            LogValue::F32(2.0),
        ];
        let mut log = MemoryBackend::<32>::new();
        log.start_write(0);
        assert!(write_message(&mut log, &row, &fields));
        log.end_write();

        let rec = log.recorded();
        assert_eq!(rec.len(), 14);
        assert_eq!(rec.get(..3), Some([HEAD_BYTE1, HEAD_BYTE2, 1].as_slice()));
        assert_eq!(rec.get(3), Some(&7));
        assert_eq!(rec.get(4..6), Some(1000u16.to_le_bytes().as_slice()));
        assert_eq!(
            rec.get(6..10),
            Some(0x0102_0304u32.to_le_bytes().as_slice())
        );
        assert_eq!(rec.get(10..14), Some(2.0f32.to_le_bytes().as_slice()));
        assert_eq!(log.ended_writes(), 1);
    }

    #[test]
    fn write_message_rejects_type_mismatch() {
        let row = test_row();
        let fields = [
            LogValue::I16(1),
            LogValue::U16(2),
            LogValue::U32(3),
            LogValue::F32(0.0),
        ];
        let mut log = MemoryBackend::<32>::new();
        assert!(!write_message(&mut log, &row, &fields));
        assert!(log.recorded().is_empty());
    }

    #[test]
    fn write_message_rejects_when_backend_full() {
        let row = test_row();
        let fields = [
            LogValue::U8(1),
            LogValue::U16(2),
            LogValue::U32(3),
            LogValue::F32(0.0),
        ];
        let mut log = MemoryBackend::<4>::new();
        assert!(!write_message(&mut log, &row, &fields));
        assert!(log.recorded().is_empty());
    }

    #[test]
    fn pack_text_zero_fills_n_field() {
        let row = LogStructure {
            msg_type: 9,
            msg_len: 7,
            name: "NAME",
            format: "n",
            labels: "N",
        };
        let mut buf = [0xFFu8; 8];
        let n = pack_message(&row, &[LogValue::Text("AB")], &mut buf).expect("pack");
        assert_eq!(n, 7);
        assert_eq!(buf.get(3..7), Some(b"AB\0\0".as_slice()));
    }
}
