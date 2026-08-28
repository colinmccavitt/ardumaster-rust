//! Log message header and FMT structure, upstream `LogStructure.h`.
//!
//! Every DataFlash packet starts with a three-byte header (`head1`,
//! `head2`, `msgid`). FMT (`LOG_FORMAT_MSG` = 128) is the message that
//! describes every other message: type, length, name, format string,
//! and column labels. This slice is that table entry and the on-wire
//! `log_Format` packet — not units, multipliers, or the Write()
//! dispatcher.

/// First magic byte of every DataFlash packet. Upstream `HEAD_BYTE1`.
pub const HEAD_BYTE1: u8 = 0xA3;
/// Second magic byte of every DataFlash packet. Upstream `HEAD_BYTE2`.
pub const HEAD_BYTE2: u8 = 0x95;
/// Bytes in `LOG_PACKET_HEADER` (`head1`, `head2`, `msgid`).
pub const LOG_PACKET_HEADER_LEN: usize = 3;

/// FMT message type. Upstream `LOG_FORMAT_MSG`; must remain 128.
pub const LOG_FORMAT_MSG: u8 = 128;

/// On-wire name length (`log_Format::name`), not including a C null.
pub const FMT_NAME_LEN: usize = 4;
/// On-wire format-string length (`log_Format::format`).
pub const FMT_FORMAT_LEN: usize = 16;
/// On-wire labels length (`log_Format::labels`).
pub const FMT_LABELS_LEN: usize = 64;

/// Table-definition name size including trailing null. Upstream `LS_NAME_SIZE`.
pub const LS_NAME_SIZE: usize = 5;
/// Table-definition format size including trailing null. Upstream `LS_FORMAT_SIZE`.
pub const LS_FORMAT_SIZE: usize = 17;
/// Table-definition labels size including trailing null. Upstream `LS_LABELS_SIZE`.
pub const LS_LABELS_SIZE: usize = 65;

/// On-wire size of `log_Format`, including the 3-byte packet header.
///
/// `LOG_PACKET_HEADER` + `type` + `length` + `name[4]` + `format[16]`
/// + `labels[64]`.
pub const LOG_FORMAT_LEN: usize =
    LOG_PACKET_HEADER_LEN + 1 + 1 + FMT_NAME_LEN + FMT_FORMAT_LEN + FMT_LABELS_LEN;

/// Packet header shared by every DataFlash message.
///
/// Upstream `LOG_PACKET_HEADER`: `uint8_t head1, head2, msgid`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogPacketHeader {
    /// Magic byte 1. Upstream `head1` / `HEAD_BYTE1`.
    pub head1: u8,
    /// Magic byte 2. Upstream `head2` / `HEAD_BYTE2`.
    pub head2: u8,
    /// Message type id. Upstream `msgid`.
    pub msgid: u8,
}

impl LogPacketHeader {
    /// Header for `msgid`, filled with [`HEAD_BYTE1`] / [`HEAD_BYTE2`].
    ///
    /// Upstream `LOG_PACKET_HEADER_INIT(id)`.
    #[must_use]
    pub const fn new(msgid: u8) -> Self {
        Self {
            head1: HEAD_BYTE1,
            head2: HEAD_BYTE2,
            msgid,
        }
    }

    /// Pack as the three leading bytes of a DataFlash packet.
    #[must_use]
    pub const fn pack(self) -> [u8; LOG_PACKET_HEADER_LEN] {
        [self.head1, self.head2, self.msgid]
    }
}

/// Table entry that defines one message format.
///
/// Upstream `struct LogStructure`. This slice keeps the five fields
/// the FMT message itself emits (`msg_type`, `msg_len`, `name`,
/// `format`, `labels`). Units, multipliers, and `streaming` land later.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogStructure {
    /// Unique-to-this-log identifier. Upstream `msg_type`.
    pub msg_type: u8,
    /// Bytes taken by this message, including the packet header.
    /// Upstream `msg_len`.
    pub msg_len: u8,
    /// Four-character message name (`"FMT"`, `"GPS"`, …). Upstream `name`.
    pub name: &'static str,
    /// Format characters for the C-storage type of each field.
    /// Upstream `format` (e.g. `"BBnNZ"`).
    pub format: &'static str,
    /// Comma-separated column labels. Upstream `labels`.
    pub labels: &'static str,
}

impl LogStructure {
    /// FMT's own table row from `LOG_COMMON_STRUCTURES`.
    ///
    /// `{ LOG_FORMAT_MSG, sizeof(log_Format), "FMT", "BBnNZ",
    /// "Type,Length,Name,Format,Columns" }`.
    #[must_use]
    pub const fn fmt() -> Self {
        Self {
            msg_type: LOG_FORMAT_MSG,
            msg_len: LOG_FORMAT_LEN as u8,
            name: "FMT",
            format: "BBnNZ",
            labels: "Type,Length,Name,Format,Columns",
        }
    }
}

/// On-wire FMT packet. Upstream `struct log_Format`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogFormat {
    /// Packet header. `msgid` is always [`LOG_FORMAT_MSG`].
    pub header: LogPacketHeader,
    /// Type of the message being defined. Upstream `type`.
    pub msg_type: u8,
    /// Length of the message being defined. Upstream `length`.
    pub msg_len: u8,
    /// Name of the message being defined. Upstream `name[4]`.
    pub name: [u8; FMT_NAME_LEN],
    /// Format string of the message being defined. Upstream `format[16]`.
    pub format: [u8; FMT_FORMAT_LEN],
    /// Column labels of the message being defined. Upstream `labels[64]`.
    pub labels: [u8; FMT_LABELS_LEN],
}

impl LogFormat {
    /// Pack a [`LogStructure`] into a FMT packet.
    ///
    /// Upstream `AP_Logger_Backend::Fill_Format`: header bytes, type,
    /// length, then `strncpy_noterm` of name / format / labels.
    #[must_use]
    pub fn fill(structure: &LogStructure) -> Self {
        Self {
            header: LogPacketHeader::new(LOG_FORMAT_MSG),
            msg_type: structure.msg_type,
            msg_len: structure.msg_len,
            name: copy_field(structure.name),
            format: copy_field(structure.format),
            labels: copy_field(structure.labels),
        }
    }

    /// Serialize as the on-wire `log_Format` bytes.
    #[must_use]
    pub fn pack(&self) -> [u8; LOG_FORMAT_LEN] {
        let mut buf = [0u8; LOG_FORMAT_LEN];
        let mut off = 0;
        off = write_bytes(&mut buf, off, &self.header.pack());
        off = write_bytes(&mut buf, off, &[self.msg_type, self.msg_len]);
        off = write_bytes(&mut buf, off, &self.name);
        off = write_bytes(&mut buf, off, &self.format);
        let _ = write_bytes(&mut buf, off, &self.labels);
        buf
    }
}

/// Pack `structure` into an on-wire FMT packet.
///
/// Upstream `AP_Logger_Backend::Fill_Format`.
#[must_use]
pub fn fill_format(structure: &LogStructure) -> LogFormat {
    LogFormat::fill(structure)
}

fn copy_field<const N: usize>(src: &str) -> [u8; N] {
    let mut out = [0u8; N];
    let bytes = src.as_bytes();
    let n = core::cmp::min(bytes.len(), N);
    if let (Some(dst), Some(src_bytes)) = (out.get_mut(..n), bytes.get(..n)) {
        dst.copy_from_slice(src_bytes);
    }
    out
}

fn write_bytes(buf: &mut [u8], off: usize, src: &[u8]) -> usize {
    let Some(end) = off.checked_add(src.len()) else {
        return off;
    };
    if let Some(dst) = buf.get_mut(off..end) {
        dst.copy_from_slice(src);
        end
    } else {
        off
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fmt_table_row_matches_upstream() {
        let row = LogStructure::fmt();
        assert_eq!(row.msg_type, 128);
        assert_eq!(row.msg_len, 89);
        assert_eq!(row.name, "FMT");
        assert_eq!(row.format, "BBnNZ");
        assert_eq!(row.labels, "Type,Length,Name,Format,Columns");
        assert_eq!(LS_NAME_SIZE, FMT_NAME_LEN + 1);
        assert_eq!(LS_FORMAT_SIZE, FMT_FORMAT_LEN + 1);
        assert_eq!(LS_LABELS_SIZE, FMT_LABELS_LEN + 1);
        assert_eq!(LOG_FORMAT_LEN, 89);
    }

    #[test]
    fn fill_format_packs_header_and_fields() {
        let pkt = fill_format(&LogStructure::fmt());
        assert_eq!(pkt.header, LogPacketHeader::new(LOG_FORMAT_MSG));
        assert_eq!(pkt.header.pack(), [HEAD_BYTE1, HEAD_BYTE2, LOG_FORMAT_MSG]);
        assert_eq!(pkt.msg_type, LOG_FORMAT_MSG);
        assert_eq!(pkt.msg_len, LOG_FORMAT_LEN as u8);
        assert_eq!(&pkt.name, b"FMT\0");
        assert_eq!(pkt.format.get(..5), Some(b"BBnNZ".as_slice()));
        assert_eq!(
            pkt.labels.get(..31),
            Some(b"Type,Length,Name,Format,Columns".as_slice())
        );

        let bytes = pkt.pack();
        assert_eq!(bytes.len(), LOG_FORMAT_LEN);
        assert_eq!(
            bytes.get(..3),
            Some([HEAD_BYTE1, HEAD_BYTE2, LOG_FORMAT_MSG].as_slice())
        );
        assert_eq!(bytes.get(3), Some(&LOG_FORMAT_MSG));
        assert_eq!(bytes.get(4), Some(&(LOG_FORMAT_LEN as u8)));
        assert_eq!(bytes.get(5..8), Some(b"FMT".as_slice()));
    }
}
