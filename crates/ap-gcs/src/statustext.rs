//! STATUSTEXT send, upstream `GCS_MAVLINK::send_text` / msgid 253.
//!
//! This slice packs severity + the 50-byte `text` field
//! (`mavlink_statustext_t`) and frames it for Write. Queueing,
//! printf/`send_textv`, chunked `id`/`chunk_seq` for strings longer
//! than 50, and STATUSTEXT receive are later slices.

use crate::framing::Frame;

/// STATUSTEXT message id, upstream `MAVLINK_MSG_ID_STATUSTEXT`.
pub const MSG_ID_STATUSTEXT: u32 = 253;

/// Packed min payload length, upstream `MAVLINK_MSG_ID_STATUSTEXT_MIN_LEN`.
pub const STATUSTEXT_MIN_LEN: usize = 51;

/// Packed payload length with extensions, upstream `MAVLINK_MSG_ID_STATUSTEXT_LEN`.
pub const STATUSTEXT_LEN: usize = 54;

/// `text` field width, upstream `MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN`.
pub const TEXT_LEN: usize = 50;

/// CRC extra, upstream `MAVLINK_MSG_ID_STATUSTEXT_CRC`.
pub const STATUSTEXT_CRC: u8 = 83;

/// `MAV_SEVERITY_EMERGENCY` — system is unusable.
pub const MAV_SEVERITY_EMERGENCY: u8 = 0;
/// `MAV_SEVERITY_ALERT` — action should be taken immediately.
pub const MAV_SEVERITY_ALERT: u8 = 1;
/// `MAV_SEVERITY_CRITICAL` — action must be taken immediately.
pub const MAV_SEVERITY_CRITICAL: u8 = 2;
/// `MAV_SEVERITY_ERROR` — error in a secondary / redundant system.
pub const MAV_SEVERITY_ERROR: u8 = 3;
/// `MAV_SEVERITY_WARNING` — possible future error if not resolved.
pub const MAV_SEVERITY_WARNING: u8 = 4;
/// `MAV_SEVERITY_NOTICE` — unusual event, not an error.
pub const MAV_SEVERITY_NOTICE: u8 = 5;
/// `MAV_SEVERITY_INFO` — normal operational message.
pub const MAV_SEVERITY_INFO: u8 = 6;
/// `MAV_SEVERITY_DEBUG` — non-operational debug text.
pub const MAV_SEVERITY_DEBUG: u8 = 7;

/// Packed STATUSTEXT fields, upstream `mavlink_statustext_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusText {
    /// Severity (`MAV_SEVERITY`).
    pub severity: u8,
    text: [u8; TEXT_LEN],
    /// Chunk id. 0 means a single-chunk short message.
    pub id: u16,
    /// Chunk index. 0 for the first (and usually only) piece.
    pub chunk_seq: u8,
}

impl StatusText {
    /// A single-chunk STATUSTEXT (`id` 0), matching a short `send_text`.
    ///
    /// `text` is copied into the 50-byte field, zero-filled, and
    /// truncated if longer. Upstream chunking of longer strings is a
    /// later slice.
    #[must_use]
    pub fn new(severity: u8, text: &str) -> Self {
        let mut buf = [0u8; TEXT_LEN];
        let src = text.as_bytes();
        let n = core::cmp::min(src.len(), TEXT_LEN);
        if let (Some(dst), Some(src_n)) = (buf.get_mut(..n), src.get(..n)) {
            dst.copy_from_slice(src_n);
        }
        Self {
            severity,
            text: buf,
            id: 0,
            chunk_seq: 0,
        }
    }

    /// The 50-byte `text` field (may contain trailing NULs).
    #[must_use]
    pub const fn text(&self) -> &[u8; TEXT_LEN] {
        &self.text
    }

    /// Pack into 54 little-endian bytes. `None` if `buf` is shorter than 54.
    ///
    /// Wire order matches `mavlink_msg_statustext_pack`: severity,
    /// text[50], `id` (LE), `chunk_seq`.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..STATUSTEXT_LEN)?;
        *dest.get_mut(0)? = self.severity;
        dest.get_mut(1..1 + TEXT_LEN)?.copy_from_slice(&self.text);
        dest.get_mut(51..53)?
            .copy_from_slice(&self.id.to_le_bytes());
        *dest.get_mut(53)? = self.chunk_seq;
        Some(STATUSTEXT_LEN)
    }

    /// Unpack at least the 51-byte min payload. `None` if `buf` is short.
    ///
    /// Extension fields (`id`, `chunk_seq`) default to 0 when the
    /// payload is the 51-byte min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..STATUSTEXT_MIN_LEN)?;
        let mut text = [0u8; TEXT_LEN];
        text.copy_from_slice(src.get(1..1 + TEXT_LEN)?);
        let (id, chunk_seq) = match buf.get(STATUSTEXT_MIN_LEN..STATUSTEXT_LEN) {
            Some(ext) => {
                let mut id_le = [0u8; 2];
                id_le.copy_from_slice(ext.get(..2)?);
                (u16::from_le_bytes(id_le), *ext.get(2)?)
            }
            None => (0, 0),
        };
        Some(Self {
            severity: *src.first()?,
            text,
            id,
            chunk_seq,
        })
    }

    /// Decode a framed STATUSTEXT. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_STATUSTEXT {
            return None;
        }
        Self::decode(frame.payload())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_writes_severity_then_zero_filled_text() {
        let st = StatusText::new(MAV_SEVERITY_WARNING, "Arm: check");
        let mut buf = [0xFFu8; STATUSTEXT_LEN];
        assert_eq!(st.encode(&mut buf), Some(STATUSTEXT_LEN));
        assert_eq!(buf.first().copied(), Some(MAV_SEVERITY_WARNING));
        assert_eq!(buf.get(1..11), Some(b"Arm: check".as_slice()));
        assert_eq!(buf.get(11), Some(&0));
        assert_eq!(buf.get(51..54), Some([0, 0, 0].as_slice()));
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(StatusText::decode(&[0, 1, 2]).is_none());
    }

    #[test]
    fn new_truncates_text_longer_than_field() {
        let long = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxYYYY";
        assert_eq!(long.len(), TEXT_LEN + 4);
        let st = StatusText::new(MAV_SEVERITY_INFO, long);
        assert_eq!(st.text().as_slice(), [b'x'; TEXT_LEN].as_slice());
    }
}
