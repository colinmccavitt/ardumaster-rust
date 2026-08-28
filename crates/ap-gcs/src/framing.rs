//! MAVLink 2 framing and CRC-16/MCRF4XX, upstream `mavlink_helpers.h`.
//!
//! A frame is STX `0xFD`, a 10-byte header, payload, and a two-byte CRC.
//! The CRC runs over the header after STX, the payload, and the
//! message-specific CRC extra. Signing (`incompat_flags` bit 0) is out of
//! scope for this slice.

/// MAVLink 2 start-of-frame, upstream `MAVLINK_STX`.
pub const STX_V2: u8 = 0xFD;

/// Header length including STX, upstream `MAVLINK_NUM_HEADER_BYTES` for v2.
pub const HEADER_LEN_V2: usize = 10;

/// Trailing CRC length.
pub const CRC_LEN: usize = 2;

/// Largest MAVLink payload, upstream `MAVLINK_MAX_PAYLOAD_LEN`.
pub const MAX_PAYLOAD_LEN: usize = 255;

/// A parsed MAVLink 2 frame without the CRC (already verified on decode).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// Sequence number (`seq`).
    pub seq: u8,
    /// Sender system id.
    pub sysid: u8,
    /// Sender component id.
    pub compid: u8,
    /// 24-bit message id.
    pub msgid: u32,
    payload: [u8; MAX_PAYLOAD_LEN],
    payload_len: u8,
}

impl Frame {
    /// Build a frame from a payload slice. `None` if the payload is too long.
    #[must_use]
    pub fn new(seq: u8, sysid: u8, compid: u8, msgid: u32, payload: &[u8]) -> Option<Self> {
        if payload.len() > MAX_PAYLOAD_LEN {
            return None;
        }
        let mut frame = Self {
            seq,
            sysid,
            compid,
            msgid,
            payload: [0; MAX_PAYLOAD_LEN],
            payload_len: payload.len() as u8,
        };
        let dest = frame.payload.get_mut(..payload.len())?;
        dest.copy_from_slice(payload);
        Some(frame)
    }

    /// Payload bytes.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        match self.payload.get(..usize::from(self.payload_len)) {
            Some(slice) => slice,
            None => &[],
        }
    }

    /// Payload length.
    #[must_use]
    pub const fn payload_len(&self) -> u8 {
        self.payload_len
    }
}

/// Why [`decode_v2`] rejected a buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Buffer shorter than a header plus CRC, or shorter than `len` claimed.
    Truncated,
    /// First byte was not [`STX_V2`].
    BadMagic,
    /// CRC extra + payload did not match the trailing checksum.
    BadCrc,
    /// `incompat_flags` asked for signing (or another unsupported feature).
    UnsupportedFlags,
}

/// CRC-16/MCRF4XX accumulate, upstream `crc_accumulate`.
#[must_use]
pub const fn crc_accumulate(crc: u16, data: u8) -> u16 {
    let mut tmp = data ^ (crc as u8);
    tmp ^= tmp << 4;
    (crc >> 8) ^ ((tmp as u16) << 8) ^ ((tmp as u16) << 3) ^ ((tmp >> 4) as u16)
}

/// CRC over `bytes` then optional CRC extra, upstream `crc_calculate` + extra.
#[must_use]
pub fn crc16(bytes: &[u8], extra: Option<u8>) -> u16 {
    let mut crc = 0xFFFF;
    for &b in bytes {
        crc = crc_accumulate(crc, b);
    }
    if let Some(e) = extra {
        crc = crc_accumulate(crc, e);
    }
    crc
}

/// CRC extra for HEARTBEAT (0), COMMAND_INT (75), COMMAND_LONG (76), and STATUSTEXT (253).
///
/// Unknown ids return `None` — this slice does not carry the dialect table.
#[must_use]
pub const fn crc_extra(msgid: u32) -> Option<u8> {
    match msgid {
        0 => Some(50),
        75 => Some(158),
        76 => Some(152),
        253 => Some(83),
        _ => None,
    }
}

/// Encode `frame` as MAVLink 2 into `out`. `None` if `out` is too small or
/// the msgid has no CRC extra in this slice.
#[must_use]
pub fn encode_v2(frame: &Frame, out: &mut [u8]) -> Option<usize> {
    let extra = crc_extra(frame.msgid)?;
    let payload_len = usize::from(frame.payload_len);
    let total = HEADER_LEN_V2
        .checked_add(payload_len)?
        .checked_add(CRC_LEN)?;
    let dest = out.get_mut(..total)?;
    let [id0, id1, id2, _] = frame.msgid.to_le_bytes();
    let header = [
        STX_V2,
        frame.payload_len,
        0,
        0,
        frame.seq,
        frame.sysid,
        frame.compid,
        id0,
        id1,
        id2,
    ];
    let header_dest = dest.get_mut(..HEADER_LEN_V2)?;
    header_dest.copy_from_slice(&header);
    let payload_dest = dest.get_mut(HEADER_LEN_V2..HEADER_LEN_V2 + payload_len)?;
    payload_dest.copy_from_slice(frame.payload());
    let crc_body = dest.get(1..HEADER_LEN_V2 + payload_len)?;
    let crc = crc16(crc_body, Some(extra));
    let crc_bytes = crc.to_le_bytes();
    let crc_dest = dest.get_mut(HEADER_LEN_V2 + payload_len..total)?;
    crc_dest.copy_from_slice(&crc_bytes);
    Some(total)
}

/// Parse one complete MAVLink 2 frame from the start of `buf`.
#[must_use]
pub fn decode_v2(buf: &[u8]) -> Result<Frame, DecodeError> {
    let magic = buf.get(0).copied().ok_or(DecodeError::Truncated)?;
    if magic != STX_V2 {
        return Err(DecodeError::BadMagic);
    }
    let payload_len = buf.get(1).copied().ok_or(DecodeError::Truncated)?;
    let incompat = buf.get(2).copied().ok_or(DecodeError::Truncated)?;
    if incompat != 0 {
        return Err(DecodeError::UnsupportedFlags);
    }
    let seq = buf.get(4).copied().ok_or(DecodeError::Truncated)?;
    let sysid = buf.get(5).copied().ok_or(DecodeError::Truncated)?;
    let compid = buf.get(6).copied().ok_or(DecodeError::Truncated)?;
    let id0 = buf.get(7).copied().ok_or(DecodeError::Truncated)?;
    let id1 = buf.get(8).copied().ok_or(DecodeError::Truncated)?;
    let id2 = buf.get(9).copied().ok_or(DecodeError::Truncated)?;
    let msgid = u32::from_le_bytes([id0, id1, id2, 0]);
    let payload_end = HEADER_LEN_V2
        .checked_add(usize::from(payload_len))
        .ok_or(DecodeError::Truncated)?;
    let total = payload_end
        .checked_add(CRC_LEN)
        .ok_or(DecodeError::Truncated)?;
    let payload = buf
        .get(HEADER_LEN_V2..payload_end)
        .ok_or(DecodeError::Truncated)?;
    let crc_bytes = buf.get(payload_end..total).ok_or(DecodeError::Truncated)?;
    let mut crc_le = [0u8; 2];
    crc_le.copy_from_slice(crc_bytes);
    let want = u16::from_le_bytes(crc_le);
    let extra = crc_extra(msgid).ok_or(DecodeError::BadCrc)?;
    let body = buf.get(1..payload_end).ok_or(DecodeError::Truncated)?;
    if crc16(body, Some(extra)) != want {
        return Err(DecodeError::BadCrc);
    }
    Frame::new(seq, sysid, compid, msgid, payload).ok_or(DecodeError::Truncated)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_accumulate_matches_init_plus_one_byte() {
        // Upstream starts at 0xFFFF; one 0x00 byte is a known step.
        assert_eq!(crc_accumulate(0xFFFF, 0x00), 0x0F87);
    }
}
