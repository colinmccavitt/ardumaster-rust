//! HEARTBEAT payload, upstream `mavlink_msg_heartbeat.h` / msgid 0.
//!
//! Wire order is size-sorted: `custom_mode` first, then the five `uint8`
//! fields. `mavlink_msg_heartbeat_pack` writes `mavlink_version` as 3 — the
//! `uint8_t_mavlink_version` magic type is not caller-writable.

use crate::framing::Frame;

/// HEARTBEAT message id, upstream `MAVLINK_MSG_ID_HEARTBEAT`.
pub const MSG_ID_HEARTBEAT: u32 = 0;

/// Packed payload length, upstream `MAVLINK_MSG_ID_HEARTBEAT_LEN`.
pub const HEARTBEAT_LEN: usize = 9;

/// `MAV_AUTOPILOT_ARDUPILOTMEGA` — what `GCS_MAVLINK::send_heartbeat` sends.
pub const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;

/// `MAV_TYPE_FIXED_WING` — Plane's `frame_type()` on a conventional airframe.
pub const MAV_TYPE_FIXED_WING: u8 = 1;

/// Packed HEARTBEAT fields, upstream `mavlink_heartbeat_t`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Heartbeat {
    /// Autopilot-specific flags (`custom_mode`). Plane encodes the flight mode.
    pub custom_mode: u32,
    /// Vehicle or component type (`type` / `MAV_TYPE`).
    pub mav_type: u8,
    /// Autopilot class (`MAV_AUTOPILOT`).
    pub autopilot: u8,
    /// System mode bitmap (`MAV_MODE_FLAG`).
    pub base_mode: u8,
    /// System status (`MAV_STATE`).
    pub system_status: u8,
    /// MAVLink version byte. Encode forces this to 3.
    pub mavlink_version: u8,
}

impl Heartbeat {
    /// A Plane-shaped heartbeat matching `GCS_MAVLINK::send_heartbeat`.
    ///
    /// Autopilot is always [`MAV_AUTOPILOT_ARDUPILOTMEGA`]; `mavlink_version`
    /// is 3. `frame_type` is upstream `gcs().frame_type()`.
    #[must_use]
    pub const fn plane(
        frame_type: u8,
        base_mode: u8,
        custom_mode: u32,
        system_status: u8,
    ) -> Self {
        Self {
            custom_mode,
            mav_type: frame_type,
            autopilot: MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode,
            system_status,
            mavlink_version: 3,
        }
    }

    /// Pack into 9 little-endian bytes. `None` if `buf` is shorter than 9.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..HEARTBEAT_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.custom_mode.to_le_bytes());
        // Encode always writes version 3, matching mavlink_msg_heartbeat_pack.
        let tail = [
            self.mav_type,
            self.autopilot,
            self.base_mode,
            self.system_status,
            3,
        ];
        dest.get_mut(4..HEARTBEAT_LEN)?.copy_from_slice(&tail);
        Some(HEARTBEAT_LEN)
    }

    /// Unpack 9 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..HEARTBEAT_LEN)?;
        let mut custom = [0u8; 4];
        custom.copy_from_slice(src.get(..4)?);
        Some(Self {
            custom_mode: u32::from_le_bytes(custom),
            mav_type: *src.get(4)?,
            autopilot: *src.get(5)?,
            base_mode: *src.get(6)?,
            system_status: *src.get(7)?,
            mavlink_version: *src.get(8)?,
        })
    }

    /// Decode a framed HEARTBEAT. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_HEARTBEAT {
            return None;
        }
        Self::decode(frame.payload())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_writes_custom_mode_first_and_version_three() {
        let hb = Heartbeat::plane(MAV_TYPE_FIXED_WING, 0x81, 5, 4);
        let mut buf = [0u8; HEARTBEAT_LEN];
        assert_eq!(hb.encode(&mut buf), Some(HEARTBEAT_LEN));
        assert_eq!(buf.get(..4), Some([5, 0, 0, 0].as_slice()));
        assert_eq!(buf.get(4).copied(), Some(MAV_TYPE_FIXED_WING));
        assert_eq!(buf.get(5).copied(), Some(MAV_AUTOPILOT_ARDUPILOTMEGA));
        assert_eq!(buf.get(6).copied(), Some(0x81));
        assert_eq!(buf.get(7).copied(), Some(4));
        assert_eq!(buf.get(8).copied(), Some(3));
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(Heartbeat::decode(&[0, 1, 2]).is_none());
    }
}
