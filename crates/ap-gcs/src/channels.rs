//! RC_CHANNELS / SERVO_OUTPUT_RAW stream send, extracted from the pinned
//! Plane-4.7.0 `modules/mavlink/message_definitions/v1.0` defs
//! (`common.xml` msgid 65, msgid 36).
//!
//! Upstream `GCS_MAVLINK::try_send_message` emits RC_CHANNELS on
//! `MSG_RC_CHANNELS` and SERVO_OUTPUT_RAW on `MSG_SERVO_OUTPUT_RAW`. This
//! slice packs both from a small channel snapshot (`send_rc_channels` /
//! `send_servo_output_raw`) and frames them for Write. Stream-rate
//! scheduling and the rest of the dialect stay for later FW-028 slices.

use crate::framing::Frame;

/// RC_CHANNELS message id, upstream `MAVLINK_MSG_ID_RC_CHANNELS`.
pub const MSG_ID_RC_CHANNELS: u32 = 65;

/// SERVO_OUTPUT_RAW message id, upstream `MAVLINK_MSG_ID_SERVO_OUTPUT_RAW`.
pub const MSG_ID_SERVO_OUTPUT_RAW: u32 = 36;

/// Packed payload length, upstream `MAVLINK_MSG_ID_RC_CHANNELS_LEN`.
pub const RC_CHANNELS_LEN: usize = 42;

/// Minimum payload length, upstream `MAVLINK_MSG_ID_RC_CHANNELS_MIN_LEN`.
pub const RC_CHANNELS_MIN_LEN: usize = 42;

/// Packed payload length, upstream `MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN`.
pub const SERVO_OUTPUT_RAW_LEN: usize = 37;

/// Minimum payload length, upstream `MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_MIN_LEN`.
pub const SERVO_OUTPUT_RAW_MIN_LEN: usize = 21;

/// CRC extra, upstream `MAVLINK_MSG_ID_RC_CHANNELS_CRC`.
pub const RC_CHANNELS_CRC: u8 = 118;

/// CRC extra, upstream `MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_CRC`.
pub const SERVO_OUTPUT_RAW_CRC: u8 = 222;

/// Reported RC channel slots, upstream `RC_CHANNELS` `chan1_raw`..`chan18_raw`.
pub const RC_CHANNELS_COUNT: usize = 18;

/// Servo output slots, upstream `SERVO_OUTPUT_RAW` `servo1_raw`..`servo16_raw`.
pub const SERVO_OUTPUT_COUNT: usize = 16;

/// Unused RC channel, upstream `UINT16_MAX`.
pub const RC_CHANNEL_UNUSED: u16 = u16::MAX;

/// Unknown RSSI, upstream `UINT8_MAX`.
pub const RSSI_UNKNOWN: u8 = u8::MAX;

/// RC / servo snapshot used by `send_rc_channels` and
/// `send_servo_output_raw`.
///
/// Mirrors the packed-unit fields those two upstream senders pull from
/// `rc().get_radio_in`, `hal.rcout->read`, `AP_HAL::millis` /
/// `AP_HAL::micros`, and `receiver_rssi()`. This is the on-wire snapshot
/// (microseconds, RSSI units), not the SI RC types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelSnapshot {
    /// `AP_HAL::millis()` — `time_boot_ms` on RC_CHANNELS.
    pub time_boot_ms: u32,
    /// `AP_HAL::micros()` — `time_usec` on SERVO_OUTPUT_RAW.
    pub time_usec: u32,
    /// Total RC channels being received (`RC_CHANNELS_MAX`, often 18).
    pub chancount: u8,
    /// Radio-in PWM, microseconds. Unused slots are [`RC_CHANNEL_UNUSED`].
    pub chan: [u16; RC_CHANNELS_COUNT],
    /// Receiver RSSI. [`RSSI_UNKNOWN`] if invalid.
    pub rssi: u8,
    /// Servo output port (`0` = MAIN, `1` = AUX).
    pub port: u8,
    /// Servo PWM, microseconds. Upstream maps `65535` to `0` before send.
    pub servo: [u16; SERVO_OUTPUT_COUNT],
}

impl ChannelSnapshot {
    /// Build the RC_CHANNELS payload from this snapshot.
    #[must_use]
    pub const fn rc_channels(&self) -> RcChannels {
        RcChannels {
            time_boot_ms: self.time_boot_ms,
            chan: self.chan,
            chancount: self.chancount,
            rssi: self.rssi,
        }
    }

    /// Build the SERVO_OUTPUT_RAW payload from this snapshot.
    #[must_use]
    pub const fn servo_output_raw(&self) -> ServoOutputRaw {
        ServoOutputRaw {
            time_usec: self.time_usec,
            servo: self.servo,
            port: self.port,
        }
    }
}

/// Packed RC_CHANNELS fields, upstream `mavlink_rc_channels_t`.
///
/// Wire order matches `mavlink_msg_rc_channels_pack`: `time_boot_ms`,
/// eighteen `chanN_raw` words, then `chancount` and `rssi`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcChannels {
    /// Timestamp (time since system boot), milliseconds.
    pub time_boot_ms: u32,
    /// RC channel 1–18 PWM, microseconds.
    pub chan: [u16; RC_CHANNELS_COUNT],
    /// Total number of RC channels being received.
    pub chancount: u8,
    /// Receive signal strength. [`RSSI_UNKNOWN`] if invalid.
    pub rssi: u8,
}

impl RcChannels {
    /// Pack into 42 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..RC_CHANNELS_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.time_boot_ms.to_le_bytes());
        write_u16_array(dest.get_mut(4..40)?, &self.chan)?;
        *dest.get_mut(40)? = self.chancount;
        *dest.get_mut(41)? = self.rssi;
        Some(RC_CHANNELS_LEN)
    }

    /// Unpack 42 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..RC_CHANNELS_MIN_LEN)?;
        Some(Self {
            time_boot_ms: u32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            chan: read_u16_array(src.get(4..40)?)?,
            chancount: *src.get(40)?,
            rssi: *src.get(41)?,
        })
    }

    /// Decode a framed RC_CHANNELS. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_RC_CHANNELS {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Packed SERVO_OUTPUT_RAW fields, upstream `mavlink_servo_output_raw_t`.
///
/// Wire order is size-sorted for the base message
/// (`mavlink_msg_servo_output_raw_pack`): `time_usec`, eight servo words,
/// `port`, then the eight-word extension block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServoOutputRaw {
    /// Timestamp (UNIX epoch or time since boot), microseconds.
    pub time_usec: u32,
    /// Servo outputs 1–16, microseconds.
    pub servo: [u16; SERVO_OUTPUT_COUNT],
    /// Servo output port (`0` = MAIN, `1` = AUX).
    pub port: u8,
}

impl ServoOutputRaw {
    /// Pack into 37 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..SERVO_OUTPUT_RAW_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.time_usec.to_le_bytes());
        write_u16_array(dest.get_mut(4..20)?, self.servo.get(..8)?)?;
        *dest.get_mut(20)? = self.port;
        write_u16_array(dest.get_mut(21..37)?, self.servo.get(8..)?)?;
        Some(SERVO_OUTPUT_RAW_LEN)
    }

    /// Unpack at least 21 bytes. Extension servos default to 0 when the
    /// buffer is shorter than [`SERVO_OUTPUT_RAW_LEN`].
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..SERVO_OUTPUT_RAW_MIN_LEN)?;
        let mut servo = [0u16; SERVO_OUTPUT_COUNT];
        let base = read_u16_array::<8>(src.get(4..20)?)?;
        let mut i = 0usize;
        while i < 8 {
            *servo.get_mut(i)? = *base.get(i)?;
            i = i.checked_add(1)?;
        }
        let ext = read_u16_array_or_zero::<8>(buf, 21);
        i = 0;
        while i < 8 {
            *servo.get_mut(i.checked_add(8)?)? = *ext.get(i)?;
            i = i.checked_add(1)?;
        }
        Some(Self {
            time_usec: u32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            servo,
            port: *src.get(20)?,
        })
    }

    /// Decode a framed SERVO_OUTPUT_RAW. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_SERVO_OUTPUT_RAW {
            return None;
        }
        Self::decode(frame.payload())
    }
}

fn write_u16_array(dest: &mut [u8], values: &[u16]) -> Option<()> {
    if dest.len() < values.len().checked_mul(2)? {
        return None;
    }
    let mut i = 0usize;
    while i < values.len() {
        let off = i.checked_mul(2)?;
        dest.get_mut(off..off.checked_add(2)?)?
            .copy_from_slice(&values.get(i)?.to_le_bytes());
        i = i.checked_add(1)?;
    }
    Some(())
}

fn read_u16_array<const N: usize>(src: &[u8]) -> Option<[u16; N]> {
    let mut out = [0u16; N];
    let mut i = 0usize;
    while i < N {
        let off = i.checked_mul(2)?;
        *out.get_mut(i)? = u16::from_le_bytes(src.get(off..off.checked_add(2)?)?.try_into().ok()?);
        i = i.checked_add(1)?;
    }
    Some(out)
}

fn read_u16_array_or_zero<const N: usize>(buf: &[u8], off: usize) -> [u16; N] {
    let need = N.saturating_mul(2);
    buf.get(off..off.saturating_add(need))
        .and_then(read_u16_array)
        .unwrap_or([0u16; N])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_writes_time_boot_ms_first() {
        let rc = RcChannels {
            time_boot_ms: 0x0102_0304,
            chan: [0u16; RC_CHANNELS_COUNT],
            chancount: 0,
            rssi: 0,
        };
        let mut buf = [0u8; RC_CHANNELS_LEN];
        assert_eq!(rc.encode(&mut buf), Some(RC_CHANNELS_LEN));
        assert_eq!(buf.get(..4), Some([0x04, 0x03, 0x02, 0x01].as_slice()));
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(RcChannels::decode(&[0, 1, 2]).is_none());
        assert!(ServoOutputRaw::decode(&[0, 1, 2]).is_none());
    }
}
