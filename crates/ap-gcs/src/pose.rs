//! ATTITUDE / GLOBAL_POSITION_INT stream send, extracted from the pinned
//! Plane-4.7.0 `modules/mavlink/message_definitions/v1.0` defs
//! (`common.xml` msgid 30, `standard.xml` msgid 33).
//!
//! Upstream `GCS_MAVLINK::try_send_message` emits ATTITUDE on `MSG_ATTITUDE`
//! and GLOBAL_POSITION_INT on `MSG_LOCATION`. This slice packs both from a
//! small pose snapshot (`send_attitude` / `send_global_position_int`) and
//! frames them for Write. Stream-rate scheduling and the rest of the dialect
//! stay for later FW-028 slices.

use crate::framing::Frame;

/// ATTITUDE message id, upstream `MAVLINK_MSG_ID_ATTITUDE`.
pub const MSG_ID_ATTITUDE: u32 = 30;

/// GLOBAL_POSITION_INT message id, upstream `MAVLINK_MSG_ID_GLOBAL_POSITION_INT`.
pub const MSG_ID_GLOBAL_POSITION_INT: u32 = 33;

/// Packed payload length, upstream `MAVLINK_MSG_ID_ATTITUDE_LEN`.
pub const ATTITUDE_LEN: usize = 28;

/// Packed payload length, upstream `MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN`.
pub const GLOBAL_POSITION_INT_LEN: usize = 28;

/// CRC extra, upstream `MAVLINK_MSG_ID_ATTITUDE_CRC`.
pub const ATTITUDE_CRC: u8 = 39;

/// CRC extra, upstream `MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC`.
pub const GLOBAL_POSITION_INT_CRC: u8 = 104;

/// AHRS / position snapshot used by `send_attitude` and
/// `send_global_position_int`.
///
/// Mirrors the fields those two upstream senders pull from `AP_AHRS` plus
/// `AP_HAL::millis()`. This is the packed-unit snapshot (rad, degE7, mm,
/// cm/s, cdeg), not the SI AHRS types.
#[derive(Debug, Clone, Copy)]
pub struct PoseSnapshot {
    /// `AP_HAL::millis()` — `time_boot_ms` on both messages.
    pub time_boot_ms: u32,
    /// Roll angle, radians (`ahrs.get_roll_rad()`).
    pub roll: f32,
    /// Pitch angle, radians (`ahrs.get_pitch_rad()`).
    pub pitch: f32,
    /// Yaw angle, radians (`ahrs.get_yaw_rad()`).
    pub yaw: f32,
    /// Roll rate, rad/s (`ahrs.get_gyro().x`).
    pub rollspeed: f32,
    /// Pitch rate, rad/s (`ahrs.get_gyro().y`).
    pub pitchspeed: f32,
    /// Yaw rate, rad/s (`ahrs.get_gyro().z`).
    pub yawspeed: f32,
    /// Latitude, degE7 (`Location.lat`).
    pub lat: i32,
    /// Longitude, degE7 (`Location.lng`).
    pub lon: i32,
    /// Altitude MSL, millimetres (`global_position_int_alt`).
    pub alt: i32,
    /// Altitude above home, millimetres (`global_position_int_relative_alt`).
    pub relative_alt: i32,
    /// North velocity, cm/s (`vel.x * 100`).
    pub vx: i16,
    /// East velocity, cm/s (`vel.y * 100`).
    pub vy: i16,
    /// Down velocity, cm/s (`vel.z * 100`).
    pub vz: i16,
    /// Compass heading, centidegrees (`ahrs.yaw_sensor`).
    pub hdg: u16,
}

impl PoseSnapshot {
    /// Build the ATTITUDE payload from this snapshot.
    #[must_use]
    pub const fn attitude(&self) -> Attitude {
        Attitude {
            time_boot_ms: self.time_boot_ms,
            roll: self.roll,
            pitch: self.pitch,
            yaw: self.yaw,
            rollspeed: self.rollspeed,
            pitchspeed: self.pitchspeed,
            yawspeed: self.yawspeed,
        }
    }

    /// Build the GLOBAL_POSITION_INT payload from this snapshot.
    #[must_use]
    pub const fn global_position_int(&self) -> GlobalPositionInt {
        GlobalPositionInt {
            time_boot_ms: self.time_boot_ms,
            lat: self.lat,
            lon: self.lon,
            alt: self.alt,
            relative_alt: self.relative_alt,
            vx: self.vx,
            vy: self.vy,
            vz: self.vz,
            hdg: self.hdg,
        }
    }
}

/// Packed ATTITUDE fields, upstream `mavlink_attitude_t`.
///
/// Wire order matches `mavlink_msg_attitude_pack`: `time_boot_ms`, then the
/// six floats (all 4-byte fields, XML order).
#[derive(Debug, Clone, Copy)]
pub struct Attitude {
    /// Timestamp (time since system boot), milliseconds.
    pub time_boot_ms: u32,
    /// Roll angle, radians.
    pub roll: f32,
    /// Pitch angle, radians.
    pub pitch: f32,
    /// Yaw angle, radians.
    pub yaw: f32,
    /// Roll angular speed, rad/s.
    pub rollspeed: f32,
    /// Pitch angular speed, rad/s.
    pub pitchspeed: f32,
    /// Yaw angular speed, rad/s.
    pub yawspeed: f32,
}

impl Attitude {
    /// Pack into 28 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..ATTITUDE_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.time_boot_ms.to_le_bytes());
        dest.get_mut(4..8)?
            .copy_from_slice(&self.roll.to_le_bytes());
        dest.get_mut(8..12)?
            .copy_from_slice(&self.pitch.to_le_bytes());
        dest.get_mut(12..16)?
            .copy_from_slice(&self.yaw.to_le_bytes());
        dest.get_mut(16..20)?
            .copy_from_slice(&self.rollspeed.to_le_bytes());
        dest.get_mut(20..24)?
            .copy_from_slice(&self.pitchspeed.to_le_bytes());
        dest.get_mut(24..28)?
            .copy_from_slice(&self.yawspeed.to_le_bytes());
        Some(ATTITUDE_LEN)
    }

    /// Unpack 28 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..ATTITUDE_LEN)?;
        Some(Self {
            time_boot_ms: u32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            roll: f32::from_le_bytes(src.get(4..8)?.try_into().ok()?),
            pitch: f32::from_le_bytes(src.get(8..12)?.try_into().ok()?),
            yaw: f32::from_le_bytes(src.get(12..16)?.try_into().ok()?),
            rollspeed: f32::from_le_bytes(src.get(16..20)?.try_into().ok()?),
            pitchspeed: f32::from_le_bytes(src.get(20..24)?.try_into().ok()?),
            yawspeed: f32::from_le_bytes(src.get(24..28)?.try_into().ok()?),
        })
    }

    /// Decode a framed ATTITUDE. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_ATTITUDE {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Packed GLOBAL_POSITION_INT fields, upstream `mavlink_global_position_int_t`.
///
/// Wire order is size-sorted (`mavlink_msg_global_position_int_pack`): five
/// 4-byte fields, then four 2-byte fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GlobalPositionInt {
    /// Timestamp (time since system boot), milliseconds.
    pub time_boot_ms: u32,
    /// Latitude, degE7.
    pub lat: i32,
    /// Longitude, degE7.
    pub lon: i32,
    /// Altitude MSL, millimetres.
    pub alt: i32,
    /// Altitude above home, millimetres.
    pub relative_alt: i32,
    /// Ground X speed (north), cm/s.
    pub vx: i16,
    /// Ground Y speed (east), cm/s.
    pub vy: i16,
    /// Ground Z speed (down), cm/s.
    pub vz: i16,
    /// Vehicle heading, centidegrees. Unknown is `u16::MAX`.
    pub hdg: u16,
}

impl GlobalPositionInt {
    /// Pack into 28 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..GLOBAL_POSITION_INT_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.time_boot_ms.to_le_bytes());
        dest.get_mut(4..8)?.copy_from_slice(&self.lat.to_le_bytes());
        dest.get_mut(8..12)?
            .copy_from_slice(&self.lon.to_le_bytes());
        dest.get_mut(12..16)?
            .copy_from_slice(&self.alt.to_le_bytes());
        dest.get_mut(16..20)?
            .copy_from_slice(&self.relative_alt.to_le_bytes());
        dest.get_mut(20..22)?
            .copy_from_slice(&self.vx.to_le_bytes());
        dest.get_mut(22..24)?
            .copy_from_slice(&self.vy.to_le_bytes());
        dest.get_mut(24..26)?
            .copy_from_slice(&self.vz.to_le_bytes());
        dest.get_mut(26..28)?
            .copy_from_slice(&self.hdg.to_le_bytes());
        Some(GLOBAL_POSITION_INT_LEN)
    }

    /// Unpack 28 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..GLOBAL_POSITION_INT_LEN)?;
        Some(Self {
            time_boot_ms: u32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            lat: i32::from_le_bytes(src.get(4..8)?.try_into().ok()?),
            lon: i32::from_le_bytes(src.get(8..12)?.try_into().ok()?),
            alt: i32::from_le_bytes(src.get(12..16)?.try_into().ok()?),
            relative_alt: i32::from_le_bytes(src.get(16..20)?.try_into().ok()?),
            vx: i16::from_le_bytes(src.get(20..22)?.try_into().ok()?),
            vy: i16::from_le_bytes(src.get(22..24)?.try_into().ok()?),
            vz: i16::from_le_bytes(src.get(24..26)?.try_into().ok()?),
            hdg: u16::from_le_bytes(src.get(26..28)?.try_into().ok()?),
        })
    }

    /// Decode a framed GLOBAL_POSITION_INT. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_GLOBAL_POSITION_INT {
            return None;
        }
        Self::decode(frame.payload())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_writes_time_boot_ms_first() {
        let att = Attitude {
            time_boot_ms: 0x0102_0304,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
        };
        let mut buf = [0u8; ATTITUDE_LEN];
        assert_eq!(att.encode(&mut buf), Some(ATTITUDE_LEN));
        assert_eq!(buf.get(..4), Some([0x04, 0x03, 0x02, 0x01].as_slice()));
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(Attitude::decode(&[0, 1, 2]).is_none());
        assert!(GlobalPositionInt::decode(&[0, 1, 2]).is_none());
    }
}
