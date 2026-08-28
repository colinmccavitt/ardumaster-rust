//! VFR_HUD / NAV_CONTROLLER_OUTPUT stream send, extracted from the pinned
//! Plane-4.7.0 `modules/mavlink/message_definitions/v1.0` defs
//! (`common.xml` msgid 74, msgid 62).
//!
//! Upstream `GCS_MAVLINK::try_send_message` emits VFR_HUD on `MSG_VFR_HUD`
//! and NAV_CONTROLLER_OUTPUT on `MSG_NAV_CONTROLLER_OUTPUT`. This slice
//! packs both from a small air-data + nav snapshot (`send_vfr_hud` /
//! `send_nav_controller_output`) and frames them for Write. Stream-rate
//! scheduling and the rest of the dialect stay for later FW-028 slices.

use crate::framing::Frame;

/// VFR_HUD message id, upstream `MAVLINK_MSG_ID_VFR_HUD`.
pub const MSG_ID_VFR_HUD: u32 = 74;

/// NAV_CONTROLLER_OUTPUT message id, upstream `MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT`.
pub const MSG_ID_NAV_CONTROLLER_OUTPUT: u32 = 62;

/// Packed payload length, upstream `MAVLINK_MSG_ID_VFR_HUD_LEN`.
pub const VFR_HUD_LEN: usize = 20;

/// Packed payload length, upstream `MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN`.
pub const NAV_CONTROLLER_OUTPUT_LEN: usize = 26;

/// CRC extra, upstream `MAVLINK_MSG_ID_VFR_HUD_CRC`.
pub const VFR_HUD_CRC: u8 = 20;

/// CRC extra, upstream `MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_CRC`.
pub const NAV_CONTROLLER_OUTPUT_CRC: u8 = 183;

/// Air-data / nav snapshot used by `send_vfr_hud` and
/// `send_nav_controller_output`.
///
/// Mirrors the packed-unit fields those two upstream senders pull from
/// `vfr_hud_airspeed`, `ahrs.groundspeed`, `ahrs.get_yaw_deg`,
/// `vfr_hud_throttle`, `vfr_hud_alt`, `vfr_hud_climbrate`, and Plane's
/// `send_nav_controller_output` (`nav_roll_cd`, `nav_pitch_cd`,
/// `nav_bearing_cd`, `target_bearing_cd`, `wp_distance`,
/// `calc_altitude_error_cm`, `airspeed_error`, `crosstrack_error`).
/// This is the on-wire snapshot (m/s, deg, %, m), not the SI AHRS types.
#[derive(Debug, Clone, Copy)]
pub struct HudSnapshot {
    /// Indicated / estimated airspeed, m/s (`vfr_hud_airspeed()`).
    pub airspeed: f32,
    /// Ground speed, m/s (`ahrs.groundspeed()`).
    pub groundspeed: f32,
    /// Compass heading, degrees (`ahrs.get_yaw_deg()`).
    pub heading: i16,
    /// Throttle setting 0–100 (`abs(vfr_hud_throttle())`).
    pub throttle: u16,
    /// Altitude MSL, metres (`vfr_hud_alt()`).
    pub alt: f32,
    /// Climb rate, m/s (`vfr_hud_climbrate()`).
    pub climb: f32,
    /// Desired roll, degrees (`nav_roll_cd * 0.01`).
    pub nav_roll: f32,
    /// Desired pitch, degrees (`nav_pitch_cd * 0.01`).
    pub nav_pitch: f32,
    /// Desired heading, degrees (`nav_bearing_cd * 0.01`).
    pub nav_bearing: i16,
    /// Bearing to current waypoint, degrees (`target_bearing_cd * 0.01`).
    pub target_bearing: i16,
    /// Distance to active waypoint, metres (`auto_state.wp_distance`).
    pub wp_dist: u16,
    /// Altitude error, metres (`calc_altitude_error_cm() * 0.01`).
    pub alt_error: f32,
    /// Airspeed error, on-wire units (`airspeed_error * 100`, PR#7933).
    pub aspd_error: f32,
    /// Crosstrack error, metres (`crosstrack_error()`).
    pub xtrack_error: f32,
}

impl HudSnapshot {
    /// Build the VFR_HUD payload from this snapshot.
    #[must_use]
    pub const fn vfr_hud(&self) -> VfrHud {
        VfrHud {
            airspeed: self.airspeed,
            groundspeed: self.groundspeed,
            heading: self.heading,
            throttle: self.throttle,
            alt: self.alt,
            climb: self.climb,
        }
    }

    /// Build the NAV_CONTROLLER_OUTPUT payload from this snapshot.
    #[must_use]
    pub const fn nav_controller_output(&self) -> NavControllerOutput {
        NavControllerOutput {
            nav_roll: self.nav_roll,
            nav_pitch: self.nav_pitch,
            nav_bearing: self.nav_bearing,
            target_bearing: self.target_bearing,
            wp_dist: self.wp_dist,
            alt_error: self.alt_error,
            aspd_error: self.aspd_error,
            xtrack_error: self.xtrack_error,
        }
    }
}

/// Packed VFR_HUD fields, upstream `mavlink_vfr_hud_t`.
///
/// Wire order is size-sorted (`mavlink_msg_vfr_hud_pack`): four 4-byte
/// fields (`airspeed`, `groundspeed`, `alt`, `climb`), then `heading`
/// and `throttle`.
#[derive(Debug, Clone, Copy)]
pub struct VfrHud {
    /// Vehicle airspeed, m/s.
    pub airspeed: f32,
    /// Ground speed, m/s.
    pub groundspeed: f32,
    /// Heading, degrees (0–360, 0 = north).
    pub heading: i16,
    /// Throttle setting, percent (0–100).
    pub throttle: u16,
    /// Altitude MSL, metres.
    pub alt: f32,
    /// Climb rate, m/s.
    pub climb: f32,
}

impl VfrHud {
    /// Pack into 20 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..VFR_HUD_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.airspeed.to_le_bytes());
        dest.get_mut(4..8)?
            .copy_from_slice(&self.groundspeed.to_le_bytes());
        dest.get_mut(8..12)?
            .copy_from_slice(&self.alt.to_le_bytes());
        dest.get_mut(12..16)?
            .copy_from_slice(&self.climb.to_le_bytes());
        dest.get_mut(16..18)?
            .copy_from_slice(&self.heading.to_le_bytes());
        dest.get_mut(18..20)?
            .copy_from_slice(&self.throttle.to_le_bytes());
        Some(VFR_HUD_LEN)
    }

    /// Unpack 20 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..VFR_HUD_LEN)?;
        Some(Self {
            airspeed: f32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            groundspeed: f32::from_le_bytes(src.get(4..8)?.try_into().ok()?),
            alt: f32::from_le_bytes(src.get(8..12)?.try_into().ok()?),
            climb: f32::from_le_bytes(src.get(12..16)?.try_into().ok()?),
            heading: i16::from_le_bytes(src.get(16..18)?.try_into().ok()?),
            throttle: u16::from_le_bytes(src.get(18..20)?.try_into().ok()?),
        })
    }

    /// Decode a framed VFR_HUD. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_VFR_HUD {
            return None;
        }
        Self::decode(frame.payload())
    }
}

/// Packed NAV_CONTROLLER_OUTPUT fields, upstream
/// `mavlink_nav_controller_output_t`.
///
/// Wire order is size-sorted (`mavlink_msg_nav_controller_output_pack`):
/// five 4-byte fields (`nav_roll`, `nav_pitch`, `alt_error`, `aspd_error`,
/// `xtrack_error`), then `nav_bearing`, `target_bearing`, `wp_dist`.
#[derive(Debug, Clone, Copy)]
pub struct NavControllerOutput {
    /// Desired roll, degrees.
    pub nav_roll: f32,
    /// Desired pitch, degrees.
    pub nav_pitch: f32,
    /// Desired heading, degrees.
    pub nav_bearing: i16,
    /// Bearing to current waypoint, degrees.
    pub target_bearing: i16,
    /// Distance to active waypoint, metres.
    pub wp_dist: u16,
    /// Altitude error, metres.
    pub alt_error: f32,
    /// Airspeed error, on-wire units.
    pub aspd_error: f32,
    /// Crosstrack error, metres.
    pub xtrack_error: f32,
}

impl NavControllerOutput {
    /// Pack into 26 little-endian bytes. `None` if `buf` is too short.
    #[must_use]
    pub fn encode(&self, buf: &mut [u8]) -> Option<usize> {
        let dest = buf.get_mut(..NAV_CONTROLLER_OUTPUT_LEN)?;
        dest.get_mut(..4)?
            .copy_from_slice(&self.nav_roll.to_le_bytes());
        dest.get_mut(4..8)?
            .copy_from_slice(&self.nav_pitch.to_le_bytes());
        dest.get_mut(8..12)?
            .copy_from_slice(&self.alt_error.to_le_bytes());
        dest.get_mut(12..16)?
            .copy_from_slice(&self.aspd_error.to_le_bytes());
        dest.get_mut(16..20)?
            .copy_from_slice(&self.xtrack_error.to_le_bytes());
        dest.get_mut(20..22)?
            .copy_from_slice(&self.nav_bearing.to_le_bytes());
        dest.get_mut(22..24)?
            .copy_from_slice(&self.target_bearing.to_le_bytes());
        dest.get_mut(24..26)?
            .copy_from_slice(&self.wp_dist.to_le_bytes());
        Some(NAV_CONTROLLER_OUTPUT_LEN)
    }

    /// Unpack 26 bytes. `None` if `buf` is shorter than the min length.
    #[must_use]
    pub fn decode(buf: &[u8]) -> Option<Self> {
        let src = buf.get(..NAV_CONTROLLER_OUTPUT_LEN)?;
        Some(Self {
            nav_roll: f32::from_le_bytes(src.get(..4)?.try_into().ok()?),
            nav_pitch: f32::from_le_bytes(src.get(4..8)?.try_into().ok()?),
            alt_error: f32::from_le_bytes(src.get(8..12)?.try_into().ok()?),
            aspd_error: f32::from_le_bytes(src.get(12..16)?.try_into().ok()?),
            xtrack_error: f32::from_le_bytes(src.get(16..20)?.try_into().ok()?),
            nav_bearing: i16::from_le_bytes(src.get(20..22)?.try_into().ok()?),
            target_bearing: i16::from_le_bytes(src.get(22..24)?.try_into().ok()?),
            wp_dist: u16::from_le_bytes(src.get(24..26)?.try_into().ok()?),
        })
    }

    /// Decode a framed NAV_CONTROLLER_OUTPUT. `None` if msgid or length is wrong.
    #[must_use]
    pub fn from_frame(frame: &Frame) -> Option<Self> {
        if frame.msgid != MSG_ID_NAV_CONTROLLER_OUTPUT {
            return None;
        }
        Self::decode(frame.payload())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_writes_airspeed_first() {
        let hud = VfrHud {
            airspeed: f32::from_le_bytes([0x04, 0x03, 0x02, 0x01]),
            groundspeed: 0.0,
            heading: 0,
            throttle: 0,
            alt: 0.0,
            climb: 0.0,
        };
        let mut buf = [0u8; VFR_HUD_LEN];
        assert_eq!(hud.encode(&mut buf), Some(VFR_HUD_LEN));
        assert_eq!(buf.get(..4), Some([0x04, 0x03, 0x02, 0x01].as_slice()));
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(VfrHud::decode(&[0, 1, 2]).is_none());
        assert!(NavControllerOutput::decode(&[0, 1, 2]).is_none());
    }
}
