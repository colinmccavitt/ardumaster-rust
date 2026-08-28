//! RC channel map and trim persist, upstream `AP_RCMapper` / `RC_Channel::set_and_save_trim`.
//!
//! `RCMAP_ROLL` / `PITCH` / `THROTTLE` / `YAW` name the 1-based receiver
//! channels that feed the four primary sticks. The map is how a transmitter
//! that cannot reorder its sticks still lands roll on roll. `set_and_save_trim`
//! copies the current `radio_in` into `RCn_TRIM` when it has changed, matching
//! `radio_trim.set_and_save_ifchanged(radio_in)`. Plane's `trim_radio` does
//! that for roll, pitch, and rudder after the surfaces have been recentered.

use crate::RcChannel;

/// Upstream `RCMAP_ROLL` default / `RCMapper::_ch_roll`.
pub const RCMAP_ROLL_DEFAULT: u8 = 1;
/// Upstream `RCMAP_PITCH` default / `RCMapper::_ch_pitch`.
pub const RCMAP_PITCH_DEFAULT: u8 = 2;
/// Upstream `RCMAP_THROTTLE` default / `RCMapper::_ch_throttle`.
pub const RCMAP_THROTTLE_DEFAULT: u8 = 3;
/// Upstream `RCMAP_YAW` default / `RCMapper::_ch_yaw`.
pub const RCMAP_YAW_DEFAULT: u8 = 4;
/// Upstream `@Range` lower bound for every `RCMAP_*` parameter.
pub const RCMAP_CHANNEL_MIN: u8 = 1;
/// Upstream `@Range` upper bound / `NUM_RC_CHANNELS`.
pub const RCMAP_CHANNEL_MAX: u8 = 16;

/// Four-stick `RCMAP_*` assignment, upstream `RCMapper`.
///
/// Channel numbers are 1-based, matching the parameters. An out-of-range
/// number is kept as stored — lookup then fails closed, the same way
/// `RC_Channels::get_rcmap_channel_nonnull` falls back to a dummy channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcMap {
    /// Upstream `RCMAP_ROLL`.
    pub roll: u8,
    /// Upstream `RCMAP_PITCH`.
    pub pitch: u8,
    /// Upstream `RCMAP_THROTTLE`.
    pub throttle: u8,
    /// Upstream `RCMAP_YAW`.
    pub yaw: u8,
}

impl Default for RcMap {
    fn default() -> Self {
        Self {
            roll: RCMAP_ROLL_DEFAULT,
            pitch: RCMAP_PITCH_DEFAULT,
            throttle: RCMAP_THROTTLE_DEFAULT,
            yaw: RCMAP_YAW_DEFAULT,
        }
    }
}

/// Stick pulses after applying [`RcMap`] to a 0-based receiver frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MappedStickPwm {
    /// PWM on the mapped roll channel, if that channel is present.
    pub roll: Option<u16>,
    /// PWM on the mapped pitch channel, if that channel is present.
    pub pitch: Option<u16>,
    /// PWM on the mapped throttle channel, if that channel is present.
    pub throttle: Option<u16>,
    /// PWM on the mapped yaw channel, if that channel is present.
    pub yaw: Option<u16>,
}

/// True when `map_ch` is a valid 1-based `RCMAP_*` channel (`@Range: 1 16`).
#[must_use]
pub const fn rcmap_channel_valid(map_ch: u8) -> bool {
    map_ch >= RCMAP_CHANNEL_MIN && map_ch <= RCMAP_CHANNEL_MAX
}

/// 0-based index for a 1-based `RCMAP_*` number.
///
/// Upstream `RC_Channels::rc_channel(rcmap_number - 1)`. Out of range is
/// `None`, matching the dummy-channel fallback when the map is invalid.
#[must_use]
pub const fn rcmap_index(map_ch: u8) -> Option<usize> {
    if rcmap_channel_valid(map_ch) {
        Some(map_ch as usize - 1)
    } else {
        None
    }
}

/// PWM on the mapped channel from a 0-based receiver frame.
///
/// Missing pulse or an invalid `RCMAP_*` returns `None`.
#[must_use]
pub fn mapped_pwm(channels: &[u16], map_ch: u8) -> Option<u16> {
    let idx = rcmap_index(map_ch)?;
    channels.get(idx).copied()
}

impl RcMap {
    /// Build a map from the four `RCMAP_*` parameters.
    #[must_use]
    pub const fn from_params(roll: u8, pitch: u8, throttle: u8, yaw: u8) -> Self {
        Self {
            roll,
            pitch,
            throttle,
            yaw,
        }
    }

    /// Apply the map to a receiver frame.
    #[must_use]
    pub fn map_sticks(self, channels: &[u16]) -> MappedStickPwm {
        MappedStickPwm {
            roll: mapped_pwm(channels, self.roll),
            pitch: mapped_pwm(channels, self.pitch),
            throttle: mapped_pwm(channels, self.throttle),
            yaw: mapped_pwm(channels, self.yaw),
        }
    }
}

/// Persist `radio_in` as `radio_trim` when it differs.
///
/// Upstream `RC_Channel::set_and_save_trim` /
/// `radio_trim.set_and_save_ifchanged(radio_in)`. Returns `true` when the
/// stored trim changed. EEPROM is not here; the caller owns that write.
#[must_use]
pub fn set_and_save_trim(ch: &mut RcChannel, radio_in: u16) -> bool {
    if ch.radio_trim == radio_in {
        return false;
    }
    ch.radio_trim = radio_in;
    true
}

/// Persist an explicit trim, upstream `RC_Channel::set_and_save_radio_trim`.
#[must_use]
pub fn set_and_save_radio_trim(ch: &mut RcChannel, val: u16) -> bool {
    set_and_save_trim(ch, val)
}

/// Persist roll / pitch / rudder trims, upstream `Plane::trim_radio`.
///
/// Throttle is not saved there. Returns `true` when any of the three
/// stored trims changed.
#[must_use]
pub fn persist_stick_trims(
    roll: &mut RcChannel,
    pitch: &mut RcChannel,
    yaw: &mut RcChannel,
    roll_in: u16,
    pitch_in: u16,
    yaw_in: u16,
) -> bool {
    let roll_changed = set_and_save_trim(roll, roll_in);
    let pitch_changed = set_and_save_trim(pitch, pitch_in);
    let yaw_changed = set_and_save_trim(yaw, yaw_in);
    roll_changed || pitch_changed || yaw_changed
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_upstream_rcmapper() {
        let map = RcMap::default();
        assert_eq!(map.roll, RCMAP_ROLL_DEFAULT);
        assert_eq!(map.pitch, RCMAP_PITCH_DEFAULT);
        assert_eq!(map.throttle, RCMAP_THROTTLE_DEFAULT);
        assert_eq!(map.yaw, RCMAP_YAW_DEFAULT);
        assert_eq!(RCMAP_CHANNEL_MIN, 1);
        assert_eq!(RCMAP_CHANNEL_MAX, 16);
    }

    #[test]
    fn default_map_reads_mode2_channel_order() {
        let frame = [1100_u16, 1200, 1300, 1400];
        let sticks = RcMap::default().map_sticks(&frame);
        assert_eq!(sticks.roll, Some(1100));
        assert_eq!(sticks.pitch, Some(1200));
        assert_eq!(sticks.throttle, Some(1300));
        assert_eq!(sticks.yaw, Some(1400));
    }

    #[test]
    fn remapped_yaw_on_channel_one_follows_the_map() {
        let map = RcMap::from_params(2, 3, 4, 1);
        let frame = [1900_u16, 1600, 1500, 1400];
        let sticks = map.map_sticks(&frame);
        assert_eq!(sticks.roll, Some(1600));
        assert_eq!(sticks.pitch, Some(1500));
        assert_eq!(sticks.throttle, Some(1400));
        assert_eq!(sticks.yaw, Some(1900));
    }

    #[test]
    fn invalid_or_missing_map_is_none() {
        assert_eq!(rcmap_index(0), None);
        assert_eq!(rcmap_index(17), None);
        assert_eq!(mapped_pwm(&[1500], 2), None);
        assert_eq!(mapped_pwm(&[1500], 0), None);
        let sticks = RcMap::from_params(1, 2, 3, 0).map_sticks(&[1500, 1500, 1500, 1500]);
        assert_eq!(sticks.yaw, None);
        assert_eq!(sticks.roll, Some(1500));
    }

    #[test]
    fn set_and_save_trim_is_ifchanged() {
        let mut ch = RcChannel::default();
        assert!(!set_and_save_trim(&mut ch, 1500));
        assert_eq!(ch.radio_trim, 1500);
        assert!(set_and_save_trim(&mut ch, 1480));
        assert_eq!(ch.radio_trim, 1480);
        assert!(!set_and_save_radio_trim(&mut ch, 1480));
        assert!(set_and_save_radio_trim(&mut ch, 1520));
        assert_eq!(ch.radio_trim, 1520);
    }

    #[test]
    fn persist_stick_trims_skips_unchanged_and_leaves_throttle() {
        let mut roll = RcChannel::default();
        let mut pitch = RcChannel::default();
        let mut yaw = RcChannel::default();
        let throttle = RcChannel::default();
        assert!(!persist_stick_trims(
            &mut roll, &mut pitch, &mut yaw, 1500, 1500, 1500
        ));
        assert!(persist_stick_trims(
            &mut roll, &mut pitch, &mut yaw, 1488, 1500, 1512
        ));
        assert_eq!(roll.radio_trim, 1488);
        assert_eq!(pitch.radio_trim, 1500);
        assert_eq!(yaw.radio_trim, 1512);
        assert_eq!(throttle.radio_trim, 1500);
    }
}
