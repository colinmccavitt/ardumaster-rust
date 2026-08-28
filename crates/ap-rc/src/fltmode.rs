//! Flight-mode switch / `FLTMODE_CH` decode, upstream `RC_Channel::read_6pos_switch`.
//!
//! `FLTMODE_CH` names the 1-based receiver channel that selects the flight
//! mode. Zero disables the switch. The pulse on that channel is a six-
//! position switch: PWM is sliced into positions 0–5 and Plane later maps
//! those onto `FLTMODE1`–`FLTMODE6`. This module is the channel + PWM
//! decode only; the six-parameter mapping is a later slice.
//!
//! Invalid pulses (`<= RC_MIN_LIMIT_PWM` or `>= RC_MAX_LIMIT_PWM`) do not
//! produce a position, matching `read_6pos_switch`'s error return. Reverse
//! does not apply: the mode switch is not an aux-function option.

use crate::aux_switch::{RC_MAX_LIMIT_PWM, RC_MIN_LIMIT_PWM};

/// Upstream `FLIGHT_MODE_CHANNEL` / `FLTMODE_CH` default (channel 8).
pub const FLTMODE_CH_DEFAULT: i8 = 8;
/// `FLTMODE_CH == 0` disables the flight-mode switch.
pub const FLTMODE_CH_DISABLED: i8 = 0;
/// Upstream `NUM_RC_CHANNELS`. `FLTMODE_CH` uses this as an exclusive max.
pub const NUM_RC_CHANNELS: i8 = 16;

/// Exclusive PWM upper bound for switch position 0 (`pulsewidth < 1231`).
pub const FLTMODE_POS0_MAX_PWM: u16 = 1231;
/// Exclusive PWM upper bound for switch position 1 (`pulsewidth < 1361`).
pub const FLTMODE_POS1_MAX_PWM: u16 = 1361;
/// Exclusive PWM upper bound for switch position 2 (`pulsewidth < 1491`).
pub const FLTMODE_POS2_MAX_PWM: u16 = 1491;
/// Exclusive PWM upper bound for switch position 3 (`pulsewidth < 1621`).
pub const FLTMODE_POS3_MAX_PWM: u16 = 1621;
/// Exclusive PWM upper bound for switch position 4 (`pulsewidth < 1750`).
pub const FLTMODE_POS4_MAX_PWM: u16 = 1750;

/// True when `FLTMODE_CH` names a live receiver channel.
///
/// Upstream `RC_Channels::flight_mode_channel`: `<= 0` is disabled,
/// `>= NUM_RC_CHANNELS` is rejected (so channel 16 is not a mode switch).
#[must_use]
pub const fn fltmode_ch_valid(fltmode_ch: i8) -> bool {
    fltmode_ch > FLTMODE_CH_DISABLED && fltmode_ch < NUM_RC_CHANNELS
}

/// 0-based receiver index for `FLTMODE_CH`.
///
/// Upstream `rc_channel(flight_mode_channel_number() - 1)`. Disabled or
/// out-of-range is `None`.
#[must_use]
pub const fn flight_mode_channel_index(fltmode_ch: i8) -> Option<usize> {
    if fltmode_ch_valid(fltmode_ch) {
        Some((fltmode_ch - 1) as usize)
    } else {
        None
    }
}

/// PWM on the `FLTMODE_CH` channel from a 0-based receiver frame.
///
/// Missing pulse, a disabled switch, or an out-of-range `FLTMODE_CH`
/// returns `None`.
#[must_use]
pub fn flight_mode_channel_pwm(channels: &[u16], fltmode_ch: i8) -> Option<u16> {
    let idx = flight_mode_channel_index(fltmode_ch)?;
    channels.get(idx).copied()
}

/// Six-position PWM decode, upstream `RC_Channel::read_6pos_switch`.
///
/// Thresholds are exclusive upper bounds. Returns `None` when the pulse
/// is outside `[RC_MIN_LIMIT_PWM, RC_MAX_LIMIT_PWM)`. Debounce is not
/// applied here — that is the aux-switch latch's job.
#[must_use]
pub fn read_6pos_switch(pwm: u16) -> Option<u8> {
    if pwm <= RC_MIN_LIMIT_PWM || pwm >= RC_MAX_LIMIT_PWM {
        return None;
    }
    Some(if pwm < FLTMODE_POS0_MAX_PWM {
        0
    } else if pwm < FLTMODE_POS1_MAX_PWM {
        1
    } else if pwm < FLTMODE_POS2_MAX_PWM {
        2
    } else if pwm < FLTMODE_POS3_MAX_PWM {
        3
    } else if pwm < FLTMODE_POS4_MAX_PWM {
        4
    } else {
        5
    })
}

/// Decode `FLTMODE_CH` plus a pulse into a 0–5 switch position.
///
/// A disabled or invalid channel, or an invalid pulse, is `None`.
#[must_use]
pub fn decode_fltmode_ch(fltmode_ch: i8, pwm: u16) -> Option<u8> {
    if !fltmode_ch_valid(fltmode_ch) {
        return None;
    }
    read_6pos_switch(pwm)
}

/// Pick `FLTMODE_CH` out of a receiver frame and decode the six-pos switch.
#[must_use]
pub fn decode_fltmode_switch(channels: &[u16], fltmode_ch: i8) -> Option<u8> {
    let pwm = flight_mode_channel_pwm(channels, fltmode_ch)?;
    read_6pos_switch(pwm)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_channel_is_eight() {
        assert_eq!(FLTMODE_CH_DEFAULT, 8);
        assert!(fltmode_ch_valid(FLTMODE_CH_DEFAULT));
        assert_eq!(flight_mode_channel_index(FLTMODE_CH_DEFAULT), Some(7));
    }

    #[test]
    fn zero_disables_the_switch() {
        assert!(!fltmode_ch_valid(FLTMODE_CH_DISABLED));
        assert_eq!(flight_mode_channel_index(0), None);
        assert_eq!(decode_fltmode_ch(0, 1500), None);
    }

    #[test]
    fn channel_sixteen_is_rejected_like_upstream() {
        assert!(!fltmode_ch_valid(NUM_RC_CHANNELS));
        assert!(!fltmode_ch_valid(NUM_RC_CHANNELS + 1));
        assert!(!fltmode_ch_valid(-1));
        assert!(fltmode_ch_valid(1));
        assert!(fltmode_ch_valid(15));
        assert_eq!(flight_mode_channel_index(16), None);
        assert_eq!(decode_fltmode_ch(16, 1500), None);
    }

    #[test]
    fn six_pos_edges_match_upstream() {
        assert_eq!(read_6pos_switch(1230), Some(0));
        assert_eq!(read_6pos_switch(1231), Some(1));
        assert_eq!(read_6pos_switch(1360), Some(1));
        assert_eq!(read_6pos_switch(1361), Some(2));
        assert_eq!(read_6pos_switch(1490), Some(2));
        assert_eq!(read_6pos_switch(1491), Some(3));
        assert_eq!(read_6pos_switch(1620), Some(3));
        assert_eq!(read_6pos_switch(1621), Some(4));
        assert_eq!(read_6pos_switch(1749), Some(4));
        assert_eq!(read_6pos_switch(1750), Some(5));
    }

    #[test]
    fn invalid_pwm_is_none() {
        assert_eq!(read_6pos_switch(800), None);
        assert_eq!(read_6pos_switch(2200), None);
        assert_eq!(decode_fltmode_ch(FLTMODE_CH_DEFAULT, 800), None);
        assert_eq!(decode_fltmode_ch(FLTMODE_CH_DEFAULT, 2200), None);
    }
}
