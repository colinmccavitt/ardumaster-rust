//! Flight-mode switch / `FLTMODE_CH` decode and `FLTMODE1`–`FLTMODE6` mapping.
//!
//! `FLTMODE_CH` names the 1-based receiver channel that selects the flight
//! mode. Zero disables the switch. The pulse on that channel is a six-
//! position switch: PWM is sliced into positions 0–5, then those slots
//! index `FLTMODE1`–`FLTMODE6`. PWM edges live in [`read_6pos_switch`];
//! the parameter table is [`FltModeTable`].
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
/// Upstream `Plane::num_flight_modes` — slots `FLTMODE1` through `FLTMODE6`.
pub const NUM_FLIGHT_MODES: u8 = 6;

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

/// Upstream `Mode::Number::MANUAL`.
pub const MODE_NUMBER_MANUAL: i8 = 0;
/// Upstream `Mode::Number::FLY_BY_WIRE_A`.
pub const MODE_NUMBER_FLY_BY_WIRE_A: i8 = 5;
/// Upstream `Mode::Number::RTL`.
pub const MODE_NUMBER_RTL: i8 = 11;

/// Upstream `FLIGHT_MODE_1` / `FLTMODE1` default (`Mode::Number::RTL`).
pub const FLTMODE1_DEFAULT: i8 = MODE_NUMBER_RTL;
/// Upstream `FLIGHT_MODE_2` / `FLTMODE2` default (`Mode::Number::RTL`).
pub const FLTMODE2_DEFAULT: i8 = MODE_NUMBER_RTL;
/// Upstream `FLIGHT_MODE_3` / `FLTMODE3` default (`Mode::Number::FLY_BY_WIRE_A`).
pub const FLTMODE3_DEFAULT: i8 = MODE_NUMBER_FLY_BY_WIRE_A;
/// Upstream `FLIGHT_MODE_4` / `FLTMODE4` default (`Mode::Number::FLY_BY_WIRE_A`).
pub const FLTMODE4_DEFAULT: i8 = MODE_NUMBER_FLY_BY_WIRE_A;
/// Upstream `FLIGHT_MODE_5` / `FLTMODE5` default (`Mode::Number::MANUAL`).
pub const FLTMODE5_DEFAULT: i8 = MODE_NUMBER_MANUAL;
/// Upstream `FLIGHT_MODE_6` / `FLTMODE6` default (`Mode::Number::MANUAL`).
pub const FLTMODE6_DEFAULT: i8 = MODE_NUMBER_MANUAL;

/// Six `FLTMODE1`–`FLTMODE6` mode numbers, upstream `Plane::flight_modes`.
///
/// Slot 0 is `FLTMODE1` (lowest PWM), slot 5 is `FLTMODE6`. The PWM
/// edges that pick the slot stay in [`read_6pos_switch`]; this table is
/// the parameter mapping only. Values are `AP_Int8` mode numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FltModeTable {
    /// Mode numbers for switch positions 0–5 (`FLTMODE1`–`FLTMODE6`).
    pub modes: [i8; NUM_FLIGHT_MODES as usize],
}

impl Default for FltModeTable {
    fn default() -> Self {
        Self {
            modes: [
                FLTMODE1_DEFAULT,
                FLTMODE2_DEFAULT,
                FLTMODE3_DEFAULT,
                FLTMODE4_DEFAULT,
                FLTMODE5_DEFAULT,
                FLTMODE6_DEFAULT,
            ],
        }
    }
}

impl FltModeTable {
    /// Build a table from the six `FLTMODE*` parameters.
    #[must_use]
    pub const fn from_params(
        fltmode1: i8,
        fltmode2: i8,
        fltmode3: i8,
        fltmode4: i8,
        fltmode5: i8,
        fltmode6: i8,
    ) -> Self {
        Self {
            modes: [fltmode1, fltmode2, fltmode3, fltmode4, fltmode5, fltmode6],
        }
    }
}

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

/// `FLTMODE[n]` for a 0-based six-pos slot.
///
/// Upstream `plane.flight_modes[new_pos].get()`. Out of range is `None`,
/// matching `mode_switch_changed` rejecting a bad position.
#[must_use]
pub fn fltmode_for_slot(table: &FltModeTable, slot: u8) -> Option<i8> {
    table.modes.get(slot as usize).copied()
}

/// PWM → `FLTMODE[n]` mode number.
///
/// Slot decode reuses [`read_6pos_switch`]; invalid pulses stay `None`.
#[must_use]
pub fn decode_fltmode_number(table: &FltModeTable, pwm: u16) -> Option<i8> {
    let slot = read_6pos_switch(pwm)?;
    fltmode_for_slot(table, slot)
}

/// Pick `FLTMODE_CH` out of a frame and map it through `FLTMODE1`–`FLTMODE6`.
#[must_use]
pub fn decode_fltmode_from_channels(
    channels: &[u16],
    fltmode_ch: i8,
    table: &FltModeTable,
) -> Option<i8> {
    let slot = decode_fltmode_switch(channels, fltmode_ch)?;
    fltmode_for_slot(table, slot)
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

    #[test]
    fn default_table_is_rtl_rtl_fbwa_fbwa_manual_manual() {
        let table = FltModeTable::default();
        assert_eq!(NUM_FLIGHT_MODES, 6);
        assert_eq!(table.modes, [11, 11, 5, 5, 0, 0]);
        assert_eq!(fltmode_for_slot(&table, 0), Some(MODE_NUMBER_RTL));
        assert_eq!(fltmode_for_slot(&table, 1), Some(MODE_NUMBER_RTL));
        assert_eq!(
            fltmode_for_slot(&table, 2),
            Some(MODE_NUMBER_FLY_BY_WIRE_A)
        );
        assert_eq!(
            fltmode_for_slot(&table, 3),
            Some(MODE_NUMBER_FLY_BY_WIRE_A)
        );
        assert_eq!(fltmode_for_slot(&table, 4), Some(MODE_NUMBER_MANUAL));
        assert_eq!(fltmode_for_slot(&table, 5), Some(MODE_NUMBER_MANUAL));
        assert_eq!(fltmode_for_slot(&table, 6), None);
    }

    #[test]
    fn pwm_maps_through_slot_to_fltmoden() {
        let table = FltModeTable::default();
        // Mid-band PWM in each slot — edges stay in read_6pos_switch tests.
        assert_eq!(decode_fltmode_number(&table, 1100), Some(MODE_NUMBER_RTL));
        assert_eq!(decode_fltmode_number(&table, 1300), Some(MODE_NUMBER_RTL));
        assert_eq!(
            decode_fltmode_number(&table, 1400),
            Some(MODE_NUMBER_FLY_BY_WIRE_A)
        );
        assert_eq!(
            decode_fltmode_number(&table, 1550),
            Some(MODE_NUMBER_FLY_BY_WIRE_A)
        );
        assert_eq!(decode_fltmode_number(&table, 1680), Some(MODE_NUMBER_MANUAL));
        assert_eq!(decode_fltmode_number(&table, 1900), Some(MODE_NUMBER_MANUAL));
        assert_eq!(decode_fltmode_number(&table, 800), None);
    }
}
