//! Option-switch PWM ranges: 2-pos vs 3-pos `AUX_FUNCTION`.
//!
//! Upstream `RC_Channel` keeps two PWM tables for option switches:
//!
//! - 3-position `RCn_OPTION` switches use `AUX_SWITCH_PWM_TRIGGER_LOW` /
//!   `AUX_SWITCH_PWM_TRIGGER_HIGH` (1200 / 1800) via `read_3pos_switch`.
//! - 2-position option / stick-gesture reads use
//!   [`AUX_PWM_TRIGGER_LOW`] / [`AUX_PWM_TRIGGER_HIGH`] (1300 / 1700) via
//!   `get_stick_gesture_pos`. The wider middle band avoids glitching on
//!   stick travel.
//!
//! Which table applies is a property of the `RCn_OPTION` function: fence /
//! Q_ASSIST / soaring consume LOW / MIDDLE / HIGH as distinct states;
//! reverse-throttle and arm/disarm assert only on HIGH.

use crate::aux_switch::{
    read_3pos_switch, AuxFunc, AuxSwitchPos, RC_MAX_LIMIT_PWM, RC_MIN_LIMIT_PWM,
};

/// Upstream `RC_Channel::AUX_PWM_TRIGGER_LOW` (2-pos / stick-gesture).
pub const AUX_PWM_TRIGGER_LOW: u16 = 1300;
/// Upstream `RC_Channel::AUX_PWM_TRIGGER_HIGH` (2-pos / stick-gesture).
pub const AUX_PWM_TRIGGER_HIGH: u16 = 1700;
/// Invalid-low PWM for `get_stick_gesture_pos` (`<= 900`).
pub const STICK_GESTURE_MIN_PWM: u16 = 900;
/// Invalid-high PWM for `get_stick_gesture_pos` (`>= 2200`).
pub const STICK_GESTURE_MAX_PWM: u16 = 2200;

/// Whether this `RCn_OPTION` is a 3-position function.
///
/// 3-pos functions consume LOW / MIDDLE / HIGH as distinct states. 2-pos
/// functions assert only on HIGH; MIDDLE is unused.
#[must_use]
pub fn option_switch_has_three_positions(func: AuxFunc) -> bool {
    matches!(func, AuxFunc::Fence | AuxFunc::QAssist | AuxFunc::Soaring)
}

fn reverse_pos(pos: AuxSwitchPos) -> AuxSwitchPos {
    match pos {
        AuxSwitchPos::Low => AuxSwitchPos::High,
        AuxSwitchPos::Middle => AuxSwitchPos::Middle,
        AuxSwitchPos::High => AuxSwitchPos::Low,
    }
}

fn map_pwm_range(pwm: u16, low: u16, high: u16, reversed: bool) -> AuxSwitchPos {
    let pos = if pwm < low {
        AuxSwitchPos::Low
    } else if pwm > high {
        AuxSwitchPos::High
    } else {
        AuxSwitchPos::Middle
    };
    if reversed {
        reverse_pos(pos)
    } else {
        pos
    }
}

/// 2-pos option PWM-range decode, `AUX_PWM_TRIGGER_*` (1300 / 1700).
///
/// Channel validity matches `read_3pos_switch` (`[800, 2200)`). Returns
/// `None` when the pulse is invalid so a caller can skip the sample.
#[must_use]
pub fn read_2pos_switch(pwm: u16, reversed: bool) -> Option<AuxSwitchPos> {
    if pwm <= RC_MIN_LIMIT_PWM || pwm >= RC_MAX_LIMIT_PWM {
        return None;
    }
    Some(map_pwm_range(
        pwm,
        AUX_PWM_TRIGGER_LOW,
        AUX_PWM_TRIGGER_HIGH,
        reversed,
    ))
}

/// Map PWM through the 2-pos / stick-gesture table.
///
/// Upstream `RC_Channel::get_stick_gesture_pos`. Invalid pulse (`<= 900`
/// or `>= 2200`) is LOW. Reverse always applies, matching `get_reverse()`
/// rather than the aux-switch `ALLOW_SWITCH_REV` gate.
#[must_use]
pub fn get_stick_gesture_pos(pwm: u16, reversed: bool) -> AuxSwitchPos {
    if pwm <= STICK_GESTURE_MIN_PWM || pwm >= STICK_GESTURE_MAX_PWM {
        return AuxSwitchPos::Low;
    }
    map_pwm_range(pwm, AUX_PWM_TRIGGER_LOW, AUX_PWM_TRIGGER_HIGH, reversed)
}

/// Decode an option switch with the PWM table that `func` requires.
///
/// 3-pos functions use `read_3pos_switch` (1200 / 1800). 2-pos functions
/// use [`read_2pos_switch`] (1300 / 1700). `DO_NOTHING` never decodes.
#[must_use]
pub fn read_option_switch(pwm: u16, reversed: bool, func: AuxFunc) -> Option<AuxSwitchPos> {
    if func == AuxFunc::DoNothing {
        return None;
    }
    if option_switch_has_three_positions(func) {
        read_3pos_switch(pwm, reversed)
    } else {
        read_2pos_switch(pwm, reversed)
    }
}

/// True when a 2-pos option function is asserted (HIGH).
///
/// Upstream `do_aux_function` for reverse-throttle / inverted / mode
/// options: the function is on only in the HIGH position.
#[must_use]
pub fn option_switch_asserted(pos: AuxSwitchPos) -> bool {
    pos == AuxSwitchPos::High
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_pos_thresholds_match_upstream() {
        assert_eq!(AUX_PWM_TRIGGER_LOW, 1300);
        assert_eq!(AUX_PWM_TRIGGER_HIGH, 1700);
        assert_eq!(STICK_GESTURE_MIN_PWM, 900);
        assert_eq!(STICK_GESTURE_MAX_PWM, 2200);
    }

    #[test]
    fn two_pos_edges_are_middle_inclusive() {
        assert_eq!(read_2pos_switch(1299, false), Some(AuxSwitchPos::Low));
        assert_eq!(read_2pos_switch(1300, false), Some(AuxSwitchPos::Middle));
        assert_eq!(read_2pos_switch(1700, false), Some(AuxSwitchPos::Middle));
        assert_eq!(read_2pos_switch(1701, false), Some(AuxSwitchPos::High));
    }

    #[test]
    fn two_pos_invalid_pwm_is_none() {
        assert_eq!(read_2pos_switch(800, false), None);
        assert_eq!(read_2pos_switch(2200, false), None);
    }

    #[test]
    fn two_pos_reverse_flips_low_and_high() {
        assert_eq!(read_2pos_switch(1100, true), Some(AuxSwitchPos::High));
        assert_eq!(read_2pos_switch(1900, true), Some(AuxSwitchPos::Low));
        assert_eq!(read_2pos_switch(1500, true), Some(AuxSwitchPos::Middle));
    }

    #[test]
    fn stick_gesture_collapses_invalid_to_low() {
        assert_eq!(get_stick_gesture_pos(900, false), AuxSwitchPos::Low);
        assert_eq!(get_stick_gesture_pos(2200, false), AuxSwitchPos::Low);
        assert_eq!(get_stick_gesture_pos(1299, false), AuxSwitchPos::Low);
        assert_eq!(get_stick_gesture_pos(1701, false), AuxSwitchPos::High);
    }

    #[test]
    fn fence_is_three_pos_reverse_throttle_is_two_pos() {
        assert!(option_switch_has_three_positions(AuxFunc::Fence));
        assert!(!option_switch_has_three_positions(AuxFunc::ReverseThrottle));
        assert_eq!(
            read_option_switch(1250, false, AuxFunc::Fence),
            Some(AuxSwitchPos::Middle)
        );
        assert_eq!(
            read_option_switch(1250, false, AuxFunc::ReverseThrottle),
            Some(AuxSwitchPos::Low)
        );
        assert_eq!(read_option_switch(1500, false, AuxFunc::DoNothing), None);
    }

    #[test]
    fn two_pos_asserted_only_on_high() {
        assert!(!option_switch_asserted(AuxSwitchPos::Low));
        assert!(!option_switch_asserted(AuxSwitchPos::Middle));
        assert!(option_switch_asserted(AuxSwitchPos::High));
    }
}
