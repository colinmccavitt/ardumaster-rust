//! RC channel PWM scaling and deadzone, upstream `libraries/RC_Channel`. FW-019.
//!
//! A receiver reports pulse widths. The vehicle flies on a signed stick in
//! `[-1, 1]`. The conversion is not a single linear map: each channel has its
//! own min/trim/max, an optional reverse, and a deadzone around trim so a
//! resting stick is zero rather than a few counts of noise.
//!
//! Scaling, the aux-function switch latch, the 2-pos vs 3-pos option-switch PWM
//! ranges, the FS_THR_VALUE / THR_FS_VALUE PWM floor, the RCMAP_* channel map
//! plus RCn_TRIM persist, the RC_OVERRIDE_TIME GCS override timeout, and the
//! FLTMODE_CH six-position flight-mode switch decode live here so radio.cpp
//! work can share one conversion. The HAL owns the raw PWM microsecond I/O;
//! Plane's failsafe hookup already reads those pulses.

#![no_std]

pub mod aux_switch;
pub mod fltmode;
pub mod fs_thr;
pub mod option_switch;
pub mod override_timeout;
pub mod rcmap;

pub use aux_switch::{
    get_aux_switch_pos, init_position_on_first_radio_read, read_3pos_switch, AuxFunc,
    AuxSwitchLatch, AuxSwitchPos, AUX_SWITCH_PWM_TRIGGER_HIGH, AUX_SWITCH_PWM_TRIGGER_LOW,
    RC_MAX_LIMIT_PWM, RC_MIN_LIMIT_PWM, SWITCH_DEBOUNCE_TIME_MS,
};
pub use fltmode::{
    decode_fltmode_ch, decode_fltmode_switch, flight_mode_channel_index, flight_mode_channel_pwm,
    fltmode_ch_valid, read_6pos_switch, FLTMODE_CH_DEFAULT, FLTMODE_CH_DISABLED,
    FLTMODE_POS0_MAX_PWM, FLTMODE_POS1_MAX_PWM, FLTMODE_POS2_MAX_PWM, FLTMODE_POS3_MAX_PWM,
    FLTMODE_POS4_MAX_PWM, NUM_RC_CHANNELS,
};
pub use fs_thr::{
    throttle_below_fs_thr_value, throttle_pwm_in_failsafe, ThrFailsafe, FS_THR_VALUE_DEFAULT,
    FS_THR_VALUE_MAX, FS_THR_VALUE_MIN, THR_FS_VALUE_DEFAULT, THR_FS_VALUE_MAX, THR_FS_VALUE_MIN,
};
pub use option_switch::{
    get_stick_gesture_pos, option_switch_asserted, option_switch_has_three_positions,
    read_2pos_switch, read_option_switch, AUX_PWM_TRIGGER_HIGH, AUX_PWM_TRIGGER_LOW,
    STICK_GESTURE_MAX_PWM, STICK_GESTURE_MIN_PWM,
};
pub use override_timeout::{
    apply_gcs_override_field, override_timeout_from_param, OverrideTimeout, RcOverride,
    RC_OVERRIDE_IGNORE, RC_OVERRIDE_RELEASE, RC_OVERRIDE_TIME_DEFAULT, RC_OVERRIDE_TIME_MAX,
    RC_OVERRIDE_TIME_MIN,
};
pub use rcmap::{
    mapped_pwm, persist_stick_trims, rcmap_channel_valid, rcmap_index, set_and_save_radio_trim,
    set_and_save_trim, MappedStickPwm, RcMap, RCMAP_CHANNEL_MAX, RCMAP_CHANNEL_MIN,
    RCMAP_PITCH_DEFAULT, RCMAP_ROLL_DEFAULT, RCMAP_THROTTLE_DEFAULT, RCMAP_YAW_DEFAULT,
};

/// Upstream `RC_CHAN_MIN_DEFAULT` / `RC_Channel::radio_min` default.
pub const RC_CHAN_MIN_DEFAULT: u16 = 1100;
/// Upstream `RC_CHAN_TRIM_DEFAULT` / `RC_Channel::radio_trim` default.
pub const RC_CHAN_TRIM_DEFAULT: u16 = 1500;
/// Upstream `RC_CHAN_MAX_DEFAULT` / `RC_Channel::radio_max` default.
pub const RC_CHAN_MAX_DEFAULT: u16 = 1900;
/// Plane stick default, upstream `channel_*->set_default_dead_zone(30)`.
pub const RC_CHAN_DEADZONE_DEFAULT: u16 = 30;

/// Per-channel calibration, upstream `RC_Channel` radio_min/trim/max/dead_zone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcChannel {
    /// Upstream `radio_min` (PWM microseconds).
    pub radio_min: u16,
    /// Upstream `radio_trim` (PWM microseconds).
    pub radio_trim: u16,
    /// Upstream `radio_max` (PWM microseconds).
    pub radio_max: u16,
    /// Upstream `dead_zone` (PWM microseconds around trim).
    pub deadzone: u16,
    /// Upstream `reversed`.
    pub reversed: bool,
}

impl Default for RcChannel {
    fn default() -> Self {
        Self {
            radio_min: RC_CHAN_MIN_DEFAULT,
            radio_trim: RC_CHAN_TRIM_DEFAULT,
            radio_max: RC_CHAN_MAX_DEFAULT,
            deadzone: RC_CHAN_DEADZONE_DEFAULT,
            reversed: false,
        }
    }
}

fn constrain_norm(value: f32) -> f32 {
    if value < -1.0 {
        -1.0
    } else if value > 1.0 {
        1.0
    } else {
        value
    }
}

/// Signed stick without deadzone, upstream `RC_Channel::norm_input`.
///
/// Below trim the span is `[radio_min, radio_trim]`; above trim it is
/// `[radio_trim, radio_max]`. A collapsed side (min ≥ trim, or max ≤ trim)
/// returns 0 on that side, matching upstream rather than dividing by zero.
#[must_use]
pub fn norm_input(pwm: u16, ch: &RcChannel) -> f32 {
    let reverse_mul = if ch.reversed { -1.0 } else { 1.0 };
    let ret = if pwm < ch.radio_trim {
        if ch.radio_min >= ch.radio_trim {
            return 0.0;
        }
        reverse_mul * (f32::from(pwm) - f32::from(ch.radio_trim))
            / (f32::from(ch.radio_trim) - f32::from(ch.radio_min))
    } else {
        if ch.radio_max <= ch.radio_trim {
            return 0.0;
        }
        reverse_mul * (f32::from(pwm) - f32::from(ch.radio_trim))
            / (f32::from(ch.radio_max) - f32::from(ch.radio_trim))
    };
    constrain_norm(ret)
}

/// Signed stick with deadzone, upstream `RC_Channel::norm_input_dz`.
///
/// The deadzone is a PWM window `[trim − dz, trim + dz]`. Inside it the
/// result is 0. Outside it the span is from the deadzone edge to min or max,
/// so a stick that just leaves the window starts from zero rather than jumping.
#[must_use]
pub fn norm_input_dz(pwm: u16, ch: &RcChannel) -> f32 {
    let reverse_mul = if ch.reversed { -1.0 } else { 1.0 };
    let dz_min = ch.radio_trim.saturating_sub(ch.deadzone);
    let dz_max = ch.radio_trim.saturating_add(ch.deadzone);
    let ret = if pwm < dz_min && dz_min > ch.radio_min {
        reverse_mul * (f32::from(pwm) - f32::from(dz_min))
            / (f32::from(dz_min) - f32::from(ch.radio_min))
    } else if pwm > dz_max && dz_max < ch.radio_max {
        reverse_mul * (f32::from(pwm) - f32::from(dz_max))
            / (f32::from(ch.radio_max) - f32::from(dz_max))
    } else {
        0.0
    };
    constrain_norm(ret)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_match_upstream_radio() {
        let ch = RcChannel::default();
        assert_eq!(ch.radio_min, RC_CHAN_MIN_DEFAULT);
        assert_eq!(ch.radio_trim, RC_CHAN_TRIM_DEFAULT);
        assert_eq!(ch.radio_max, RC_CHAN_MAX_DEFAULT);
        assert_eq!(ch.deadzone, RC_CHAN_DEADZONE_DEFAULT);
        assert!(!ch.reversed);
    }

    #[test]
    fn norm_input_is_neutral_at_trim() {
        let ch = RcChannel::default();
        assert!((norm_input(1500, &ch) - 0.0).abs() < 1e-6);
    }

    #[test]
    fn norm_input_reaches_extremes_at_limits() {
        let ch = RcChannel::default();
        assert!((norm_input(1100, &ch) + 1.0).abs() < 1e-6);
        assert!((norm_input(1900, &ch) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn norm_input_dz_swallows_pwm_inside_deadzone() {
        let ch = RcChannel::default();
        assert!((norm_input_dz(1500, &ch) - 0.0).abs() < 1e-6);
        assert!((norm_input_dz(1470, &ch) - 0.0).abs() < 1e-6);
        assert!((norm_input_dz(1530, &ch) - 0.0).abs() < 1e-6);
        assert!((norm_input_dz(1469, &ch) - 0.0).abs() > 1e-4);
        assert!((norm_input_dz(1531, &ch) - 0.0).abs() > 1e-4);
    }

    #[test]
    fn norm_input_dz_reaches_extremes_at_limits() {
        let ch = RcChannel::default();
        assert!((norm_input_dz(1100, &ch) + 1.0).abs() < 1e-6);
        assert!((norm_input_dz(1900, &ch) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn reversed_flips_signed_stick() {
        let ch = RcChannel {
            reversed: true,
            ..RcChannel::default()
        };
        assert!((norm_input(1100, &ch) - 1.0).abs() < 1e-6);
        assert!((norm_input(1900, &ch) + 1.0).abs() < 1e-6);
        assert!((norm_input_dz(1100, &ch) - 1.0).abs() < 1e-6);
        assert!((norm_input_dz(1900, &ch) + 1.0).abs() < 1e-6);
    }

    #[test]
    fn collapsed_range_is_zero_not_nan() {
        let ch = RcChannel {
            radio_min: 1500,
            radio_trim: 1500,
            radio_max: 1500,
            deadzone: 30,
            reversed: false,
        };
        assert!((norm_input(1500, &ch) - 0.0).abs() < 1e-6);
        assert!((norm_input_dz(1400, &ch) - 0.0).abs() < 1e-6);
    }
}
