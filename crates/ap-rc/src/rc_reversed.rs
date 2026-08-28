//! Per-channel input reverse, upstream `RC_Channel::reversed` / `RCn_REVERSED`.
//!
//! Each receiver channel stores an `AP_Int8` named `REVERSED` (0 = Normal,
//! 1 = Reversed). `get_reverse()` is `bool(reversed.get())` — any non-zero
//! stored value is reversed. PWM scaling already multiplies the signed stick
//! by ±1; this module is the parameter decode and the apply that writes
//! that flag onto [`RcChannel`]. Range-type `control_in` also mirrors the
//! pulse about the min/max span (`radio_max - (radio_in - radio_min)`).

use crate::RcChannel;

/// Upstream `RC_Channel::reversed` default / `RCn_REVERSED = 0`.
pub const RC_REVERSED_DEFAULT: i8 = 0;
/// Upstream `@Values: 0:Normal`.
pub const RC_REVERSED_NORMAL: i8 = 0;
/// Upstream `@Values: 1:Reversed`.
pub const RC_REVERSED_REVERSED: i8 = 1;

/// Decoded `RCn_REVERSED`, upstream `RC_Channel::reversed` after `get_reverse`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RcReversed {
    /// True when the channel input is reversed.
    pub reversed: bool,
}

impl Default for RcReversed {
    fn default() -> Self {
        Self {
            reversed: get_reverse(RC_REVERSED_DEFAULT),
        }
    }
}

impl RcReversed {
    /// Wrap a stored `RCn_REVERSED` (`AP_Int8`) via [`get_reverse`].
    #[must_use]
    pub const fn from_param(reversed: i8) -> Self {
        Self {
            reversed: get_reverse(reversed),
        }
    }

    /// Stamp the decoded flag onto a channel's `reversed` field.
    #[must_use]
    pub const fn apply_to(self, ch: RcChannel) -> RcChannel {
        RcChannel {
            reversed: self.reversed,
            radio_min: ch.radio_min,
            radio_trim: ch.radio_trim,
            radio_max: ch.radio_max,
            deadzone: ch.deadzone,
        }
    }
}

/// Decode a stored `RCn_REVERSED` as upstream `RC_Channel::get_reverse`.
///
/// `bool(reversed.get())`: any non-zero `AP_Int8` is reversed.
#[must_use]
pub const fn get_reverse(reversed: i8) -> bool {
    reversed != 0
}

/// Apply `RCn_REVERSED` as the per-channel reverse flag.
#[must_use]
pub const fn apply_rc_reversed(reversed: i8) -> RcReversed {
    RcReversed::from_param(reversed)
}

/// Mirror a constrained pulse about the min/max span when reversed.
///
/// Upstream `pwm_to_range_dz`: `r_in = constrain(radio_in, min, max)` then,
/// if reversed, `r_in = radio_max - (r_in - radio_min)`. Normal channels
/// return the constrained pulse unchanged.
#[must_use]
pub fn reverse_range_pwm(pwm: u16, ch: &RcChannel) -> u16 {
    let r_in = if pwm < ch.radio_min {
        ch.radio_min
    } else if pwm > ch.radio_max {
        ch.radio_max
    } else {
        pwm
    };
    if ch.reversed {
        ch.radio_max
            .saturating_sub(r_in.saturating_sub(ch.radio_min))
    } else {
        r_in
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{norm_input, RcChannel};

    #[test]
    fn defaults_match_upstream_rcn_reversed() {
        assert_eq!(RC_REVERSED_DEFAULT, 0);
        assert_eq!(RC_REVERSED_NORMAL, 0);
        assert_eq!(RC_REVERSED_REVERSED, 1);
        assert!(!RcReversed::default().reversed);
        assert!(!get_reverse(RC_REVERSED_DEFAULT));
    }

    #[test]
    fn get_reverse_is_bool_of_stored_int8() {
        assert!(!get_reverse(0));
        assert!(get_reverse(1));
        assert!(get_reverse(-1));
        assert!(get_reverse(2));
    }

    #[test]
    fn apply_stamps_flag_and_flips_signed_stick() {
        let ch = apply_rc_reversed(RC_REVERSED_REVERSED).apply_to(RcChannel::default());
        assert!(ch.reversed);
        assert!((norm_input(1100, &ch) - 1.0).abs() < 1e-6);
        assert!((norm_input(1900, &ch) + 1.0).abs() < 1e-6);
        let normal = apply_rc_reversed(RC_REVERSED_NORMAL).apply_to(RcChannel::default());
        assert!(!normal.reversed);
        assert!((norm_input(1100, &normal) + 1.0).abs() < 1e-6);
    }

    #[test]
    fn range_pwm_mirrors_about_min_max_when_reversed() {
        let ch = apply_rc_reversed(1).apply_to(RcChannel::default());
        assert_eq!(reverse_range_pwm(1100, &ch), 1900);
        assert_eq!(reverse_range_pwm(1900, &ch), 1100);
        assert_eq!(reverse_range_pwm(1500, &ch), 1500);
        assert_eq!(reverse_range_pwm(1300, &ch), 1700);
        let normal = RcChannel::default();
        assert_eq!(reverse_range_pwm(1100, &normal), 1100);
        assert_eq!(reverse_range_pwm(800, &normal), 1100);
        assert_eq!(reverse_range_pwm(2200, &normal), 1900);
    }
}
