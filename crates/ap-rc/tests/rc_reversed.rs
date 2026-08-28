//! `RCn_REVERSED` / per-channel reverse, upstream `RC_Channel::get_reverse`.
//!
//! Each channel stores an `AP_Int8` (`0` = Normal, `1` = Reversed). The
//! decode is `bool(reversed.get())`; applying it stamps [`RcChannel::reversed`]
//! so PWM scaling and range-type `control_in` both see the same flag.

use ap_rc::{
    apply_rc_reversed, get_reverse, norm_input, reverse_range_pwm, RcChannel, RcReversed,
    RC_REVERSED_DEFAULT, RC_REVERSED_NORMAL, RC_REVERSED_REVERSED,
};

#[test]
fn defaults_match_upstream_rcn_reversed() {
    assert_eq!(RC_REVERSED_DEFAULT, 0);
    assert_eq!(RC_REVERSED_NORMAL, 0);
    assert_eq!(RC_REVERSED_REVERSED, 1);
    assert!(!RcReversed::default().reversed);
    assert_eq!(
        apply_rc_reversed(RC_REVERSED_DEFAULT),
        RcReversed::default()
    );
}

#[test]
fn get_reverse_treats_any_nonzero_as_reversed() {
    assert!(!get_reverse(0));
    assert!(get_reverse(1));
    assert!(get_reverse(-1));
    assert!(get_reverse(i8::MAX));
}

#[test]
fn apply_writes_flag_onto_channel_and_flips_stick() {
    let ch = apply_rc_reversed(RC_REVERSED_REVERSED).apply_to(RcChannel::default());
    assert!(ch.reversed);
    assert!((norm_input(1100, &ch) - 1.0).abs() < 1e-6);
    assert!((norm_input(1900, &ch) + 1.0).abs() < 1e-6);

    let normal = apply_rc_reversed(RC_REVERSED_NORMAL).apply_to(RcChannel::default());
    assert!(!normal.reversed);
    assert!((norm_input(1100, &normal) + 1.0).abs() < 1e-6);
    assert!((norm_input(1900, &normal) - 1.0).abs() < 1e-6);
}

#[test]
fn range_type_reverse_mirrors_pwm_about_min_max() {
    let ch = apply_rc_reversed(1).apply_to(RcChannel::default());
    assert_eq!(reverse_range_pwm(1100, &ch), 1900);
    assert_eq!(reverse_range_pwm(1900, &ch), 1100);
    assert_eq!(reverse_range_pwm(1500, &ch), 1500);
    assert_eq!(reverse_range_pwm(1000, &ch), 1900);
    assert_eq!(reverse_range_pwm(2000, &ch), 1100);

    let normal = apply_rc_reversed(0).apply_to(RcChannel::default());
    assert_eq!(reverse_range_pwm(1100, &normal), 1100);
    assert_eq!(reverse_range_pwm(1900, &normal), 1900);
    assert_eq!(reverse_range_pwm(800, &normal), 1100);
}
