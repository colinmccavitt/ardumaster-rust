//! `RC_OPTIONS` bitfield, upstream `RC_Channels::Option`.
//!
//! Decode the bitmask, then apply ignore-overrides / honor-overrides,
//! ignore-receiver, ignore-failsafe, and the RC arming-check bits.

use ap_rc::{
    apply_arming_rc_checks, apply_radio_in, apply_rc_options, apply_receiver_failsafe,
    apply_switch_reversed, RcOption, RcOptions, RC_OPTIONS_DEFAULT,
};

#[test]
fn rc_options_default_is_arming_check_throttle() {
    assert_eq!(RC_OPTIONS_DEFAULT, 32);
    assert_eq!(RcOption::ArmingCheckThrottle.bit(), 1 << 5);
    let opts = RcOptions::default();
    assert_eq!(opts.bits, RC_OPTIONS_DEFAULT);
    assert!(opts.option_is_enabled(RcOption::ArmingCheckThrottle));
    assert!(opts.arming_check_throttle());
    assert!(opts.honor_overrides());
    assert!(!opts.ignore_overrides());
    assert!(!opts.ignore_receiver());
    assert!(!opts.ignore_failsafe());
    assert!(!opts.arming_skip_check_rpy());
    assert!(!opts.allow_switch_rev());
}

#[test]
fn decode_ignore_and_arming_bits() {
    let bits = RcOption::IgnoreReceiver.bit()
        | RcOption::IgnoreOverrides.bit()
        | RcOption::IgnoreFailsafe.bit()
        | RcOption::ArmingCheckThrottle.bit()
        | RcOption::ArmingSkipCheckRpy.bit()
        | RcOption::AllowSwitchRev.bit();
    let opts = RcOptions::from_bits(bits);
    assert!(opts.ignore_receiver());
    assert!(opts.ignore_overrides());
    assert!(!opts.honor_overrides());
    assert!(opts.ignore_failsafe());
    assert!(opts.arming_check_throttle());
    assert!(opts.arming_skip_check_rpy());
    assert!(opts.allow_switch_rev());
    assert_eq!(RcOption::IgnoreReceiver.bit(), 1);
    assert_eq!(RcOption::IgnoreOverrides.bit(), 2);
    assert_eq!(RcOption::IgnoreFailsafe.bit(), 4);
    assert_eq!(RcOption::ArmingSkipCheckRpy.bit(), 64);
    assert_eq!(RcOption::AllowSwitchRev.bit(), 128);
}

#[test]
fn apply_honors_overrides_unless_ignored() {
    let honor = RcOptions::default();
    assert_eq!(apply_radio_in(honor, true, true, 1720, 1490), Some(1720));
    assert_eq!(apply_radio_in(honor, false, true, 1720, 1490), Some(1490));

    let ignore = RcOptions::from_bits(RcOption::IgnoreOverrides.bit());
    assert_eq!(apply_radio_in(ignore, true, true, 1720, 1490), Some(1490));
}

#[test]
fn apply_ignore_receiver_needs_a_live_override() {
    let ignore_rx = RcOptions::from_bits(RcOption::IgnoreReceiver.bit());
    assert_eq!(apply_radio_in(ignore_rx, false, true, 1600, 1510), None);
    assert_eq!(
        apply_radio_in(ignore_rx, true, true, 1600, 1510),
        Some(1600)
    );

    let both =
        RcOptions::from_bits(RcOption::IgnoreReceiver.bit() | RcOption::IgnoreOverrides.bit());
    assert_eq!(apply_radio_in(both, true, true, 1600, 1510), None);
}

#[test]
fn apply_failsafe_and_arming_and_switch() {
    let default = RcOptions::default();
    assert!(apply_receiver_failsafe(default, true));
    assert!(!apply_receiver_failsafe(default, false));
    let ignore_fs = RcOptions::from_bits(RcOption::IgnoreFailsafe.bit());
    assert!(!apply_receiver_failsafe(ignore_fs, true));

    let checks = apply_arming_rc_checks(default);
    assert!(checks.check_throttle_idle);
    assert!(checks.check_rpy_neutral);
    let skip_rpy = RcOptions::from_bits(RcOption::ArmingSkipCheckRpy.bit());
    let checks = apply_arming_rc_checks(skip_rpy);
    assert!(!checks.check_throttle_idle);
    assert!(!checks.check_rpy_neutral);

    assert!(!apply_switch_reversed(default, true));
    let rev = RcOptions::from_bits(RcOption::AllowSwitchRev.bit());
    assert!(apply_switch_reversed(rev, true));
    assert!(!apply_switch_reversed(rev, false));
}

#[test]
fn apply_rc_options_composes_decode() {
    let bits = RcOption::IgnoreOverrides.bit()
        | RcOption::IgnoreFailsafe.bit()
        | RcOption::ArmingCheckThrottle.bit()
        | RcOption::AllowSwitchRev.bit();
    let applied = apply_rc_options(
        RcOptions::from_bits(bits),
        true,
        true,
        1800,
        1400,
        true,
        true,
    );
    assert_eq!(applied.radio_in, Some(1400));
    assert!(!applied.in_failsafe);
    assert!(applied.check_throttle_idle);
    assert!(applied.check_rpy_neutral);
    assert!(applied.switch_reversed);
}
