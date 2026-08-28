//! ARMING_CHECK enable bitmask: bits set run named checks; bit 0 is ALL.

use ap_arming::check_bitmask::{
    arming_check_enabled, arming_check_skips_all, arming_from_check, skipchk_from_arming_check,
    ARMING_CHECK_ALL, ARMING_CHECK_DEFAULT,
};
use ap_arming::{
    Arming, Check, NamedCheck, PreArmOutcome, Required, ARMING_SKIPCHK_DEFAULT, CHECK_MASK,
};

fn baro(ok: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Baro,
        name: "BARO",
        ok,
    }
}

fn compass(ok: bool) -> NamedCheck {
    NamedCheck {
        check: Check::Compass,
        name: "COMPASS",
        ok,
    }
}

#[test]
fn default_arming_check_is_all() {
    assert_eq!(ARMING_CHECK_DEFAULT, ARMING_CHECK_ALL);
    assert!(arming_check_enabled(ARMING_CHECK_DEFAULT, Check::Baro));
    assert!(arming_check_enabled(ARMING_CHECK_DEFAULT, Check::Compass));
    assert!(arming_check_enabled(ARMING_CHECK_DEFAULT, Check::Ins));
    assert!(!arming_check_skips_all(ARMING_CHECK_DEFAULT));
}

#[test]
fn zero_disables_every_named_check() {
    assert!(arming_check_skips_all(0));
    assert!(!arming_check_enabled(0, Check::Baro));
    assert!(!arming_check_enabled(0, Check::Compass));
    assert_eq!(skipchk_from_arming_check(0), u32::MAX);
}

#[test]
fn a_named_bit_without_all_enables_only_that_check() {
    let mask = Check::Baro.as_u32();
    assert!(arming_check_enabled(mask, Check::Baro));
    assert!(!arming_check_enabled(mask, Check::Compass));
    assert!(!arming_check_skips_all(mask));
    assert_eq!(skipchk_from_arming_check(mask), (!mask) & CHECK_MASK);
}

#[test]
fn all_converts_to_skip_nothing() {
    assert_eq!(
        skipchk_from_arming_check(ARMING_CHECK_ALL),
        ARMING_SKIPCHK_DEFAULT
    );
    assert_eq!(
        skipchk_from_arming_check(ARMING_CHECK_ALL | Check::Baro.as_u32()),
        0
    );
}

#[test]
fn converted_all_refuses_a_failing_named_check() {
    let arming = arming_from_check(Required::YesMinPwm, ARMING_CHECK_ALL);
    assert_eq!(arming.checks_to_skip, 0);
    assert!(arming.check_enabled(Check::Baro));
    assert_eq!(
        arming.pre_arm_checks(&[baro(false)]),
        PreArmOutcome::Refused {
            check: Check::Baro,
            name: "BARO",
        }
    );
}

#[test]
fn converted_zero_allows_a_failing_named_check() {
    let arming = arming_from_check(Required::YesMinPwm, 0);
    assert!(arming.should_skip_all_checks());
    assert_eq!(arming.pre_arm_checks(&[baro(false)]), PreArmOutcome::Allowed);
}

#[test]
fn converted_baro_only_skips_a_failing_compass() {
    let arming = arming_from_check(Required::YesMinPwm, Check::Baro.as_u32());
    assert!(arming.check_enabled(Check::Baro));
    assert!(!arming.check_enabled(Check::Compass));
    assert_eq!(
        arming.pre_arm_checks(&[baro(true), compass(false)]),
        PreArmOutcome::Allowed
    );
    assert_eq!(
        arming.pre_arm_checks(&[baro(false), compass(false)]),
        PreArmOutcome::Refused {
            check: Check::Baro,
            name: "BARO",
        }
    );
}

#[test]
fn converted_arming_matches_direct_skipchk() {
    let from_check = arming_from_check(Required::YesMinPwm, ARMING_CHECK_ALL);
    assert_eq!(from_check, Arming::new());
}
