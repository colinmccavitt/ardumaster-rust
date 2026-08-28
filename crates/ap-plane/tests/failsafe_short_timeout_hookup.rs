//! FS_SHORT_TIMEOUT short-failsafe entry delay, upstream `check_short_failsafe`.
//!
//! RC loss stays in `FAILSAFE_NONE` until `FS_SHORT_TIMEOUT` seconds have
//! elapsed since `failsafe.last_valid_rc_ms`, then raises SHORT so
//! `FS_SHORT_ACTN` can fire.

use ap_plane::failsafe_long_timeout_hookup::FailsafeState;
use ap_plane::failsafe_short_timeout_hookup::{
    check_rc_short_failsafe, ShortTimeoutDecision, ShortTimeoutInputs, FS_SHORT_TIMEOUT_DEFAULT,
    FS_SHORT_TIMEOUT_MAX, FS_SHORT_TIMEOUT_MIN,
};

fn lost_since(last_valid_rc_ms: u32, now_ms: u32) -> ShortTimeoutInputs {
    ShortTimeoutInputs {
        now_ms,
        last_valid_rc_ms,
        rc_failsafe: true,
        timeout_s: FS_SHORT_TIMEOUT_DEFAULT,
        state: FailsafeState::None,
        landing: false,
    }
}

#[test]
fn fs_short_timeout_values_match_upstream() {
    assert!((FS_SHORT_TIMEOUT_DEFAULT - 1.5).abs() < 1e-6);
    assert!((FS_SHORT_TIMEOUT_MIN - 1.0).abs() < 1e-6);
    assert!((FS_SHORT_TIMEOUT_MAX - 100.0).abs() < 1e-6);
}

#[test]
fn healthy_rc_never_enters() {
    let mut inp = lost_since(1_000, 1_000 + 60_000);
    inp.rc_failsafe = false;
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
}

#[test]
fn none_holds_until_deadline_then_enters() {
    let last = 2_000;
    let fresh = lost_since(last, last);
    assert_eq!(check_rc_short_failsafe(&fresh), ShortTimeoutDecision::Hold);
    // `age > FS_SHORT_TIMEOUT * 1000` — exclusive at the deadline.
    let at_deadline = lost_since(last, last + 1_500);
    assert_eq!(
        check_rc_short_failsafe(&at_deadline),
        ShortTimeoutDecision::Hold
    );
    let past = lost_since(last, last + 1_501);
    assert_eq!(
        check_rc_short_failsafe(&past),
        ShortTimeoutDecision::EnterShort
    );
}

#[test]
fn already_short_long_or_gcs_does_not_reenter() {
    let mut inp = lost_since(100, 100 + 1_501);
    inp.state = FailsafeState::Short;
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
    inp.state = FailsafeState::Long;
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
    inp.state = FailsafeState::Gcs;
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
}

#[test]
fn landing_blocks_entry_not_recovery() {
    let mut inp = lost_since(100, 100 + 1_501);
    inp.landing = true;
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);

    inp.state = FailsafeState::Short;
    inp.rc_failsafe = false;
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Recover);
}

#[test]
fn short_recovers_when_rc_returns() {
    let inp = ShortTimeoutInputs {
        now_ms: 10_000,
        last_valid_rc_ms: 10_000,
        rc_failsafe: false,
        timeout_s: FS_SHORT_TIMEOUT_DEFAULT,
        state: FailsafeState::Short,
        landing: false,
    };
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Recover);
}

#[test]
fn long_and_gcs_do_not_recover_on_rc_return() {
    let mut inp = ShortTimeoutInputs {
        now_ms: 10_000,
        last_valid_rc_ms: 10_000,
        rc_failsafe: false,
        timeout_s: FS_SHORT_TIMEOUT_DEFAULT,
        state: FailsafeState::Long,
        landing: false,
    };
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
    inp.state = FailsafeState::Gcs;
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
}

#[test]
fn custom_timeout_uses_configured_seconds() {
    let mut inp = lost_since(0, 1_000);
    inp.timeout_s = FS_SHORT_TIMEOUT_MIN;
    assert_eq!(check_rc_short_failsafe(&inp), ShortTimeoutDecision::Hold);
    inp.now_ms = 1_001;
    assert_eq!(
        check_rc_short_failsafe(&inp),
        ShortTimeoutDecision::EnterShort
    );
}
