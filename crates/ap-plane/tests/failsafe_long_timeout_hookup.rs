//! FS_LONG_TIMEOUT short-to-long promotion timer, upstream `check_long_failsafe`.
//!
//! RC loss stays in short failsafe until `FS_LONG_TIMEOUT` seconds have
//! elapsed since `failsafe.last_valid_rc_ms`, then promotes to LONG.

use ap_plane::failsafe_long_timeout_hookup::{
    check_rc_long_failsafe, FailsafeState, LongTimeoutDecision, LongTimeoutInputs,
    FS_LONG_TIMEOUT_DEFAULT, FS_LONG_TIMEOUT_MAX, FS_LONG_TIMEOUT_MIN,
};

fn short_since(last_valid_rc_ms: u32, now_ms: u32) -> LongTimeoutInputs {
    LongTimeoutInputs {
        now_ms,
        last_valid_rc_ms,
        rc_failsafe: true,
        timeout_s: FS_LONG_TIMEOUT_DEFAULT,
        state: FailsafeState::Short,
        landing: false,
    }
}

#[test]
fn fs_long_timeout_values_match_upstream() {
    assert!((FS_LONG_TIMEOUT_DEFAULT - 5.0).abs() < 1e-6);
    assert!((FS_LONG_TIMEOUT_MIN - 1.0).abs() < 1e-6);
    assert!((FS_LONG_TIMEOUT_MAX - 300.0).abs() < 1e-6);
}

#[test]
fn healthy_rc_never_promotes() {
    let mut inp = short_since(1_000, 1_000 + 60_000);
    inp.rc_failsafe = false;
    inp.state = FailsafeState::None;
    assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Hold);
}

#[test]
fn short_hold_until_deadline_then_promote() {
    let last = 2_000;
    let fresh = short_since(last, last);
    assert_eq!(check_rc_long_failsafe(&fresh), LongTimeoutDecision::Hold);
    // `age > FS_LONG_TIMEOUT * 1000` — exclusive at the deadline.
    let at_deadline = short_since(last, last + 5_000);
    assert_eq!(
        check_rc_long_failsafe(&at_deadline),
        LongTimeoutDecision::Hold
    );
    let past = short_since(last, last + 5_001);
    assert_eq!(
        check_rc_long_failsafe(&past),
        LongTimeoutDecision::PromoteLong
    );
}

#[test]
fn none_and_short_both_promote_after_timeout() {
    let mut inp = short_since(500, 500 + 5_001);
    inp.state = FailsafeState::None;
    assert_eq!(
        check_rc_long_failsafe(&inp),
        LongTimeoutDecision::PromoteLong
    );
    inp.state = FailsafeState::Short;
    assert_eq!(
        check_rc_long_failsafe(&inp),
        LongTimeoutDecision::PromoteLong
    );
}

#[test]
fn already_long_or_gcs_does_not_reenter() {
    let mut inp = short_since(100, 100 + 5_001);
    inp.state = FailsafeState::Long;
    assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Hold);
    inp.state = FailsafeState::Gcs;
    assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Hold);
}

#[test]
fn landing_blocks_promotion_not_recovery() {
    let mut inp = short_since(100, 100 + 5_001);
    inp.landing = true;
    assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Hold);

    inp.state = FailsafeState::Long;
    inp.rc_failsafe = false;
    assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Recover);
}

#[test]
fn long_recovers_when_rc_returns() {
    let inp = LongTimeoutInputs {
        now_ms: 10_000,
        last_valid_rc_ms: 10_000,
        rc_failsafe: false,
        timeout_s: FS_LONG_TIMEOUT_DEFAULT,
        state: FailsafeState::Long,
        landing: false,
    };
    assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Recover);
}

#[test]
fn gcs_state_does_not_recover_on_rc_return() {
    let inp = LongTimeoutInputs {
        now_ms: 10_000,
        last_valid_rc_ms: 10_000,
        rc_failsafe: false,
        timeout_s: FS_LONG_TIMEOUT_DEFAULT,
        state: FailsafeState::Gcs,
        landing: false,
    };
    assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Hold);
}

#[test]
fn custom_timeout_uses_configured_seconds() {
    let mut inp = short_since(0, 1_000);
    inp.timeout_s = FS_LONG_TIMEOUT_MIN;
    assert_eq!(check_rc_long_failsafe(&inp), LongTimeoutDecision::Hold);
    inp.now_ms = 1_001;
    assert_eq!(
        check_rc_long_failsafe(&inp),
        LongTimeoutDecision::PromoteLong
    );
}
