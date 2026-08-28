//! ARMING_CRASH_IF_DISARMED / crash-check-while-disarmed.

use ap_arming::crash_if_disarmed::{
    crash_check_while_disarmed, crash_dump_allows_arm, crash_if_disarmed_named_check,
    CrashIfDisarmed, ARMING_CRASH_IF_DISARMED_DEFAULT, CRASH_DUMP_CHECK_NAME,
};
use ap_arming::{Arming, Check, PreArmOutcome};

#[test]
fn plane_default_does_not_crash_check_while_disarmed() {
    assert_eq!(ARMING_CRASH_IF_DISARMED_DEFAULT, CrashIfDisarmed::Disabled);
    assert_eq!(ARMING_CRASH_IF_DISARMED_DEFAULT.as_u8(), 0);
    assert!(!CrashIfDisarmed::Disabled.enabled());
    assert!(CrashIfDisarmed::Enabled.enabled());
}

#[test]
fn from_u8_decodes_only_the_two_upstream_values() {
    assert_eq!(CrashIfDisarmed::from_u8(0), Some(CrashIfDisarmed::Disabled));
    assert_eq!(CrashIfDisarmed::from_u8(1), Some(CrashIfDisarmed::Enabled));
    assert_eq!(CrashIfDisarmed::from_u8(2), None);
}

#[test]
fn disarmed_crash_check_runs_only_when_option_enabled() {
    assert!(!crash_check_while_disarmed(CrashIfDisarmed::Disabled, false));
    assert!(crash_check_while_disarmed(CrashIfDisarmed::Enabled, false));
}

#[test]
fn armed_path_is_not_this_option() {
    assert!(!crash_check_while_disarmed(CrashIfDisarmed::Disabled, true));
    assert!(!crash_check_while_disarmed(CrashIfDisarmed::Enabled, true));
}

#[test]
fn unacked_dump_refuses_only_when_disarmed_gate_is_on() {
    assert!(crash_dump_allows_arm(
        CrashIfDisarmed::Disabled,
        false,
        true,
        false,
    ));
    assert!(!crash_dump_allows_arm(
        CrashIfDisarmed::Enabled,
        false,
        true,
        false,
    ));
    assert!(crash_dump_allows_arm(
        CrashIfDisarmed::Enabled,
        false,
        true,
        true,
    ));
    assert!(crash_dump_allows_arm(
        CrashIfDisarmed::Enabled,
        false,
        false,
        false,
    ));
}

#[test]
fn armed_sample_does_not_refuse_on_this_gate() {
    assert!(crash_dump_allows_arm(
        CrashIfDisarmed::Enabled,
        true,
        true,
        false,
    ));
}

#[test]
fn registry_refuses_when_disarmed_gate_sees_unacked_dump() {
    let arming = Arming::new();
    let named = crash_if_disarmed_named_check(CrashIfDisarmed::Enabled, false, true, false);
    assert_eq!(named.check, Check::Parameters);
    assert_eq!(named.name, CRASH_DUMP_CHECK_NAME);
    assert!(!named.ok);
    assert_eq!(
        arming.pre_arm_checks(&[named]),
        PreArmOutcome::Refused {
            check: Check::Parameters,
            name: CRASH_DUMP_CHECK_NAME,
        }
    );
}

#[test]
fn registry_allows_when_dump_acked_or_absent() {
    let arming = Arming::new();
    let acked = crash_if_disarmed_named_check(CrashIfDisarmed::Enabled, false, true, true);
    assert!(acked.ok);
    assert_eq!(arming.pre_arm_checks(&[acked]), PreArmOutcome::Allowed);

    let absent = crash_if_disarmed_named_check(CrashIfDisarmed::Enabled, false, false, false);
    assert!(absent.ok);
    assert_eq!(arming.pre_arm_checks(&[absent]), PreArmOutcome::Allowed);
}

#[test]
fn registry_allows_when_option_is_off_even_with_unacked_dump() {
    let arming = Arming::new();
    let named = crash_if_disarmed_named_check(CrashIfDisarmed::Disabled, false, true, false);
    assert!(named.ok);
    assert_eq!(arming.pre_arm_checks(&[named]), PreArmOutcome::Allowed);
}
