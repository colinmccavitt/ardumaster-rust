//! ArduCopter's pre-arm checks, against the real firmware.
//!
//! Every check returns the same `false`, so the recording carries the reason
//! as well — captured from `AP_Arming::check_failed`, which is where the
//! refusal leaves the code. A fixture of return values alone would be
//! satisfied by any check refusing for any reason.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::arming::{
    alt_check, battery_failsafe_check, gcs_failsafe_check, interlock_checks, pre_arm_checks_apply,
    rc_throttle_failsafe_check, system_initialised_check, ArmRefusal, InterlockSwitches,
    RcFailsafeState, FS_THR_DISABLED,
};

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_arming.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out = Vec::new();
    let mut current = "";
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag;
            continue;
        }
        if line.is_empty() || line.starts_with(|c: char| c.is_alphabetic()) {
            continue;
        }
        if current == section {
            out.push(line.split(',').map(str::to_owned).collect());
        }
    }
    out
}

/// Compare a port refusal against the recorded outcome and message.
fn check(got: Option<ArmRefusal>, passed: bool, reason: &str, context: &str) {
    match got {
        None => assert!(
            passed,
            "{context}: the port passes, upstream refused with {reason:?}"
        ),
        Some(refusal) => {
            assert!(
                !passed,
                "{context}: the port refuses with {refusal:?}, upstream passed"
            );
            assert_eq!(
                refusal.message(),
                reason,
                "{context}: refused with {refusal:?}, upstream said {reason:?}"
            );
        }
    }
}

#[test]
fn the_rc_throttle_failsafe_check_matches_upstream() {
    let rows = rows("rc_failsafe");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut refusals = std::collections::BTreeMap::new();
    let mut passed_count = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 9, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let state = RcFailsafeState {
            rc_check_enabled: b(&r[1]),
            failsafe_throttle: r[2].trim().parse().expect("fs_thr"),
            has_had_rc_receiver: b(&r[3]),
            has_had_rc_override: b(&r[4]),
            throttle_radio_in: r[5].trim().parse().expect("radio_in"),
            failsafe_throttle_value: r[6].trim().parse().expect("threshold"),
        };
        let passed = b(&r[7]);
        let reason = r[8].trim();

        let got = rc_throttle_failsafe_check(&state);
        check(got, passed, reason, &format!("rc row {idx}"));

        if let Some(refusal) = got {
            *refusals.entry(refusal.message()).or_insert(0_usize) += 1;
        } else {
            passed_count += 1;
        }
    }

    assert_eq!(
        refusals.len(),
        2,
        "both refusals should be reached: {refusals:?}"
    );
    assert!(passed_count > 0, "no row passed");
    println!(
        "{} RC rows, {passed_count} passed, refusals {refusals:?}",
        rows.len()
    );
}

/// Turning the throttle failsafe off also turns off the no-pulses check.
///
/// Upstream's comment says so explicitly: a radio that has sent nothing
/// leaves `radio_in` at zero, which is below any threshold, so `FS_THR_ENABLE`
/// gates that failure too. An operator disabling the throttle failsafe is
/// therefore also disabling the check that would notice a receiver which
/// never spoke — worth knowing, because it is not what the parameter's name
/// suggests.
#[test]
fn disabling_the_throttle_failsafe_also_disables_the_no_pulses_check() {
    let no_radio_at_all = RcFailsafeState {
        rc_check_enabled: true,
        failsafe_throttle: FS_THR_DISABLED,
        has_had_rc_receiver: false,
        has_had_rc_override: false,
        throttle_radio_in: 0,
        failsafe_throttle_value: 975,
    };
    assert_eq!(
        rc_throttle_failsafe_check(&no_radio_at_all),
        None,
        "with the failsafe disabled, even a silent radio passes"
    );

    // Enable it and the same state is refused.
    let enabled = RcFailsafeState {
        failsafe_throttle: 1,
        ..no_radio_at_all
    };
    assert_eq!(
        rc_throttle_failsafe_check(&enabled),
        Some(ArmRefusal::RcNotFound)
    );

    // And ARMING_CHECK's RC bit switches the whole thing off ahead of that.
    let unchecked = RcFailsafeState {
        rc_check_enabled: false,
        ..enabled
    };
    assert_eq!(rc_throttle_failsafe_check(&unchecked), None);
}

/// A ground station's RC overrides count as a receiver.
///
/// A vehicle flown entirely from a companion computer has no radio and must
/// still be able to arm.
#[test]
fn an_override_counts_as_a_receiver() {
    let base = RcFailsafeState {
        rc_check_enabled: true,
        failsafe_throttle: 1,
        has_had_rc_receiver: false,
        has_had_rc_override: true,
        throttle_radio_in: 1500,
        failsafe_throttle_value: 975,
    };
    assert_eq!(rc_throttle_failsafe_check(&base), None);

    // Neither source: refused.
    assert_eq!(
        rc_throttle_failsafe_check(&RcFailsafeState {
            has_had_rc_override: false,
            ..base
        }),
        Some(ArmRefusal::RcNotFound)
    );
}

/// The throttle test is strictly below the threshold.
#[test]
fn the_throttle_threshold_is_exclusive() {
    let at_threshold = RcFailsafeState {
        rc_check_enabled: true,
        failsafe_throttle: 1,
        has_had_rc_receiver: true,
        has_had_rc_override: false,
        throttle_radio_in: 975,
        failsafe_throttle_value: 975,
    };
    assert_eq!(
        rc_throttle_failsafe_check(&at_threshold),
        None,
        "exactly at the threshold is not below it"
    );
    assert_eq!(
        rc_throttle_failsafe_check(&RcFailsafeState {
            throttle_radio_in: 974,
            ..at_threshold
        }),
        Some(ArmRefusal::ThrottleBelowFailsafe)
    );
}

#[test]
fn the_gcs_failsafe_check_matches_upstream() {
    let rows = rows("gcs");
    assert_eq!(rows.len(), 2, "both states should be recorded");

    for r in &rows {
        assert_eq!(r.len(), 3, "malformed row");
        let failsafe = b(&r[0]);
        check(
            gcs_failsafe_check(failsafe),
            b(&r[1]),
            r[2].trim(),
            &format!("gcs failsafe {failsafe}"),
        );
    }
}

#[test]
fn the_alt_check_matches_upstream() {
    let rows = rows("alt");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut manual = 0_usize;
    let mut automatic = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 4, "malformed row");
        let mode: i32 = r[0].trim().parse().expect("mode");
        let manual_throttle = b(&r[1]);
        let passed = b(&r[2]);
        let reason = r[3].trim();

        // The EKF has no altitude estimate in the harness, which is what
        // makes the manual-throttle exemption visible: the modes that need an
        // estimate are the ones refused.
        let got = alt_check(manual_throttle, false);
        check(got, passed, reason, &format!("mode {mode}"));

        if manual_throttle {
            manual += 1;
            assert!(passed, "a manual-throttle mode should not need an estimate");
        } else {
            automatic += 1;
        }
    }

    assert!(
        manual > 0 && automatic > 0,
        "the exemption never varies: {manual} manual, {automatic} automatic"
    );
    println!(
        "{} alt rows: {manual} manual-throttle, {automatic} not",
        rows.len()
    );
}

/// A manual-throttle mode needs no altitude estimate.
///
/// Nothing is trying to hold a height, so requiring one would stop a pilot
/// taking off in Stabilize on a day the barometer is unhappy — which is
/// exactly when they most want the mode that does not depend on it.
#[test]
fn a_manual_throttle_mode_needs_no_altitude_estimate() {
    assert_eq!(alt_check(true, false), None);
    assert_eq!(alt_check(true, true), None);
    assert_eq!(alt_check(false, true), None);
    assert_eq!(alt_check(false, false), Some(ArmRefusal::NeedAltEstimate));
}

/// Both interlock problems are reported, not just the first.
///
/// Upstream records each failure and carries on rather than returning early,
/// so a vehicle with both is told about both. A port returning the first
/// refusal would leave the operator fixing one problem and discovering the
/// other on the next attempt.
#[test]
fn both_interlock_problems_are_reported() {
    let both = InterlockSwitches {
        motor_interlock_assigned: true,
        motor_estop_assigned: true,
        arm_emergency_stop_assigned: false,
        using_interlock: true,
        motor_interlock_switch: true,
    };
    assert_eq!(
        interlock_checks(&both),
        [
            Some(ArmRefusal::InterlockEstopConflict),
            Some(ArmRefusal::MotorInterlockEnabled)
        ]
    );

    // Either emergency-stop assignment conflicts with the interlock.
    let via_arm_estop = InterlockSwitches {
        motor_estop_assigned: false,
        arm_emergency_stop_assigned: true,
        ..both
    };
    assert_eq!(
        interlock_checks(&via_arm_estop)[0],
        Some(ArmRefusal::InterlockEstopConflict)
    );

    // An interlock with no emergency stop is fine.
    let interlock_only = InterlockSwitches {
        motor_estop_assigned: false,
        arm_emergency_stop_assigned: false,
        ..both
    };
    assert_eq!(interlock_checks(&interlock_only)[0], None);

    // An emergency stop with no interlock is fine too — the conflict needs
    // both.
    let estop_only = InterlockSwitches {
        motor_interlock_assigned: false,
        motor_estop_assigned: true,
        ..both
    };
    assert_eq!(interlock_checks(&estop_only)[0], None);

    // The switch must be disabled to arm, and only matters when in use.
    assert_eq!(
        interlock_checks(&InterlockSwitches {
            motor_interlock_switch: false,
            ..interlock_only
        })[1],
        None
    );
    assert_eq!(
        interlock_checks(&InterlockSwitches {
            using_interlock: false,
            ..interlock_only
        })[1],
        None
    );
}

/// An already-armed vehicle passes without running a check.
#[test]
fn an_armed_vehicle_skips_the_checks() {
    assert!(!pre_arm_checks_apply(true));
    assert!(pre_arm_checks_apply(false));
}

/// Nothing is trusted before the system is up.
#[test]
fn an_uninitialised_system_refuses_before_anything_else() {
    assert_eq!(
        system_initialised_check(false),
        Some(ArmRefusal::SystemNotInitialised)
    );
    assert_eq!(system_initialised_check(true), None);
}

/// The battery check is gated by the voltage bit.
#[test]
fn the_battery_check_is_gated_by_arming_check() {
    assert_eq!(
        battery_failsafe_check(true, true),
        Some(ArmRefusal::BatteryFailsafe)
    );
    assert_eq!(battery_failsafe_check(true, false), None);
    assert_eq!(
        battery_failsafe_check(false, true),
        None,
        "with the voltage check disabled a battery failsafe does not block arming"
    );
}
