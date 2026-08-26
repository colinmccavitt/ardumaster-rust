//! The final arm checks, against the real firmware.
//!
//! The property worth recording is the ordering: three checks sit above the
//! skip-all shortcut, so disabling `ARMING_CHECK` does not disable them. If
//! that shortcut ever moved to the top of the function, nothing about a
//! healthy vehicle's return value would change — which is why the sweep sets
//! skip-all against each of those three failing.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::arming::{
    arm_checks, combine_mandatory_checks, lean_angle_rad, oa_check_message, proximity_check,
    ArmCheckRefusal, ArmCheckState, ArmingMethod, PROXIMITY_TOLERANCE_M,
};

/// Upstream's `AP_Arming::Method` numbers for the three the sweep uses.
const METHOD_RUDDER: i32 = 0;
const METHOD_MAVLINK: i32 = 1;

/// Mode numbers, for the message that names the mode.
fn mode_name(number: i32) -> &'static str {
    match number {
        0 => "Stabilize",
        5 => "Loiter",
        9 => "Land",
        other => panic!("unexpected recorded mode {other}"),
    }
}

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_arm_checks.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("idx,"))
        .map(|l| l.splitn(12, ',').map(str::to_owned).collect())
        .collect()
}

#[test]
fn the_arm_checks_match_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut seen = std::collections::BTreeSet::new();
    let mut passed_count = 0_usize;
    let mut refused_despite_skip = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 12, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let mode: i32 = r[1].trim().parse().expect("mode");
        let method_num: i32 = r[2].trim().parse().expect("method");
        let skip_all = b(&r[4]);
        let throttle_in: i32 = r[8].trim().parse().expect("throttle");

        let method = match method_num {
            METHOD_RUDDER => ArmingMethod::Pilot,
            METHOD_MAVLINK => ArmingMethod::GroundStation,
            _ => ArmingMethod::Scripting,
        };

        let state = ArmCheckState {
            ahrs_healthy: b(&r[3]),
            // The harness reports a non-compass yaw source throughout, so the
            // compass rung is skipped; it has its own test.
            using_noncompass_for_yaw: true,
            compass_healthy: true,
            mode_allows_arming: b(&r[9]),
            skip_all_checks: skip_all,
            ins_check_enabled: b(&r[5]),
            // The vehicle is level in the harness.
            lean_angle_rad: 0.0,
            lean_angle_max_rad: 0.5,
            parameter_check_enabled: b(&r[6]),
            adsb_failsafe: b(&r[7]),
            rc_check_enabled: b(&r[6]),
            method,
            // None of the three swept modes permits it.
            mode_allows_gcs_arming_with_throttle_high: false,
            pilot_climb_rate_positive: false,
            manual_throttle_mode: mode == 0,
            throttle_control_in_positive: throttle_in > 0,
            // The harness has no safety switch fitted, so the firmware reads
            // it as disarmed; the recording shows that as the last rung.
            safety_switch_disarmed: true,
        };

        let passed = b(&r[10]);
        let first = r[11].trim();

        let got = arm_checks(&state);
        match got {
            None => {
                assert!(
                    passed,
                    "row {idx}: the port passes, upstream said {first:?}"
                );
                passed_count += 1;
            }
            Some(refusal) => {
                assert!(!passed, "row {idx}: the port refuses, upstream passed");
                let expected = match refusal {
                    ArmCheckRefusal::AhrsNotHealthy => "AHRS not healthy".to_owned(),
                    ArmCheckRefusal::CompassNotHealthy => "Compass not healthy".to_owned(),
                    ArmCheckRefusal::ModeNotArmable => {
                        format!("{} mode not armable", mode_name(mode))
                    }
                    ArmCheckRefusal::Leaning => "Leaning".to_owned(),
                    ArmCheckRefusal::AdsbThreatDetected => "ADSB threat detected".to_owned(),
                    ArmCheckRefusal::ThrottleTooHigh => "Throttle too high".to_owned(),
                    ArmCheckRefusal::SafetySwitch => "Safety Switch".to_owned(),
                };
                assert_eq!(expected, first, "row {idx}: refused with {refusal:?}");
                seen.insert(format!("{refusal:?}"));

                // The property this recording exists for.
                if skip_all
                    && matches!(
                        refusal,
                        ArmCheckRefusal::AhrsNotHealthy
                            | ArmCheckRefusal::CompassNotHealthy
                            | ArmCheckRefusal::ModeNotArmable
                    )
                {
                    refused_despite_skip += 1;
                }
            }
        }
    }

    assert!(
        refused_despite_skip > 0,
        "no row refused while every check was disabled, so the ordering that \
         keeps three checks above the shortcut is untested"
    );
    assert!(
        seen.len() >= 4,
        "only {} refusals reached: {seen:?}",
        seen.len()
    );
    println!(
        "{} rows, {passed_count} passed, {refused_despite_skip} refused despite \
         ARMING_CHECK being off, refusals {seen:?}",
        rows.len()
    );
}

/// Disabling every check does not disable the three that matter.
///
/// The AHRS health, compass health and mode-allows-arming tests sit above the
/// skip-all shortcut. An operator who turns off `ARMING_CHECK` to get a
/// vehicle airborne still cannot arm one whose estimator is unhealthy or
/// whose mode refuses. That is the difference between a parameter that skips
/// the advisory checks and one that disables the safety interlocks, and it
/// would vanish silently if the shortcut moved to the top.
#[test]
fn disabling_every_check_leaves_three_standing() {
    let healthy = ArmCheckState {
        ahrs_healthy: true,
        using_noncompass_for_yaw: false,
        compass_healthy: true,
        mode_allows_arming: true,
        skip_all_checks: true,
        ins_check_enabled: false,
        lean_angle_rad: 0.0,
        lean_angle_max_rad: 0.5,
        parameter_check_enabled: false,
        adsb_failsafe: true,
        rc_check_enabled: false,
        method: ArmingMethod::Pilot,
        mode_allows_gcs_arming_with_throttle_high: false,
        pilot_climb_rate_positive: true,
        manual_throttle_mode: true,
        throttle_control_in_positive: true,
        safety_switch_disarmed: true,
    };
    // Everything below the shortcut is hostile and none of it fires.
    assert_eq!(arm_checks(&healthy), None);

    // Each of the three above it still refuses.
    assert_eq!(
        arm_checks(&ArmCheckState {
            ahrs_healthy: false,
            ..healthy
        }),
        Some(ArmCheckRefusal::AhrsNotHealthy)
    );
    assert_eq!(
        arm_checks(&ArmCheckState {
            compass_healthy: false,
            ..healthy
        }),
        Some(ArmCheckRefusal::CompassNotHealthy)
    );
    assert_eq!(
        arm_checks(&ArmCheckState {
            mode_allows_arming: false,
            ..healthy
        }),
        Some(ArmCheckRefusal::ModeNotArmable)
    );
}

/// A vehicle taking heading from something other than the compass does not
/// need a healthy one.
///
/// Requiring it would ground a working aircraft for a sensor it is not using.
#[test]
fn a_non_compass_yaw_source_skips_the_compass() {
    let base = ArmCheckState {
        ahrs_healthy: true,
        using_noncompass_for_yaw: true,
        compass_healthy: false,
        mode_allows_arming: true,
        skip_all_checks: true,
        ins_check_enabled: false,
        lean_angle_rad: 0.0,
        lean_angle_max_rad: 0.5,
        parameter_check_enabled: false,
        adsb_failsafe: false,
        rc_check_enabled: false,
        method: ArmingMethod::Pilot,
        mode_allows_gcs_arming_with_throttle_high: false,
        pilot_climb_rate_positive: false,
        manual_throttle_mode: false,
        throttle_control_in_positive: false,
        safety_switch_disarmed: false,
    };
    assert_eq!(arm_checks(&base), None);
    assert_eq!(
        arm_checks(&ArmCheckState {
            using_noncompass_for_yaw: false,
            ..base
        }),
        Some(ArmCheckRefusal::CompassNotHealthy)
    );
}

/// The throttle exemption is for software, not for pilots.
///
/// A ground station or script arming a mode that permits it skips the
/// throttle test: a raised stick is a stale physical control when the
/// decision to arm came from software. A pilot arming with a raised stick is
/// refused, because for them the stick *is* the instruction.
#[test]
fn the_throttle_exemption_is_for_software_not_pilots() {
    let raised = ArmCheckState {
        ahrs_healthy: true,
        using_noncompass_for_yaw: true,
        compass_healthy: true,
        mode_allows_arming: true,
        skip_all_checks: false,
        ins_check_enabled: false,
        lean_angle_rad: 0.0,
        lean_angle_max_rad: 0.5,
        parameter_check_enabled: false,
        adsb_failsafe: false,
        rc_check_enabled: true,
        method: ArmingMethod::Pilot,
        mode_allows_gcs_arming_with_throttle_high: true,
        pilot_climb_rate_positive: true,
        manual_throttle_mode: false,
        throttle_control_in_positive: false,
        safety_switch_disarmed: false,
    };

    assert_eq!(
        arm_checks(&raised),
        Some(ArmCheckRefusal::ThrottleTooHigh),
        "a pilot is not exempt however the mode is configured"
    );

    for method in [ArmingMethod::GroundStation, ArmingMethod::Scripting] {
        assert_eq!(
            arm_checks(&ArmCheckState { method, ..raised }),
            None,
            "{method:?} arming a permitting mode skips the throttle test"
        );
        // But only if the mode permits it.
        assert_eq!(
            arm_checks(&ArmCheckState {
                method,
                mode_allows_gcs_arming_with_throttle_high: false,
                ..raised
            }),
            Some(ArmCheckRefusal::ThrottleTooHigh)
        );
    }

    // A manual-throttle mode is refused for the stick position as well as the
    // climb rate — two separate tests, either of which fires.
    assert_eq!(
        arm_checks(&ArmCheckState {
            pilot_climb_rate_positive: false,
            manual_throttle_mode: true,
            throttle_control_in_positive: true,
            ..raised
        }),
        Some(ArmCheckRefusal::ThrottleTooHigh)
    );
}

/// The lean angle is the tilt from vertical, not the larger of roll and pitch.
///
/// A vehicle leaning thirty degrees in both axes is tilted further than one
/// leaning thirty in either alone, and this is the angle between its thrust
/// axis and vertical.
#[test]
fn the_lean_angle_is_the_tilt_from_vertical() {
    // Level.
    assert!(lean_angle_rad(1.0, 1.0).abs() < 1e-6);

    // Thirty degrees of roll alone.
    let thirty = 30.0_f32.to_radians();
    let roll_only = lean_angle_rad(thirty.cos(), 1.0);
    assert!((roll_only - thirty).abs() < 1e-5);

    // Thirty in both is more than thirty.
    let both = lean_angle_rad(thirty.cos(), thirty.cos());
    assert!(
        both > thirty,
        "combined tilt {both} should exceed a single axis {thirty}"
    );
    assert!(both < 2.0 * thirty, "and it is not simply additive");
}

/// The mandatory checks combine bitwise so all of them run.
///
/// These are what run when `ARMING_SKIPCHK` skips everything else or arming
/// is forced — exactly when running all of them matters most.
#[test]
fn the_mandatory_checks_all_run() {
    assert!(combine_mandatory_checks(true, true, true));
    assert!(!combine_mandatory_checks(false, true, true));
    assert!(!combine_mandatory_checks(true, false, true));
    assert!(!combine_mandatory_checks(true, true, false));
    assert!(!combine_mandatory_checks(false, false, false));
}

/// A silent object-avoidance refusal is given a message.
#[test]
fn a_silent_avoidance_refusal_gets_a_generic_message() {
    assert_eq!(oa_check_message(true, ""), None);
    assert_eq!(oa_check_message(true, "ignored"), None);
    assert_eq!(oa_check_message(false, ""), Some("Check Object Avoidance"));
    assert_eq!(oa_check_message(false, "no path"), Some("no path"));
}

/// Proximity only refuses when the vehicle is actually avoiding obstacles.
///
/// A vehicle not using avoidance has no reason to refuse arming next to a
/// wall, and a sensor reading close to one is normal for a machine sitting in
/// a hangar.
#[test]
fn proximity_only_matters_when_avoidance_is_on() {
    assert!(proximity_check(true, false, Some(0.1)), "avoidance off");
    assert!(proximity_check(false, true, Some(0.1)), "check disabled");
    assert!(proximity_check(true, true, None), "no object seen");

    assert!(!proximity_check(true, true, Some(PROXIMITY_TOLERANCE_M)));
    assert!(
        !proximity_check(true, true, Some(PROXIMITY_TOLERANCE_M - 0.01)),
        "closer than the tolerance"
    );
    assert!(proximity_check(
        true,
        true,
        Some(PROXIMITY_TOLERANCE_M + 0.01)
    ));
}

/// A vehicle leaning past its limit is refused, and exactly at the limit is
/// not.
///
/// Nothing else exercises this rung: the recorded vehicle sits level, so the
/// comparison was never anything but false and both `>=` and `==` survived
/// mutation in its place. A machine parked on a slope, or one whose
/// `ANGLE_MAX` has been lowered under it, sits at a real lean angle, and
/// whether it arms is a decision rather than an accident.
#[test]
fn a_leaning_vehicle_is_refused_but_not_at_exactly_the_limit() {
    let level = ArmCheckState {
        ahrs_healthy: true,
        using_noncompass_for_yaw: true,
        compass_healthy: true,
        mode_allows_arming: true,
        skip_all_checks: false,
        ins_check_enabled: true,
        lean_angle_rad: 0.0,
        lean_angle_max_rad: 0.5,
        parameter_check_enabled: false,
        adsb_failsafe: false,
        rc_check_enabled: false,
        method: ArmingMethod::Pilot,
        mode_allows_gcs_arming_with_throttle_high: false,
        pilot_climb_rate_positive: false,
        manual_throttle_mode: false,
        throttle_control_in_positive: false,
        safety_switch_disarmed: false,
    };
    assert_eq!(arm_checks(&level), None);

    // Exactly at the limit is allowed: upstream compares strictly greater.
    assert_eq!(
        arm_checks(&ArmCheckState {
            lean_angle_rad: 0.5,
            ..level
        }),
        None,
        "a vehicle exactly at its maximum lean should still arm"
    );

    // A hair past it is refused.
    assert_eq!(
        arm_checks(&ArmCheckState {
            lean_angle_rad: 0.5 + 1e-6,
            ..level
        }),
        Some(ArmCheckRefusal::Leaning)
    );
    assert_eq!(
        arm_checks(&ArmCheckState {
            lean_angle_rad: 1.2,
            ..level
        }),
        Some(ArmCheckRefusal::Leaning)
    );

    // The rung is behind the INS bit, and behind the skip-all shortcut.
    assert_eq!(
        arm_checks(&ArmCheckState {
            lean_angle_rad: 1.2,
            ins_check_enabled: false,
            ..level
        }),
        None
    );
    assert_eq!(
        arm_checks(&ArmCheckState {
            lean_angle_rad: 1.2,
            skip_all_checks: true,
            ..level
        }),
        None
    );

    // And a real attitude reaches it: half a radian is about 29 degrees, so a
    // vehicle leaning 30 in both axes is past a 0.5 rad limit.
    let thirty = 30.0_f32.to_radians();
    assert_eq!(
        arm_checks(&ArmCheckState {
            lean_angle_rad: lean_angle_rad(thirty.cos(), thirty.cos()),
            ..level
        }),
        Some(ArmCheckRefusal::Leaning)
    );
}
