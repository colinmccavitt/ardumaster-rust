//! The RC calibration pre-arm check, against the real firmware.
//!
//! What is worth pinning here is not the returned bool but *how many*
//! messages arrive. A channel that was never calibrated is wrong at both
//! ends, and upstream reports both; a port returning at the first fault
//! gives the same bool and half the messages, and the operator fixes one
//! problem and discovers the other on the next attempt.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::arming::{
    altitude_disparity_check, combine_rc_calibration, ekf_attitude_check, rc_calibration_passes,
    rc_channel_calibration_faults, RcCalibrationFault, RcChannelCalibration,
    RC_CALIBRATION_CHANNEL_NAMES, RC_CALIB_MAX_LIMIT_PWM, RC_CALIB_MIN_LIMIT_PWM,
};

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

fn n(s: &str) -> u16 {
    s.trim().parse().expect("pwm")
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_rc_calibration.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("idx,"))
        .map(|l| l.splitn(13, ',').map(str::to_owned).collect())
        .collect()
}

/// Upstream's message for one channel's fault.
fn message(channel: &str, fault: RcCalibrationFault) -> String {
    match fault {
        RcCalibrationFault::MinTooHigh => format!("{channel} radio min too high"),
        RcCalibrationFault::MaxTooLow => format!("{channel} radio max too low"),
    }
}

#[test]
fn the_rc_calibration_check_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut multi_fault_rows = 0_usize;
    let mut most_faults = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 13, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let enabled = b(&r[1]);

        let channels = [
            RcChannelCalibration {
                radio_min: n(&r[2]),
                radio_max: n(&r[3]),
            },
            RcChannelCalibration {
                radio_min: n(&r[4]),
                radio_max: n(&r[5]),
            },
            RcChannelCalibration {
                radio_min: n(&r[6]),
                radio_max: n(&r[7]),
            },
            RcChannelCalibration {
                radio_min: n(&r[8]),
                radio_max: n(&r[9]),
            },
        ];

        let passed = b(&r[10]);
        let calls: usize = r[11].trim().parse().expect("calls");
        let first = r[12].trim();

        assert_eq!(
            rc_calibration_passes(enabled, &channels),
            passed,
            "row {idx}: pass/fail"
        );

        // Every fault upstream would report, in channel order then min-then-max.
        let mut faults: Vec<String> = Vec::new();
        if enabled {
            for (channel, name) in channels.iter().zip(RC_CALIBRATION_CHANNEL_NAMES) {
                for fault in rc_channel_calibration_faults(channel).into_iter().flatten() {
                    faults.push(message(name, fault));
                }
            }
        }

        assert_eq!(
            faults.len(),
            calls,
            "row {idx}: the port would report {} faults, upstream reported \
             {calls} (first {first:?}) — a check that returned early would \
             match the bool and not the count",
            faults.len()
        );
        if let Some(expected_first) = faults.first() {
            assert_eq!(expected_first, first, "row {idx}: first message");
        }

        if calls > 1 {
            multi_fault_rows += 1;
        }
        most_faults = most_faults.max(calls);
    }

    assert!(
        multi_fault_rows > 0,
        "no row reported more than one fault, so the accumulate-rather-than-\
         return behaviour is untested"
    );
    assert!(
        most_faults >= 3,
        "the deepest row reported only {most_faults} faults"
    );
    println!(
        "{} rows, {multi_fault_rows} with several faults, deepest {most_faults}",
        rows.len()
    );
}

/// A channel can be wrong at both ends and is reported twice.
#[test]
fn an_uncalibrated_channel_is_wrong_at_both_ends() {
    let never_calibrated = RcChannelCalibration {
        radio_min: 1500,
        radio_max: 1500,
    };
    assert_eq!(
        rc_channel_calibration_faults(&never_calibrated),
        [
            Some(RcCalibrationFault::MinTooHigh),
            Some(RcCalibrationFault::MaxTooLow)
        ]
    );

    // The limits themselves pass: the tests are strictly outside.
    let at_limits = RcChannelCalibration {
        radio_min: RC_CALIB_MIN_LIMIT_PWM,
        radio_max: RC_CALIB_MAX_LIMIT_PWM,
    };
    assert_eq!(rc_channel_calibration_faults(&at_limits), [None, None]);

    // One microsecond outside each.
    assert_eq!(
        rc_channel_calibration_faults(&RcChannelCalibration {
            radio_min: RC_CALIB_MIN_LIMIT_PWM + 1,
            ..at_limits
        }),
        [Some(RcCalibrationFault::MinTooHigh), None]
    );
    assert_eq!(
        rc_channel_calibration_faults(&RcChannelCalibration {
            radio_max: RC_CALIB_MAX_LIMIT_PWM - 1,
            ..at_limits
        }),
        [None, Some(RcCalibrationFault::MaxTooLow)]
    );
}

/// The two halves are combined bitwise so both run.
///
/// Upstream writes `&` with a comment saying it "ensures all checks are run".
/// A logical `&&` would short-circuit and a vehicle failing the first half
/// would never run the second. The two spellings are one character apart and
/// return the same bool; the difference is entirely in which messages the
/// pilot gets.
#[test]
fn both_halves_of_the_rc_check_always_run() {
    assert!(combine_rc_calibration(true, true));
    assert!(!combine_rc_calibration(true, false));
    assert!(!combine_rc_calibration(false, true));
    assert!(!combine_rc_calibration(false, false));
}

/// The altitude disparity check, which the recording cannot reach.
///
/// It needs the EKF's prediction-status flags and a relative height, none of
/// which a harness can produce. Pinned here and labelled, so the fixture is
/// not read as covering it.
///
/// The interesting part is when it applies at all: only where the estimator
/// is using an absolute height reference. A vehicle flying terrain-relative
/// legitimately disagrees with the barometer as the baro drifts, and checking
/// regardless would refuse arming in exactly the configuration where the
/// disparity is expected.
#[test]
fn the_altitude_disparity_check_only_applies_to_absolute_references() {
    // Absolute reference, heights agree.
    assert!(altitude_disparity_check(true, false, true, 10.0, 10.5));
    // Absolute reference, heights disagree by more than a metre.
    assert!(!altitude_disparity_check(true, false, true, 10.0, 11.5));
    // Exactly a metre apart is allowed; the test is strictly greater.
    assert!(altitude_disparity_check(true, false, true, 10.0, 11.0));

    // Ground-relative: not checked however far apart they are.
    assert!(altitude_disparity_check(true, true, true, 0.0, 500.0));
    // Neither status: also not checked.
    assert!(altitude_disparity_check(true, false, false, 0.0, 500.0));

    // And the whole check is behind ARMING_CHECK's baro bit.
    assert!(altitude_disparity_check(false, false, true, 0.0, 500.0));
}

/// The EKF attitude check is behind the INS bit.
#[test]
fn the_ekf_attitude_check_is_gated() {
    assert!(ekf_attitude_check(true, true));
    assert!(!ekf_attitude_check(true, false));
    assert!(
        ekf_attitude_check(false, false),
        "with the INS check disabled a bad EKF attitude does not block arming"
    );
}
