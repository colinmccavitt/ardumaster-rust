//! The mandatory position pre-arm checks, against the real firmware.
//!
//! The EKF's answers are injected — without a running estimator the first
//! rung refuses every time and the other five are unreachable, which an
//! earlier version of this recording demonstrated by covering one branch of
//! six while looking complete.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::arming::{
    fence_requires_position, gps_hdop_check, mandatory_position_checks, mode_requires_gps,
    mode_requires_position, EkfVariance, MandatoryPositionState, PositionRefusal,
};

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

/// Upstream's message for a refusal, as `check_failed` formats it.
fn message(refusal: PositionRefusal) -> String {
    match refusal {
        PositionRefusal::Ahrs => "AHRS: EKF3 not started".to_owned(),
        PositionRefusal::NeedPositionEstimate => "Need Position Estimate".to_owned(),
        PositionRefusal::FenceNeedsPositionEstimate => {
            "Fence enabled, need position estimate".to_owned()
        }
        PositionRefusal::GpsGlitching => "GPS glitching".to_owned(),
        PositionRefusal::EkfVariance(v) => format!("EKF {} variance", v.name()),
        PositionRefusal::HighGpsHdop => "High GPS HDOP".to_owned(),
    }
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_position_checks.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("idx,"))
        .map(|l| l.splitn(12, ',').map(str::to_owned).collect())
        .collect()
}

#[test]
fn the_mandatory_position_checks_match_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut seen = std::collections::BTreeSet::new();
    let mut passed_count = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 12, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let thresh_tenths: i32 = r[7].trim().parse().expect("thresh");
        #[allow(
            clippy::cast_precision_loss,
            reason = "the harness recorded the threshold in tenths to keep the \
fixture integral; the values are small and exact"
        )]
        let fs_ekf_thresh = thresh_tenths as f32 / 10.0;

        let var_index: u8 = r[8].trim().parse().expect("variance index");
        let over = 0.9_f32;

        let state = MandatoryPositionState {
            ahrs_pre_arm_ok: b(&r[5]),
            mode_requires_position: b(&r[2]),
            require_location: b(&r[3]),
            // The fence is not swept; see the test below.
            fence_requires_position: false,
            position_ok: b(&r[4]),
            filter_status_available: true,
            gps_glitching: b(&r[6]),
            fs_ekf_thresh,
            compass_variance: if var_index == 1 { over } else { 0.0 },
            position_variance: if var_index == 2 { over } else { 0.0 },
            velocity_variance: if var_index == 3 { over } else { 0.0 },
            height_variance: if var_index == 4 { over } else { 0.0 },
        };

        let passed = b(&r[9]);
        let first = r[11].trim();

        let got = mandatory_position_checks(&state);
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
                assert_eq!(
                    message(refusal),
                    first,
                    "row {idx}: refused with {refusal:?}, state {state:?}"
                );
                seen.insert(format!("{refusal:?}"));
            }
        }
    }

    assert!(passed_count > 0, "no row passed");
    assert!(
        seen.len() >= 6,
        "only {} distinct refusals reached: {seen:?}",
        seen.len()
    );
    println!(
        "{} rows, {passed_count} passed, refusals {seen:?}",
        rows.len()
    );
}

/// A fence needing a position gets its own message, and it is not the general
/// one.
///
/// Upstream's comment says the second message exists "to clarify to user why
/// they need GPS in non-GPS flight mode". A pilot in Stabilize told they need
/// a position estimate would reasonably think the aircraft was broken, when
/// in fact they enabled a fence. Not reachable in the recording — the fence
/// is not swept — so pinned here and labelled.
#[test]
fn a_fence_gets_its_own_message_for_the_same_missing_position() {
    let base = MandatoryPositionState {
        ahrs_pre_arm_ok: true,
        mode_requires_position: false,
        require_location: false,
        fence_requires_position: true,
        position_ok: false,
        filter_status_available: true,
        gps_glitching: false,
        fs_ekf_thresh: 0.0,
        compass_variance: 0.0,
        position_variance: 0.0,
        velocity_variance: 0.0,
        height_variance: 0.0,
    };
    assert_eq!(
        mandatory_position_checks(&base),
        Some(PositionRefusal::FenceNeedsPositionEstimate)
    );

    // The mode needing a position takes precedence and gives the other
    // message, even with the fence also enabled.
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            mode_requires_position: true,
            ..base
        }),
        Some(PositionRefusal::NeedPositionEstimate)
    );

    // With a position, neither fires.
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            position_ok: true,
            ..base
        }),
        None
    );

    // Only a circle or polygon fence needs a position; an altitude fence can
    // be enforced from the barometer alone.
    assert!(!fence_requires_position(false, false));
    assert!(fence_requires_position(true, false));
    assert!(fence_requires_position(false, true));
}

/// Nothing below the position test runs when no position is needed.
///
/// The `else` returns immediately, so a mode that does not need a position is
/// never refused for a GPS glitch or an EKF variance it will not use.
#[test]
fn a_mode_needing_no_position_skips_the_estimator_checks() {
    let hostile = MandatoryPositionState {
        ahrs_pre_arm_ok: true,
        mode_requires_position: false,
        require_location: false,
        fence_requires_position: false,
        position_ok: false,
        filter_status_available: true,
        gps_glitching: true,
        fs_ekf_thresh: 0.5,
        compass_variance: 99.0,
        position_variance: 99.0,
        velocity_variance: 99.0,
        height_variance: 99.0,
    };
    assert_eq!(mandatory_position_checks(&hostile), None);

    // Requiring a location brings all of it back.
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            require_location: true,
            position_ok: true,
            ..hostile
        }),
        Some(PositionRefusal::GpsGlitching)
    );
}

/// The variances are reported in upstream's order, and the test is `>=`.
///
/// Which comes first decides what a pilot is told when several are bad. And
/// `FS_EKF_THRESH` is the *failsafe* threshold, so a vehicle sitting exactly
/// on it is one the failsafe would fire for — upstream continues only while
/// strictly below.
#[test]
fn the_variance_checks_report_in_order_and_include_the_threshold() {
    let base = MandatoryPositionState {
        ahrs_pre_arm_ok: true,
        mode_requires_position: true,
        require_location: false,
        fence_requires_position: false,
        position_ok: true,
        filter_status_available: true,
        gps_glitching: false,
        fs_ekf_thresh: 0.8,
        compass_variance: 0.0,
        position_variance: 0.0,
        velocity_variance: 0.0,
        height_variance: 0.0,
    };

    // Exactly at the threshold refuses.
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            compass_variance: 0.8,
            ..base
        }),
        Some(PositionRefusal::EkfVariance(EkfVariance::Compass))
    );
    // A hair below passes.
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            compass_variance: 0.8 - 1e-6,
            ..base
        }),
        None
    );

    // All four bad: the compass is named, because it is checked first.
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            compass_variance: 9.0,
            position_variance: 9.0,
            velocity_variance: 9.0,
            height_variance: 9.0,
            ..base
        }),
        Some(PositionRefusal::EkfVariance(EkfVariance::Compass))
    );
    // With the compass clear, position is next, and so on down.
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            position_variance: 9.0,
            velocity_variance: 9.0,
            height_variance: 9.0,
            ..base
        }),
        Some(PositionRefusal::EkfVariance(EkfVariance::Position))
    );
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            velocity_variance: 9.0,
            height_variance: 9.0,
            ..base
        }),
        Some(PositionRefusal::EkfVariance(EkfVariance::Velocity))
    );
    assert_eq!(
        mandatory_position_checks(&MandatoryPositionState {
            height_variance: 9.0,
            ..base
        }),
        Some(PositionRefusal::EkfVariance(EkfVariance::Height))
    );

    // A threshold of zero or below disables the whole group.
    for thresh in [0.0_f32, -1.0] {
        assert_eq!(
            mandatory_position_checks(&MandatoryPositionState {
                fs_ekf_thresh: thresh,
                compass_variance: 99.0,
                ..base
            }),
            None
        );
    }
}

/// Needing a position is not the same as needing GPS.
///
/// A vehicle holding position from optical flow or motion capture needs a
/// position and no GPS at all, and upstream skips the GPS checks for it —
/// otherwise indoor flight could not be armed.
#[test]
fn a_non_gps_position_source_skips_the_gps_checks() {
    assert!(mode_requires_position(true, false, false));
    assert!(mode_requires_position(false, true, false));
    assert!(
        mode_requires_position(false, false, true),
        "super-simple mode rotates sticks by the bearing from home, so it \
         needs to know where home is"
    );
    assert!(!mode_requires_position(false, false, false));

    // But GPS specifically is only needed when the estimator is using it.
    assert!(mode_requires_gps(true, true));
    assert!(!mode_requires_gps(false, true), "flying without GPS");
    assert!(!mode_requires_gps(true, false));
}

/// HDOP is reported separately from a missing fix, and only when a GPS is
/// fitted.
#[test]
fn the_hdop_check_needs_a_gps_to_be_meaningful() {
    assert_eq!(gps_hdop_check(0, 9999, 140), None, "no GPS fitted");
    assert_eq!(
        gps_hdop_check(1, 141, 140),
        Some(PositionRefusal::HighGpsHdop)
    );
    assert_eq!(gps_hdop_check(1, 140, 140), None, "exactly good is good");
    assert_eq!(gps_hdop_check(2, 100, 140), None);
}
