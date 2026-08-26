//! ArduCopter's parameter pre-arm checks, against the real firmware.
//!
//! One rung of this ladder warns without blocking, so a single call can
//! produce two messages. The recording carries the first and the last, which
//! is what makes a row that warned and then refused distinguishable from one
//! that only refused — a distinction no return value could express, since
//! both come back `false`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::arming::{parameter_checks, ParameterRefusal, ParameterState, TerrainSource};

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

/// Upstream's message for each refusal, as `check_failed` formats it.
fn message(refusal: ParameterRefusal) -> &'static str {
    match refusal {
        ParameterRefusal::CheckFsThrValue => "Check FS_THR_VALUE",
        ParameterRefusal::FsGcsEnable2Removed => "FS_GCS_ENABLE=2 removed, see FS_OPTIONS",
        ParameterRefusal::CheckAcroBalance => "Check ACRO_BAL_ROLL/PITCH",
        ParameterRefusal::CheckPilotSpdUp => "Check PILOT_SPD_UP",
        ParameterRefusal::InvalidMulticopterFrameClass => "Invalid MultiCopter FRAME_CLASS",
        ParameterRefusal::RtlTerrainNoData => "RTL_ALT_TYPE is above-terrain but no terrain data",
        ParameterRefusal::RtlTerrainNoRangefinder => {
            "RTL_ALT_TYPE is above-terrain but no rangefinder"
        }
        ParameterRefusal::RtlAltAboveRangefinderMax => {
            "RTL_ALT_TYPE is above-terrain but RTL_ALT_M above RNGFND_MAX"
        }
        ParameterRefusal::AdsbThreatDetected => "ADSB threat detected",
        ParameterRefusal::BadPositionControllerParameter => "Bad parameter: PSC",
        ParameterRefusal::BadAttitudeControllerParameter => "Bad parameter: ATC",
    }
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_param_checks.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("idx,"))
        // Fifteen pieces: the last is the joined message pair, which
        // contains commas of its own and must not be split further.
        .map(|l| l.splitn(15, ',').map(str::to_owned).collect())
        .collect()
}

#[test]
fn the_parameter_checks_match_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut two_message_rows = 0_usize;
    let mut warned_only = 0_usize;
    let mut seen = std::collections::BTreeSet::new();

    for r in &rows {
        assert_eq!(r.len(), 15, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let state = ParameterState {
            parameter_check_enabled: b(&r[1]),
            failsafe_throttle: r[2].trim().parse().expect("fs_thr"),
            throttle_radio_min: r[3].trim().parse().expect("rc3_min"),
            failsafe_throttle_value: r[4].trim().parse().expect("fs_thr_value"),
            failsafe_gcs: r[5].trim().parse().expect("fs_gcs"),
            acro_balance_roll: f(&r[6]),
            acro_balance_pitch: f(&r[7]),
            angle_roll_p: f(&r[8]),
            angle_pitch_p: f(&r[9]),
            pilot_speed_up_ms: f(&r[10]),
            frame_class_is_heli: false,
            // The RTL terrain rung is kept out of the recording; it has its
            // own inputs and its own test below.
            rtl_alt_type_is_terrain: false,
            terrain_source: TerrainSource::Database,
            rangefinder_available: true,
            rtl_altitude_m: 0.0,
            rangefinder_max_distance_m: 0.0,
            adsb_failsafe: b(&r[11]),
            pos_control_ok: true,
            attitude_control_ok: true,
        };

        let passed = b(&r[12]);
        let calls: usize = r[13].trim().parse().expect("calls");
        let (first, last) = r[14]
            .trim()
            .split_once('|')
            .expect("the message field should hold both messages");

        let got = parameter_checks(&state);
        let raised: Vec<ParameterRefusal> = got.iter().flatten().copied().collect();

        assert_eq!(
            raised.len(),
            calls,
            "row {idx}: the port raised {} refusals, upstream reported {calls} \
             ({first:?} .. {last:?})",
            raised.len()
        );

        if let Some(first_refusal) = raised.first() {
            assert_eq!(
                message(*first_refusal),
                first,
                "row {idx}: first refusal {first_refusal:?}"
            );
        }
        if let Some(last_refusal) = raised.last() {
            assert_eq!(
                message(*last_refusal),
                last,
                "row {idx}: last refusal {last_refusal:?}"
            );
        }

        // Whether arming is blocked is the *blocking* refusals, not all of
        // them: the warning does not ground the aircraft.
        let blocked = raised.iter().any(|r| r.blocks_arming());
        assert_eq!(
            !blocked, passed,
            "row {idx}: port blocks={blocked}, upstream passed={passed}"
        );

        for refusal in &raised {
            seen.insert(format!("{refusal:?}"));
        }
        if calls == 2 {
            two_message_rows += 1;
        }
        if calls == 1 && !blocked {
            warned_only += 1;
        }
    }

    assert!(
        two_message_rows > 0,
        "no row produced two messages, so the non-blocking rung is untested"
    );
    assert!(
        warned_only > 0,
        "no row warned and still passed, which is the whole point of that rung"
    );
    assert!(
        seen.len() >= 4,
        "only {} refusals reached: {seen:?}",
        seen.len()
    );

    println!(
        "{} rows, {two_message_rows} with two messages, {warned_only} warned \
         but passed, refusals {seen:?}",
        rows.len()
    );
}

/// The removed `FS_GCS_ENABLE=2` warns without grounding the aircraft.
///
/// Upstream calls `check_failed` and does *not* return, so the operator is
/// told their parameter is obsolete and arming continues. Every other rung
/// returns false. It is easy to miss reading the function and easy to "tidy"
/// into a return, which would ground every vehicle carrying an old parameter
/// file.
#[test]
fn the_removed_gcs_parameter_warns_without_blocking() {
    let mut state = ParameterState {
        parameter_check_enabled: true,
        failsafe_throttle: 0,
        throttle_radio_min: 1000,
        failsafe_throttle_value: 975,
        failsafe_gcs: 2,
        acro_balance_roll: 1.0,
        acro_balance_pitch: 1.0,
        angle_roll_p: 4.5,
        angle_pitch_p: 4.5,
        pilot_speed_up_ms: 2.5,
        frame_class_is_heli: false,
        rtl_alt_type_is_terrain: false,
        terrain_source: TerrainSource::Database,
        rangefinder_available: true,
        rtl_altitude_m: 0.0,
        rangefinder_max_distance_m: 0.0,
        adsb_failsafe: false,
        pos_control_ok: true,
        attitude_control_ok: true,
    };

    let raised = parameter_checks(&state);
    assert_eq!(raised[0], Some(ParameterRefusal::FsGcsEnable2Removed));
    assert_eq!(raised[1], None, "nothing else should have fired");
    assert!(
        !raised.iter().flatten().any(|r| r.blocks_arming()),
        "the warning must not block arming"
    );

    // And a later rung still runs after it, producing two refusals.
    state.pilot_speed_up_ms = 0.0;
    let both = parameter_checks(&state);
    assert_eq!(both[0], Some(ParameterRefusal::FsGcsEnable2Removed));
    assert_eq!(both[1], Some(ParameterRefusal::CheckPilotSpdUp));
    assert!(both.iter().flatten().any(|r| r.blocks_arming()));
}

/// The throttle failsafe parameters are checked against each other.
///
/// `FS_THR_VALUE` must sit above the PPM encoder's loss-of-signal output of
/// 900, and below `RC3_MIN` by a margin. Between them: the threshold has to
/// be distinguishable from a dead link at one end and from a pilot holding
/// the stick fully down at the other. A value satisfying neither would either
/// never fire or fire constantly.
#[test]
fn the_throttle_failsafe_parameters_must_bracket_the_threshold() {
    let ok = ParameterState {
        parameter_check_enabled: true,
        failsafe_throttle: 1,
        throttle_radio_min: 1000,
        failsafe_throttle_value: 975,
        failsafe_gcs: 0,
        acro_balance_roll: 1.0,
        acro_balance_pitch: 1.0,
        angle_roll_p: 4.5,
        angle_pitch_p: 4.5,
        pilot_speed_up_ms: 2.5,
        frame_class_is_heli: false,
        rtl_alt_type_is_terrain: false,
        terrain_source: TerrainSource::Database,
        rangefinder_available: true,
        rtl_altitude_m: 0.0,
        rangefinder_max_distance_m: 0.0,
        adsb_failsafe: false,
        pos_control_ok: true,
        attitude_control_ok: true,
    };
    assert_eq!(parameter_checks(&ok)[0], None);

    // Below the encoder floor.
    assert_eq!(
        parameter_checks(&ParameterState {
            failsafe_throttle_value: 909,
            ..ok
        })[0],
        Some(ParameterRefusal::CheckFsThrValue)
    );

    // Too close to RC3_MIN: the margin is ten microseconds.
    assert_eq!(
        parameter_checks(&ParameterState {
            throttle_radio_min: 985,
            ..ok
        })[0],
        Some(ParameterRefusal::CheckFsThrValue),
        "RC3_MIN exactly at value+10 is not clear of it"
    );
    assert_eq!(
        parameter_checks(&ParameterState {
            throttle_radio_min: 986,
            ..ok
        })[0],
        None
    );

    // With the throttle failsafe off, neither is checked.
    assert_eq!(
        parameter_checks(&ParameterState {
            failsafe_throttle: 0,
            failsafe_throttle_value: 0,
            throttle_radio_min: 0,
            ..ok
        })[0],
        None
    );
}

/// The acro balance gains must be positive and no larger than the angle gains.
#[test]
fn the_acro_balance_gains_are_bounded_by_the_angle_gains() {
    let base = ParameterState {
        parameter_check_enabled: true,
        failsafe_throttle: 0,
        throttle_radio_min: 1000,
        failsafe_throttle_value: 975,
        failsafe_gcs: 0,
        acro_balance_roll: 1.0,
        acro_balance_pitch: 1.0,
        angle_roll_p: 4.5,
        angle_pitch_p: 4.5,
        pilot_speed_up_ms: 2.5,
        frame_class_is_heli: false,
        rtl_alt_type_is_terrain: false,
        terrain_source: TerrainSource::Database,
        rangefinder_available: true,
        rtl_altitude_m: 0.0,
        rangefinder_max_distance_m: 0.0,
        adsb_failsafe: false,
        pos_control_ok: true,
        attitude_control_ok: true,
    };
    assert_eq!(parameter_checks(&base)[0], None);

    for bad in [
        ParameterState {
            acro_balance_roll: -0.1,
            ..base
        },
        ParameterState {
            acro_balance_pitch: -0.1,
            ..base
        },
        ParameterState {
            acro_balance_roll: 4.6,
            ..base
        },
        ParameterState {
            acro_balance_pitch: 4.6,
            ..base
        },
    ] {
        assert_eq!(
            parameter_checks(&bad)[0],
            Some(ParameterRefusal::CheckAcroBalance),
            "{bad:?}"
        );
    }

    // Exactly equal to the angle gain is allowed; the test is strictly
    // greater.
    assert_eq!(
        parameter_checks(&ParameterState {
            acro_balance_roll: 4.5,
            ..base
        })[0],
        None
    );
}

/// The above-terrain RTL rungs, which the recording keeps out of the way.
///
/// They have their own inputs — a terrain source, a rangefinder, and two
/// altitudes — and mixing them into the main sweep would have multiplied it
/// without adding coverage of the rungs above. Pinned here instead, and
/// labelled so nobody reads the recording as covering them.
#[test]
fn the_above_terrain_rtl_rungs_need_a_usable_terrain_source() {
    let base = ParameterState {
        parameter_check_enabled: true,
        failsafe_throttle: 0,
        throttle_radio_min: 1000,
        failsafe_throttle_value: 975,
        failsafe_gcs: 0,
        acro_balance_roll: 1.0,
        acro_balance_pitch: 1.0,
        angle_roll_p: 4.5,
        angle_pitch_p: 4.5,
        pilot_speed_up_ms: 2.5,
        frame_class_is_heli: false,
        rtl_alt_type_is_terrain: true,
        terrain_source: TerrainSource::Unavailable,
        rangefinder_available: true,
        rtl_altitude_m: 10.0,
        rangefinder_max_distance_m: 20.0,
        adsb_failsafe: false,
        pos_control_ok: true,
        attitude_control_ok: true,
    };

    assert_eq!(
        parameter_checks(&base)[0],
        Some(ParameterRefusal::RtlTerrainNoData)
    );

    let rangefinder = ParameterState {
        terrain_source: TerrainSource::Rangefinder,
        ..base
    };
    assert_eq!(parameter_checks(&rangefinder)[0], None);

    assert_eq!(
        parameter_checks(&ParameterState {
            rangefinder_available: false,
            ..rangefinder
        })[0],
        Some(ParameterRefusal::RtlTerrainNoRangefinder)
    );
    assert_eq!(
        parameter_checks(&ParameterState {
            rtl_altitude_m: 21.0,
            ..rangefinder
        })[0],
        Some(ParameterRefusal::RtlAltAboveRangefinderMax)
    );

    // The database source is checked by the shared AP_Arming, not here.
    assert_eq!(
        parameter_checks(&ParameterState {
            terrain_source: TerrainSource::Database,
            ..base
        })[0],
        None
    );

    // And none of it applies unless RTL_ALT_TYPE is above-terrain.
    assert_eq!(
        parameter_checks(&ParameterState {
            rtl_alt_type_is_terrain: false,
            ..base
        })[0],
        None
    );
}

/// Disabling the parameter checks disables all of them.
#[test]
fn disabling_the_parameter_check_skips_every_rung() {
    let hostile = ParameterState {
        parameter_check_enabled: false,
        failsafe_throttle: 1,
        throttle_radio_min: 0,
        failsafe_throttle_value: 0,
        failsafe_gcs: 2,
        acro_balance_roll: -5.0,
        acro_balance_pitch: -5.0,
        angle_roll_p: 4.5,
        angle_pitch_p: 4.5,
        pilot_speed_up_ms: -1.0,
        frame_class_is_heli: true,
        rtl_alt_type_is_terrain: true,
        terrain_source: TerrainSource::Unavailable,
        rangefinder_available: false,
        rtl_altitude_m: 100.0,
        rangefinder_max_distance_m: 1.0,
        adsb_failsafe: true,
        pos_control_ok: false,
        attitude_control_ok: false,
    };
    assert_eq!(parameter_checks(&hostile), [None, None]);
}

/// A helper for the boundary tests: a configuration that passes everything.
fn passing_state() -> ParameterState {
    ParameterState {
        parameter_check_enabled: true,
        failsafe_throttle: 1,
        throttle_radio_min: 1000,
        failsafe_throttle_value: 975,
        failsafe_gcs: 0,
        acro_balance_roll: 1.0,
        acro_balance_pitch: 1.0,
        angle_roll_p: 4.5,
        angle_pitch_p: 4.5,
        pilot_speed_up_ms: 2.5,
        frame_class_is_heli: false,
        rtl_alt_type_is_terrain: false,
        terrain_source: TerrainSource::Database,
        rangefinder_available: true,
        rtl_altitude_m: 0.0,
        rangefinder_max_distance_m: 0.0,
        adsb_failsafe: false,
        pos_control_ok: true,
        attitude_control_ok: true,
    }
}

/// Every threshold, at the exact value an operator can type.
///
/// These are the comparisons the recorded sweep could not distinguish from
/// their `<=` or `>=` twins, because no row landed on equality. Each is a
/// number someone can put in a parameter file, and each decides whether the
/// aircraft arms.
#[test]
fn the_parameter_thresholds_are_exact_at_the_boundary() {
    // FS_THR_VALUE: 910 is the PPM encoder's loss-of-signal floor, and the
    // test is strictly below it, so 910 itself is allowed.
    let at_floor = ParameterState {
        failsafe_throttle_value: 910,
        throttle_radio_min: 1000,
        ..passing_state()
    };
    assert_eq!(parameter_checks(&at_floor)[0], None, "910 is allowed");
    assert_eq!(
        parameter_checks(&ParameterState {
            failsafe_throttle_value: 909,
            ..at_floor
        })[0],
        Some(ParameterRefusal::CheckFsThrValue),
        "909 is below the floor"
    );

    // ACRO_BAL_ROLL and PITCH: zero is allowed, negative is not. Both axes,
    // because testing one and assuming the other is how the pitch twin of
    // this comparison went untested in the first place.
    for zeroed in [
        ParameterState {
            acro_balance_roll: 0.0,
            ..passing_state()
        },
        ParameterState {
            acro_balance_pitch: 0.0,
            ..passing_state()
        },
    ] {
        assert_eq!(
            parameter_checks(&zeroed)[0],
            None,
            "zero balance is allowed"
        );
    }
    for negative in [
        ParameterState {
            acro_balance_roll: -f32::MIN_POSITIVE,
            ..passing_state()
        },
        ParameterState {
            acro_balance_pitch: -f32::MIN_POSITIVE,
            ..passing_state()
        },
    ] {
        assert_eq!(
            parameter_checks(&negative)[0],
            Some(ParameterRefusal::CheckAcroBalance),
            "the smallest negative balance is refused"
        );
    }

    // Equal to the angle gain is allowed on both axes; greater is not.
    for equal in [
        ParameterState {
            acro_balance_roll: 4.5,
            ..passing_state()
        },
        ParameterState {
            acro_balance_pitch: 4.5,
            ..passing_state()
        },
    ] {
        assert_eq!(
            parameter_checks(&equal)[0],
            None,
            "equal to the gain is allowed"
        );
    }
    for greater in [
        ParameterState {
            acro_balance_roll: 4.5 + 1e-6,
            ..passing_state()
        },
        ParameterState {
            acro_balance_pitch: 4.5 + 1e-6,
            ..passing_state()
        },
    ] {
        assert_eq!(
            parameter_checks(&greater)[0],
            Some(ParameterRefusal::CheckAcroBalance),
            "a hair above the gain is refused"
        );
    }

    // PILOT_SPD_UP: zero is refused, so the test is not strict.
    assert_eq!(
        parameter_checks(&ParameterState {
            pilot_speed_up_ms: 0.0,
            ..passing_state()
        })[0],
        Some(ParameterRefusal::CheckPilotSpdUp),
        "zero climb rate is not a usable setting"
    );
    assert_eq!(
        parameter_checks(&ParameterState {
            pilot_speed_up_ms: f32::MIN_POSITIVE,
            ..passing_state()
        })[0],
        None
    );

    // RTL_ALT_M exactly at the rangefinder's maximum is within range.
    let at_max = ParameterState {
        rtl_alt_type_is_terrain: true,
        terrain_source: TerrainSource::Rangefinder,
        rtl_altitude_m: 20.0,
        rangefinder_max_distance_m: 20.0,
        ..passing_state()
    };
    assert_eq!(
        parameter_checks(&at_max)[0],
        None,
        "at the limit is in range"
    );
    assert_eq!(
        parameter_checks(&ParameterState {
            rtl_altitude_m: 20.0 + 1e-4,
            ..at_max
        })[0],
        Some(ParameterRefusal::RtlAltAboveRangefinderMax)
    );
}
