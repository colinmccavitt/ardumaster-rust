//! Copter land-detector leftover, upstream `ArduCopter/land_detector.cpp`.

use ap_copter::land_detector::{
    land_trigger_iterations, large_angle_request, maybe_trigger_iterations, non_takeoff_throttle,
    update_land_detector, LandDetectorInputs, LandDetectorUpdate, WowState,
    LAND_AIRMODE_DETECTOR_TRIGGER_SEC, LAND_CHECK_ANGLE_ERROR_DEG, LAND_CHECK_LARGE_ANGLE_RAD,
    LAND_DETECTOR_ACCEL_MAX, LAND_DETECTOR_MAYBE_TRIGGER_SEC, LAND_DETECTOR_TRIGGER_SEC,
    LAND_DETECTOR_VEL_Z_MAX, LAND_RANGEFINDER_MIN_ALT_M,
};
use ap_motors::spool::SpoolState;

#[test]
fn constants_match_upstream_defines() {
    assert_eq!(LAND_DETECTOR_TRIGGER_SEC, 1.0);
    assert_eq!(LAND_AIRMODE_DETECTOR_TRIGGER_SEC, 3.0);
    assert_eq!(LAND_DETECTOR_MAYBE_TRIGGER_SEC, 0.2);
    assert_eq!(LAND_DETECTOR_ACCEL_MAX, 1.0);
    assert_eq!(LAND_DETECTOR_VEL_Z_MAX, 1.0);
    assert_eq!(LAND_RANGEFINDER_MIN_ALT_M, 2.0);
    assert_eq!(LAND_CHECK_ANGLE_ERROR_DEG, 30.0);
    assert_eq!(LAND_CHECK_LARGE_ANGLE_RAD, 15.0_f32.to_radians());
    assert_eq!(land_trigger_iterations(false, 400), 400.0);
    assert_eq!(land_trigger_iterations(true, 400), 1200.0);
    assert_eq!(maybe_trigger_iterations(400), 80.0);
}

#[test]
fn non_takeoff_throttle_is_half_hover_floored_at_zero() {
    assert_eq!(non_takeoff_throttle(0.5), 0.25);
    assert_eq!(non_takeoff_throttle(0.0), 0.0);
    assert_eq!(non_takeoff_throttle(-0.4), 0.0);
}

#[test]
fn large_angle_request_is_strictly_above_fifteen_degrees() {
    assert!(!large_angle_request(LAND_CHECK_LARGE_ANGLE_RAD, 0.0));
    assert!(!large_angle_request(0.0, 0.0));
    assert!(large_angle_request(
        LAND_CHECK_LARGE_ANGLE_RAD + 1.0e-4,
        0.0
    ));
}

fn settling() -> LandDetectorInputs {
    LandDetectorInputs::default()
}

fn landed(count: u32) -> LandDetectorInputs {
    LandDetectorInputs {
        land_complete: true,
        land_detector_count: count,
        ..settling()
    }
}

#[test]
fn default_inputs_accumulate() {
    let got = update_land_detector(&settling());
    assert_eq!(
        got,
        LandDetectorUpdate {
            land_complete: false,
            land_complete_maybe: false,
            count: 1,
            unexpected_takeoff: false,
        }
    );
}

#[test]
fn disarmed_is_always_landed() {
    let flying = LandDetectorInputs {
        armed: false,
        land_complete: false,
        land_detector_count: 50,
        ..settling()
    };
    let got = update_land_detector(&flying);
    assert!(got.land_complete);
    assert!(got.land_complete_maybe);
    assert_eq!(got.count, 0, "flying-to-landed zeros the count");
    assert!(!got.unexpected_takeoff);

    let already = LandDetectorInputs {
        armed: false,
        land_complete: true,
        land_detector_count: 17,
        ..settling()
    };
    let got = update_land_detector(&already);
    assert!(got.land_complete);
    assert_eq!(
        got.count, 17,
        "already-landed disarm must not wipe the count"
    );
}

#[test]
fn already_landed_stays_landed_unless_throttle_and_spool_clear_it() {
    let stay = update_land_detector(&landed(0));
    assert!(stay.land_complete);
    assert!(stay.land_complete_maybe);
    assert!(!stay.unexpected_takeoff);

    let taking_off = LandDetectorInputs {
        is_taking_off: true,
        throttle_out: 0.8,
        spool_state: SpoolState::ThrottleUnlimited,
        ..landed(0)
    };
    assert!(update_land_detector(&taking_off).land_complete);

    let spooling = LandDetectorInputs {
        throttle_out: 0.8,
        spool_state: SpoolState::SpoolingUp,
        ..landed(0)
    };
    assert!(update_land_detector(&spooling).land_complete);

    let exactly = LandDetectorInputs {
        throttle_out: non_takeoff_throttle(0.5),
        spool_state: SpoolState::ThrottleUnlimited,
        ..landed(3)
    };
    let got = update_land_detector(&exactly);
    assert!(got.land_complete);
    assert_eq!(got.count, 3);
    assert!(!got.unexpected_takeoff);
}

#[test]
fn high_throttle_unlimited_while_landed_is_an_unexpected_takeoff() {
    let inp = LandDetectorInputs {
        throttle_out: 0.26,
        throttle_hover: 0.5,
        spool_state: SpoolState::ThrottleUnlimited,
        land_detector_count: 9,
        ..landed(9)
    };
    let got = update_land_detector(&inp);
    assert!(!got.land_complete);
    assert!(!got.land_complete_maybe);
    assert_eq!(got.count, 0);
    assert!(got.unexpected_takeoff);
}

#[test]
fn standby_zeros_the_count_and_does_not_run_the_criteria() {
    let inp = LandDetectorInputs {
        standby_active: true,
        land_detector_count: 40,
        ..settling()
    };
    let got = update_land_detector(&inp);
    assert!(!got.land_complete);
    assert!(!got.land_complete_maybe);
    assert_eq!(got.count, 0);
}

#[test]
fn any_failed_criterion_resets_the_count() {
    let cases: &[LandDetectorInputs] = &[
        LandDetectorInputs {
            motor_at_lower_limit: false,
            land_detector_count: 12,
            ..settling()
        },
        LandDetectorInputs {
            throttle_mix_at_min: false,
            land_detector_count: 12,
            ..settling()
        },
        LandDetectorInputs {
            target_roll_rad: LAND_CHECK_LARGE_ANGLE_RAD + 0.01,
            land_detector_count: 12,
            ..settling()
        },
        LandDetectorInputs {
            att_error_angle_deg: LAND_CHECK_ANGLE_ERROR_DEG + 0.1,
            land_detector_count: 12,
            ..settling()
        },
        LandDetectorInputs {
            filtered_accel_ms2: LAND_DETECTOR_ACCEL_MAX + 0.01,
            land_detector_count: 12,
            ..settling()
        },
        LandDetectorInputs {
            vel_d_ms: LAND_DETECTOR_VEL_Z_MAX,
            land_detector_count: 12,
            ..settling()
        },
        LandDetectorInputs {
            rangefinder_alt_ok: true,
            rangefinder_alt_m: LAND_RANGEFINDER_MIN_ALT_M,
            land_detector_count: 12,
            ..settling()
        },
        LandDetectorInputs {
            wow: WowState::NoWow,
            land_detector_count: 12,
            ..settling()
        },
    ];
    for inp in cases {
        let got = update_land_detector(inp);
        assert_eq!(got.count, 0, "failed criterion should reset: {inp:?}");
        assert!(!got.land_complete);
        assert!(!got.land_complete_maybe);
    }
}

#[test]
fn threshold_edges_match_upstream_comparisons() {
    // Accel uses `<=`, so exactly 1 m/s² is still stationary.
    let accel = LandDetectorInputs {
        filtered_accel_ms2: LAND_DETECTOR_ACCEL_MAX,
        ..settling()
    };
    assert_eq!(update_land_detector(&accel).count, 1);

    // Vertical speed uses `<`, so exactly 1 m/s is not low.
    let vel = LandDetectorInputs {
        vel_d_ms: LAND_DETECTOR_VEL_Z_MAX,
        ..settling()
    };
    assert_eq!(update_land_detector(&vel).count, 0);

    // Angle error uses `>`, so exactly 30 deg is not large.
    let err = LandDetectorInputs {
        att_error_angle_deg: LAND_CHECK_ANGLE_ERROR_DEG,
        ..settling()
    };
    assert_eq!(update_land_detector(&err).count, 1);

    // A healthy rangefinder at exactly 2 m is not below the floor.
    let rf = LandDetectorInputs {
        rangefinder_alt_ok: true,
        rangefinder_alt_m: LAND_RANGEFINDER_MIN_ALT_M,
        ..settling()
    };
    assert_eq!(update_land_detector(&rf).count, 0);

    // Unhealthy rangefinder is ignored even at a high reading.
    let rf_bad = LandDetectorInputs {
        rangefinder_alt_ok: false,
        rangefinder_alt_m: 20.0,
        ..settling()
    };
    assert_eq!(update_land_detector(&rf_bad).count, 1);
}

#[test]
fn wow_known_doubles_the_accel_and_speed_thresholds() {
    let wow = LandDetectorInputs {
        wow: WowState::Wow,
        filtered_accel_ms2: 1.5,
        vel_d_ms: 1.5,
        ..settling()
    };
    assert_eq!(update_land_detector(&wow).count, 1);

    let unknown = LandDetectorInputs {
        wow: WowState::Unknown,
        filtered_accel_ms2: 1.5,
        ..settling()
    };
    assert_eq!(
        update_land_detector(&unknown).count,
        0,
        "unknown WoW must not loosen the thresholds"
    );
}

#[test]
fn airmode_forces_mix_min_and_uses_the_three_second_trigger() {
    let mix_not_min = LandDetectorInputs {
        has_manual_throttle: true,
        airmode_enabled: true,
        throttle_mix_at_min: false,
        land_detector_count: 399,
        ..settling()
    };
    let got = update_land_detector(&mix_not_min);
    assert_eq!(got.count, 400, "airmode forces mix-min true");
    assert!(!got.land_complete, "3 s trigger is 1200 ticks at 400 Hz");

    let no_airmode = LandDetectorInputs {
        has_manual_throttle: true,
        airmode_enabled: false,
        throttle_mix_at_min: false,
        land_detector_count: 12,
        ..settling()
    };
    assert_eq!(update_land_detector(&no_airmode).count, 0);

    let at_trigger = LandDetectorInputs {
        has_manual_throttle: true,
        airmode_enabled: true,
        land_detector_count: 1200,
        ..settling()
    };
    let got = update_land_detector(&at_trigger);
    assert!(got.land_complete);
    assert_eq!(got.count, 0);
}

#[test]
fn maybe_raises_at_point_two_seconds_then_land_resets_the_count() {
    let maybe = LandDetectorInputs {
        land_detector_count: 79,
        ..settling()
    };
    let got = update_land_detector(&maybe);
    assert_eq!(got.count, 80);
    assert!(got.land_complete_maybe);
    assert!(!got.land_complete);

    let before = LandDetectorInputs {
        land_detector_count: 78,
        ..settling()
    };
    let got = update_land_detector(&before);
    assert_eq!(got.count, 79);
    assert!(!got.land_complete_maybe);

    let land = LandDetectorInputs {
        land_detector_count: 400,
        ..settling()
    };
    let got = update_land_detector(&land);
    assert!(got.land_complete);
    assert!(got.land_complete_maybe);
    assert_eq!(got.count, 0, "set_land_complete zeros the count");
}

#[test]
fn failed_velocity_read_is_zero_and_still_settled() {
    // The unused-result path initialises vel_d_ms to 0.0. That is low.
    let inp = LandDetectorInputs {
        vel_d_ms: 0.0,
        ..settling()
    };
    assert_eq!(update_land_detector(&inp).count, 1);
}
