//! Copter crash-check leftover, upstream `ArduCopter/crash_check.cpp`.

use ap_copter::crash_check::{
    crash_check, crash_trigger_count, lean_angle_deg, CrashCheck, CrashCheckInputs,
    CRASH_CHECK_ACCEL_MAX_MS2, CRASH_CHECK_ANGLE_DEVIATION_DEG, CRASH_CHECK_ANGLE_MIN_DEG,
    CRASH_CHECK_SPEED_MAX_MS, CRASH_CHECK_TRIGGER_SEC, FS_CRASH_CHECK_DEFAULT,
};

#[test]
fn constants_match_upstream_defines() {
    assert_eq!(CRASH_CHECK_TRIGGER_SEC, 2);
    assert_eq!(CRASH_CHECK_ANGLE_DEVIATION_DEG, 30.0);
    assert_eq!(CRASH_CHECK_ANGLE_MIN_DEG, 15.0);
    assert_eq!(CRASH_CHECK_SPEED_MAX_MS, 10.0);
    assert_eq!(CRASH_CHECK_ACCEL_MAX_MS2, 3.0);
    assert_eq!(FS_CRASH_CHECK_DEFAULT, 1);
    assert_eq!(crash_trigger_count(400), 800);
    assert_eq!(crash_trigger_count(100), 200);
}

#[test]
fn lean_angle_is_zero_when_level() {
    let angle = lean_angle_deg(1.0, 1.0);
    assert!(angle.abs() < 1.0e-5);
}

#[test]
fn lean_angle_is_ninety_when_on_its_side() {
    let angle = lean_angle_deg(0.0, 1.0);
    assert!((angle - 90.0).abs() < 1.0e-4);
}

fn crashing() -> CrashCheckInputs {
    CrashCheckInputs::default()
}

#[test]
fn default_inputs_accumulate() {
    let decision = crash_check(&crashing());
    assert_eq!(decision, CrashCheck::Accumulating { counter: 1 });
    assert!(!decision.should_disarm());
    assert_eq!(decision.counter(), 1);
}

#[test]
fn first_gate_resets_when_disarmed_landed_or_disabled() {
    let disarmed = CrashCheckInputs {
        armed: false,
        crash_counter: 17,
        ..crashing()
    };
    assert_eq!(crash_check(&disarmed), CrashCheck::Clear);

    let landed = CrashCheckInputs {
        land_complete: true,
        crash_counter: 17,
        ..crashing()
    };
    assert_eq!(crash_check(&landed), CrashCheck::Clear);

    let disabled = CrashCheckInputs {
        fs_crash_check: 0,
        crash_counter: 17,
        ..crashing()
    };
    assert_eq!(crash_check(&disabled), CrashCheck::Clear);
}

#[test]
fn standby_and_autorotate_and_mode_disable_reset() {
    let standby = CrashCheckInputs {
        standby_active: true,
        crash_counter: 9,
        ..crashing()
    };
    assert_eq!(crash_check(&standby), CrashCheck::Clear);

    let acro = CrashCheckInputs {
        crash_check_enabled: false,
        crash_counter: 9,
        ..crashing()
    };
    assert_eq!(crash_check(&acro), CrashCheck::Clear);

    let auto_rot = CrashCheckInputs {
        in_autorotate: true,
        crash_counter: 9,
        ..crashing()
    };
    assert_eq!(crash_check(&auto_rot), CrashCheck::Clear);
}

#[test]
fn force_flying_resets_unless_landing() {
    let force = CrashCheckInputs {
        force_flying: true,
        is_landing: false,
        crash_counter: 4,
        ..crashing()
    };
    assert_eq!(crash_check(&force), CrashCheck::Clear);

    let landing = CrashCheckInputs {
        force_flying: true,
        is_landing: true,
        crash_counter: 4,
        ..crashing()
    };
    assert_eq!(
        crash_check(&landing),
        CrashCheck::Accumulating { counter: 5 }
    );
}

#[test]
fn accel_lean_and_angle_error_gates_reset() {
    let accel = CrashCheckInputs {
        filtered_accel_ms2: CRASH_CHECK_ACCEL_MAX_MS2,
        crash_counter: 3,
        ..crashing()
    };
    assert_eq!(crash_check(&accel), CrashCheck::Clear);

    let still_accel = CrashCheckInputs {
        filtered_accel_ms2: 3.1,
        crash_counter: 3,
        ..crashing()
    };
    assert_eq!(crash_check(&still_accel), CrashCheck::Clear);

    let just_under_accel = CrashCheckInputs {
        filtered_accel_ms2: 2.9,
        crash_counter: 3,
        ..crashing()
    };
    assert_eq!(
        crash_check(&just_under_accel),
        CrashCheck::Accumulating { counter: 4 }
    );

    let upright = CrashCheckInputs {
        lean_angle_deg: CRASH_CHECK_ANGLE_MIN_DEG,
        crash_counter: 3,
        ..crashing()
    };
    assert_eq!(crash_check(&upright), CrashCheck::Clear);

    let tracking = CrashCheckInputs {
        att_error_angle_deg: CRASH_CHECK_ANGLE_DEVIATION_DEG,
        crash_counter: 3,
        ..crashing()
    };
    assert_eq!(crash_check(&tracking), CrashCheck::Clear);
}

#[test]
fn missing_velocity_does_not_reset_but_fast_flight_does() {
    let no_vel = CrashCheckInputs {
        vel_ned_ms: None,
        crash_counter: 2,
        ..crashing()
    };
    assert_eq!(crash_check(&no_vel), CrashCheck::Accumulating { counter: 3 });

    let slow = CrashCheckInputs {
        vel_ned_ms: Some(9.9),
        crash_counter: 2,
        ..crashing()
    };
    assert_eq!(crash_check(&slow), CrashCheck::Accumulating { counter: 3 });

    let fast = CrashCheckInputs {
        vel_ned_ms: Some(CRASH_CHECK_SPEED_MAX_MS),
        crash_counter: 2,
        ..crashing()
    };
    assert_eq!(crash_check(&fast), CrashCheck::Clear);
}

#[test]
fn two_seconds_at_loop_rate_disarms_and_keeps_the_counter() {
    let almost = CrashCheckInputs {
        crash_counter: 799,
        loop_rate_hz: 400,
        ..crashing()
    };
    let decision = crash_check(&almost);
    assert_eq!(decision, CrashCheck::Disarm { counter: 800 });
    assert!(decision.should_disarm());
    assert_eq!(decision.counter(), 800);

    let one_short = CrashCheckInputs {
        crash_counter: 798,
        loop_rate_hz: 400,
        ..crashing()
    };
    assert_eq!(
        crash_check(&one_short),
        CrashCheck::Accumulating { counter: 799 }
    );

    // Upstream does not reset on disarm; the next iteration increments again.
    let already = CrashCheckInputs {
        crash_counter: 800,
        loop_rate_hz: 400,
        ..crashing()
    };
    assert_eq!(crash_check(&already), CrashCheck::Disarm { counter: 801 });
}

#[test]
fn a_cleared_wobble_does_not_inherit_its_count() {
    let wobble = CrashCheckInputs {
        crash_counter: 400,
        lean_angle_deg: 10.0,
        ..crashing()
    };
    assert_eq!(crash_check(&wobble), CrashCheck::Clear);

    let again = CrashCheckInputs {
        crash_counter: 0,
        ..crashing()
    };
    assert_eq!(crash_check(&again), CrashCheck::Accumulating { counter: 1 });
}
