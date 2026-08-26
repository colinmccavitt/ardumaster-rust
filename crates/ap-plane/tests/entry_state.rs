//! What a Plane mode change clears, against the real firmware.
//!
//! Every field is filled with a distinct non-default sentinel before the
//! firmware's `enter()` runs, so the thing being tested for is *omission*: a
//! field the port forgets to reset arrives here still holding its sentinel.
//! A sweep of plausible values could not find that, because a forgotten field
//! would simply keep whatever the sweep put there and nothing would notice.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_plane::entry_state::{AutoState, CrashState, ModeEntryState, SteerState};

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

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/plane_mode_entry.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("idx,"))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

/// The sentinel state the harness writes before each recorded call. Every
/// field differs from what a reset would leave, so anything still holding one
/// of these afterwards was not reset.
fn sentinel() -> ModeEntryState {
    ModeEntryState {
        auto: AutoState {
            inverted_flight: true,
            next_wp_crosstrack: true,
            checked_for_autoland: true,
            highest_airspeed: 33.25,
            initial_pitch_cd: -9999,
            fbwa_tdrag_takeoff_mode: true,
            rotation_complete: true,
            vtol_mode: true,
            vtol_loiter: true,
            idle_mode: true,
        },
        steer: SteerState {
            locked_course: true,
            locked_course_err: 1.5,
        },
        crash: CrashState {
            is_crashed: true,
            impact_detected: true,
        },
        waiting_for_rudder_neutral: true,
        loiter_start_time_ms: 123_456,
        new_airspeed_cm: 777,
        long_failsafe_pending: true,
        throttle_suppressed: false,
    }
}

#[test]
fn the_mode_entry_reset_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut vtol_rows = 0_usize;
    let mut suppressed_rows = 0_usize;
    let mut pitches = std::collections::BTreeSet::new();

    for r in &rows {
        assert_eq!(r.len(), 25, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let mode = &r[1];

        let pitch_cd: i16 = r[2].trim().parse().expect("pitch");
        let is_vtol = b(&r[3]);
        let ok = b(&r[4]);

        let mut state = sentinel();
        state.reset(pitch_cd, is_vtol);
        if ok {
            state.after_enter(b(&r[24]));
        }

        let want = ModeEntryState {
            auto: AutoState {
                inverted_flight: b(&r[5]),
                next_wp_crosstrack: b(&r[7]),
                checked_for_autoland: b(&r[8]),
                highest_airspeed: f(&r[13]),
                initial_pitch_cd: r[14].trim().parse().expect("initial pitch"),
                fbwa_tdrag_takeoff_mode: b(&r[15]),
                rotation_complete: b(&r[16]),
                vtol_mode: b(&r[18]),
                vtol_loiter: b(&r[19]),
                idle_mode: b(&r[22]),
            },
            steer: SteerState {
                locked_course: b(&r[9]),
                locked_course_err: f(&r[10]),
            },
            crash: CrashState {
                is_crashed: b(&r[11]),
                impact_detected: b(&r[12]),
            },
            waiting_for_rudder_neutral: b(&r[6]),
            loiter_start_time_ms: r[17].trim().parse().expect("loiter ms"),
            new_airspeed_cm: r[20].trim().parse().expect("airspeed"),
            long_failsafe_pending: b(&r[21]),
            throttle_suppressed: b(&r[23]),
        };

        assert_eq!(
            state, want,
            "row {idx} ({mode}, pitch {pitch_cd}): the port's state after a \
             mode change differs from the firmware's"
        );

        if is_vtol {
            vtol_rows += 1;
        }
        if want.throttle_suppressed {
            suppressed_rows += 1;
        }
        pitches.insert(pitch_cd);
    }

    // The two seeded fields must actually have varied, or the recording would
    // pass just as happily against a port that zeroed them.
    assert!(
        pitches.len() > 1 && pitches.iter().any(|p| *p != 0),
        "the seeded pitch never varied: {pitches:?}"
    );
    // No recorded row enters a VTOL mode, and that is not an oversight:
    // calling enter() on one with no quadplane available segfaults, which is
    // what Plane::set_mode's VTOL guard prevents. So the recording pins the
    // false case only, and `the_seeded_fields_are_not_cleared` pins the true
    // one. If a quadplane is ever brought up in a harness this should start
    // failing, which is the point of asserting it rather than ignoring it.
    assert_eq!(
        vtol_rows, 0,
        "a VTOL mode was recorded after all — the reasoning below about \
         vtol_mode can now be replaced by the recording"
    );
    assert!(
        suppressed_rows > 0 && suppressed_rows < rows.len(),
        "throttle suppression never varied"
    );

    println!(
        "{} rows, {} distinct seeded pitches, {vtol_rows} VTOL, \
         {suppressed_rows} with the throttle suppressed",
        rows.len(),
        pitches.len()
    );
}

/// Nothing the reset touches keeps its previous value.
///
/// The point of the struct is that a mode change leaves no state behind. This
/// checks it directly rather than through the recording: reset from a state
/// where every field is set, and nothing that should be cleared is still set.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "exactness is the assertion: a reset writes a literal zero, and a value merely near zero would mean the field kept part of what it held"
)]
fn a_mode_change_leaves_nothing_behind() {
    let mut state = sentinel();
    state.reset(0, false);

    assert!(!state.auto.inverted_flight);
    assert!(!state.waiting_for_rudder_neutral);
    assert!(!state.auto.next_wp_crosstrack);
    assert!(!state.auto.checked_for_autoland);
    assert!(!state.steer.locked_course);
    assert_eq!(state.steer.locked_course_err, 0.0);
    assert!(!state.crash.is_crashed);
    assert!(!state.crash.impact_detected);
    assert_eq!(state.auto.highest_airspeed, 0.0);
    assert!(!state.auto.fbwa_tdrag_takeoff_mode);
    assert!(!state.auto.rotation_complete);
    assert_eq!(state.loiter_start_time_ms, 0);
    assert!(!state.auto.vtol_loiter);
    assert!(!state.long_failsafe_pending);
    assert!(!state.auto.idle_mode);
}

/// The two seeded fields are seeded, not zeroed.
///
/// They sit in the same list as twenty-odd fields being cleared, which is
/// what makes them easy to flatten. Zeroing `initial_pitch_cd` would tell a
/// takeoff the aircraft started level when it started on a ramp; zeroing
/// `vtol_mode` would tell the vehicle it had entered a fixed-wing mode when
/// it had entered a hover.
#[test]
fn the_seeded_fields_are_not_cleared() {
    for pitch in [-4500_i16, -137, 0, 250, 8999] {
        for vtol in [false, true] {
            let mut state = sentinel();
            state.reset(pitch, vtol);
            assert_eq!(
                state.auto.initial_pitch_cd, pitch,
                "the initial pitch should be the attitude, not zero"
            );
            assert_eq!(
                state.auto.vtol_mode, vtol,
                "vtol_mode should follow the entered mode"
            );
        }
    }
}

/// The "nothing requested" airspeed is −1, not zero.
///
/// Zero is a speed a mission could legitimately ask for, so the sentinel has
/// to be a value that cannot be confused with a request.
#[test]
fn the_unrequested_airspeed_is_not_a_valid_request() {
    let mut state = sentinel();
    state.reset(0, false);
    assert_eq!(state.new_airspeed_cm, -1);
}

/// Throttle suppression is decided after the mode starts, from its own answer.
#[test]
fn the_throttle_suppression_follows_the_entered_mode() {
    let mut auto_throttle = sentinel();
    auto_throttle.reset(0, false);
    auto_throttle.after_enter(true);
    assert!(auto_throttle.throttle_suppressed);

    let mut manual = sentinel();
    manual.reset(0, false);
    manual.after_enter(false);
    assert!(!manual.throttle_suppressed);
}
