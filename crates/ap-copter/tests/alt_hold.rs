//! ArduCopter's altitude-hold state machine, against the real firmware.
//!
//! The recording is exhaustive rather than sampled: the machine reads six
//! booleans and a climb rate, so every combination of the booleans is swept
//! against all five spool states and five climb rates chosen to sit on both
//! sides of the two different zero thresholds. There is no input the machine
//! can see that is not in the fixture.
//!
//! # Two layers, deliberately
//!
//! Upstream's machine commands the motors by calling
//! `set_desired_spool_state`, and the multirotor override of that method
//! refuses the request outright while disarmed or while the motor interlock
//! is open. The fixture therefore records the two composed, and so does this
//! test. Comparing only the mode's ask would pass even if the safety
//! constraint had been dropped on the way into the port — which is exactly
//! what nearly happened, since `Spool::set_desired` had a note saying the
//! check belonged to a caller that did not yet exist.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::alt_hold::{alt_hold_state, AltHoldInputs, AltHoldModeState};
use ap_motors::spool::{DesiredSpoolState, Spool, SpoolState};

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

fn spool_state(s: &str) -> SpoolState {
    match s.trim() {
        "0" => SpoolState::ShutDown,
        "1" => SpoolState::GroundIdle,
        "2" => SpoolState::SpoolingUp,
        "3" => SpoolState::ThrottleUnlimited,
        "4" => SpoolState::SpoolingDown,
        other => panic!("unknown recorded spool state {other}"),
    }
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/alt_hold_state.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter(|l| !l.starts_with(|c: char| c.is_alphabetic()))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

#[test]
fn the_alt_hold_state_machine_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut states_seen = std::collections::BTreeSet::new();
    let mut desired_seen = std::collections::BTreeSet::new();
    let mut uncommanded = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 12, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let armed = b(&r[1]);
        let motor_interlock = b(&r[7]);

        let inputs = AltHoldInputs {
            armed,
            spool_state: spool_state(&r[2]),
            takeoff_running: b(&r[3]),
            auto_armed: b(&r[4]),
            land_complete: b(&r[5]),
            using_interlock: b(&r[6]),
            target_climb_rate_ms: f(&r[8]),
        };

        let decision = alt_hold_state(&inputs);

        let want_state: i32 = r[9].trim().parse().expect("state");
        let got_state = decision.state as i32;
        assert_eq!(
            got_state, want_state,
            "row {idx}: state {got_state} against upstream {want_state}, \
             inputs {inputs:?}"
        );
        states_seen.insert(want_state);

        // The command, composed with the motors' safety constraint exactly as
        // upstream composes them.
        let commanded = b(&r[11]);
        let want_desired: i32 = r[10].trim().parse().expect("desired");

        match decision.desired_spool {
            None => {
                uncommanded += 1;
                assert!(
                    !commanded,
                    "row {idx}: the port issued no spool command where \
                     upstream commanded {want_desired}"
                );
            }
            Some(d) => {
                assert!(
                    commanded,
                    "row {idx}: the port commanded a spool state where \
                     upstream issued no command"
                );
                let mut spool = Spool::new();
                spool.set_desired_spool_state(d, armed, motor_interlock);
                let got = spool.desired() as i32;
                assert_eq!(
                    got, want_desired,
                    "row {idx}: desired spool {got} against upstream \
                     {want_desired}, inputs {inputs:?}"
                );
                desired_seen.insert(want_desired);
            }
        }
    }

    // A sweep that never reached a state would be pinning less than it looks
    // like it pins.
    assert_eq!(
        states_seen.len(),
        5,
        "the recording does not reach every state: {states_seen:?}"
    );
    assert!(
        uncommanded > 0,
        "no row reached the branch that issues no spool command"
    );
    assert!(
        desired_seen.contains(&(DesiredSpoolState::ShutDown as i32))
            && desired_seen.contains(&(DesiredSpoolState::GroundIdle as i32))
            && desired_seen.contains(&(DesiredSpoolState::ThrottleUnlimited as i32)),
        "the recording does not reach every spool command: {desired_seen:?}"
    );

    println!(
        "{} rows, all five states, {} spool commands and {} uncommanded rows",
        rows.len(),
        desired_seen.len(),
        uncommanded
    );
}

/// The two zero thresholds are different, and that difference is load-bearing.
///
/// `triggered_ms` rejects a climb rate of exactly zero, so a centred stick
/// does not start a takeoff. The landed branch commands ground idle only
/// below zero, so that same centred stick does not spool down either. A port
/// that made both `<` or both `<=` would put the aircraft in the wrong place
/// for the most common stick position there is.
#[test]
fn a_centred_stick_neither_takes_off_nor_settles() {
    let ready = AltHoldInputs {
        armed: true,
        spool_state: SpoolState::ThrottleUnlimited,
        takeoff_running: false,
        auto_armed: true,
        land_complete: true,
        using_interlock: false,
        target_climb_rate_ms: 0.0,
    };

    let centred = alt_hold_state(&ready);
    assert_ne!(
        centred.state,
        AltHoldModeState::Takeoff,
        "a centred stick should not start a takeoff"
    );
    assert_eq!(
        centred.desired_spool,
        Some(DesiredSpoolState::ThrottleUnlimited),
        "a centred stick should not spool the aircraft down"
    );

    // A hair either side moves it, in opposite directions.
    let up = alt_hold_state(&AltHoldInputs {
        target_climb_rate_ms: f32::MIN_POSITIVE,
        ..ready
    });
    assert_eq!(up.state, AltHoldModeState::Takeoff);

    let down = alt_hold_state(&AltHoldInputs {
        target_climb_rate_ms: -f32::MIN_POSITIVE,
        ..ready
    });
    assert_eq!(down.desired_spool, Some(DesiredSpoolState::GroundIdle));
}

/// A disarmed aircraft shuts down whatever else is true.
///
/// This is the one branch with no escape: every other input the machine reads
/// is varied underneath it and the command does not change.
#[test]
fn disarmed_always_shuts_down() {
    for flags in 0..32_u8 {
        for spool in [
            SpoolState::ShutDown,
            SpoolState::GroundIdle,
            SpoolState::SpoolingUp,
            SpoolState::ThrottleUnlimited,
            SpoolState::SpoolingDown,
        ] {
            let d = alt_hold_state(&AltHoldInputs {
                armed: false,
                spool_state: spool,
                takeoff_running: flags & 1 != 0,
                auto_armed: flags & 2 != 0,
                land_complete: flags & 4 != 0,
                using_interlock: flags & 8 != 0,
                target_climb_rate_ms: if flags & 16 != 0 { 5.0 } else { -5.0 },
            });
            assert_eq!(
                d.desired_spool,
                Some(DesiredSpoolState::ShutDown),
                "disarmed with spool {spool:?} and flags {flags} did not shut down"
            );
        }
    }
}
