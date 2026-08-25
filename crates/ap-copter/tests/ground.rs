//! Ground handling and mode exit, against the real firmware.
//!
//! Each decision is observed where it leaves the firmware: the spool command
//! in the motors' stored request, the two calls by wrapping symbols the
//! linker resolves, the EKF reset method by reading the member its setter
//! assigns. See `tools/parity/gen_ground_handling.py`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::ground::{
    ekf_reset_method, is_disarmed_or_landed, make_safe_ground_handling,
    smooth_throttle_transition_on_exit, zero_throttle_spool, EkfResetMethod,
};
use ap_motors::spool::{DesiredSpoolState, Spool, SpoolState};

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

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/ground_handling.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out = Vec::new();
    let mut current = "";
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag;
            continue;
        }
        if line.is_empty() || line.starts_with("idx,") {
            continue;
        }
        if current == section {
            out.push(line.split(',').map(str::to_owned).collect());
        }
    }
    out
}

/// The spool command the firmware stored, which is the mode's ask after the
/// motors' own safety constraint. The Rust side composes the same two.
fn stored(desired: DesiredSpoolState, armed: bool, interlock: bool) -> i32 {
    let mut spool = Spool::new();
    spool.set_desired_spool_state(desired, armed, interlock);
    spool.desired() as i32
}

#[test]
fn is_disarmed_or_landed_matches_upstream() {
    let rows = rows("disarmed_or_landed");
    assert_eq!(
        rows.len(),
        8,
        "the sweep should be exhaustive over three bools"
    );

    let mut trues = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 5, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let got = is_disarmed_or_landed(b(&r[1]), b(&r[2]), b(&r[3]));
        let want = b(&r[4]);
        assert_eq!(got, want, "row {idx}: {got} against upstream {want}");
        if want {
            trues += 1;
        }
    }
    // Seven of the eight combinations are "not flying"; only all-three-good is
    // not. If that ratio ever changes the predicate has changed shape.
    assert_eq!(trues, 7, "the predicate is no longer a three-way or");
}

#[test]
fn make_safe_ground_handling_matches_upstream() {
    let rows = rows("ground_handling");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut reset_yes = 0_usize;
    let mut reset_no = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 7, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let armed = b(&r[3]);
        let interlock = b(&r[4]);
        let got = make_safe_ground_handling(b(&r[1]), spool_state(&r[2]));

        let want_desired: i32 = r[5].trim().parse().expect("desired");
        let want_reset = b(&r[6]);

        assert_eq!(
            stored(got.desired_spool, armed, interlock),
            want_desired,
            "row {idx}: spool command {:?} against upstream {want_desired}",
            got.desired_spool
        );
        assert_eq!(
            got.reset_yaw_target_and_rate, want_reset,
            "row {idx}: yaw reset {} against upstream {want_reset}",
            got.reset_yaw_target_and_rate
        );

        if want_reset {
            reset_yes += 1;
        } else {
            reset_no += 1;
        }
    }

    assert!(
        reset_yes > 0 && reset_no > 0,
        "the yaw reset never varies: {reset_yes} on, {reset_no} off"
    );
    println!(
        "{} rows, yaw reset on in {reset_yes} and off in {reset_no}",
        rows.len()
    );
}

#[test]
fn zero_throttle_spool_matches_upstream() {
    let rows = rows("zero_throttle");
    assert!(!rows.is_empty(), "no recorded rows");

    for r in &rows {
        assert_eq!(r.len(), 5, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let got = zero_throttle_spool(b(&r[1]));
        let want: i32 = r[4].trim().parse().expect("desired");
        assert_eq!(
            stored(got, b(&r[2]), b(&r[3])),
            want,
            "row {idx}: {got:?} against upstream {want}"
        );
    }
}

#[test]
fn the_throttle_handover_matches_upstream() {
    let rows = rows("exit_mode");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut seeded = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 6, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let got = smooth_throttle_transition_on_exit(b(&r[1]), b(&r[2]), b(&r[3]), b(&r[4]));
        let want = b(&r[5]);
        assert_eq!(got, want, "row {idx}: {got} against upstream {want}");
        if want {
            seeded += 1;
        }
    }
    assert!(
        seeded > 0 && seeded < rows.len(),
        "the handover never varies across {} rows",
        rows.len()
    );
}

#[test]
fn the_ekf_reset_method_matches_upstream() {
    let rows = rows("ekf_reset");
    assert!(!rows.is_empty(), "no recorded rows");

    for r in &rows {
        assert_eq!(r.len(), 3, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let got = ekf_reset_method(b(&r[1]));
        let want: i32 = r[2].trim().parse().expect("method");
        assert_eq!(
            got as i32, want,
            "row {idx}: {got:?} against upstream {want}"
        );
    }
}

/// The yaw target is reset only while the motors are idle or stopped.
///
/// While they are spooling or unlimited the aircraft is holding a heading on
/// purpose, and snapping the target to the current heading every iteration
/// would throw away a demand a pilot or a mission is making. The condition is
/// on where the motors *are*, not on what they were just asked for — so
/// forcing them unlimited does not stop the reset until they get there.
#[test]
fn the_yaw_reset_follows_the_motors_not_the_command() {
    for force in [false, true] {
        for (state, expect) in [
            (SpoolState::ShutDown, true),
            (SpoolState::GroundIdle, true),
            (SpoolState::SpoolingUp, false),
            (SpoolState::ThrottleUnlimited, false),
            (SpoolState::SpoolingDown, false),
        ] {
            let got = make_safe_ground_handling(force, state);
            assert_eq!(
                got.reset_yaw_target_and_rate, expect,
                "with force {force} and motors {state:?}"
            );
        }
    }

    // And the command itself depends only on the flag, never on the state.
    for state in [
        SpoolState::ShutDown,
        SpoolState::GroundIdle,
        SpoolState::SpoolingUp,
        SpoolState::ThrottleUnlimited,
        SpoolState::SpoolingDown,
    ] {
        assert_eq!(
            make_safe_ground_handling(true, state).desired_spool,
            DesiredSpoolState::ThrottleUnlimited
        );
        assert_eq!(
            make_safe_ground_handling(false, state).desired_spool,
            DesiredSpoolState::GroundIdle
        );
    }
}

/// The throttle handover happens only where the discontinuity could be felt.
///
/// All four conditions are required: leaving a manual-throttle mode, entering
/// one that is not, armed, and not landed. Dropping any one of them means
/// there is no altitude being held on the pilot's stick for the controller to
/// inherit.
#[test]
fn the_handover_needs_every_condition() {
    assert!(smooth_throttle_transition_on_exit(true, false, true, false));

    assert!(
        !smooth_throttle_transition_on_exit(false, false, true, false),
        "the old mode was not on the pilot's throttle"
    );
    assert!(
        !smooth_throttle_transition_on_exit(true, true, true, false),
        "the new mode is on the pilot's throttle too, so nothing is handed over"
    );
    assert!(
        !smooth_throttle_transition_on_exit(true, false, false, false),
        "disarmed, there is no altitude to lose"
    );
    assert!(
        !smooth_throttle_transition_on_exit(true, false, true, true),
        "landed, there is no altitude to lose"
    );
}

/// The two reset methods are opposites, and the mapping is not arbitrary.
#[test]
fn the_reset_method_maps_both_ways() {
    assert_eq!(ekf_reset_method(true), EkfResetMethod::MoveVehicle);
    assert_eq!(ekf_reset_method(false), EkfResetMethod::MoveTarget);
}
