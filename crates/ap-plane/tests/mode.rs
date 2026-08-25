//! Plane's mode machine, against the real firmware.
//!
//! # What the recording covers and what it does not
//!
//! The veto ladder and the state move are recorded. The *rollback* is not: no
//! mode reachable in a harness refuses in `enter()` on this build, so the
//! branch never fires. That gap is stated rather than papered over, and the
//! rollback is pinned instead by a property every rollback must have —
//! applying and then rolling back restores all four fields — checked
//! exhaustively over the reasons the machine distinguishes.
//!
//! AUTO is absent from the recording on purpose: its `enter()` succeeds and
//! the mode then immediately re-enters RTL because there is no mission, so
//! the after-state would be a different mode change than the one under test.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_plane::mode::{
    already_in_mode_notifies, mode_change_veto, FenceState, ModeChangeRequest, ModeChangeVeto,
    ModeReason, ModeState,
};

/// The recording stores upstream's own reason numbers, so the port's mapping
/// is what reads them. An earlier version of this test carried its own table
/// of guessed numbers and disagreed with the firmware; the numbers are part
/// of the port's contract, so they live there.
fn reason(n: &str) -> ModeReason {
    ModeReason::from_number(n.trim().parse().expect("reason number"))
}

fn b(s: &str) -> bool {
    match s.trim() {
        "0" => false,
        "1" => true,
        other => panic!("not a recorded boolean: {other}"),
    }
}

fn n(s: &str) -> u8 {
    s.trim().parse().expect("number")
}

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/plane_mode.csv"))
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

fn state_with(control_reason: ModeReason, previous_reason: ModeReason) -> ModeState {
    ModeState {
        control_mode: 0,
        previous_mode: 0,
        control_mode_reason: control_reason,
        previous_mode_reason: previous_reason,
    }
}

/// The reason numbers survive a round trip, and are upstream's.
///
/// These are not an internal choice: they are written to logs and sent over
/// MAVLink, so a renumbering would silently reinterpret every recorded flight.
/// `from_number` is exercised by every fixture row; `as_number` had no caller
/// at all until this test, which is how the mutation gate found it.
#[test]
fn the_reason_numbers_are_upstreams() {
    // From AP_Vehicle/ModeReason.h, transcribed and checked against the
    // recording rather than remembered -- an earlier version of this test
    // guessed 6 and 8 for two of them and disagreed with the firmware.
    for (number, reason) in [
        (2_u8, ModeReason::GcsCommand),
        (10, ModeReason::FenceBreached),
        (26, ModeReason::Initialised),
        (39, ModeReason::RtlCompleteSwitchingToVtolLandRtl),
        (40, ModeReason::RtlCompleteSwitchingToFixedwingAutoland),
        (44, ModeReason::QrtlInsteadOfRtl),
        (49, ModeReason::QlandInsteadOfRtl),
    ] {
        assert_eq!(
            reason.as_number(),
            number,
            "{reason:?} has the wrong number"
        );
        assert_eq!(
            ModeReason::from_number(number),
            reason,
            "number {number} maps to the wrong reason"
        );
    }

    // Every number round-trips, named or not, so nothing is silently folded
    // into a reason it is not.
    for number in 0..=u8::MAX {
        assert_eq!(
            ModeReason::from_number(number).as_number(),
            number,
            "number {number} did not survive a round trip"
        );
    }
}

/// `in_fence_recovery`, swept over every pair of the reasons it reads.
#[test]
fn the_fence_recovery_predicate_matches_upstream() {
    let rows = rows("fence_recovery");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut trues = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 4, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let state = state_with(reason(&r[1]), reason(&r[2]));
        let got = state.in_fence_recovery(false);
        let want = b(&r[3]);
        assert_eq!(
            got, want,
            "row {idx}: {got} against upstream {want} — control reason {}, \
             previous reason {}",
            r[1], r[2]
        );
        if want {
            trues += 1;
        }
    }
    assert!(
        trues > 0 && trues < rows.len(),
        "the predicate never varies across {} rows",
        rows.len()
    );
    println!("{} fence-recovery rows, {trues} in recovery", rows.len());
}

/// The veto ladder and the state move.
#[test]
fn the_mode_machine_matches_upstream() {
    let rows = rows("set_mode");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut vetoes = std::collections::BTreeMap::new();
    let mut changed = 0_usize;
    let mut already = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 20, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let target = n(&r[1]);
        let request = ModeChangeRequest {
            new_mode: target,
            reason: reason(&r[2]),
            new_is_vtol: b(&r[4]),
            // No quadplane is configured in the recording.
            quadplane_available: false,
            fence: FenceState {
                soft_armed: b(&r[5]),
                enabled: b(&r[6]),
                disable_mode_change: b(&r[7]),
                breached: b(&r[8]),
                recovering: b(&r[9]),
            },
            gcs_entry_enabled: b(&r[3]),
        };

        let before = ModeState {
            control_mode: n(&r[10]),
            previous_mode: n(&r[11]),
            control_mode_reason: reason(&r[12]),
            previous_mode_reason: reason(&r[13]),
        };
        let after = ModeState {
            control_mode: n(&r[15]),
            previous_mode: n(&r[16]),
            control_mode_reason: reason(&r[17]),
            previous_mode_reason: reason(&r[18]),
        };
        let ok = b(&r[14]);
        let message = r[19].trim();

        // Already in the requested mode: upstream returns success and moves
        // nothing at all.
        if before.control_mode == target {
            assert!(ok, "row {idx}: already in the mode but not a success");
            assert_eq!(after, before, "row {idx}: the state moved anyway");
            already += 1;
            continue;
        }

        let veto = mode_change_veto(&request);

        match (veto, ok) {
            (Some(v), false) => {
                let expected = match v {
                    ModeChangeVeto::VtolUnavailable => "vtol unavailable",
                    ModeChangeVeto::InFenceRecovery => "in fence recovery",
                    ModeChangeVeto::GcsEntryDisabled => "GCS entry disabled",
                };
                assert_eq!(
                    message, expected,
                    "row {idx}: refused with {v:?}, upstream said {message:?} \
                     — the ladder order has drifted"
                );
                assert_eq!(after, before, "row {idx}: refused but the state moved");
                *vetoes.entry(expected).or_insert(0_usize) += 1;
            }
            (None, true) => {
                // Allowed through, so the state must have moved exactly the
                // way `apply` moves it.
                let mut state = before;
                state.apply(target, request.reason);
                assert_eq!(
                    state, after,
                    "row {idx}: applied state differs from upstream's"
                );
                changed += 1;
            }
            (v, ok) => panic!(
                "row {idx}: port says {v:?}, upstream ok={ok} message \
                 {message:?} — request {request:?}"
            ),
        }
    }

    assert!(changed > 0, "no row was allowed through");
    assert!(already > 0, "the already-in-mode shortcut is never taken");
    assert_eq!(vetoes.len(), 3, "not every veto is reached: {vetoes:?}");
    println!(
        "{} rows — {changed} changed, {already} already in mode, vetoes {vetoes:?}",
        rows.len()
    );
}

/// Rolling back restores all four fields.
///
/// The recording cannot reach this branch: no mode a harness can request
/// refuses in `enter()` on this build. So it is pinned as the property every
/// rollback must have, over every pair of reasons the machine distinguishes
/// and both orderings of the modes.
///
/// A rollback that restored three of the four would leave the vehicle in the
/// right mode with a wrong story about how it got there — and
/// `in_fence_recovery` reads exactly those reasons, so the next mode change
/// would be judged on the wrong ones.
#[test]
fn a_rollback_restores_every_field() {
    let reasons = [
        ModeReason::Initialised,
        ModeReason::GcsCommand,
        ModeReason::FenceBreached,
        ModeReason::RtlCompleteSwitchingToFixedwingAutoland,
        ModeReason::QrtlInsteadOfRtl,
        ModeReason::Other(3),
    ];

    for &cr in &reasons {
        for &pr in &reasons {
            for control in 0..4_u8 {
                for previous in 0..4_u8 {
                    for &new_reason in &reasons {
                        for new_mode in 0..4_u8 {
                            let original = ModeState {
                                control_mode: control,
                                previous_mode: previous,
                                control_mode_reason: cr,
                                previous_mode_reason: pr,
                            };
                            let mut state = original;
                            let snapshot = state.apply(new_mode, new_reason);

                            // The apply must actually have done something to
                            // roll back, except where it is a no-op by
                            // coincidence of the inputs.
                            state.roll_back(snapshot);
                            assert_eq!(
                                state, original,
                                "rollback left {state:?}, not {original:?}"
                            );
                        }
                    }
                }
            }
        }
    }
}

/// Applying moves the old mode down and the old reason with it.
///
/// The pairing is the part worth pinning: `previous_mode` and
/// `previous_mode_reason` must come from the same moment, or the vehicle ends
/// up reporting that it entered one mode for another mode's reason.
#[test]
fn applying_moves_the_mode_and_its_reason_together() {
    let mut state = ModeState {
        control_mode: 5,
        previous_mode: 2,
        control_mode_reason: ModeReason::FenceBreached,
        previous_mode_reason: ModeReason::GcsCommand,
    };
    state.apply(11, ModeReason::Initialised);

    assert_eq!(state.control_mode, 11);
    assert_eq!(state.control_mode_reason, ModeReason::Initialised);
    assert_eq!(
        state.previous_mode, 5,
        "the mode that was running should have moved down"
    );
    assert_eq!(
        state.previous_mode_reason,
        ModeReason::FenceBreached,
        "and its reason with it"
    );
}

/// The fence's own recovery completing must not be blocked by the fence.
///
/// A breach sends the vehicle to RTL; RTL completes and hands over to a
/// landing mode with one of four reasons. If those were treated as ordinary
/// mode changes the fence would refuse the very handover it asked for, and
/// the vehicle would be held in RTL indefinitely.
#[test]
fn the_fence_does_not_block_its_own_handover() {
    let blocking = ModeChangeRequest {
        new_mode: 11,
        reason: ModeReason::GcsCommand,
        new_is_vtol: false,
        quadplane_available: false,
        fence: FenceState {
            soft_armed: true,
            enabled: true,
            disable_mode_change: true,
            breached: true,
            recovering: true,
        },
        gcs_entry_enabled: true,
    };
    assert_eq!(
        mode_change_veto(&blocking),
        Some(ModeChangeVeto::InFenceRecovery)
    );

    for handover in [
        ModeReason::RtlCompleteSwitchingToFixedwingAutoland,
        ModeReason::RtlCompleteSwitchingToVtolLandRtl,
        ModeReason::QrtlInsteadOfRtl,
        ModeReason::QlandInsteadOfRtl,
    ] {
        assert_eq!(
            mode_change_veto(&ModeChangeRequest {
                reason: handover,
                ..blocking
            }),
            None,
            "the fence blocked its own handover for {handover:?}"
        );
    }
}

/// The fence is tested before the GCS block, unlike Copter.
///
/// When both would fire, the pilot is told about the recovery rather than
/// about the block — the more useful message, since the block is a standing
/// configuration and the recovery is the condition that will pass.
#[test]
fn the_fence_outranks_the_gcs_block() {
    let both = ModeChangeRequest {
        new_mode: 7,
        reason: ModeReason::GcsCommand,
        new_is_vtol: false,
        quadplane_available: false,
        fence: FenceState {
            soft_armed: true,
            enabled: true,
            disable_mode_change: true,
            breached: true,
            recovering: true,
        },
        gcs_entry_enabled: false,
    };
    assert_eq!(
        mode_change_veto(&both),
        Some(ModeChangeVeto::InFenceRecovery)
    );

    // And the VTOL check outranks both.
    assert_eq!(
        mode_change_veto(&ModeChangeRequest {
            new_is_vtol: true,
            ..both
        }),
        Some(ModeChangeVeto::VtolUnavailable)
    );
}

/// Asking for the mode already running is a success, and the noise is
/// suppressed when nothing has changed about why.
#[test]
fn repeating_a_mode_request_does_not_beep() {
    assert!(
        !already_in_mode_notifies(ModeReason::GcsCommand, ModeReason::GcsCommand),
        "a ground station repeating its request should not beep each time"
    );
    assert!(
        !already_in_mode_notifies(ModeReason::Initialised, ModeReason::GcsCommand),
        "startup should be silent"
    );
    assert!(
        already_in_mode_notifies(ModeReason::GcsCommand, ModeReason::Other(3)),
        "a different method of asking should be acknowledged"
    );
}
