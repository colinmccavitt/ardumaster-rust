//! `Copter::set_mode`'s veto ladder, against the real firmware.
//!
//! The recording carries upstream's own failure *message*, not just its
//! return value. Every veto returns the same `false`, so a fixture of return
//! values alone would be satisfied by any permutation of the ladder — and the
//! order is the part that decides which explanation reaches the pilot.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::mode_entry::{mode_entry, FenceState, ModeEntry, ModeEntryRequest, ModeEntryVeto};

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
        .map(|p| p.join("fixtures/mode_entry.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter(|l| !l.starts_with("idx,"))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

/// Upstream's message for each veto, so the fixture pins the rung and not
/// merely the refusal.
fn message(veto: ModeEntryVeto) -> &'static str {
    match veto {
        ModeEntryVeto::GcsEntryDisabled => "GCS entry disabled (FLTMODE_GCSBLOCK)",
        // The one rung that does not go through mode_change_failed; it
        // calls notify_no_such_mode instead, which the harness wraps
        // separately and records under this name.
        ModeEntryVeto::NoSuchMode => "no such mode",
        ModeEntryVeto::ThrottleTooHigh => "throttle too high",
        ModeEntryVeto::RequiresPosition => "requires position",
        ModeEntryVeto::NeedAltEstimate => "need alt estimate",
        ModeEntryVeto::InFenceRecovery => "in fence recovery",
        ModeEntryVeto::InRcFailsafe => "in RC failsafe",
        ModeEntryVeto::InitFailed => "init failed",
    }
}

#[test]
fn the_mode_entry_ladder_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut vetoes = std::collections::BTreeMap::new();
    let mut entered = 0_usize;
    let mut already = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 20, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let mode = &r[1];

        let request = ModeEntryRequest {
            target_is_current: b(&r[2]),
            // AUTO_RTL is not among the candidates; the delegation is covered
            // by its own test below.
            target_is_auto_rtl: false,
            reason_is_gcs_command: b(&r[3]),
            gcs_entry_enabled: b(&r[4]),
            mode_exists: b(&r[5]),
            armed: b(&r[6]),
            land_complete: b(&r[7]),
            new_has_manual_throttle: b(&r[8]),
            new_is_drift: b(&r[9]),
            current_has_manual_throttle: b(&r[10]),
            pilot_throttle: f(&r[11]),
            non_takeoff_throttle: f(&r[12]),
            new_requires_position: b(&r[13]),
            position_ok: b(&r[14]),
            ekf_alt_ok: b(&r[15]),
            // The fence is quiet throughout the recording; see the module
            // documentation on the harness.
            fence: FenceState {
                enabled: false,
                disable_mode_change: false,
                breached: false,
                entered_for_breach: false,
            },
            in_rc_failsafe: b(&r[16]),
            new_allows_entry_in_rc_failsafe: b(&r[17]),
            // Every candidate mode's init succeeds under these conditions;
            // the recording carries no init failure, so this is what upstream
            // saw. InitFailed is covered by its own test.
            init_ok: true,
        };

        let got = mode_entry(&request);
        let want_result: i32 = r[18].trim().parse().expect("result");
        let want_reason = r[19].trim();

        match (got, want_result) {
            (ModeEntry::AlreadyInMode, 2) => already += 1,
            (ModeEntry::Entered, 1) => entered += 1,
            (ModeEntry::Refused(veto), 0) => {
                assert_eq!(
                    message(veto),
                    want_reason,
                    "row {idx} ({mode}): refused with {veto:?}, upstream said \
                     {want_reason:?} — the ladder order has drifted"
                );
                *vetoes.entry(want_reason.to_owned()).or_insert(0_usize) += 1;
            }
            _ => panic!(
                "row {idx} ({mode}): got {got:?}, upstream result {want_result} \
                 reason {want_reason:?} — request {request:?}"
            ),
        }
    }

    assert!(entered > 0, "no row was allowed through");
    assert!(already > 0, "the already-in-mode shortcut is never taken");
    assert!(
        vetoes.len() >= 4,
        "only {} distinct vetoes reached: {vetoes:?}",
        vetoes.len()
    );

    println!(
        "{} rows — {entered} entered, {already} already in mode, vetoes {vetoes:?}",
        rows.len()
    );
}

/// A disarmed aircraft can be put into any mode.
///
/// `ignore_checks` is exactly `!armed`, so every flight-safety rung is
/// suppressed on the bench: loading a mission should not need a GPS fix, and
/// nothing the aircraft does before it arms can hurt anyone. The arming checks
/// are where the real gate is.
#[test]
fn a_disarmed_aircraft_can_enter_any_mode() {
    let hostile = ModeEntryRequest {
        target_is_current: false,
        target_is_auto_rtl: false,
        reason_is_gcs_command: false,
        gcs_entry_enabled: true,
        mode_exists: true,
        armed: false,
        // Everything a flight-safety rung could object to, all at once.
        land_complete: true,
        new_has_manual_throttle: true,
        new_is_drift: false,
        current_has_manual_throttle: false,
        pilot_throttle: 1.0,
        non_takeoff_throttle: 0.1,
        new_requires_position: true,
        position_ok: false,
        ekf_alt_ok: false,
        fence: FenceState {
            enabled: true,
            disable_mode_change: true,
            breached: true,
            entered_for_breach: true,
        },
        in_rc_failsafe: false,
        new_allows_entry_in_rc_failsafe: true,
        init_ok: true,
    };

    assert_eq!(mode_entry(&hostile), ModeEntry::Entered);

    // Arming the same aircraft refuses it, so the suppression is doing the
    // work rather than the conditions failing to hold.
    let armed = ModeEntryRequest {
        armed: true,
        ..hostile
    };
    assert_eq!(
        mode_entry(&armed),
        ModeEntry::Refused(ModeEntryVeto::ThrottleTooHigh)
    );
}

/// Two rungs are not suppressed by being disarmed, and both are about
/// something other than whether flying would be safe.
///
/// The GCS block is about who is allowed to ask. The RC failsafe rung is
/// about a mode that refuses to run without a pilot, which does not become
/// acceptable on the bench.
#[test]
fn the_bench_exemption_has_two_exceptions() {
    let disarmed = ModeEntryRequest {
        target_is_current: false,
        target_is_auto_rtl: false,
        reason_is_gcs_command: true,
        gcs_entry_enabled: false,
        mode_exists: true,
        armed: false,
        land_complete: false,
        new_has_manual_throttle: false,
        new_is_drift: false,
        current_has_manual_throttle: false,
        pilot_throttle: 0.0,
        non_takeoff_throttle: 0.5,
        new_requires_position: false,
        position_ok: true,
        ekf_alt_ok: true,
        fence: FenceState {
            enabled: false,
            disable_mode_change: false,
            breached: false,
            entered_for_breach: false,
        },
        in_rc_failsafe: false,
        new_allows_entry_in_rc_failsafe: true,
        init_ok: true,
    };

    assert_eq!(
        mode_entry(&disarmed),
        ModeEntry::Refused(ModeEntryVeto::GcsEntryDisabled),
        "a GCS-blocked mode was allowed because the aircraft was disarmed"
    );

    let failsafe = ModeEntryRequest {
        reason_is_gcs_command: false,
        gcs_entry_enabled: true,
        in_rc_failsafe: true,
        new_allows_entry_in_rc_failsafe: false,
        ..disarmed
    };
    assert_eq!(
        mode_entry(&failsafe),
        ModeEntry::Refused(ModeEntryVeto::InRcFailsafe),
        "an RC-failsafe-refusing mode was allowed because the aircraft was \
         disarmed"
    );
}

/// Already being in the mode short-circuits every check.
///
/// Including the ones that would otherwise refuse. A check that fired here
/// would be refusing the status quo — the aircraft is already flying in that
/// mode, and saying no changes nothing except to alarm the pilot.
#[test]
fn being_already_in_the_mode_answers_before_any_check() {
    let hopeless = ModeEntryRequest {
        target_is_current: true,
        target_is_auto_rtl: false,
        reason_is_gcs_command: true,
        gcs_entry_enabled: false,
        mode_exists: false,
        armed: true,
        land_complete: true,
        new_has_manual_throttle: true,
        new_is_drift: true,
        current_has_manual_throttle: false,
        pilot_throttle: 1.0,
        non_takeoff_throttle: 0.0,
        new_requires_position: true,
        position_ok: false,
        ekf_alt_ok: false,
        fence: FenceState {
            enabled: true,
            disable_mode_change: true,
            breached: true,
            entered_for_breach: true,
        },
        in_rc_failsafe: true,
        new_allows_entry_in_rc_failsafe: false,
        init_ok: false,
    };
    assert_eq!(mode_entry(&hopeless), ModeEntry::AlreadyInMode);
}

/// The fence rung, which the recording does not reach.
///
/// Getting there needs a configured fence, a real breach, and the current
/// mode to have been entered by the fence itself — none of which a harness
/// can stand up. So this is derived from upstream's source rather than
/// recorded, and is labelled as such: it pins that all six conditions are
/// required together, which is what makes the rung narrow enough to be safe.
#[test]
fn the_fence_rung_needs_every_condition_together() {
    let recovering = ModeEntryRequest {
        target_is_current: false,
        target_is_auto_rtl: false,
        reason_is_gcs_command: false,
        gcs_entry_enabled: true,
        mode_exists: true,
        armed: true,
        land_complete: false,
        new_has_manual_throttle: false,
        new_is_drift: false,
        current_has_manual_throttle: false,
        pilot_throttle: 0.0,
        non_takeoff_throttle: 0.5,
        new_requires_position: false,
        position_ok: true,
        ekf_alt_ok: true,
        fence: FenceState {
            enabled: true,
            disable_mode_change: true,
            breached: true,
            entered_for_breach: true,
        },
        in_rc_failsafe: false,
        new_allows_entry_in_rc_failsafe: true,
        init_ok: true,
    };
    assert_eq!(
        mode_entry(&recovering),
        ModeEntry::Refused(ModeEntryVeto::InFenceRecovery)
    );

    // Dropping any one condition lets the change through, so none of them is
    // decoration.
    let relaxations: [(&str, ModeEntryRequest); 5] = [
        (
            "fence disabled",
            ModeEntryRequest {
                fence: FenceState {
                    enabled: false,
                    ..recovering.fence
                },
                ..recovering
            },
        ),
        (
            "option off",
            ModeEntryRequest {
                fence: FenceState {
                    disable_mode_change: false,
                    ..recovering.fence
                },
                ..recovering
            },
        ),
        (
            "no breach",
            ModeEntryRequest {
                fence: FenceState {
                    breached: false,
                    ..recovering.fence
                },
                ..recovering
            },
        ),
        (
            "current mode not entered for the breach",
            ModeEntryRequest {
                fence: FenceState {
                    entered_for_breach: false,
                    ..recovering.fence
                },
                ..recovering
            },
        ),
        (
            "already landed",
            ModeEntryRequest {
                land_complete: true,
                ..recovering
            },
        ),
    ];

    for (what, request) in relaxations {
        assert_eq!(
            mode_entry(&request),
            ModeEntry::Entered,
            "with {what}, the fence rung should not have fired"
        );
    }
}

/// AUTO_RTL is answered by the mission logic, and only after the GCS block.
///
/// The ordering matters: a ground station blocked from commanding AUTO_RTL
/// must be refused rather than quietly handed to the mission code, which
/// would otherwise change the aircraft's flight path on a command the
/// operator had disabled.
#[test]
fn auto_rtl_is_delegated_but_not_before_the_gcs_block() {
    let request = ModeEntryRequest {
        target_is_current: false,
        target_is_auto_rtl: true,
        reason_is_gcs_command: false,
        gcs_entry_enabled: true,
        mode_exists: true,
        armed: true,
        land_complete: false,
        new_has_manual_throttle: false,
        new_is_drift: false,
        current_has_manual_throttle: false,
        pilot_throttle: 0.0,
        non_takeoff_throttle: 0.5,
        new_requires_position: false,
        position_ok: true,
        ekf_alt_ok: true,
        fence: FenceState {
            enabled: false,
            disable_mode_change: false,
            breached: false,
            entered_for_breach: false,
        },
        in_rc_failsafe: false,
        new_allows_entry_in_rc_failsafe: true,
        init_ok: true,
    };
    assert_eq!(mode_entry(&request), ModeEntry::DelegatedToAutoRtl);

    let blocked = ModeEntryRequest {
        reason_is_gcs_command: true,
        gcs_entry_enabled: false,
        ..request
    };
    assert_eq!(
        mode_entry(&blocked),
        ModeEntry::Refused(ModeEntryVeto::GcsEntryDisabled),
        "a blocked ground station reached the mission logic anyway"
    );
}
