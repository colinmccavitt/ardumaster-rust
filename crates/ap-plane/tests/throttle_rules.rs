//! Which corrections a Plane mode's throttle is subject to, against the
//! firmware.
//!
//! The manual-throttle set is not asserted by the harness: upstream compares
//! `this` against five specific modes, so every fixed-wing mode is swept and
//! the firmware decides which five they are.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_plane::throttle_rules::{
    allow_fw_systemid, is_vtol_man_throttle, manual_use_battery_compensation,
    manual_use_throttle_limits, use_battery_compensation, use_throttle_limits, SystemIdContext,
    ThrottleContext,
};

/// The five modes upstream names as manual-throttle: STABILIZE, TRAINING,
/// ACRO, FBWA, AUTOTUNE. Asserted against the recording rather than trusted.
const MANUAL_THROTTLE_MODES: &[u8] = &[2, 3, 4, 5, 8];

/// MANUAL, the one mode that overrides both predicates outright. It is
/// deliberately *not* in the list above — the base implementation never sees
/// it.
const MANUAL_MODE: u8 = 0;

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
        .map(|p| p.join("fixtures/plane_throttle_rules.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#') && !l.starts_with("idx,"))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

#[test]
fn the_throttle_rules_match_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    let mut disagreements = 0_usize;
    let mut manual_seen = std::collections::BTreeSet::new();

    for r in &rows {
        assert_eq!(r.len(), 7, "malformed row");
        let idx: usize = r[0].parse().expect("idx");
        let mode: u8 = r[1].trim().parse().expect("mode");

        let context = ThrottleContext {
            // Not reachable in this recording; see the tests below.
            nav_scripting_active: false,
            manual_throttle_mode: MANUAL_THROTTLE_MODES.contains(&mode),
            throttle_passthru_stabilize: b(&r[3]),
            guided_throttle_passthru: b(&r[2]) && b(&r[4]),
            in_vtol_mode: false,
            allow_forward_throttle_in_vtol: false,
        };

        let want_limits = b(&r[5]);
        let want_battery = b(&r[6]);

        // MANUAL is the only mode that overrides these, and it overrides
        // both. No quadplane is configured in the recording, so its throttle
        // limits are off throughout.
        let (got_limits, got_battery) = if mode == MANUAL_MODE {
            (
                manual_use_throttle_limits(false, false),
                manual_use_battery_compensation(),
            )
        } else {
            (
                use_throttle_limits(&context),
                use_battery_compensation(&context),
            )
        };

        assert_eq!(
            got_limits, want_limits,
            "row {idx} (mode {mode}): throttle limits, context {context:?}"
        );
        assert_eq!(
            got_battery, want_battery,
            "row {idx} (mode {mode}): battery compensation, context {context:?}"
        );

        if want_limits != want_battery {
            disagreements += 1;
            manual_seen.insert(mode);
        }
    }

    // The two functions must actually disagree somewhere in the recording, or
    // it would pass just as happily against a port that merged them.
    assert!(
        disagreements > 0,
        "the two predicates never disagree, so nothing here would catch them \
         being merged"
    );
    assert_eq!(
        manual_seen.iter().copied().collect::<Vec<_>>(),
        MANUAL_THROTTLE_MODES,
        "the modes where they disagree are not the five manual-throttle modes"
    );

    println!(
        "{} rows, {disagreements} where the two predicates disagree, on modes \
         {manual_seen:?}",
        rows.len()
    );
}

/// The two predicates differ in a manual-throttle mode, and the difference is
/// deliberate.
///
/// Battery compensation is always off there; the limits are off only when
/// `THR_PASS_STAB` asks for a direct mapping. A pilot flying on the stick
/// expects a stick position to mean a throttle position, and silently
/// rescaling it as the battery sags would change what the same stick does as
/// the flight went on. The configured limits are a different matter — the
/// pilot has to ask for those to be bypassed.
#[test]
fn a_manual_throttle_mode_keeps_its_limits_but_not_its_compensation() {
    let manual = ThrottleContext {
        nav_scripting_active: false,
        manual_throttle_mode: true,
        throttle_passthru_stabilize: false,
        guided_throttle_passthru: false,
        in_vtol_mode: false,
        allow_forward_throttle_in_vtol: false,
    };

    assert!(
        use_throttle_limits(&manual),
        "the configured limits should still apply"
    );
    assert!(
        !use_battery_compensation(&manual),
        "the stick should mean the same throttle as the battery sags"
    );

    // And asking for a direct mapping drops the limits too.
    let passthru = ThrottleContext {
        throttle_passthru_stabilize: true,
        ..manual
    };
    assert!(!use_throttle_limits(&passthru));
    assert!(!use_battery_compensation(&passthru));
}

/// In a VTOL mode the two differ again: compensation is off unconditionally,
/// the limits defer to the quadplane.
///
/// Not reachable in the recording — no quadplane is configured — so it is
/// pinned here and labelled.
#[test]
fn a_vtol_mode_defers_its_limits_but_not_its_compensation() {
    let vtol = ThrottleContext {
        nav_scripting_active: false,
        manual_throttle_mode: false,
        throttle_passthru_stabilize: false,
        guided_throttle_passthru: false,
        in_vtol_mode: true,
        allow_forward_throttle_in_vtol: true,
    };

    assert!(
        use_throttle_limits(&vtol),
        "the quadplane allowed forward throttle, so the limits apply"
    );
    assert!(!use_battery_compensation(&vtol));

    let no_forward = ThrottleContext {
        allow_forward_throttle_in_vtol: false,
        ..vtol
    };
    assert!(!use_throttle_limits(&no_forward));
    assert!(!use_battery_compensation(&no_forward));
}

/// A running script suppresses both, before anything else is consulted.
///
/// Not reachable in the recording either: it needs a Lua script actually
/// flying the aircraft.
#[test]
fn a_running_script_suppresses_both() {
    // Every other input set to what would otherwise turn them on.
    let scripted = ThrottleContext {
        nav_scripting_active: true,
        manual_throttle_mode: false,
        throttle_passthru_stabilize: false,
        guided_throttle_passthru: false,
        in_vtol_mode: false,
        allow_forward_throttle_in_vtol: true,
    };
    assert!(!use_throttle_limits(&scripted));
    assert!(!use_battery_compensation(&scripted));

    // Without the script, the same state turns both on — so the script is
    // doing the work rather than the rest failing to.
    let unscripted = ThrottleContext {
        nav_scripting_active: false,
        ..scripted
    };
    assert!(use_throttle_limits(&unscripted));
    assert!(use_battery_compensation(&unscripted));
}

/// System identification runs only when the fixed wing alone is answering.
///
/// It injects deliberate disturbances to measure the airframe's response.
/// Every rejection is a phase where something other than the wing would
/// answer — the ground, or the VTOL motors — so the measurement would
/// describe something that is not what it claims to.
#[test]
fn system_identification_needs_the_wing_alone() {
    let flying = SystemIdContext {
        mode_supports: true,
        taking_off: false,
        landing: false,
        quadplane_available: false,
        in_assisted_flight: false,
        transition_complete: true,
    };
    assert!(allow_fw_systemid(&flying));

    for (what, context) in [
        (
            "mode does not support it",
            SystemIdContext {
                mode_supports: false,
                ..flying
            },
        ),
        (
            "taking off",
            SystemIdContext {
                taking_off: true,
                ..flying
            },
        ),
        (
            "landing",
            SystemIdContext {
                landing: true,
                ..flying
            },
        ),
        (
            "VTOL motors assisting",
            SystemIdContext {
                quadplane_available: true,
                in_assisted_flight: true,
                ..flying
            },
        ),
        (
            "still transitioning",
            SystemIdContext {
                quadplane_available: true,
                transition_complete: false,
                ..flying
            },
        ),
    ] {
        assert!(
            !allow_fw_systemid(&context),
            "system ID was allowed while {what}"
        );
    }

    // The quadplane conditions are only consulted when there is a quadplane,
    // so a fixed wing is not refused for a transition it cannot have.
    assert!(allow_fw_systemid(&SystemIdContext {
        quadplane_available: false,
        in_assisted_flight: true,
        transition_complete: false,
        ..flying
    }));
}

/// The vertical throttle's manual flag is the *negation* of the forward
/// throttle's automatic one.
///
/// Only on a tailsitter that has fully transitioned to Q-assisted forward
/// flight, where the forward throttle directly drives the vertical one.
/// Upstream's own comment flags the confusion: the forward throttle asks
/// `does_auto_throttle` and the vertical asks `is_vtol_man_throttle`, and the
/// two booleans mean opposite things — so this returns the negation rather
/// than a copy.
#[test]
fn the_vertical_throttle_flag_is_inverted_not_copied() {
    assert!(
        is_vtol_man_throttle(true, true, false),
        "a manual forward throttle means a manual vertical throttle"
    );
    assert!(
        !is_vtol_man_throttle(true, true, true),
        "an automatic forward throttle means an automatic vertical one — a \
         copy rather than a negation would get this backwards"
    );

    // And it is false everywhere else, whatever the forward throttle says.
    for auto_throttle in [false, true] {
        assert!(!is_vtol_man_throttle(false, true, auto_throttle));
        assert!(!is_vtol_man_throttle(true, false, auto_throttle));
        assert!(!is_vtol_man_throttle(false, false, auto_throttle));
    }
}

/// MANUAL replaces both predicates rather than being listed among the
/// manual-throttle modes.
///
/// In MANUAL the stick is the output, so neither the configured limits nor a
/// battery correction applies — the same stick must mean the same throttle
/// for the whole flight. The exception is a quadplane with IDLE_GOV_MANUAL,
/// where the idle governor needs the limits to hold the motor at idle rather
/// than letting a closed stick stop it.
#[test]
fn manual_replaces_both_predicates() {
    assert!(!manual_use_throttle_limits(false, false));
    assert!(!manual_use_throttle_limits(true, false));
    assert!(
        !manual_use_throttle_limits(false, true),
        "the option means nothing without a quadplane to apply it"
    );
    assert!(
        manual_use_throttle_limits(true, true),
        "the idle governor needs the limits"
    );

    // Battery compensation has no such exception.
    assert!(!manual_use_battery_compensation());

    // And MANUAL is not in the base's manual-throttle list, which is why the
    // override is needed rather than an entry there.
    assert!(
        !MANUAL_THROTTLE_MODES.contains(&MANUAL_MODE),
        "MANUAL should not be in the base's list; it overrides instead"
    );
}
