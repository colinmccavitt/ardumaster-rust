//! The slope landing's stage predicates against the real ArduPlane firmware.
//!
//! The first parity test linked against ArduPlane rather than ArduCopter.
//! `AP_Landing` exists in only one of the two binaries.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_landing::slope_stage::{
    abort_decision, should_recalculate_slope, target_airspeed_cm, AbortDecision, FlareConfig,
    LandingAirspeedParams, RangefinderState, SlopeStage, TransitionInputs,
};

/// Bit-exact float from the fixture's `%u` column.
fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/slope_stage.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out = Vec::new();
    let mut current = "";
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag;
            continue;
        }
        if line.is_empty() || line.chars().next().is_some_and(char::is_alphabetic) {
            continue;
        }
        if current == section {
            out.push(line.split(',').map(str::to_owned).collect());
        }
    }
    out
}

fn stage_of(n: u8) -> SlopeStage {
    match n {
        0 => SlopeStage::Normal,
        1 => SlopeStage::Approach,
        2 => SlopeStage::Preflare,
        3 => SlopeStage::Final,
        other => panic!("unknown stage {other}"),
    }
}

/// The five stage predicates, on every stage.
///
/// They overlap deliberately. `is_on_final` covers preflare *and* final —
/// committed, past the point where the approach would be flown again — while
/// `is_on_approach` covers approach *and* preflare. Preflare is in both, and
/// the two are not a partition: it is simultaneously the end of the approach
/// and the beginning of the commitment, and callers of each ask different
/// questions about it.
///
/// The recording carries both the private `type_slope_*` predicates and the
/// public wrappers, which must agree once a landing is in progress. That
/// second set is what pins the type dispatch.
#[test]
fn the_stage_predicates_match_upstream() {
    let rows = rows("predicates");
    assert_eq!(rows.len(), 4, "expected one row per stage");

    for r in &rows {
        assert_eq!(r.len(), 11, "malformed predicates row");
        let n: u8 = r[0].parse().expect("stage");
        let stage = stage_of(n);

        let got = [
            stage.is_flaring(),
            stage.is_on_final(),
            stage.is_on_approach(),
            stage.is_expecting_impact(),
            stage.is_complete(),
        ];
        let labels = [
            "is_flaring",
            "is_on_final",
            "is_on_approach",
            "is_expecting_impact",
            "is_complete",
        ];

        for (i, label) in labels.iter().enumerate() {
            let want = r[1 + i].trim() == "1";
            assert_eq!(got[i], want, "stage {n} {label}");

            // And the public wrapper, which reaches the same answer through
            // the landing-type dispatch.
            let want_public = r[6 + i].trim() == "1";
            assert_eq!(
                want, want_public,
                "stage {n} {label}: the private predicate and its public \
                 wrapper disagree in the recording, which means the type \
                 dispatch is not reaching the slope implementation"
            );
        }
    }

    // The overlap is the point; if preflare were in only one of the two, a
    // port could implement either as the other.
    assert!(
        SlopeStage::Preflare.is_on_final() && SlopeStage::Preflare.is_on_approach(),
        "preflare must be both on-final and on-approach"
    );
    assert!(
        !SlopeStage::Final.is_on_approach(),
        "final must not be on-approach"
    );

    println!(
        "{} stages, all five predicates and their public wrappers agree",
        rows.len()
    );
}

/// The roll constraint, which applies only at the flare.
///
/// This is the one place the stage machine reaches into the attitude command.
/// Wings level matters more than tracking at the moment of touchdown: a wing
/// down puts a tip into the ground.
#[test]
fn the_flare_roll_constraint_matches_upstream() {
    let rows = rows("roll");
    assert!(!rows.is_empty(), "no roll rows");

    let mut clamped = 0_usize;
    let mut passed_through = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 4, "malformed roll row");
        let n: u8 = r[0].parse().expect("stage");
        let desired: i32 = r[1].parse().expect("desired");
        let limit: i32 = r[2].parse().expect("limit");
        let want: i32 = r[3].parse().expect("out");

        let got = stage_of(n).constrain_roll(desired, limit);
        assert_eq!(got, want, "stage {n}, desired {desired}, limit {limit}");

        if got == desired {
            passed_through += 1;
        } else {
            clamped += 1;
        }
    }

    // Both outcomes must appear, or the test passes with the constraint
    // applied always or never.
    assert!(
        clamped > 0 && passed_through > 0,
        "the constraint must both bind and not ({clamped} clamped, \
         {passed_through} passed through)"
    );

    // And it must bind only at the flare.
    for stage in [
        SlopeStage::Normal,
        SlopeStage::Approach,
        SlopeStage::Preflare,
    ] {
        assert_eq!(
            stage.constrain_roll(9000, 500),
            9000,
            "{stage:?} must not constrain roll"
        );
    }
    assert_eq!(SlopeStage::Final.constrain_roll(9000, 500), 500);

    println!(
        "{} roll rows, clamped on {clamped}, passed through on {passed_through}",
        rows.len()
    );
}

/// The landing target airspeed, per stage.
///
/// # Two inputs the harness cannot drive
///
/// TECS reports no landing airspeed (−1) and the AHRS no head wind (0) in
/// every recorded row. So the recording covers the *fallback* base speed and
/// none of the TECS branch, and the head-wind term contributes zero
/// throughout — which also makes the `wind_comp` sweep inert. Both are covered
/// by [`the_tecs_airspeed_and_head_wind_paths_behave`] instead, and recorded
/// here rather than left to be discovered.
///
/// What the recording does cover: the per-stage overrides, the pre-flare
/// substitution, and the ceiling chosen by the options bit.
#[test]
fn the_landing_airspeed_matches_upstream() {
    let rows = rows("airspeed");
    assert!(!rows.is_empty(), "no airspeed rows");

    let mut checked = 0_usize;
    let mut distinct = std::collections::BTreeSet::new();

    for r in &rows {
        assert_eq!(r.len(), 10, "malformed airspeed row");
        let stage = stage_of(r[0].trim().parse().expect("stage"));

        let params = LandingAirspeedParams {
            airspeed_cruise_ms: r[1].trim().parse::<f32>().expect("cruise"),
            airspeed_min_ms: r[2].trim().parse::<f32>().expect("min"),
            airspeed_max_ms: r[3].trim().parse::<f32>().expect("max"),
            land_airspeed_ms: f(&r[4]),
            pre_flare_airspeed_ms: f(&r[5]),
            wind_comp_pct: f(&r[6]),
            allow_max_airspeed: r[7].trim() == "1",
        };

        let got = target_airspeed_cm(stage, &params, f(&r[8]));
        let want: i32 = r[9].trim().parse().expect("out");
        assert_eq!(got, want, "stage {:?}, params {params:?}", stage);
        checked += 1;
        distinct.insert(got);
    }

    assert!(
        distinct.len() > 4,
        "only {} distinct airspeeds across {} rows",
        distinct.len(),
        rows.len()
    );

    println!(
        "{} airspeed rows, {checked} values, {} distinct results",
        rows.len(),
        distinct.len()
    );
}

/// The two paths the recording pins.
///
/// TECS's landing airspeed wins when it is set. Otherwise the fallback is the
/// *mean* of cruise and minimum — not cruise, and not minimum. Landing wants
/// slower than cruise for a shorter roll-out, but a margin above minimum
/// because an approach is exactly where a stall is unrecoverable.
///
/// The head-wind term can only ever add: the final constrain's lower bound is
/// the target itself. In a tail wind the head wind goes negative, and without
/// that floor the aircraft would be told to fly slower than its landing speed
/// on the approach where it already has the least margin.
#[test]
fn the_tecs_airspeed_and_head_wind_paths_behave() {
    let base = LandingAirspeedParams {
        airspeed_cruise_ms: 22.0,
        airspeed_min_ms: 12.0,
        airspeed_max_ms: 30.0,
        land_airspeed_ms: -1.0,
        pre_flare_airspeed_ms: 0.0,
        wind_comp_pct: 50.0,
        allow_max_airspeed: true,
    };

    // Unset: the mean of cruise and minimum, so 17 m/s.
    let fallback = target_airspeed_cm(SlopeStage::Approach, &base, 0.0);
    assert_eq!(
        fallback, 1700,
        "the fallback is the mean, not either endpoint"
    );

    // Set: TECS wins outright.
    let with_tecs = LandingAirspeedParams {
        land_airspeed_ms: 19.0,
        ..base
    };
    assert_eq!(
        target_airspeed_cm(SlopeStage::Approach, &with_tecs, 0.0),
        1900,
        "a set landing airspeed must win"
    );

    // Zero is set, not unset — the test is `>= 0`.
    let zero_tecs = LandingAirspeedParams {
        land_airspeed_ms: 0.0,
        ..base
    };
    assert_eq!(
        target_airspeed_cm(SlopeStage::Approach, &zero_tecs, 0.0),
        0,
        "zero is a set landing airspeed, not an absent one"
    );

    // Head wind adds half of itself at 50 percent.
    assert_eq!(
        target_airspeed_cm(SlopeStage::Approach, &base, 8.0),
        1700 + 400,
        "half of an eight metre head wind is four"
    );

    // Tail wind must not subtract.
    assert_eq!(
        target_airspeed_cm(SlopeStage::Approach, &base, -8.0),
        1700,
        "a tail wind must not reduce the approach speed below the target"
    );

    // And the compensation percentage is itself bounded to 0..100.
    let over = LandingAirspeedParams {
        wind_comp_pct: 400.0,
        ..base
    };
    assert_eq!(
        target_airspeed_cm(SlopeStage::Approach, &over, 8.0),
        1700 + 800,
        "the compensation is capped at the whole head wind, not four times it"
    );
}

/// The ceiling can sit below the target, and the constrain must not be a clamp.
///
/// Reachable: a TECS landing airspeed above cruise, with the maximum not
/// allowed on landing, puts the target above its own ceiling. `i32::clamp` is
/// ill-formed with crossed bounds; upstream's `constrain_int32` is not, and
/// which bound it returns depends on the wind.
///
/// With no wind the amount equals the target, which is the low bound, so the
/// low test does not fire and the ceiling wins. Only a tail wind pushes the
/// sum below the target — and then the floor returns the target, which is the
/// whole reason the floor is the target rather than zero.
#[test]
fn a_target_above_its_ceiling_is_constrained_not_clamped() {
    let params = LandingAirspeedParams {
        airspeed_cruise_ms: 15.0,
        airspeed_min_ms: 10.0,
        airspeed_max_ms: 30.0,
        // Above cruise, which is the ceiling when the option is off.
        land_airspeed_ms: 25.0,
        pre_flare_airspeed_ms: 0.0,
        wind_comp_pct: 50.0,
        allow_max_airspeed: false,
    };

    // No wind: the amount equals the low bound, so the ceiling binds.
    assert_eq!(
        target_airspeed_cm(SlopeStage::Approach, &params, 0.0),
        1500,
        "with the bounds crossed and no wind, the ceiling wins"
    );

    // Tail wind: the sum drops below the target, and the floor returns it.
    assert_eq!(
        target_airspeed_cm(SlopeStage::Approach, &params, -10.0),
        2500,
        "a tail wind must not reduce the speed below the target, even when \
         the target is above its own ceiling"
    );

    // With the option on the ceiling is the maximum and nothing binds.
    let allowed = LandingAirspeedParams {
        allow_max_airspeed: true,
        ..params
    };
    assert_eq!(
        target_airspeed_cm(SlopeStage::Approach, &allowed, 0.0),
        2500
    );
}

/// The stage machine, over 86016 recorded transitions.
///
/// All ten reachable stage pairs appear, including Normal straight to
/// Preflare in a single call — which only happens because the flare test
/// reads the stage *after* the approach transition, so a leg that enters the
/// approach this cycle can also leave it this cycle.
///
/// # Three inputs the harness cannot drive
///
/// The navigation controller reports a stale solution in every recorded row,
/// which makes `heading_lined_up` and `on_flight_line` false throughout. So
/// every Normal-to-Approach transition here came through the
/// `wp_proportion > 0.5` backstop, and the two quality paths are not covered.
/// Neither is the loiter-to-altitude entry, since the mission has no previous
/// command. All three have port-side tests in
/// [`the_uncovered_approach_entries_behave`].
#[test]
fn the_stage_transitions_match_upstream() {
    let rows = rows("transition");
    assert!(!rows.is_empty(), "no transition rows");

    let mut seen: std::collections::BTreeSet<(u8, u8)> = Default::default();

    for r in &rows {
        assert_eq!(r.len(), 18, "malformed transition row");
        let from_n: u8 = r[0].trim().parse().expect("from");
        let to_n: u8 = r[17].trim().parse().expect("to");

        let inp = TransitionInputs {
            wp_proportion: f(&r[1]),
            height: f(&r[2]),
            sink_rate: f(&r[3]),
            bearing_error_cd: r[12].trim().parse().expect("bearing"),
            crosstrack_error_m: f(&r[13]),
            nav_data_is_stale: r[14].trim() == "1",
            below_prev_wp: r[16].trim() == "1",
            prev_cmd_is_loiter_to_alt: r[15].trim() == "1",
            rangefinder_in_range: r[9].trim() == "1",
            is_flying: r[10].trim() == "1",
            crash_detection_enable: r[11].trim() == "1",
        };
        let cfg = FlareConfig {
            flare_alt: f(&r[4]),
            flare_sec: f(&r[5]),
            pre_flare_alt: f(&r[6]),
            pre_flare_sec: f(&r[7]),
            pre_flare_airspeed: f(&r[8]),
        };

        let got = stage_of(from_n).next(&inp, &cfg);
        assert_eq!(
            got,
            stage_of(to_n),
            "from stage {from_n}: inputs {inp:?}, config {cfg:?}"
        );

        seen.insert((from_n, to_n));
    }

    // Every reachable pair must appear, or the machine could be wrong in a
    // direction the sweep never asks about.
    assert_eq!(
        seen.len(),
        10,
        "expected all ten reachable stage pairs, saw {seen:?}"
    );
    assert!(
        seen.contains(&(0, 2)),
        "Normal straight to Preflare must appear; it is what shows the flare \
         test reads the stage after the approach transition"
    );

    println!(
        "{} transitions, all {} reachable stage pairs covered",
        rows.len(),
        seen.len()
    );
}

/// The three ways into the approach that the recording cannot reach.
///
/// The harness navigation solution is stale on every row, so `heading_lined_up`
/// and `on_flight_line` are false throughout and only the `wp_proportion > 0.5`
/// backstop fires. The mission has no previous command, so the
/// loiter-to-altitude entry is never taken either.
///
/// That backstop is worth its own note. It has no quality test at all: past
/// the midpoint the aircraft is committed whether or not it ever lined up, and
/// refusing to enter the approach would leave it descending with the approach
/// logic switched off.
#[test]
fn the_uncovered_approach_entries_behave() {
    let base = TransitionInputs {
        // Below the backstop, so only the quality paths can fire.
        wp_proportion: 0.3,
        height: 100.0,
        sink_rate: 1.0,
        bearing_error_cd: 0,
        crosstrack_error_m: 0.0,
        nav_data_is_stale: true,
        below_prev_wp: false,
        prev_cmd_is_loiter_to_alt: false,
        rangefinder_in_range: true,
        is_flying: true,
        crash_detection_enable: false,
    };
    let cfg = FlareConfig {
        flare_alt: 3.0,
        flare_sec: 2.0,
        pre_flare_alt: 0.0,
        pre_flare_sec: 0.0,
        pre_flare_airspeed: 0.0,
    };

    // Stale navigation: neither quality path fires.
    assert_eq!(SlopeStage::Normal.next(&base, &cfg), SlopeStage::Normal);

    // Fresh, lined up and on the flight line: in.
    let fresh = TransitionInputs {
        nav_data_is_stale: false,
        ..base
    };
    assert_eq!(SlopeStage::Normal.next(&fresh, &cfg), SlopeStage::Approach);

    // Lined up but off the flight line, and not below the previous waypoint:
    // still out.
    let off_line = TransitionInputs {
        crosstrack_error_m: 20.0,
        ..fresh
    };
    assert_eq!(SlopeStage::Normal.next(&off_line, &cfg), SlopeStage::Normal);

    // Off the line but below the previous waypoint, past fifteen percent: in.
    let descending = TransitionInputs {
        below_prev_wp: true,
        ..off_line
    };
    assert_eq!(
        SlopeStage::Normal.next(&descending, &cfg),
        SlopeStage::Approach
    );

    // ...but not before fifteen percent.
    let too_early = TransitionInputs {
        wp_proportion: 0.1,
        ..descending
    };
    assert_eq!(
        SlopeStage::Normal.next(&too_early, &cfg),
        SlopeStage::Normal
    );

    // A loiter-to-altitude beforehand counts on its own, stale or not: the
    // aircraft has already been positioned deliberately.
    let loitered = TransitionInputs {
        prev_cmd_is_loiter_to_alt: true,
        ..base
    };
    assert_eq!(
        SlopeStage::Normal.next(&loitered, &cfg),
        SlopeStage::Approach
    );

    // And the heading test needs fresh data even when the error is zero.
    let stale_but_aligned = TransitionInputs {
        crosstrack_error_m: 0.0,
        below_prev_wp: true,
        wp_proportion: 0.3,
        ..base
    };
    assert_eq!(
        SlopeStage::Normal.next(&stale_but_aligned, &cfg),
        SlopeStage::Normal,
        "a stale solution must not satisfy the heading test however good it looks"
    );
}

/// Whether a rangefinder correction should trigger a slope recalculation.
///
/// Compared against whether the firmware's slope actually moved, which is the
/// observable consequence of the decision.
///
/// The test is on the change in the *magnitude* of the correction, not on the
/// change in the correction itself — so a swing from +3 to −3, six metres of
/// reversal, registers as no change at all, while +3 to +6 triggers. That is
/// deliberate: the recalculation handles the barometer being wrong, and how
/// wrong it is does not depend on the sign. The sign is the abort's business.
#[test]
fn the_slope_recalculation_trigger_matches_upstream() {
    let rows = rows("rangefinder");
    assert!(!rows.is_empty(), "no rangefinder rows");

    let mut fired = 0_usize;
    let mut held = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 13, "malformed rangefinder row");
        let idx: usize = r[0].parse().expect("idx");

        let state = RangefinderState {
            in_use: r[1].trim() == "1",
            correction: f(&r[2]),
            last_stable_correction: f(&r[3]),
        };
        let shallow = f(&r[4]);

        let want = (f(&r[9]) - f(&r[7])).abs() > 1e-9;
        let got = should_recalculate_slope(&state, shallow);
        assert_eq!(
            got,
            want,
            "row {idx}: state {state:?}, threshold {shallow} — the firmware's \
             slope {} but the port {}",
            if want { "moved" } else { "held" },
            if got {
                "would recalculate"
            } else {
                "would not"
            }
        );

        if want {
            fired += 1;
        } else {
            held += 1;
        }
    }

    assert!(
        fired > 100 && held > 100,
        "the trigger must both fire and hold ({fired} fired, {held} held)"
    );

    println!(
        "{} rangefinder rows, recalculated on {fired}, held on {held}",
        rows.len()
    );
}

/// The abort decision, which the recording cannot reach.
///
/// The harness vehicle reports zero adjusted altitude, so the corrected
/// altitude the recalculation derives comes out at or below the landing
/// point and the recomputed slope is *negative* on every recorded row.
/// `new_slope_deg - initial_slope_deg` is therefore never positive and cannot
/// exceed a positive threshold, so the abort branch never runs there.
///
/// It is the branch that matters most, so it is checked here directly against
/// what upstream's source says.
#[test]
fn the_abort_decision_behaves() {
    // A positive correction means the aircraft is LOWER than the barometer
    // said, so the new slope is shallower — always flyable, always continue.
    // Upstream says so in a comment and takes no action at all.
    assert_eq!(
        abort_decision(5.0, 0.5, 0.0, 1.0, false),
        AbortDecision::Continue,
        "a shallower slope is always flyable"
    );

    // Negative correction, steep enough, not yet aborted: go around, carrying
    // the offset so the next approach starts from the corrected altitude.
    assert_eq!(
        abort_decision(-5.0, 0.5, 0.0, 1.0, false),
        AbortDecision::GoAround { alt_offset: -5.0 },
        "a much steeper slope should abort"
    );

    // Not steep enough: continue.
    assert_eq!(
        abort_decision(-5.0, 0.02, 0.0, 20.0, false),
        AbortDecision::Continue,
        "a slope within the threshold should be flown"
    );

    // Already aborted once: never again. A go-around triggered this way
    // records its offset, so the next approach should not have the same
    // problem — and if it somehow does, aborting again would loop forever
    // without ever landing.
    assert_eq!(
        abort_decision(-5.0, 0.5, 0.0, 1.0, true),
        AbortDecision::Continue,
        "the abort latches; it must only happen once"
    );

    // A non-positive threshold disables it.
    assert_eq!(
        abort_decision(-5.0, 0.5, 0.0, 0.0, false),
        AbortDecision::Continue,
        "a zero threshold disables the abort"
    );

    // The comparison is between slope ANGLES, so the threshold means the same
    // thing at any approach angle. Two slopes differing by 0.1 are 5.7 degrees
    // apart near level and under 2 degrees apart at a steep angle.
    let shallow_pair = abort_decision(-1.0, 0.1, 0.0, 3.0, false);
    let steep_pair = abort_decision(-1.0, 1.1, 1.0, 3.0, false);
    assert_eq!(
        shallow_pair,
        AbortDecision::GoAround { alt_offset: -1.0 },
        "0.1 against 0.0 is 5.7 degrees, past a 3 degree threshold"
    );
    assert_eq!(
        steep_pair,
        AbortDecision::Continue,
        "the same 0.1 of slope near 45 degrees is under 2 degrees, and must \
         not abort — which is why the threshold is in degrees"
    );
}
