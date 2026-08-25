//! The slope landing's stage predicates against the real ArduPlane firmware.
//!
//! The first parity test linked against ArduPlane rather than ArduCopter.
//! `AP_Landing` exists in only one of the two binaries.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_landing::slope_stage::{target_airspeed_cm, LandingAirspeedParams, SlopeStage};

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
