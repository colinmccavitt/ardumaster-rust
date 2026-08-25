//! The slope landing's stage predicates against the real ArduPlane firmware.
//!
//! The first parity test linked against ArduPlane rather than ArduCopter.
//! `AP_Landing` exists in only one of the two binaries.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_landing::slope_stage::SlopeStage;

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
