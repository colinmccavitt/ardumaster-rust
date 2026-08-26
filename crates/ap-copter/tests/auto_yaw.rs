//! Copter's AutoYaw mode machine, against the real firmware.
//!
//! Every source the rate can come from is given a distinct value in the
//! recording, so the four are separable. An earlier version left the position
//! controller at zero, which made "reads the position controller"
//! indistinguishable from "assigns zero" — a port confusing those two would
//! have passed.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::auto_yaw::{
    default_yaw_mode, yaw_mode_entry, yaw_rate_source, WpYawBehaviour, YawMode, YawModeEntry,
    YawRateSource,
};

/// The values the harness put on each rate source.
const PILOT_RATE: f32 = 1.75;
const POS_CONTROL_RATE: f32 = 0.875;
const SENTINEL_RATE: f32 = -3.25;

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

fn mode(s: &str) -> YawMode {
    let n: u8 = s.trim().parse().expect("mode number");
    YawMode::from_number(n).unwrap_or_else(|| panic!("unknown recorded yaw mode {n}"))
}

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/copter_auto_yaw.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    let mut out = Vec::new();
    let mut current = "";
    for line in text.lines() {
        if let Some(tag) = line.strip_prefix('#') {
            current = tag;
            continue;
        }
        if line.is_empty() || line.starts_with(|c: char| c.is_alphabetic()) {
            continue;
        }
        if current == section {
            out.push(line.split(',').map(str::to_owned).collect());
        }
    }
    out
}

/// `default_mode`, over every parameter value including out-of-range ones.
#[test]
fn the_default_yaw_mode_matches_upstream() {
    let rows = rows("default_mode");
    assert!(!rows.is_empty(), "no recorded rows");

    let mut out_of_range = 0_usize;
    for r in &rows {
        assert_eq!(r.len(), 3, "malformed row");
        let raw: i32 = r[0].trim().parse().expect("behaviour");
        let rtl = b(&r[1]);

        #[allow(
            clippy::cast_sign_loss,
            clippy::cast_possible_truncation,
            reason = "reproduces the harness's cast through the parameter's \
own signed type, which is how an out-of-range stored value reaches the switch"
        )]
        let number = raw as u8;

        let behaviour = WpYawBehaviour::from_number(number);
        let got = default_yaw_mode(behaviour, rtl);
        let want = mode(&r[2]);

        assert_eq!(
            got, want,
            "WP_YAW_BEHAVIOR {raw} rtl={rtl}: {got:?} against upstream {want:?}"
        );

        if !matches!(raw, 0..=3) {
            out_of_range += 1;
            assert_eq!(
                got,
                YawMode::LookAtNextWp,
                "an out-of-range behaviour should fall to the default arm"
            );
        }
    }
    assert!(out_of_range > 0, "no out-of-range value was recorded");
    println!("{} default-mode rows", rows.len());
}

/// Only RTL-aware behaviour treats a return differently.
#[test]
fn only_one_behaviour_cares_about_rtl() {
    for behaviour in [
        WpYawBehaviour::Never,
        WpYawBehaviour::LookAtNextWp,
        WpYawBehaviour::LookAhead,
    ] {
        assert_eq!(
            default_yaw_mode(behaviour, false),
            default_yaw_mode(behaviour, true),
            "{behaviour:?} should not depend on rtl"
        );
    }

    assert_eq!(
        default_yaw_mode(WpYawBehaviour::LookAtNextWpExceptRtl, false),
        YawMode::LookAtNextWp
    );
    assert_eq!(
        default_yaw_mode(WpYawBehaviour::LookAtNextWpExceptRtl, true),
        YawMode::Hold,
        "on the way home the last heading is kept rather than facing home"
    );
}

/// Every transition between all eleven modes.
#[test]
fn the_mode_entry_effects_match_upstream() {
    let rows = rows("set_mode");
    assert_eq!(
        rows.len(),
        121,
        "every pair of eleven modes should be swept"
    );

    let mut unchanged = 0_usize;
    let mut seeded = 0_usize;
    let mut zeroed = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 5, "malformed row");
        let from = mode(&r[0]);
        let to = mode(&r[1]);
        let look_ahead_changed = b(&r[2]);
        let rate_changed = b(&r[3]);

        let got = yaw_mode_entry(from, to);

        match got {
            None => {
                unchanged += 1;
                assert_eq!(from, to, "the port skipped a real transition");
                assert!(
                    !look_ahead_changed && !rate_changed,
                    "re-selecting {from:?} initialised something; upstream \
                     returns before the switch"
                );
            }
            Some(YawModeEntry::SeedLookAheadFromCurrentYaw) => {
                seeded += 1;
                assert!(
                    look_ahead_changed && !rate_changed,
                    "{from:?} -> {to:?} should seed the look-ahead heading only"
                );
            }
            Some(YawModeEntry::ZeroYawRate) => {
                zeroed += 1;
                assert!(
                    rate_changed && !look_ahead_changed,
                    "{from:?} -> {to:?} should zero the rate only"
                );
            }
            Some(YawModeEntry::Nothing) => {
                assert!(
                    !look_ahead_changed && !rate_changed,
                    "{from:?} -> {to:?} initialised something the port does not"
                );
            }
        }
    }

    assert_eq!(unchanged, 11, "one no-op transition per mode");
    assert_eq!(seeded, 10, "entering LOOK_AHEAD from each other mode");
    assert_eq!(zeroed, 10, "entering RATE from each other mode");
    println!(
        "{} transitions: {unchanged} no-ops, {seeded} seeded, {zeroed} zeroed",
        rows.len()
    );
}

/// Re-selecting a mode does not re-run its initialisation.
///
/// Upstream returns before the switch when the mode is unchanged. That is not
/// an optimisation: asking for RATE while already in RATE must leave the
/// commanded rate alone, and re-running the initialisation would stop a turn
/// that a `DO_CONDITIONAL_YAW` had started.
#[test]
fn reselecting_a_mode_is_not_a_transition() {
    for m in [
        YawMode::Hold,
        YawMode::LookAtNextWp,
        YawMode::Roi,
        YawMode::Fixed,
        YawMode::LookAhead,
        YawMode::ResetToArmedYaw,
        YawMode::AngleRate,
        YawMode::Rate,
        YawMode::Circle,
        YawMode::PilotRate,
        YawMode::Weathervane,
    ] {
        assert_eq!(
            yaw_mode_entry(m, m),
            None,
            "re-selecting {m:?} should not be a transition"
        );
    }

    // And the two modes with initialisation do run it on a real transition.
    assert_eq!(
        yaw_mode_entry(YawMode::Hold, YawMode::Rate),
        Some(YawModeEntry::ZeroYawRate)
    );
    assert_eq!(
        yaw_mode_entry(YawMode::Hold, YawMode::LookAhead),
        Some(YawModeEntry::SeedLookAheadFromCurrentYaw)
    );
}

/// Where each mode's yaw rate comes from.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "exactness is the assertion: these modes assign a literal zero, \
and a value merely near zero would mean the rate was computed rather than \
forced -- which is the distinction the test exists to make"
)]
fn the_yaw_rate_source_matches_upstream() {
    let rows = rows("rate");
    assert_eq!(rows.len(), 11, "one row per mode");

    let mut seen = std::collections::BTreeSet::new();
    for r in &rows {
        assert_eq!(r.len(), 3, "malformed row");
        let m = mode(&r[0]);
        let out = f(&r[1]);
        let assigned = b(&r[2]);

        let source = yaw_rate_source(m);
        seen.insert(format!("{source:?}"));

        match source {
            YawRateSource::Zero => {
                assert!(assigned, "{m:?}: upstream did not assign");
                assert_eq!(out, 0.0, "{m:?}: should be forced to zero");
            }
            YawRateSource::PositionController => {
                assert!(assigned, "{m:?}: upstream did not assign");
                assert!(
                    (out - POS_CONTROL_RATE).abs() < 1e-6,
                    "{m:?}: expected the position controller's rate, got {out}"
                );
            }
            YawRateSource::Pilot => {
                assert!(assigned, "{m:?}: upstream did not assign");
                assert!(
                    (out - PILOT_RATE).abs() < 1e-6,
                    "{m:?}: expected the pilot's rate, got {out}"
                );
            }
            YawRateSource::Unchanged => {
                assert!(
                    !assigned,
                    "{m:?}: the port expects the rate left alone, upstream \
                     assigned {out}"
                );
                assert!(
                    (out - SENTINEL_RATE).abs() < 1e-6,
                    "{m:?}: the stored rate should have survived, got {out}"
                );
            }
        }
    }

    assert_eq!(seen.len(), 4, "not every rate source is reached: {seen:?}");
    println!("11 modes across {} rate sources", seen.len());
}

/// Three modes leave the rate alone, and that is not the same as zero.
///
/// `ANGLE_RATE`, `RATE` and `WEATHERVANE` fall through upstream's switch
/// without assigning, so a rate set by a `DO_CONDITIONAL_YAW` command
/// survives to the next iteration. Reading those three as "no case, so zero"
/// would stop every commanded yaw turn dead on the iteration after it was
/// commanded.
#[test]
fn a_commanded_turn_is_not_cancelled_by_the_next_iteration() {
    for m in [YawMode::AngleRate, YawMode::Rate, YawMode::Weathervane] {
        assert_eq!(
            yaw_rate_source(m),
            YawRateSource::Unchanged,
            "{m:?} must not overwrite a commanded rate"
        );
    }

    // The angle-holding modes are the ones that force zero, because a
    // non-zero rate alongside an angle demand would fight it.
    for m in [
        YawMode::Hold,
        YawMode::Roi,
        YawMode::Fixed,
        YawMode::LookAhead,
        YawMode::ResetToArmedYaw,
        YawMode::Circle,
    ] {
        assert_eq!(yaw_rate_source(m), YawRateSource::Zero, "{m:?}");
    }
}

/// The mode numbers round-trip, and the pilot-excluded set is the documented
/// one.
#[test]
fn the_yaw_mode_numbers_round_trip() {
    for n in 0..=10_u8 {
        let m = YawMode::from_number(n).expect("mode should exist");
        assert_eq!(m.as_number(), n);
    }
    for n in 11..=u8::MAX {
        assert_eq!(YawMode::from_number(n), None, "{n} should not be a mode");
    }

    // Upstream marks these three "no pilot input accepted" in the enum's own
    // comments.
    for m in [YawMode::LookAtNextWp, YawMode::Roi, YawMode::Fixed] {
        assert!(m.is_pilot_excluded(), "{m:?}");
    }
    for m in [YawMode::Hold, YawMode::PilotRate, YawMode::Rate] {
        assert!(!m.is_pilot_excluded(), "{m:?}");
    }
}
