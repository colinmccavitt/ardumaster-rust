//! Full rangefinder bump against the real ArduPlane firmware recording.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted"
)]

use ap_landing::go_around::{LandingFlags, SlopeLandingFlags};
use ap_landing::rangefinder_bump::{
    adjust_landing_slope_for_rangefinder_bump, RangefinderBumpConfig, RangefinderBumpInputs,
    RangefinderBumpState,
};
use ap_landing::slope_stage::RangefinderState;
use ap_landing::{SlopeConfig, SlopeInputs};
use ap_math::location::{AltContext, AltFrame, Location};

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn rows() -> Vec<Vec<String>> {
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
        if current == "rangefinder" {
            out.push(line.split(',').map(str::to_owned).collect());
        }
    }
    out
}

fn harness_locations() -> (Location, Location, Location) {
    let prev = Location::new_with_alt(-353_632_621, 1_491_652_374, 12_000, AltFrame::Absolute);
    let next = Location::new_with_alt(-353_600_000, 1_491_700_000, 4_000, AltFrame::Absolute);
    let cur = Location::new_with_alt(-353_620_000, 1_491_680_000, 8_000, AltFrame::Absolute);
    (prev, next, cur)
}

fn harness_slope_inputs(prev: Location, next: Location, cur: Location) -> SlopeInputs {
    SlopeInputs {
        prev_wp: prev,
        next_wp: next,
        current: cur,
        groundspeed: 0.5,
        land_sinkrate: 0.25,
        alt_ctx: AltContext {
            home_alt_cm: Some(0),
            origin_alt_cm: Some(0),
            terrain_alt_cm: Some(0),
        },
    }
}

/// The full bump path: slope recalculation, go-around, and abort latch.
#[test]
fn the_rangefinder_bump_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no rangefinder rows");

    let (prev, next, cur) = harness_locations();
    let alt_ctx = AltContext {
        home_alt_cm: Some(0),
        origin_alt_cm: Some(0),
        terrain_alt_cm: Some(0),
    };
    let slope_cfg = SlopeConfig {
        flare_sec: 2.0,
        flare_alt: 3.0,
        flare_effectivness_pct: 50,
    };
    let slope_inp = harness_slope_inputs(prev, next, cur);

    let mut checked = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 13, "malformed rangefinder row");
        let idx: usize = r[0].parse().expect("idx");

        let rf = RangefinderState {
            in_use: r[1].trim() == "1",
            correction: f(&r[2]),
            last_stable_correction: f(&r[3]),
        };
        let bump_cfg = RangefinderBumpConfig {
            shallow_threshold: f(&r[4]),
            steep_threshold_deg: f(&r[5]),
        };
        let had_aborted = r[6].trim() == "1";
        let slope_before = f(&r[7]);
        let initial_slope = f(&r[8]);
        let want_slope_after = f(&r[9]);
        let want_go_around = r[10].trim() == "1";
        let want_alt_offset = f(&r[11]);
        let want_aborted_after = r[12].trim() == "1";

        let mut state = RangefinderBumpState {
            slope: slope_before,
            initial_slope,
            landing: LandingFlags::default(),
            slope_flags: SlopeLandingFlags {
                has_aborted_due_to_slope_recalc: had_aborted,
                alt_offset: 0.0,
            },
            rf,
        };

        let bump_inp = RangefinderBumpInputs {
            rf,
            prev_wp: prev,
            next_wp: next,
            current: cur,
            wp_distance_m: 300.0,
            // Harness drives bump with offset_cm=0; vehicle adjusted alt is
            // not cur.alt (see tools/parity/gen_slope_stage.py).
            adjusted_altitude_cm: 0,
            alt_ctx,
        };

        let got = adjust_landing_slope_for_rangefinder_bump(
            &bump_cfg,
            &slope_cfg,
            &slope_inp,
            &mut state,
            &bump_inp,
        );

        let slope_moved = (got.slope - slope_before).abs() > 1e-9;
        let want_recalc = (want_slope_after - slope_before).abs() > 1e-9;
        assert_eq!(slope_moved, want_recalc, "row {idx}: slope recalc");

        assert_eq!(got.go_around, want_go_around, "row {idx}: go_around");
        assert!(
            (got.alt_offset - want_alt_offset).abs() < 1e-4,
            "row {idx}: alt_offset got {} want {}",
            got.alt_offset,
            want_alt_offset
        );
        assert_eq!(
            state.slope_flags.has_aborted_due_to_slope_recalc,
            want_aborted_after,
            "row {idx}: aborted_after"
        );
        checked += 1;
    }

    println!("{checked} rangefinder bump rows matched upstream");
}
