//! The glide-slope geometry against the real ArduPlane firmware.
//!
//! This closes the gap the first FW-029 slice recorded honestly: the upstream
//! function writes through the vehicle's callbacks rather than returning, so
//! verifying it needed a real vehicle. `plane_link` now provides one, and the
//! harness rebinds those two callbacks to recorders — which captures exactly
//! the values the port returns in its `SlopeResult`.
//!
//! # One coverage limit
//!
//! The harness AHRS reports zero ground speed, so upstream's
//! `groundspeed < 0.5 → 0.5` clamp is taken on every recorded row. The sink
//! time is therefore always computed from half a metre per second, and the
//! ground-speed *variation* is not covered. Everything downstream of it is.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_landing::{setup_landing_glide_slope, SlopeConfig, SlopeInputs};
use ap_math::location::{AltContext, AltFrame, Location};

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn i32_of(s: &str) -> i32 {
    s.trim().parse().expect("integer")
}

fn rows() -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/slope_geometry.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && l.chars().next().is_some_and(|c| c.is_ascii_digit()))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

/// The slope, the aim point, the altitude offset and the proportion.
///
/// Every one of these is an output the vehicle acts on: the aim point is where
/// it flies, the slope sets how steeply, the offset feeds the altitude
/// controller and the proportion says how far along it is. They are compared
/// together because they are computed together — the aim point cannot be
/// placed without first knowing the flare distance, which needs the sink rate,
/// which needs the slope.
#[test]
fn the_glide_slope_geometry_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no geometry rows");

    let mut checked = 0_usize;
    let mut distinct_slopes = std::collections::BTreeSet::new();

    for r in &rows {
        assert_eq!(r.len(), 21, "malformed geometry row");
        let idx: usize = r[0].parse().expect("idx");

        let cfg = SlopeConfig {
            flare_sec: f(&r[1]),
            flare_alt: f(&r[2]),
            flare_effectivness_pct: u8::try_from(i32_of(&r[3])).expect("percent"),
        };

        let inp = SlopeInputs {
            prev_wp: Location::new_with_alt(
                i32_of(&r[6]),
                i32_of(&r[7]),
                i32_of(&r[8]),
                AltFrame::Absolute,
            ),
            next_wp: Location::new_with_alt(
                i32_of(&r[9]),
                i32_of(&r[10]),
                i32_of(&r[11]),
                AltFrame::Absolute,
            ),
            current: Location::new_with_alt(
                i32_of(&r[12]),
                i32_of(&r[13]),
                i32_of(&r[14]),
                AltFrame::Absolute,
            ),
            groundspeed: f(&r[4]),
            land_sinkrate: f(&r[5]),
            alt_ctx: AltContext::default(),
        };

        // Zero forces upstream's first-calculation path, which the harness
        // also does before every row.
        let mut slope = 0.0_f32;
        let got = setup_landing_glide_slope(&cfg, &inp, &mut slope)
            .unwrap_or_else(|| panic!("row {idx}: the port declined to compute a slope"));

        for (label, value, want) in [
            ("slope", slope, f(&r[15])),
            ("altitude_proportion", got.altitude_proportion, f(&r[20])),
        ] {
            let diff = (value - want).abs();
            assert!(
                diff < 3e-5,
                "row {idx} {label}: {value} != upstream {want} (diff {diff})"
            );
            checked += 1;
        }

        for (label, value, want) in [
            (
                "target_altitude_offset_cm",
                got.target_altitude_offset_cm,
                i32_of(&r[16]),
            ),
            ("aim_lat", got.aim_point.lat, i32_of(&r[17])),
            ("aim_lng", got.aim_point.lng, i32_of(&r[18])),
            ("aim_alt", got.aim_point.alt, i32_of(&r[19])),
        ] {
            assert_eq!(value, want, "row {idx} {label}");
            checked += 1;
        }

        assert!(
            got.first_calculation,
            "row {idx}: the slope started at zero, so this is a first calculation"
        );

        distinct_slopes.insert(slope.to_bits());
    }

    // A sweep that produced one slope would pass with the geometry replaced by
    // a constant.
    assert!(
        distinct_slopes.len() > 20,
        "only {} distinct slopes across {} rows",
        distinct_slopes.len(),
        rows.len()
    );

    println!(
        "{} geometry rows, {checked} values, {} distinct slopes",
        rows.len(),
        distinct_slopes.len()
    );
}
