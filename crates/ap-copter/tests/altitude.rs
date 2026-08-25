//! The height-above-ground ladder, against the real firmware.
//!
//! The two upper sources are injected at the link boundary rather than
//! brought up — a rangefinder height needs a running estimator with an origin,
//! and an above-terrain altitude needs a loaded terrain database. The ladder
//! itself is the firmware's, unmodified. See
//! `tools/parity/gen_alt_above_ground.py`.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_copter::altitude::alt_above_ground_m;

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
        .map(|p| p.join("fixtures/alt_above_ground.csv"))
        .expect("workspace root");
    let text = std::fs::read_to_string(&path).expect("fixture");

    text.lines()
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter(|l| !l.starts_with(|c: char| c.is_alphabetic()))
        .map(|l| l.split(',').map(str::to_owned).collect())
        .collect()
}

#[test]
fn the_height_above_ground_matches_upstream() {
    let rows = rows();
    assert!(!rows.is_empty(), "no recorded rows");

    // Which rung each row landed on, so the sweep can be shown to reach all
    // four rather than assumed to.
    let (mut rf, mut uninit, mut terrain, mut flat) = (0_usize, 0, 0, 0);
    let mut distinct = std::collections::BTreeSet::new();

    for r in &rows {
        assert_eq!(r.len(), 8, "malformed row");
        let idx: usize = r[0].parse().expect("idx");

        let rangefinder = b(&r[1]).then(|| f(&r[2]));
        let initialised = b(&r[3]);
        let terrain_m = b(&r[4]).then(|| f(&r[5]));
        let alt_cm: i32 = r[6].trim().parse().expect("alt");

        let got = alt_above_ground_m(rangefinder, initialised, terrain_m, alt_cm);
        let want = f(&r[7]);

        assert!(
            (got - want).abs() < 1e-6,
            "row {idx}: {got} against upstream {want} — rangefinder \
             {rangefinder:?}, initialised {initialised}, terrain \
             {terrain_m:?}, alt {alt_cm} cm"
        );

        if rangefinder.is_some() {
            rf += 1;
        } else if !initialised {
            uninit += 1;
        } else if terrain_m.is_some() {
            terrain += 1;
        } else {
            flat += 1;
        }
        distinct.insert(want.to_bits());
    }

    assert!(
        rf > 0 && uninit > 0 && terrain > 0 && flat > 0,
        "the sweep does not reach every rung: rangefinder {rf}, \
         uninitialised {uninit}, terrain {terrain}, flat earth {flat}"
    );
    println!(
        "{} rows, {} distinct heights — {rf} rangefinder, {uninit} \
         uninitialised, {terrain} terrain, {flat} flat earth",
        rows.len(),
        distinct.len()
    );
}

/// The rangefinder wins over everything, including a position the vehicle
/// does not have.
///
/// This is the ordering decision that is easiest to get wrong: it would look
/// tidier to check `initialised` first and bail early, and that would throw
/// away a perfectly good distance measurement during startup — exactly when a
/// vehicle is most likely to be near the ground.
#[test]
#[allow(
    clippy::float_cmp,
    reason = "exactness is the assertion: the uninitialised rung returns a literal zero rather than a computed altitude, and a value near zero would mean it fell through to the flat-earth rung instead"
)]
fn a_rangefinder_reading_beats_an_unknown_position() {
    assert!(
        (alt_above_ground_m(Some(2.5), false, None, 90_000) - 2.5).abs() < 1e-9,
        "an uninitialised position discarded a rangefinder reading"
    );
    assert!(
        (alt_above_ground_m(Some(2.5), true, Some(40.0), 90_000) - 2.5).abs() < 1e-9,
        "terrain data outranked a rangefinder reading"
    );

    // And with no rangefinder, the same uninitialised position gives zero
    // rather than the flat-earth reading of that same altitude.
    assert_eq!(alt_above_ground_m(None, false, None, 90_000), 0.0);
}

/// Terrain outranks the flat-earth assumption, and the assumption is the last
/// resort rather than a default.
#[test]
fn terrain_beats_assuming_the_earth_is_flat() {
    let alt_cm = 12_345_i32;
    let flat = alt_above_ground_m(None, true, None, alt_cm);
    assert!(
        (flat - 123.45).abs() < 1e-3,
        "the flat-earth rung should be the altitude in metres, got {flat}"
    );

    let with_terrain = alt_above_ground_m(None, true, Some(8.0), alt_cm);
    assert!((with_terrain - 8.0).abs() < 1e-9);
}

/// The flat-earth rung keeps the centimetre, which a truncating conversion
/// would lose.
#[test]
fn the_flat_earth_rung_keeps_the_centimetre() {
    for (cm, want) in [(37_i32, 0.37_f32), (-450, -4.5), (1, 0.01), (-1, -0.01)] {
        let got = alt_above_ground_m(None, true, None, cm);
        assert!(
            (got - want).abs() < 1e-6,
            "{cm} cm gave {got}, not {want} m"
        );
    }
}
