//! `Mode`'s roll and pitch stick conversions, against the real firmware.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use core::f32::consts::FRAC_PI_6;

use ap_copter::stick_nav::{pilot_desired_lean_angles_rad, pilot_desired_velocity_ne};

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("float bits"))
}

fn rows(section: &str) -> Vec<Vec<String>> {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/stick_nav.csv"))
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

/// The lean-angle conversion, over both sticks against both limits.
#[test]
fn the_lean_angles_match_upstream() {
    let rows = rows("lean");
    assert!(!rows.is_empty(), "no recorded lean rows");

    let mut largest = 0.0_f32;
    let mut nonzero = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 7, "malformed lean row");
        let idx: usize = r[0].parse().expect("idx");

        let (roll, pitch) =
            pilot_desired_lean_angles_rad(f(&r[1]), f(&r[2]), f(&r[3]), f(&r[4]), true);
        let (want_roll, want_pitch) = (f(&r[5]), f(&r[6]));

        for (got, want, axis) in [(roll, want_roll, "roll"), (pitch, want_pitch, "pitch")] {
            let diff = (got - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < 1e-6,
                "lean row {idx} {axis}: {got} against upstream {want} (diff {diff})"
            );
            if want != 0.0 {
                nonzero += 1;
            }
        }
    }

    assert!(
        nonzero > rows.len(),
        "most recorded outputs are zero, so the sweep is pinning the guard \
         rather than the conversion"
    );
    println!(
        "{} lean rows, {nonzero} non-zero outputs, largest difference {largest:e}",
        rows.len()
    );
}

/// The earth-frame velocity conversion, over both sticks and seven headings.
#[test]
fn the_pilot_velocity_matches_upstream() {
    let rows = rows("velocity");
    assert!(!rows.is_empty(), "no recorded velocity rows");

    let mut largest = 0.0_f32;
    let mut zero_rows = 0_usize;

    for r in &rows {
        assert_eq!(r.len(), 8, "malformed velocity row");
        let idx: usize = r[0].parse().expect("idx");

        let got = pilot_desired_velocity_ne(f(&r[1]), f(&r[2]), f(&r[3]), f(&r[4]), f(&r[5]), true);
        let (want_n, want_e) = (f(&r[6]), f(&r[7]));

        if want_n == 0.0 && want_e == 0.0 {
            zero_rows += 1;
        }

        for (got, want, axis) in [(got.x, want_n, "north"), (got.y, want_e, "east")] {
            let diff = (got - want).abs();
            largest = largest.max(diff);
            assert!(
                diff < 1e-5,
                "velocity row {idx} {axis}: {got} against upstream {want} (diff {diff})"
            );
        }
    }

    assert!(
        zero_rows > 0,
        "the centred-stick guard is never reached by the recording"
    );
    assert!(
        zero_rows < rows.len() / 2,
        "most rows are the centred-stick guard, so the sweep pins little else"
    );
    println!(
        "{} velocity rows, {zero_rows} centred, largest difference {largest:e}",
        rows.len()
    );
}

/// The failsafe returns neutral, and it is the guard doing it rather than the
/// sticks happening to be centred.
#[test]
fn no_valid_radio_gives_neutral() {
    let (roll, pitch) = pilot_desired_lean_angles_rad(0.8, -0.6, FRAC_PI_6, FRAC_PI_6, false);
    assert_eq!((roll, pitch), (0.0, 0.0));
    let (roll, pitch) = pilot_desired_lean_angles_rad(0.8, -0.6, FRAC_PI_6, FRAC_PI_6, true);
    assert!(roll != 0.0 && pitch != 0.0, "the sticks were not deflected");

    let v = pilot_desired_velocity_ne(0.8, -0.6, 5.0, 1.0, 0.0, false);
    assert_eq!((v.x, v.y), (0.0, 0.0));
    let v = pilot_desired_velocity_ne(0.8, -0.6, 5.0, 1.0, 0.0, true);
    assert!(v.x != 0.0 && v.y != 0.0);
}

/// The top speed depends on the heading, which is upstream's behaviour and
/// almost certainly not upstream's intent.
///
/// The scaling divides by the distance to the edge of the ±1 square in the
/// direction of travel, so a stick pushed to a corner does not command √2
/// times the speed of one pushed to an edge. Upstream's comment calls this
/// transforming "square input range to circular output".
///
/// It does not achieve that, because the rotation into earth frame happens
/// *first*. The square being normalised against is fixed to the compass, not
/// to the sticks, and the two coincide only when the aircraft points along a
/// cardinal. Recorded from the firmware at `vel_max` 5 m/s with full roll:
///
/// ```text
///   heading     speed
///     0 deg     5.0000
///    45 deg     3.5355     <- vel_max / sqrt(2)
///    90 deg     5.0000
///   135 deg     3.5355
/// ```
///
/// So a pilot holding full stick loses 29% of their commanded speed by yawing
/// 45 degrees, with no other input changing. This test pins that rather than
/// asserting the envelope is round, because the port reproduces upstream and
/// a test that claimed otherwise would be describing a port that does not
/// exist. See DIVERGENCES.md D-027 for the proposal to correct it.
#[test]
fn the_speed_envelope_is_square_in_earth_frame() {
    let vel_max = 7.5_f32;

    // Full roll deflection, along a cardinal and then at 45 degrees to it.
    let cardinal = pilot_desired_velocity_ne(1.0, 0.0, vel_max, 1.0, 0.0, true);
    let diagonal = {
        let c = core::f32::consts::FRAC_1_SQRT_2;
        pilot_desired_velocity_ne(1.0, 0.0, vel_max, c, c, true)
    };

    assert!(
        (cardinal.length() - vel_max).abs() < 1e-5,
        "pointing north, full stick should give vel_max, got {}",
        cardinal.length()
    );
    assert!(
        (diagonal.length() - vel_max * core::f32::consts::FRAC_1_SQRT_2).abs() < 1e-5,
        "pointing 45 degrees, full stick gives vel_max/sqrt(2) upstream, got {}",
        diagonal.length()
    );

    // Within a fixed heading the scaling does do its job: the corner of the
    // stick's travel is not faster than its edge, which is the artefact the
    // transform was written to remove.
    let edge = pilot_desired_velocity_ne(1.0, 0.0, vel_max, 1.0, 0.0, true);
    let corner = pilot_desired_velocity_ne(1.0, 1.0, vel_max, 1.0, 0.0, true);
    assert!(
        (edge.length() - corner.length()).abs() < 1e-5,
        "the corner of the stick's travel commands {} against the edge's {}",
        corner.length(),
        edge.length()
    );

    // And a stick short of the edge is proportionally slower, so the scaling
    // has not simply normalised everything to the maximum.
    let half = pilot_desired_velocity_ne(0.5, 0.0, vel_max, 1.0, 0.0, true);
    assert!(
        (half.length() - vel_max * 0.5).abs() < 1e-5,
        "half deflection gave {}, not half speed",
        half.length()
    );
}

/// A centred stick divides by zero twice over without its guard.
#[test]
fn a_centred_stick_is_guarded() {
    let v = pilot_desired_velocity_ne(0.0, 0.0, 5.0, 0.6, 0.8, true);
    assert_eq!((v.x, v.y), (0.0, 0.0));
    assert!(v.x.is_finite() && v.y.is_finite());
}
