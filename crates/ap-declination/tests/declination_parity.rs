//! Parity test: the IGRF world magnetic model lookup against upstream.
//!
//! 17,213 points: a two-degree sweep of the whole globe, the table's four
//! edges with a hair either side of each, every exact grid point (where both
//! interpolation fractions are zero and the answer should be the table entry
//! itself), and a set of real places whose coordinates are not nice numbers.
//!
//! Dense because it is cheap. The lookup is a table read and six multiplies,
//! so sixteen thousand points cost nothing and leave nowhere for an
//! off-by-one in the indexing to hide in a region nobody sampled.
//!
//! The coverage flag is compared too. Upstream returns it as the function's
//! value and `get_declination` discards it, which makes it the part most
//! likely to be got wrong without anyone noticing.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_declination::{get_mag_field_ef, Coverage};

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/declination_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_declination_fixture.py",
            path.display()
        )
    })
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

#[test]
fn the_magnetic_model_matches_upstream_everywhere() {
    let text = fixture();
    let mut checked = 0_usize;
    let mut exact = 0_usize;
    let mut worst_rel = 0.0_f64;
    let mut worst_where = String::new();
    let mut outside = 0_usize;

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("lat,") {
            continue;
        }
        let row: Vec<&str> = line.split(',').collect();
        assert_eq!(row.len(), 6, "malformed row: {line}");

        let lat = f(row[0]);
        let lon = f(row[1]);
        let want = (f(row[2]), f(row[3]), f(row[4]));
        let want_covered = row[5] == "1";

        let (got, coverage) = get_mag_field_ef(lat, lon);

        assert_eq!(
            coverage == Coverage::Interpolated,
            want_covered,
            "coverage at ({lat}, {lon}): {coverage:?} against upstream {want_covered}"
        );
        if !want_covered {
            outside += 1;
        }

        for (label, g, w) in [
            ("intensity", got.intensity_gauss, want.0),
            ("declination", got.declination_deg, want.1),
            ("inclination", got.inclination_deg, want.2),
        ] {
            if same(g, w) {
                exact += 1;
            } else {
                let denom = f64::from(w).abs().max(1.0e-3);
                let rel = ((f64::from(g) - f64::from(w)) / denom).abs();
                assert!(
                    rel < 1.0e-6,
                    "{label} at ({lat}, {lon}): {g} against upstream {w}"
                );
                if rel > worst_rel {
                    worst_rel = rel;
                    worst_where = format!("{label} at ({lat}, {lon})");
                }
            }
            checked += 1;
        }
    }

    assert!(
        checked > 50_000,
        "fixture looks truncated: {checked} values"
    );
    assert!(
        outside > 20,
        "the edge cases should be in here, got {outside}"
    );
    println!(
        "{checked} values across {} points, {exact} bit-exact ({:.2}%); {outside} outside the table; worst relative {worst_rel:e} {worst_where}",
        checked / 3,
        100.0 * exact as f64 / checked as f64
    );
}

/// At an exact grid point both interpolation fractions are zero, so the answer
/// must be the table entry with no arithmetic in between. If the port's tables
/// had drifted by a single ulp from upstream's, this is where it would show.
#[test]
fn exact_grid_points_return_the_table_untouched() {
    let text = fixture();
    let mut grid_points = 0_usize;

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("lat,") {
            continue;
        }
        let row: Vec<&str> = line.split(',').collect();
        let lat = f(row[0]);
        let lon = f(row[1]);

        // Only the exact ten-degree grid, and only inside the table.
        if lat % 10.0 != 0.0 || lon % 10.0 != 0.0 {
            continue;
        }
        if lat.abs() >= 90.0 || lon.abs() >= 180.0 {
            continue;
        }

        let (got, _) = get_mag_field_ef(lat, lon);
        assert!(
            same(got.intensity_gauss, f(row[2]))
                && same(got.declination_deg, f(row[3]))
                && same(got.inclination_deg, f(row[4])),
            "grid point ({lat}, {lon}) is not bit-exact — the tables have drifted"
        );
        grid_points += 1;
    }

    assert!(
        grid_points > 500,
        "expected the grid to be well covered, got {grid_points}"
    );
    println!("{grid_points} exact grid points bit-identical to upstream's tables");
}
