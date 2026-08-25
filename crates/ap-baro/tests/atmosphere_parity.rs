//! Parity test: the 1976 standard atmosphere against upstream's own model.
//!
//! Covers every entry point the fixed-wing path uses: the geometric and
//! geopotential conversions, pressure/temperature/density/EAS2TAS forward from
//! altitude, altitude back from pressure, the altitude difference between two
//! pressures, and the numerical sea-level solver.
//!
//! The altitudes straddle every layer boundary from −5000 m to 90 km, and both
//! sides of each — a layer lookup off by one would show up immediately.
//!
//! # Tolerance
//!
//! The model runs `logf`, `expf`, `powf` and `sqrtf`, and D-017 records that
//! upstream links glibc while the port links libm. Bit-exactness is not
//! available here. The test therefore reports the worst disagreement it sees
//! and holds it to a bound stated in the units that matter — metres of
//! altitude, pascals of pressure — rather than a raw ulp count, because a ulp
//! at 100 kPa and a ulp at 0.37 Pa are not comparable quantities.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_baro::{
    air_density_for_alt_amsl, altitude_difference, altitude_from_pressure, eas2tas_extended,
    eas2tas_for_alt_amsl, geometric_to_geopotential, geopotential_to_geometric,
    pressure_temperature_for_alt_amsl, sealevel_pressure, temperature_from_altitude,
};

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/atmosphere_parity.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_atmosphere_fixture.py",
            path.display()
        )
    })
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

/// One category of comparison, with the bound that matters for it.
struct Bound {
    /// Absolute tolerance, in the quantity's own units.
    abs: f64,
    /// Relative tolerance, for quantities spanning six orders of magnitude.
    rel: f64,
}

fn bound_for(kind: &str) -> Bound {
    match kind {
        // Metres. A tenth of a metre is far below what a barometer resolves.
        "geo2pot" | "pot2geo" | "alt_from_p" | "alt_diff" => Bound {
            abs: 0.1,
            rel: 1e-5,
        },
        // Kelvin.
        "temp_from_alt" => Bound {
            abs: 1e-3,
            rel: 1e-5,
        },
        // Pascals, spanning 177 kPa down to 0.37 Pa — relative is the only
        // meaningful bound across that range.
        "pt" | "sealevel" => Bound {
            abs: 1e-2,
            rel: 1e-5,
        },
        // kg/m^3, likewise spanning six orders.
        "density" => Bound {
            abs: 1e-9,
            rel: 1e-5,
        },
        // Dimensionless ratio.
        "eas2tas_sitl" | "eas2tas_ext" => Bound {
            abs: 1e-4,
            rel: 1e-5,
        },
        _ => Bound { abs: 0.0, rel: 0.0 },
    }
}

fn close(got: f32, want: f32, b: &Bound) -> bool {
    if got.to_bits() == want.to_bits() {
        return true;
    }
    if !got.is_finite() || !want.is_finite() {
        return false;
    }
    let (g, w) = (f64::from(got), f64::from(want));
    let diff = (g - w).abs();
    diff <= b.abs || diff <= b.rel * w.abs()
}

#[test]
fn the_atmosphere_model_matches_upstream() {
    let text = fixture();
    let mut checked = 0_usize;
    let mut exact = 0_usize;
    let mut worst_rel = 0.0_f64;
    let mut worst_note = String::new();

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("kind,") {
            continue;
        }
        let row: Vec<&str> = line.split(',').collect();
        assert_eq!(row.len(), 4, "malformed row: {line}");

        let kind = row[0];
        let input = f(row[1]);
        let want_a = f(row[2]);
        let want_b = f(row[3]);

        let (got_a, got_b) = match kind {
            "geo2pot" => (geometric_to_geopotential(input), 0.0),
            "pot2geo" => (geopotential_to_geometric(input), 0.0),
            "pt" => {
                let (p, t) = pressure_temperature_for_alt_amsl(input);
                (p, t)
            }
            "density" => (air_density_for_alt_amsl(input), 0.0),
            "eas2tas_sitl" => (eas2tas_for_alt_amsl(input), 0.0),
            "eas2tas_ext" => (eas2tas_extended(input), 0.0),
            "temp_from_alt" => (temperature_from_altitude(input), 0.0),
            "alt_from_p" => (
                altitude_from_pressure(input).expect("the fixture only drives real pressures"),
                0.0,
            ),
            "alt_diff" => (
                altitude_difference(101_325.0, input).expect("real pressures"),
                0.0,
            ),
            "sealevel" => {
                // input is the altitude; column a is the pressure upstream
                // computed for it, and b is the sea-level pressure it solved.
                (want_a, sealevel_pressure(want_a, input))
            }
            other => panic!("unknown fixture kind {other}"),
        };

        let b = bound_for(kind);

        // For "sealevel" the first column is an input echo, not a result.
        if kind != "sealevel" {
            assert!(
                close(got_a, want_a, &b),
                "{kind}({input}): {got_a} against upstream {want_a}"
            );
            if got_a.to_bits() == want_a.to_bits() {
                exact += 1;
            }
            let rel = ((f64::from(got_a) - f64::from(want_a)) / f64::from(want_a).abs()).abs();
            if rel.is_finite() && rel > worst_rel {
                worst_rel = rel;
                worst_note = format!("{kind}({input})");
            }
            checked += 1;
        }

        if kind == "pt" || kind == "sealevel" {
            assert!(
                close(got_b, want_b, &b),
                "{kind}({input}) second value: {got_b} against upstream {want_b}"
            );
            if got_b.to_bits() == want_b.to_bits() {
                exact += 1;
            }
            let rel = ((f64::from(got_b) - f64::from(want_b)) / f64::from(want_b).abs()).abs();
            if rel.is_finite() && rel > worst_rel {
                worst_rel = rel;
                worst_note = format!("{kind}({input}) second");
            }
            checked += 1;
        }
    }

    assert!(
        checked > 300,
        "fixture looks truncated: {checked} comparisons"
    );
    println!(
        "{checked} values compared, {exact} bit-exact ({:.0}%); worst relative {worst_rel:e} at {worst_note}",
        100.0 * exact as f64 / checked as f64
    );
}

/// The round trip a barometer actually performs, driven from upstream's own
/// pressures rather than the port's.
///
/// This is the check that matters operationally: feed the port the pressure
/// upstream computed for an altitude, and it should recover that altitude.
#[test]
fn upstreams_pressures_read_back_as_the_right_altitude() {
    let text = fixture();
    let mut worst = 0.0_f64;
    let mut worst_alt = 0.0_f32;

    for line in text.lines() {
        if !line.starts_with("pt,") {
            continue;
        }
        let row: Vec<&str> = line.split(',').collect();
        let alt = f(row[1]);
        let upstream_pressure = f(row[2]);

        // Above the table the inverse is not meaningful.
        if alt > 84_000.0 {
            continue;
        }
        let back = altitude_from_pressure(upstream_pressure).expect("a real pressure");
        let err = f64::from(back - alt).abs();
        if err > worst {
            worst = err;
            worst_alt = alt;
        }
    }

    println!("worst round-trip error {worst:.4} m, at {worst_alt} m");
    assert!(
        worst < 1.0,
        "the port should recover upstream's altitudes to within a metre, worst {worst} m at {worst_alt} m"
    );
}
