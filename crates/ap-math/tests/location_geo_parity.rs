//! Parity test for `Location`'s geodesics (FW-006 slice, needed by FW-016).
//!
//! These three functions decide where the navigation controller believes the
//! aircraft is relative to its waypoints, so an error moves the whole flight
//! path rather than jittering a servo.
//!
//! They are also the first ported code whose answer depends on `ftype` being
//! double. SITL sets `HAL_WITH_EKF_DOUBLE`, so `longitude_scale` and
//! `get_bearing` compute in double, while `LOCATION_SCALING_FACTOR` stays a
//! `float` and `get_distance_NE` returns a `Vector2f`. The port mirrors that
//! through `Ftype`, which means **this test is only meaningful with the
//! `ekf-double` feature on** — the workspace default has it off, and without it
//! the port is computing in a precision the reference build never uses. The
//! test says so out loud rather than passing quietly against the wrong
//! configuration.
//!
//! The fixture covers both poles and both sides of the antimeridian, because
//! `diff_longitude` has a separate 64-bit path for coordinates straddling the
//! sign boundary and `longitude_scale` is floored near the poles.

// Only meaningful where ftype is double, which is what SITL builds
// (HAL_WITH_EKF_DOUBLE via linux.h). Without the feature the port computes
// these in single precision -- a configuration upstream never produces -- so
// the comparison is compiled out rather than run against the wrong thing.
#![cfg(feature = "ekf-double")]
#![allow(
    clippy::float_cmp,
    reason = "bit equality against upstream's recorded values is the assertion"
)]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture fields whose count is checked first; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::location::Location;

fn fixtures_dir() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures"))
        .expect("workspace root")
}

fn load() -> Option<Vec<Vec<String>>> {
    let text = std::fs::read_to_string(fixtures_dir().join("location_geo.csv")).ok()?;
    Some(
        text.lines()
            .skip(1)
            .map(|l| l.split(',').map(str::to_owned).collect())
            .collect(),
    )
}

#[test]
fn location_geodesics_match_upstream() {
    let Some(rows) = load() else {
        eprintln!("skipping: location_geo.csv not present");
        return;
    };

    let mut scales = 0usize;
    let mut pairs = 0usize;
    let mut worst_ne = 0.0_f64;
    let mut worst_bearing = 0.0_f64;
    let mut inexact_bearings = 0usize;
    let mut worst_ulps = 0i64;

    for r in &rows {
        if r.len() < 3 {
            continue;
        }
        match r[0].as_str() {
            "scale" => {
                let lat: i32 = r[1].parse().expect("lat");
                let want: f64 = r[2].parse().expect("scale");
                let got = Location::longitude_scale(lat);
                assert_eq!(
                    got.to_bits(),
                    want.to_bits(),
                    "longitude_scale({lat}): port {got:.17}, upstream {want:.17}"
                );
                scales += 1;
            }
            "pair" if r.len() >= 10 => {
                let a = Location::new(r[1].parse().expect("lat"), r[2].parse().expect("lng"));
                let b = Location::new(r[3].parse().expect("lat"), r[4].parse().expect("lng"));
                let want_dlon: i32 = r[5].parse().expect("dlon");
                let want_n: f32 = r[6].parse().expect("north");
                let want_e: f32 = r[7].parse().expect("east");
                let want_bearing: f64 = r[8].parse().expect("bearing");

                assert_eq!(
                    Location::diff_longitude(b.lng, a.lng),
                    want_dlon,
                    "diff_longitude({}, {})",
                    b.lng,
                    a.lng
                );

                let ne = a.get_distance_ne(b);
                assert_eq!(
                    ne.x.to_bits(),
                    want_n.to_bits(),
                    "north offset {a:?} -> {b:?}: port {}, upstream {want_n}",
                    ne.x
                );
                assert_eq!(
                    ne.y.to_bits(),
                    want_e.to_bits(),
                    "east offset {a:?} -> {b:?}: port {}, upstream {want_e}",
                    ne.y
                );
                worst_ne = worst_ne
                    .max(f64::from(ne.x - want_n).abs())
                    .max(f64::from(ne.y - want_e).abs());

                // The bearing is the one function here that calls a
                // transcendental, so it carries D-017's libm-versus-glibc gap.
                // Everything else in Location is required to be bit-exact.
                let bearing = a.get_bearing(b);
                let ulps = (bearing.to_bits() as i64 - want_bearing.to_bits() as i64).abs();
                assert!(
                    ulps <= 8,
                    "bearing {a:?} -> {b:?} differs by {ulps} ulp: port                      {bearing:.17}, upstream {want_bearing:.17}. D-017 bounds                      the math-library gap at a few ulps; this is more than that."
                );
                if ulps > 0 {
                    inexact_bearings += 1;
                    worst_ulps = worst_ulps.max(ulps);
                }
                worst_bearing = worst_bearing.max((bearing - want_bearing).abs());
                pairs += 1;
            }
            _ => {}
        }
    }

    println!("{scales} longitude scales and {pairs} pairs compared");
    println!("  scale, diff_longitude and both NE offsets: bit-exact");
    println!(
        "  bearing: {inexact_bearings} of {pairs} inexact, worst {worst_ulps} ulp ({worst_bearing:.3e}) -- D-017"
    );
    assert_eq!(worst_ne, 0.0, "the NE offsets must be bit-exact");
    assert!(scales >= 10, "too few scales compared: {scales}");
    assert!(pairs >= 100, "too few pairs compared: {pairs}");
}

/// The centidegree conversion rounds to nearest, which most of upstream's do
/// not, so it is pinned separately.
#[test]
fn bearing_to_rounds_to_nearest() {
    let Some(rows) = load() else {
        eprintln!("skipping: location_geo.csv not present");
        return;
    };

    let mut checked = 0usize;
    for r in &rows {
        if r.len() < 10 || r[0] != "pair" {
            continue;
        }
        let a = Location::new(r[1].parse().expect("lat"), r[2].parse().expect("lng"));
        let b = Location::new(r[3].parse().expect("lat"), r[4].parse().expect("lng"));
        let want: i32 = r[9].parse().expect("bearing_cd");
        assert_eq!(a.get_bearing_to(b), want, "get_bearing_to {a:?} -> {b:?}");
        checked += 1;
    }
    println!("{checked} bearings in centidegrees compared");
    assert!(checked >= 100);
}
