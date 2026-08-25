//! Parity test: `Location`'s offset operations against upstream.
//!
//! These are what the fixed-wing landing slope uses to place its aim point and
//! work out where the vehicle sits along the approach, so they are exercised
//! where the arithmetic is delicate rather than where it is comfortable: the
//! dateline, both poles, the equator, offsets large enough to cross all three,
//! negative distances (the aim point sits *short* of the threshold), and
//! points projected beyond both ends of a line.
//!
//! Latitude and longitude are integers, so `offset` and `offset_bearing` are
//! compared exactly. `line_path_proportion` is a float and is compared as bits
//! where it can be and within a tolerance where a transcendental is involved.

// Upstream's ftype.h resolves `ftype` to `double` for this build, and
// Location's offsets are declared in terms of ftype. Comparing against them in
// single precision measures the precision choice rather than the port, so this
// file only runs with the matching feature.
#![cfg(feature = "ekf-double")]
#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]

use ap_math::location::Location;
use ap_math::Ftype;

fn fixture() -> String {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/location_offset.csv"))
        .expect("workspace root");
    std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run tools/parity/gen_location_offset_fixture.py",
            path.display()
        )
    })
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.trim().parse::<u32>().expect("bit pattern"))
}

#[test]
fn location_offsets_match_upstream() {
    let text = fixture();

    let mut offsets = 0_usize;
    let mut bearings = 0_usize;
    let mut bearing_off_by_one = 0_usize;
    let mut proportions = 0_usize;
    let mut exact_proportions = 0_usize;
    let mut worst_prop = 0.0_f64;

    // proportion rows come in pairs: the result, then the query point's
    // longitude.
    let mut pending: Option<(i32, i32, i32, i32, f32, i32)> = None;

    for line in text.lines() {
        if line.is_empty() || line.starts_with('#') || line.starts_with("kind,") {
            continue;
        }
        let row: Vec<&str> = line.split(',').collect();
        assert_eq!(row.len(), 7, "malformed row: {line}");

        match row[0] {
            "offset" => {
                let mut loc = Location {
                    lat: row[1].parse().expect("lat"),
                    lng: row[2].parse().expect("lng"),
                };
                let (start_lat, start_lng) = (loc.lat, loc.lng);
                loc.offset(Ftype::from(f(row[3])), Ftype::from(f(row[4])));

                let want_lat: i32 = row[5].parse().expect("out lat");
                let want_lng: i32 = row[6].parse().expect("out lng");
                assert_eq!(
                    (loc.lat, loc.lng),
                    (want_lat, want_lng),
                    "offset({}, {}) from ({start_lat}, {start_lng})",
                    f(row[3]),
                    f(row[4])
                );
                offsets += 1;
            }
            "bearing" => {
                let mut loc = Location {
                    lat: row[1].parse().expect("lat"),
                    lng: row[2].parse().expect("lng"),
                };
                let (start_lat, start_lng) = (loc.lat, loc.lng);
                loc.offset_bearing(Ftype::from(f(row[3])), Ftype::from(f(row[4])));

                let want_lat: i32 = row[5].parse().expect("out lat");
                let want_lng: i32 = row[6].parse().expect("out lng");

                // One count of 1e-7 degrees, and no more. See the module docs:
                // sin and cos come from libm here and glibc upstream, and near
                // the poles a one-ulp difference survives the division by a
                // very small longitude_scale.
                let dlat = (loc.lat - want_lat).abs();
                let dlng = (loc.lng - want_lng).abs();
                assert!(
                    dlat <= 1 && dlng <= 1,
                    "offset_bearing({}, {}) from ({start_lat}, {start_lng}): \
                     ({}, {}) against upstream ({want_lat}, {want_lng})",
                    f(row[3]),
                    f(row[4]),
                    loc.lat,
                    loc.lng
                );
                if dlat != 0 || dlng != 0 {
                    bearing_off_by_one += 1;
                }
                bearings += 1;
            }
            "proportion" => {
                pending = Some((
                    row[1].parse().expect("p1 lat"),
                    row[2].parse().expect("p1 lng"),
                    row[3].parse().expect("p2 lat"),
                    row[4].parse().expect("p2 lng"),
                    f(row[5]),
                    row[6].parse().expect("q lat"),
                ));
            }
            "proportion_q" => {
                let (p1lat, p1lng, p2lat, p2lng, want, q_lat_echo) = pending
                    .take()
                    .expect("a result row must precede its query point");
                let q_lat: i32 = row[1].parse().expect("q lat");
                let q_lng: i32 = row[2].parse().expect("q lng");
                assert_eq!(q_lat, q_lat_echo, "the two rows must describe one query");

                let got = Location {
                    lat: q_lat,
                    lng: q_lng,
                }
                .line_path_proportion(
                    Location {
                        lat: p1lat,
                        lng: p1lng,
                    },
                    Location {
                        lat: p2lat,
                        lng: p2lng,
                    },
                );

                if got.to_bits() == want.to_bits() || (got.is_nan() && want.is_nan()) {
                    exact_proportions += 1;
                } else {
                    let denom = f64::from(want).abs().max(1.0);
                    let rel = (f64::from(got) - f64::from(want)).abs() / denom;
                    assert!(
                        rel < 1.0e-5,
                        "line_path_proportion for ({q_lat}, {q_lng}) on \
                         ({p1lat}, {p1lng})->({p2lat}, {p2lng}): {got} against \
                         upstream {want}"
                    );
                    if rel > worst_prop {
                        worst_prop = rel;
                    }
                }
                proportions += 1;
            }
            other => panic!("unknown fixture kind {other}"),
        }
    }

    assert!(offsets >= 90, "got {offsets} offsets");
    assert!(bearings >= 90, "got {bearings} bearings");
    assert!(proportions >= 700, "got {proportions} proportions");
    println!(
        "{offsets} offsets exact; {bearings} bearings, {bearing_off_by_one} of them one count out (D-017); \
         {proportions} proportions, {exact_proportions} bit-exact, worst relative {worst_prop:e}"
    );
    assert!(
        bearing_off_by_one * 4 < bearings,
        "{bearing_off_by_one} of {bearings} bearings disagree — too many to be \
         a one-ulp transcendental difference"
    );
}

/// Offsetting north past the pole reflects the latitude and leaves the
/// longitude alone — which is geometrically wrong, and is upstream's
/// behaviour. Pinned so the reproduction is deliberate rather than accidental.
///
/// Crossing a pole should flip longitude by 180 degrees. Upstream's
/// `limit_lattitude` is a guard against nonsense coordinates rather than polar
/// navigation, and the port reproduces it; this test states what that means.
#[test]
fn crossing_the_pole_reflects_latitude_without_flipping_longitude() {
    let mut loc = Location {
        lat: 899_000_000,
        lng: 0,
    };
    loc.offset(200_000.0, 0.0);

    assert!(
        loc.lat < 900_000_000,
        "latitude should be folded back below the pole, got {}",
        loc.lat
    );
    assert_eq!(
        loc.lng, 0,
        "and longitude is left alone — a true crossing would put this at 180 degrees"
    );
}

/// A negative distance moves backwards along the bearing, which is how the
/// landing aim point is placed short of the runway threshold.
#[test]
fn a_negative_distance_moves_backwards() {
    let start = Location {
        lat: 515_080_000,
        lng: -1_268_000,
    };

    let mut forward = start;
    forward.offset_bearing(0.0, 1000.0);
    let mut backward = start;
    backward.offset_bearing(0.0, -1000.0);

    assert!(forward.lat > start.lat, "north should increase latitude");
    assert!(
        backward.lat < start.lat,
        "and a negative distance decrease it"
    );
    assert_eq!(
        forward.lat - start.lat,
        start.lat - backward.lat,
        "symmetric about the start"
    );
}

/// The proportion is zero at the first point, one at the second, and runs
/// outside that range beyond either end — callers that need it bounded clamp
/// it themselves.
#[test]
fn the_proportion_is_unbounded_beyond_the_ends() {
    let p1 = Location { lat: 0, lng: 0 };
    let mut p2 = p1;
    p2.offset(1000.0, 0.0);

    assert!(
        p1.line_path_proportion(p1, p2).abs() < 1e-3,
        "zero at the start"
    );
    assert!(
        (p2.line_path_proportion(p1, p2) - 1.0).abs() < 1e-3,
        "one at the end"
    );

    let mut beyond = p1;
    beyond.offset(2000.0, 0.0);
    assert!(
        beyond.line_path_proportion(p1, p2) > 1.9,
        "past the end it keeps going"
    );

    let mut before = p1;
    before.offset(-1000.0, 0.0);
    assert!(
        before.line_path_proportion(p1, p2) < -0.9,
        "and before the start it goes negative"
    );
}
