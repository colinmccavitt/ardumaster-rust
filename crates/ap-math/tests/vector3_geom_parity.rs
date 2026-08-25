//! Parity test: the Vector3 geometry members against upstream `vector3.cpp`.
//!
//! Covers `rotate_xy`, `rotate_inverse`, `row_times_mat`, `mul_rowcol`,
//! `distance_to_segment`, `point_on_line_closest_to_other_point`,
//! `closest_distance_between_line_and_point`, `segment_plane_intersect` and
//! `segment_to_segment_closest_point`.
//!
//! The segment pairs are chosen to reach every branch of
//! `segment_to_segment_closest_point` — near-parallel, collinear, degenerate,
//! and pairs whose nearest approach falls at each end — rather than only the
//! generic crossing case, which would exercise one branch of six and still look
//! thorough.
//!
//! Values are raw bit patterns, so every comparison is exact.

#![allow(
    clippy::indexing_slicing,
    reason = "indexes fixture rows whose field count is asserted; in a test an \
index fault is a test failure, which is the desired outcome"
)]
#![allow(
    clippy::float_cmp,
    reason = "bit-exact comparison against upstream is the point of the test"
)]

use ap_math::matrix3::Matrix3f;
use ap_math::rotations_gen::{rotate_inverse, Rotation};
use ap_math::vector3::Vector3f;
use std::collections::HashMap;

fn fixture_path() -> std::path::PathBuf {
    std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(|p| p.parent())
        .map(|p| p.join("fixtures/vector3_geom_parity.csv"))
        .expect("workspace root")
}

fn f(s: &str) -> f32 {
    f32::from_bits(s.parse::<u32>().expect("bit pattern"))
}

fn vec3(field: &str) -> Vector3f {
    let v: Vec<f32> = field.split_whitespace().map(f).collect();
    assert_eq!(v.len(), 3, "expected 3 components in {field:?}");
    Vector3f::new(v[0], v[1], v[2])
}

fn same(a: f32, b: f32) -> bool {
    (a.is_nan() && b.is_nan()) || a.to_bits() == b.to_bits()
}

fn same3(a: Vector3f, b: Vector3f) -> bool {
    same(a.x, b.x) && same(a.y, b.y) && same(a.z, b.z)
}

struct Fx {
    rows: Vec<(String, Vec<String>)>,
    pts: HashMap<usize, Vector3f>,
    segs: HashMap<usize, [Vector3f; 4]>,
}

fn load() -> Option<Fx> {
    let path = fixture_path();
    if !path.exists() {
        eprintln!("skipping: {} not present", path.display());
        return None;
    }
    let text = std::fs::read_to_string(&path).expect("read fixture");
    let mut rows = Vec::new();
    let mut pts = HashMap::new();
    let mut segs = HashMap::new();
    let mut section = String::new();
    let mut header_pending = false;

    for line in text.lines() {
        if line.is_empty() {
            continue;
        }
        if let Some(name) = line.strip_prefix('#') {
            section = name.to_string();
            header_pending = true;
            continue;
        }
        if header_pending {
            header_pending = false;
            continue;
        }
        let c: Vec<String> = line.split(',').map(str::to_string).collect();
        match section.as_str() {
            "pts" => {
                pts.insert(c[0].parse::<usize>().expect("idx"), vec3(&c[1]));
            }
            "segs" => {
                let v: Vec<f32> = c[1].split_whitespace().map(f).collect();
                assert_eq!(v.len(), 12, "segment row");
                segs.insert(
                    c[0].parse::<usize>().expect("idx"),
                    [
                        Vector3f::new(v[0], v[1], v[2]),
                        Vector3f::new(v[3], v[4], v[5]),
                        Vector3f::new(v[6], v[7], v[8]),
                        Vector3f::new(v[9], v[10], v[11]),
                    ],
                );
            }
            _ => rows.push((section.clone(), c)),
        }
    }
    Some(Fx { rows, pts, segs })
}

#[test]
fn vector3_geometry_matches_upstream() {
    let Some(fx) = load() else { return };
    let seg = |i: usize| *fx.segs.get(&i).expect("segment");
    let pt = |i: usize| *fx.pts.get(&i).expect("point");

    let mut counts = std::collections::BTreeMap::<&str, usize>::new();

    for (section, c) in &fx.rows {
        match section.as_str() {
            "seg2seg" => {
                let s = seg(c[0].parse().expect("case"));
                let got = Vector3f::segment_to_segment_closest_point(s[0], s[1], s[2], s[3]);
                let want = vec3(&c[1]);
                assert!(
                    same3(got, want),
                    "segment_to_segment_closest_point case {}: port {got:?}, upstream {want:?}",
                    c[0]
                );
                *counts.entry("segment_to_segment").or_default() += 1;
            }
            "pointline" => {
                let s = seg(c[0].parse().expect("seg"));
                let p = pt(c[1].parse().expect("pt"));
                let closest = Vector3f::point_on_line_closest_to_other_point(s[0], s[1], p);
                let want = vec3(&c[2]);
                assert!(
                    same3(closest, want),
                    "point_on_line seg {} pt {}: port {closest:?}, upstream {want:?}",
                    c[0],
                    c[1]
                );
                let dist = Vector3f::closest_distance_between_line_and_point(s[0], s[1], p);
                let want_dist = f(&c[3]);
                assert!(
                    same(dist, want_dist),
                    "closest_distance seg {} pt {}: port {dist}, upstream {want_dist}",
                    c[0],
                    c[1]
                );
                let dseg = p.distance_to_segment(s[0], s[1]);
                let want_dseg = f(&c[4]);
                assert!(
                    same(dseg, want_dseg),
                    "distance_to_segment seg {} pt {}: port {dseg}, upstream {want_dseg}",
                    c[0],
                    c[1]
                );
                *counts.entry("point_on_line").or_default() += 1;
                *counts.entry("closest_distance").or_default() += 1;
                *counts.entry("distance_to_segment").or_default() += 1;
            }
            "plane" => {
                let s = seg(c[0].parse().expect("seg"));
                let planes: [([f32; 3], [f32; 3]); 5] = [
                    ([0.0, 0.0, 1.0], [0.0, 0.0, 0.0]),
                    ([0.0, 0.0, 1.0], [0.0, 0.0, 5.0]),
                    ([1.0, 0.0, 0.0], [5.0, 0.0, 0.0]),
                    ([1.0, 1.0, 1.0], [0.0, 0.0, 0.0]),
                    ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                ];
                let k: usize = c[1].parse().expect("plane");
                let (n, pp) = planes[k];
                let got = Vector3f::segment_plane_intersect(
                    s[0],
                    s[1],
                    Vector3f::new(n[0], n[1], n[2]),
                    Vector3f::new(pp[0], pp[1], pp[2]),
                );
                assert_eq!(
                    got,
                    c[3] == "1",
                    "segment_plane_intersect seg {} plane {k}",
                    c[0]
                );
                *counts.entry("segment_plane_intersect").or_default() += 1;
            }
            "matops" => {
                let p = pt(c[0].parse().expect("pt"));
                let m = Matrix3f::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 10.0);
                let rtm = p.row_times_mat(&m);
                let want_rtm = vec3(&c[1]);
                assert!(
                    same3(rtm, want_rtm),
                    "row_times_mat pt {}: port {rtm:?}, upstream {want_rtm:?}",
                    c[0]
                );

                let mrc = p.mul_rowcol(Vector3f::new(1.5, -2.5, 0.75));
                let want: Vec<f32> = c[2].split_whitespace().map(f).collect();
                assert_eq!(want.len(), 9, "mul_rowcol row");
                for (i, row) in [mrc.a, mrc.b, mrc.c].iter().enumerate() {
                    assert!(
                        same3(
                            *row,
                            Vector3f::new(want[i * 3], want[i * 3 + 1], want[i * 3 + 2])
                        ),
                        "mul_rowcol pt {} row {i}",
                        c[0]
                    );
                }
                *counts.entry("row_times_mat").or_default() += 1;
                *counts.entry("mul_rowcol").or_default() += 1;
            }
            "rotxy" => {
                let mut p = pt(c[0].parse().expect("pt"));
                p.rotate_xy(f(&c[1]));
                let want = vec3(&c[2]);
                assert!(
                    same3(p, want),
                    "rotate_xy pt {} angle {}: port {p:?}, upstream {want:?}",
                    c[0],
                    f(&c[1])
                );
                *counts.entry("rotate_xy").or_default() += 1;
            }
            "rotinv" => {
                let raw: u8 = c[0].parse().expect("rotation");
                let r = Rotation::from_u8(raw).expect("fixture rotation must be valid");
                let mut p = pt(c[1].parse().expect("pt"));
                rotate_inverse(&mut p, r).expect("fixture rotations are concrete");
                let want = vec3(&c[2]);
                assert!(
                    same3(p, want),
                    "rotate_inverse {r:?} pt {}: port {p:?}, upstream {want:?}",
                    c[1]
                );
                *counts.entry("rotate_inverse").or_default() += 1;
            }
            other => panic!("unhandled section {other}"),
        }
    }

    let total: usize = counts.values().sum();
    println!("{total} Vector3 geometry cases matched upstream exactly:");
    for (k, v) in &counts {
        println!("  {k:<26} {v}");
    }

    for name in [
        "segment_to_segment",
        "point_on_line",
        "closest_distance",
        "distance_to_segment",
        "segment_plane_intersect",
        "row_times_mat",
        "mul_rowcol",
        "rotate_xy",
        "rotate_inverse",
    ] {
        assert!(
            counts.get(name).copied().unwrap_or(0) > 0,
            "{name} contributed no cases"
        );
    }
    assert!(total > 400, "expected the whole fixture, got {total}");
}

/// `rotate_inverse` must undo `rotate` for every concrete rotation.
///
/// Independent of upstream: a parity test would agree with upstream even if
/// both directions were wrong in the same way.
#[test]
fn rotate_inverse_undoes_rotate() {
    use ap_math::rotations_gen::rotate;

    for raw in 0..=103u8 {
        let Some(r) = Rotation::from_u8(raw) else {
            continue;
        };
        let start = Vector3f::new(1.25, -3.5, 7.75);
        let mut v = start;
        if rotate(&mut v, r).is_err() {
            continue;
        }
        rotate_inverse(&mut v, r).expect("the forward rotation succeeded");
        for (got, want) in [(v.x, start.x), (v.y, start.y), (v.z, start.z)] {
            assert!(
                (got - want).abs() < 1e-5,
                "{r:?}: round trip gave {v:?}, expected {start:?}"
            );
        }
    }
}
