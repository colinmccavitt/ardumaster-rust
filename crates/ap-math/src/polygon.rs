//! Point-in-polygon and polygon distance, ported from `AP_Math/polygon.cpp`.
//!
//! Used by the geofence and by object avoidance (`AP_OABendyRuler`,
//! `AP_OADijkstra`) to decide which side of a boundary the aircraft is on and
//! how close it is.
//!
//! # Slices instead of pointer and count
//!
//! Upstream takes `(const Vector2f *V, unsigned N)`. The port takes `&[Vector2f]`,
//! so the count cannot disagree with the array. That is not cosmetic here — see
//! D-011.
//!
//! # Scope: the float instantiation only
//!
//! Upstream instantiates `Polygon_outside` and `Polygon_complete` for both
//! `Vector2f` and `Vector2l` (`int32_t`). Only the float form is ported, because
//! the port's `Vector2<T>` is bound to `Real` and has no integer instantiation
//! yet. Every fixed-wing caller uses the float form; the integer form reaches
//! `Polygon_outside` only through `AC_Avoid`, which is Copter and Rover.
//!
//! The parity fixture already carries upstream's integer results alongside the
//! float ones, so when an integer `Vector2` lands the oracle is waiting.

use crate::scalar::Real;
use crate::vector2::Vector2f;

/// Whether a polygon is closed, upstream `Polygon_complete`.
///
/// Complete means at least four points with the last equal to the first, which
/// is the minimum [`polygon_outside`] needs. Note the comparison is
/// `Vector2`'s, which is epsilon-based rather than exact.
pub fn polygon_complete(v: &[Vector2f]) -> bool {
    match (v.first(), v.last()) {
        (Some(first), Some(last)) if v.len() >= 4 => last == first,
        _ => false,
    }
}

/// Whether a point lies outside a polygon, upstream `Polygon_outside`.
///
/// Crossing count against each edge, after the method at
/// <https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html>. Ignores the
/// curvature of the earth, which is negligible over fence-sized distances.
///
/// A closed polygon has its repeated last point dropped first, so passing the
/// same boundary with or without the closing point gives the same answer.
pub fn polygon_outside(p: Vector2f, v: &[Vector2f]) -> bool {
    // drop the repeated closing point, if there is one
    let n = if polygon_complete(v) {
        v.len() - 1
    } else {
        v.len()
    };

    let Some(ring) = v.get(..n) else {
        return true;
    };

    let mut outside = true;
    for (&vi, &vj) in edges(ring) {
        // the edge must straddle the test point's latitude
        if (vi.y > p.y) == (vj.y > p.y) {
            continue;
        }

        let dx1 = p.x - vi.x;
        let dx2 = vj.x - vi.x;
        let dy1 = p.y - vi.y;
        let dy2 = vj.y - vi.y;

        // Upstream compares the signs first so the products can be skipped in
        // the common case, which matters on the integer instantiation where
        // they would be 64-bit multiplies. Kept, because the sign comparison
        // also decides the outcome outright whenever the signs differ; only
        // the ambiguous case falls through to the products.
        let sgn = |x: f32| if x < 0.0 { -1i8 } else { 1i8 };
        let m1 = sgn(dx1) * sgn(dy2);
        let m2 = sgn(dx2) * sgn(dy1);

        // Upstream spells this as toggle / continue / compare-products per
        // branch. As one boolean the two directions are visibly mirror images.
        let crosses = if dy2 < 0.0 {
            m1 > m2 || (m1 == m2 && dx1 * dy2 > dx2 * dy1)
        } else {
            m1 < m2 || (m1 == m2 && dx1 * dy2 < dx2 * dy1)
        };
        if crosses {
            outside = !outside;
        }
    }
    outside
}

/// Each vertex paired with its successor, wrapping the last back to the first.
///
/// Upstream's `j = i + 1; if (j >= n) { j = 0; }`, without the index. An empty
/// ring yields nothing: `Cycle` over an empty iterator is itself empty, so this
/// cannot spin.
fn edges(ring: &[Vector2f]) -> impl Iterator<Item = (&Vector2f, &Vector2f)> {
    ring.iter().zip(ring.iter().cycle().skip(1))
}

/// Where a line from `p1` to `p2` first crosses a polygon's boundary,
/// upstream `Polygon_intersects`.
///
/// Returns the crossing closest to `p1`, or `None` if the line misses the
/// polygon entirely. Upstream returns a `bool` and leaves its out-parameter
/// untouched on failure.
pub fn polygon_intersects(v: &[Vector2f], p1: Vector2f, p2: Vector2f) -> Option<Vector2f> {
    let n = if polygon_complete(v) {
        v.len() - 1
    } else {
        v.len()
    };

    let ring = v.get(..n)?;

    let mut best: Option<(f32, Vector2f)> = None;
    for (&v1, &v2) in edges(ring) {
        // skip edges that lie wholly to one side of the line's bounding box
        if v1.x > p1.x && v2.x > p1.x && v1.x > p2.x && v2.x > p2.x {
            continue;
        }
        if v1.y > p1.y && v2.y > p1.y && v1.y > p2.y && v2.y > p2.y {
            continue;
        }
        if v1.x < p1.x && v2.x < p1.x && v1.x < p2.x && v2.x < p2.x {
            continue;
        }
        if v1.y < p1.y && v2.y < p1.y && v1.y < p2.y && v2.y < p2.y {
            continue;
        }

        if let Some(hit) = Vector2f::segment_intersection(v1, v2, p1, p2) {
            let dist_sq = (hit.x - p1.x) * (hit.x - p1.x) + (hit.y - p1.y) * (hit.y - p1.y);
            if best.is_none_or(|(d, _)| dist_sq < d) {
                best = Some((dist_sq, hit));
            }
        }
    }
    best.map(|(_, hit)| hit)
}

/// Closest distance from the line `p1`..`p2` to a polygon's boundary,
/// upstream `Polygon_closest_distance_line`.
///
/// A negative result means the line crosses into the polygon, its magnitude
/// being the distance from `p2` back to the crossing nearest `p1`.
///
/// # DIVERGENCE D-011
///
/// Upstream iterates its edges with `for (uint8_t i = 0; i < N-1; i++)` where
/// `N` is `unsigned`. Two things go wrong, and the port's slice-based loop has
/// neither:
///
/// * `N == 0` makes `N-1` wrap to `UINT_MAX`, so the loop reads past the array
///   and never terminates.
/// * `N >= 257` makes the `uint8_t` counter wrap back to 0 before reaching the
///   bound, so the loop never terminates either. `AP_OABendyRuler` takes its
///   point count as a `uint16_t` straight from `AC_PolyFence_loader`, so this
///   is reachable rather than theoretical.
///
/// With fewer than two points there are no edges, and this returns
/// `f32::MAX.sqrt()` — the value upstream's own `closest_sq = FLT_MAX`
/// initialiser yields when its loop body does not run, which it already does
/// for `N == 1`. See DIVERGENCES.md.
pub fn polygon_closest_distance_line(v: &[Vector2f], p1: Vector2f, p2: Vector2f) -> f32 {
    if let Some(hit) = polygon_intersects(v, p1, p2) {
        let dx = hit.x - p2.x;
        let dy = hit.y - p2.y;
        return -Real::sqrt(dx * dx + dy * dy);
    }

    let mut closest_sq = f32::MAX;
    // Upstream walks consecutive pairs and does NOT wrap to close the polygon,
    // relying on the caller passing a closed boundary. Reproduced as-is: adding
    // the closing edge would change results for open polygons.
    for (a, b) in v.iter().zip(v.iter().skip(1)) {
        let d = Vector2f::closest_distance_between_lines_squared(*a, *b, p1, p2);
        if d < closest_sq {
            closest_sq = d;
        }
    }
    Real::sqrt(closest_sq)
}

/// Vector from `p` to the closest point on a polygon's boundary, upstream
/// `Polygon_closest_distance_point`.
///
/// `None` when the polygon has fewer than three distinct points, where there is
/// no boundary to be close to.
pub fn polygon_closest_distance_point(v: &[Vector2f], p: Vector2f) -> Option<Vector2f> {
    let mut n = v.len();
    if polygon_complete(v) && n > 0 {
        n -= 1;
    }
    if n < 3 {
        return None;
    }

    let ring = v.get(..n)?;

    let mut closest_sq = f32::MAX;
    let mut best = Vector2f::new(0.0, 0.0);
    for (&a, &b) in edges(ring) {
        // closest point on segment ab to p, which handles a == b
        let q = Vector2f::closest_point(p, a, b);
        let vec = q - p;
        let vsq = vec.length_squared();
        if vsq < closest_sq {
            closest_sq = vsq;
            best = vec;
        }
    }
    Some(best)
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::indexing_slicing,
        reason = "indexes fixed-size literal arrays declared in the test itself"
    )]
    #![allow(
        clippy::float_cmp,
        reason = "these assert exact geometry on whole-number coordinates, where the results are exactly representable and an epsilon would only hide a real change"
    )]

    use super::*;

    fn square() -> [Vector2f; 5] {
        [
            Vector2f::new(0.0, 0.0),
            Vector2f::new(0.0, 100.0),
            Vector2f::new(100.0, 100.0),
            Vector2f::new(100.0, 0.0),
            Vector2f::new(0.0, 0.0),
        ]
    }

    /// A closed polygon and the same polygon without its repeated last point
    /// must classify every point identically. Upstream achieves this by
    /// trimming, and the trimming is easy to get wrong.
    #[test]
    fn closed_and_open_forms_agree() {
        let closed = square();
        let open = &closed[..4];
        for x in (-150..=150).step_by(10) {
            for y in (-150..=150).step_by(10) {
                let p = Vector2f::new(x as f32, y as f32);
                assert_eq!(
                    polygon_outside(p, &closed),
                    polygon_outside(p, open),
                    "disagreement at ({x}, {y})"
                );
            }
        }
    }

    #[test]
    fn complete_requires_four_points_and_a_repeated_first() {
        let sq = square();
        assert!(polygon_complete(&sq));
        assert!(!polygon_complete(&sq[..4]), "last point is not the first");
        assert!(!polygon_complete(&sq[..3]), "fewer than four points");
        assert!(!polygon_complete(&[]), "empty");
    }

    /// D-011: upstream would loop forever on an empty boundary, because
    /// `N - 1` wraps. The port has no edges to walk and says so.
    #[test]
    fn d011_closest_distance_line_terminates_without_edges() {
        let p1 = Vector2f::new(0.0, 0.0);
        let p2 = Vector2f::new(1.0, 1.0);
        for v in [&[][..], &[Vector2f::new(5.0, 5.0)][..]] {
            let d = polygon_closest_distance_line(v, p1, p2);
            assert!(d.is_finite(), "must terminate with a finite result");
            assert_eq!(d, Real::sqrt(f32::MAX), "no edges means nothing is close");
        }
    }

    /// D-011: upstream's `uint8_t` counter wraps before reaching the bound
    /// above 256 points, so the loop never ends. `AP_OABendyRuler` passes a
    /// `uint16_t` count, so this size is reachable.
    #[test]
    fn d011_closest_distance_line_handles_more_than_256_points() {
        // a many-sided ring, well away from the probe line. no_std, so a
        // fixed-size array rather than a Vec.
        let mut v = [Vector2f::new(0.0, 0.0); 400];
        for (i, e) in v.iter_mut().enumerate() {
            let a = (i as f32) * core::f32::consts::TAU / 400.0;
            *e = Vector2f::new(1000.0 * a.cos(), 1000.0 * a.sin());
        }
        let d = polygon_closest_distance_line(&v, Vector2f::new(0.0, 0.0), Vector2f::new(1.0, 0.0));
        assert!(d.is_finite() && d > 0.0, "got {d}");
    }

    /// A point strictly inside is not outside, and vice versa. Cheap, but it
    /// is the property the whole function exists for.
    #[test]
    fn inside_and_outside_are_classified() {
        let sq = square();
        assert!(!polygon_outside(Vector2f::new(50.0, 50.0), &sq));
        assert!(polygon_outside(Vector2f::new(150.0, 50.0), &sq));
        assert!(polygon_outside(Vector2f::new(-1.0, 50.0), &sq));
    }

    #[test]
    fn intersection_is_the_one_nearest_p1() {
        let sq = square();
        // a line straight through the square from the left
        let hit = polygon_intersects(&sq, Vector2f::new(-50.0, 50.0), Vector2f::new(150.0, 50.0))
            .expect("should cross");
        assert_eq!(hit.x, 0.0, "nearest crossing is the left edge, got {hit:?}");
        assert_eq!(hit.y, 50.0);
    }

    #[test]
    fn a_line_that_misses_does_not_intersect() {
        let sq = square();
        assert!(polygon_intersects(
            &sq,
            Vector2f::new(200.0, 200.0),
            Vector2f::new(300.0, 300.0)
        )
        .is_none());
    }

    #[test]
    fn closest_distance_point_needs_three_points() {
        let sq = square();
        assert!(polygon_closest_distance_point(&sq, Vector2f::new(50.0, 50.0)).is_some());
        assert!(polygon_closest_distance_point(&sq[..2], Vector2f::new(0.0, 0.0)).is_none());
        assert!(polygon_closest_distance_point(&[], Vector2f::new(0.0, 0.0)).is_none());
    }
}
