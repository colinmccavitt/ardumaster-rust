//! `AC_PolyFence_loader` leftover: inclusion / exclusion circles,
//! vertex inclusion / exclusion polygons, and `breached(loc)`.
//! EEPROM scan / index / SD stay later. Upstream
//! `libraries/AC_Fence/AC_PolyFence_loader.cpp`. Tracked as **COP-025**.

use ap_math::location::Location;
use ap_math::polygon::polygon_closest_distance_point;
use ap_math::scalar::{is_equal, is_positive};
use ap_math::vector2::Vector2f;

/// `FENCE_OPTIONS` bit 1. Upstream `AC_Fence::OPTIONS::INCLUSION_UNION`.
pub const OPTION_INCLUSION_UNION: u16 = 1 << 1;

/// In-memory inclusion circles this leftover can hold. EEPROM stays later.
pub const MAX_INCLUSION_CIRCLES: usize = 8;
/// In-memory exclusion circles. Upstream `_loaded_circle_exclusion_boundary`.
pub const MAX_EXCLUSION_CIRCLES: usize = 8;
/// In-memory inclusion polygons. Upstream `_loaded_inclusion_boundary`.
pub const MAX_INCLUSION_POLYGONS: usize = 4;
/// In-memory exclusion polygons. Upstream `_loaded_exclusion_boundary`.
pub const MAX_EXCLUSION_POLYGONS: usize = 4;
/// Vertices stored per in-memory polygon. EEPROM item count stays later.
pub const MAX_POLYGON_VERTICES: usize = 16;

/// One circular inclusion zone. Upstream `AC_PolyFence_loader::InclusionCircle`.
///
/// `point` is absolute lat/lng (1e-7 deg). `radius` is metres.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InclusionCircle {
    /// Latitude, 1e-7 degrees. Upstream `InclusionCircle::point.x`.
    pub lat: i32,
    /// Longitude, 1e-7 degrees. Upstream `InclusionCircle::point.y`.
    pub lng: i32,
    /// Radius in metres.
    pub radius_m: f32,
}

impl InclusionCircle {
    /// Seat a circular inclusion zone.
    #[must_use]
    pub const fn new(lat: i32, lng: i32, radius_m: f32) -> Self {
        Self { lat, lng, radius_m }
    }

    /// Absolute centre. Upstream builds a `Location` from `point.x` / `point.y`.
    #[must_use]
    pub const fn center(self) -> Location {
        Location::new(self.lat, self.lng)
    }
}

/// One circular exclusion zone. Upstream `AC_PolyFence_loader::ExclusionCircle`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExclusionCircle {
    /// Latitude, 1e-7 degrees. Upstream `ExclusionCircle::point.x`.
    pub lat: i32,
    /// Longitude, 1e-7 degrees. Upstream `ExclusionCircle::point.y`.
    pub lng: i32,
    /// Radius in metres.
    pub radius_m: f32,
}

impl ExclusionCircle {
    /// Seat a circular exclusion zone.
    #[must_use]
    pub const fn new(lat: i32, lng: i32, radius_m: f32) -> Self {
        Self { lat, lng, radius_m }
    }

    /// Absolute centre.
    #[must_use]
    pub const fn center(self) -> Location {
        Location::new(self.lat, self.lng)
    }
}

/// One lat/lng vertex. Upstream `Vector2l` (`x` = lat, `y` = lng).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vertex {
    /// Latitude, 1e-7 degrees.
    pub lat: i32,
    /// Longitude, 1e-7 degrees.
    pub lng: i32,
}

impl Vertex {
    /// Seat a vertex.
    #[must_use]
    pub const fn new(lat: i32, lng: i32) -> Self {
        Self { lat, lng }
    }

    /// Absolute location.
    #[must_use]
    pub const fn location(self) -> Location {
        Location::new(self.lat, self.lng)
    }
}

/// In-memory inclusion or exclusion polygon. Upstream `InclusionBoundary` /
/// `ExclusionBoundary` without the cm-from-origin cache.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VertexPolygon {
    vertices: [Vertex; MAX_POLYGON_VERTICES],
    count: u8,
}

impl Default for VertexPolygon {
    fn default() -> Self {
        Self::new()
    }
}

impl VertexPolygon {
    /// Empty polygon.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            vertices: [Vertex::new(0, 0); MAX_POLYGON_VERTICES],
            count: 0,
        }
    }

    /// How many vertices are seated.
    #[must_use]
    pub const fn count(&self) -> u8 {
        self.count
    }

    /// Seated vertices, in order.
    #[must_use]
    pub fn vertices(&self) -> &[Vertex] {
        self.vertices
            .get(..usize::from(self.count))
            .unwrap_or(&[])
    }

    /// Append one vertex. False when the leftover is full.
    pub fn push_vertex(&mut self, vertex: Vertex) -> bool {
        let i = usize::from(self.count);
        let Some(slot) = self.vertices.get_mut(i) else {
            return false;
        };
        *slot = vertex;
        self.count = self.count.saturating_add(1);
        true
    }
}

/// `AC_PolyFence_loader::breached(loc, distance, direction)` leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BreachedLeftover {
    /// C++ return: the location is outside the inclusion set or inside
    /// an exclusion zone.
    pub breached: bool,
    /// `distance_outside_fence` out-param, metres. Positive is outside.
    pub distance_outside_m: f32,
    /// `!loaded() || total_fence_count() == 0`.
    pub skipped: bool,
    /// Inclusion circles and polygons considered.
    pub num_inclusion: u16,
    /// How many of those the location was outside.
    pub num_inclusion_outside: u16,
    /// An exclusion polygon or circle returned true immediately.
    pub exclusion_hit: bool,
}

/// `AC_PolyFence_loader` leftover. Holds in-memory circles and vertex
/// polygons — no EEPROM index, no SD.
#[derive(Debug, Clone, PartialEq)]
pub struct PolyFence {
    inclusion: [InclusionCircle; MAX_INCLUSION_CIRCLES],
    inclusion_count: u8,
    exclusion: [ExclusionCircle; MAX_EXCLUSION_CIRCLES],
    exclusion_count: u8,
    inclusion_poly: [VertexPolygon; MAX_INCLUSION_POLYGONS],
    inclusion_poly_count: u8,
    exclusion_poly: [VertexPolygon; MAX_EXCLUSION_POLYGONS],
    exclusion_poly_count: u8,
    options: u16,
    /// `_load_time_ms != 0`. EEPROM does not set this; the leftover does.
    loaded: bool,
}

impl Default for PolyFence {
    fn default() -> Self {
        Self::new()
    }
}

impl PolyFence {
    /// Empty loader. `loaded()` is false until a zone is seated or
    /// [`Self::set_loaded`] is called.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            inclusion: [InclusionCircle::new(0, 0, 0.0); MAX_INCLUSION_CIRCLES],
            inclusion_count: 0,
            exclusion: [ExclusionCircle::new(0, 0, 0.0); MAX_EXCLUSION_CIRCLES],
            exclusion_count: 0,
            inclusion_poly: [VertexPolygon::new(); MAX_INCLUSION_POLYGONS],
            inclusion_poly_count: 0,
            exclusion_poly: [VertexPolygon::new(); MAX_EXCLUSION_POLYGONS],
            exclusion_poly_count: 0,
            options: 0,
            loaded: false,
        }
    }

    /// `loaded()` — `_load_time_ms != 0`.
    #[must_use]
    pub const fn loaded(&self) -> bool {
        self.loaded
    }

    /// Seed `loaded()` without EEPROM. Used by tests and the fence leftover.
    pub fn set_loaded(&mut self, loaded: bool) {
        self.loaded = loaded;
    }

    /// `FENCE_OPTIONS` bits passed into the loader.
    #[must_use]
    pub const fn options(&self) -> u16 {
        self.options
    }

    /// Set `FENCE_OPTIONS`.
    pub fn set_options(&mut self, options: u16) {
        self.options = options;
    }

    /// Bit 1 — union of inclusion areas instead of intersection.
    #[must_use]
    pub const fn inclusion_union(&self) -> bool {
        self.options & OPTION_INCLUSION_UNION != 0
    }

    /// `get_inclusion_circle_count`.
    #[must_use]
    pub const fn inclusion_circle_count(&self) -> u8 {
        self.inclusion_count
    }

    /// `get_exclusion_circle_count`.
    #[must_use]
    pub const fn exclusion_circle_count(&self) -> u8 {
        self.exclusion_count
    }

    /// `get_inclusion_polygon_count`.
    #[must_use]
    pub const fn inclusion_polygon_count(&self) -> u8 {
        self.inclusion_poly_count
    }

    /// `get_exclusion_polygon_count`.
    #[must_use]
    pub const fn exclusion_polygon_count(&self) -> u8 {
        self.exclusion_poly_count
    }

    /// `total_fence_count` — polygons and circles.
    #[must_use]
    pub const fn total_fence_count(&self) -> u16 {
        (self.inclusion_count as u16)
            .saturating_add(self.exclusion_count as u16)
            .saturating_add(self.inclusion_poly_count as u16)
            .saturating_add(self.exclusion_poly_count as u16)
    }

    /// Drop seated zones and mark the leftover unloaded.
    pub fn clear(&mut self) {
        self.inclusion_count = 0;
        self.exclusion_count = 0;
        self.inclusion_poly_count = 0;
        self.exclusion_poly_count = 0;
        self.loaded = false;
    }

    /// Seat one inclusion circle. Returns false when the leftover is full.
    ///
    /// Seating a zone also marks the leftover loaded — the C++ path
    /// does that only after EEPROM read; this slice has no storage.
    pub fn push_inclusion_circle(&mut self, circle: InclusionCircle) -> bool {
        let i = usize::from(self.inclusion_count);
        let Some(slot) = self.inclusion.get_mut(i) else {
            return false;
        };
        *slot = circle;
        self.inclusion_count = self.inclusion_count.saturating_add(1);
        self.loaded = true;
        true
    }

    /// Seat one exclusion circle. Returns false when the leftover is full.
    pub fn push_exclusion_circle(&mut self, circle: ExclusionCircle) -> bool {
        let i = usize::from(self.exclusion_count);
        let Some(slot) = self.exclusion.get_mut(i) else {
            return false;
        };
        *slot = circle;
        self.exclusion_count = self.exclusion_count.saturating_add(1);
        self.loaded = true;
        true
    }

    /// Seat one inclusion polygon. False when full or fewer than 3 vertices.
    pub fn push_inclusion_polygon(&mut self, poly: VertexPolygon) -> bool {
        if poly.count < 3 {
            return false;
        }
        let i = usize::from(self.inclusion_poly_count);
        let Some(slot) = self.inclusion_poly.get_mut(i) else {
            return false;
        };
        *slot = poly;
        self.inclusion_poly_count = self.inclusion_poly_count.saturating_add(1);
        self.loaded = true;
        true
    }

    /// Seat one exclusion polygon. False when full or fewer than 3 vertices.
    pub fn push_exclusion_polygon(&mut self, poly: VertexPolygon) -> bool {
        if poly.count < 3 {
            return false;
        }
        let i = usize::from(self.exclusion_poly_count);
        let Some(slot) = self.exclusion_poly.get_mut(i) else {
            return false;
        };
        *slot = poly;
        self.exclusion_poly_count = self.exclusion_poly_count.saturating_add(1);
        self.loaded = true;
        true
    }

    /// `AC_PolyFence_loader::check_inclusion_circle_margin`.
    ///
    /// False when any seated inclusion radius is smaller than `margin`.
    #[must_use]
    pub fn check_inclusion_circle_margin(&self, margin: f32) -> bool {
        for i in 0..usize::from(self.inclusion_count) {
            let Some(circle) = self.inclusion.get(i) else {
                break;
            };
            if circle.radius_m < margin {
                return false;
            }
        }
        true
    }

    /// `breached(loc)` — true when the location is outside the inclusion
    /// set or inside an exclusion zone.
    #[must_use]
    pub fn breached(&self, loc: Location) -> bool {
        self.breached_at(loc).breached
    }

    /// `breached(loc, distance_outside_fence, fence_direction)`.
    ///
    /// Walks inclusion polygons, then exclusion polygons, then exclusion
    /// circles, then inclusion circles — the C++ order. `fence_direction`
    /// is not written; circle loops leave it alone and this leftover
    /// reports only the scalar distance.
    #[must_use]
    pub fn breached_at(&self, loc: Location) -> BreachedLeftover {
        if !self.loaded || self.total_fence_count() == 0 {
            return BreachedLeftover {
                breached: false,
                distance_outside_m: 0.0,
                skipped: true,
                num_inclusion: 0,
                num_inclusion_outside: 0,
                exclusion_hit: false,
            };
        }

        let num_inclusion = u16::from(self.inclusion_count)
            .saturating_add(u16::from(self.inclusion_poly_count));
        let mut num_inclusion_outside = 0_u16;
        // Upstream `-FLT_MAX`.
        let mut distance_outside_m = -f32::MAX;

        for i in 0..usize::from(self.inclusion_poly_count) {
            let Some(poly) = self.inclusion_poly.get(i) else {
                break;
            };
            let outside = polygon_outside_lla(Vertex::new(loc.lat, loc.lng), poly.vertices());
            if let Some(distance) = polygon_distance_m(loc, poly) {
                if outside {
                    if is_positive(distance_outside_m) {
                        distance_outside_m = distance_outside_m.min(distance);
                    } else {
                        distance_outside_m = distance;
                    }
                } else {
                    distance_outside_m = distance_outside_m.max(-distance);
                }
            }
            if outside {
                num_inclusion_outside = num_inclusion_outside.saturating_add(1);
            }
        }

        for i in 0..usize::from(self.exclusion_poly_count) {
            let Some(poly) = self.exclusion_poly.get(i) else {
                break;
            };
            let outside = polygon_outside_lla(Vertex::new(loc.lat, loc.lng), poly.vertices());
            if !outside {
                distance_outside_m = polygon_distance_m(loc, poly).unwrap_or(0.0);
                return BreachedLeftover {
                    breached: true,
                    distance_outside_m,
                    skipped: false,
                    num_inclusion,
                    num_inclusion_outside,
                    exclusion_hit: true,
                };
            }
            if let Some(distance) = polygon_distance_m(loc, poly) {
                distance_outside_m = distance_outside_m.max(-distance);
            }
        }

        for i in 0..usize::from(self.exclusion_count) {
            let Some(circle) = self.exclusion.get(i) else {
                break;
            };
            let center = circle.center();
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_precision_loss,
                reason = "upstream stores distance as float metres from Location::get_distance"
            )]
            let diff_m = loc.get_distance(center) as f32;
            let diff_cm = diff_m * 100.0;
            distance_outside_m = distance_outside_m.max(circle.radius_m - diff_m);
            if diff_cm < circle.radius_m * 100.0 {
                return BreachedLeftover {
                    breached: true,
                    distance_outside_m,
                    skipped: false,
                    num_inclusion,
                    num_inclusion_outside,
                    exclusion_hit: true,
                };
            }
        }

        for i in 0..usize::from(self.inclusion_count) {
            let Some(circle) = self.inclusion.get(i) else {
                break;
            };
            let center = circle.center();
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_precision_loss,
                reason = "upstream stores distance as float metres from Location::get_distance"
            )]
            let diff_m = loc.get_distance(center) as f32;
            let diff_cm = diff_m * 100.0;
            distance_outside_m = distance_outside_m.max(diff_m - circle.radius_m);
            if diff_cm > circle.radius_m * 100.0 {
                num_inclusion_outside = num_inclusion_outside.saturating_add(1);
            }
        }

        let breached = if self.inclusion_union() {
            num_inclusion > 0 && num_inclusion == num_inclusion_outside
        } else {
            num_inclusion_outside > 0
        };

        if is_equal(distance_outside_m, -f32::MAX) {
            distance_outside_m = 0.0;
        }

        BreachedLeftover {
            breached,
            distance_outside_m,
            skipped: false,
            num_inclusion,
            num_inclusion_outside,
            exclusion_hit: false,
        }
    }
}

/// Integer `Polygon_outside` leftover. Upstream instantiates
/// `Polygon_outside<int32_t>` on lat/lng (`Vector2l`).
fn polygon_outside_lla(p: Vertex, v: &[Vertex]) -> bool {
    let n = if polygon_complete_lla(v) {
        v.len().saturating_sub(1)
    } else {
        v.len()
    };
    let Some(ring) = v.get(..n) else {
        return true;
    };

    let mut outside = true;
    for i in 0..n {
        let j = if i + 1 >= n { 0 } else { i + 1 };
        let Some(vi) = ring.get(i) else {
            continue;
        };
        let Some(vj) = ring.get(j) else {
            continue;
        };
        if (vi.lng > p.lng) == (vj.lng > p.lng) {
            continue;
        }
        let dx1 = i64::from(p.lat) - i64::from(vi.lat);
        let dx2 = i64::from(vj.lat) - i64::from(vi.lat);
        let dy1 = i64::from(p.lng) - i64::from(vi.lng);
        let dy2 = i64::from(vj.lng) - i64::from(vi.lng);
        let sgn = |x: i64| -> i8 {
            if x < 0 {
                -1
            } else {
                1
            }
        };
        let m1 = sgn(dx1) * sgn(dy2);
        let m2 = sgn(dx2) * sgn(dy1);
        let crosses = if dy2 < 0 {
            m1 > m2 || (m1 == m2 && dx1.saturating_mul(dy2) > dx2.saturating_mul(dy1))
        } else {
            m1 < m2 || (m1 == m2 && dx1.saturating_mul(dy2) < dx2.saturating_mul(dy1))
        };
        if crosses {
            outside = !outside;
        }
    }
    outside
}

fn polygon_complete_lla(v: &[Vertex]) -> bool {
    match (v.first(), v.last()) {
        (Some(first), Some(last)) if v.len() >= 4 => first == last,
        _ => false,
    }
}

/// Closest-boundary distance in metres. Origin is `(0, 0)` — this leftover
/// has no EEPROM `loaded_origin`.
fn polygon_distance_m(loc: Location, poly: &VertexPolygon) -> Option<f32> {
    let origin = Location::new(0, 0);
    let mut pts = [Vector2f::new(0.0, 0.0); MAX_POLYGON_VERTICES];
    let n = usize::from(poly.count);
    for i in 0..n {
        let Some(v) = poly.vertices.get(i) else {
            break;
        };
        if let Some(slot) = pts.get_mut(i) {
            *slot = origin.get_distance_ne(v.location());
        }
    }
    let ring = pts.get(..n)?;
    let pos = origin.get_distance_ne(loc);
    let closest = polygon_closest_distance_point(ring, pos)?;
    Some(closest.length())
}
