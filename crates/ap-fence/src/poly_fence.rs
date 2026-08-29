//! `AC_PolyFence_loader` leftover: inclusion / exclusion circles,
//! vertex inclusion / exclusion polygons, `breached(loc)`,
//! `load_from_storage`, and SD init. Upstream
//! `libraries/AC_Fence/AC_PolyFence_loader.cpp`. Tracked as **COP-025**.

use crate::poly_fence_storage::{
    index_eeprom, index_fence_count, init_sdcard_storage, read_f32_from_storage,
    read_latlon_from_storage, read_u32_from_storage, scale_latlon_from_origin, FenceIndex,
    PolyFenceType, SdcardFenceContext, SdcardInitLeftover,
};
use ap_math::location::Location;
use ap_math::polygon::polygon_closest_distance_point;
use ap_math::scalar::{is_equal, is_positive};
use ap_math::vector2::Vector2f;

/// `FENCE_OPTIONS` bit 1. Upstream `AC_Fence::OPTIONS::INCLUSION_UNION`.
pub const OPTION_INCLUSION_UNION: u16 = 1 << 1;

/// In-memory inclusion circles this leftover can hold.
pub const MAX_INCLUSION_CIRCLES: usize = 8;
/// In-memory exclusion circles. Upstream `_loaded_circle_exclusion_boundary`.
pub const MAX_EXCLUSION_CIRCLES: usize = 8;
/// In-memory inclusion polygons. Upstream `_loaded_inclusion_boundary`.
pub const MAX_INCLUSION_POLYGONS: usize = 4;
/// In-memory exclusion polygons. Upstream `_loaded_exclusion_boundary`.
pub const MAX_EXCLUSION_POLYGONS: usize = 4;
/// Vertices stored per in-memory polygon.
pub const MAX_POLYGON_VERTICES: usize = 16;

/// Injected leftover of `AP::ahrs().get_origin` and `AP_HAL::millis()`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadFromStorageContext {
    /// EKF origin. `None` is the C++ `!get_origin` path.
    pub origin: Option<Location>,
    /// `AP_HAL::millis()` leftover of `_load_time_ms`.
    pub now_ms: u32,
}

/// `AC_PolyFence_loader::load_from_storage` leftover.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoadFromStorageLeftover {
    /// C++ return.
    pub ok: bool,
    /// `_load_attempted` after the call.
    pub load_attempted: bool,
    /// `_load_time_ms` after the call.
    pub load_time_ms: u32,
    /// Origin was missing — `_load_attempted` was not set.
    pub origin_missing: bool,
    /// `_eeprom_item_count == 0` success path.
    pub empty: bool,
    /// `_load_attempted` was already true; storage was not re-read.
    pub already_attempted: bool,
    /// `!check_indexed()`.
    pub index_failed: bool,
    /// Allocation leftover — more fences than the in-memory leftover holds.
    pub alloc_failed: bool,
    /// Corrupt / invalid item during the walk.
    pub corrupt: bool,
    /// `_loaded_return_point_lla`.
    pub return_point: Option<Vertex>,
    /// Inclusion circles seated.
    pub inclusion_circles: u8,
    /// Exclusion circles seated.
    pub exclusion_circles: u8,
    /// Inclusion polygons seated.
    pub inclusion_polygons: u8,
    /// Exclusion polygons seated.
    pub exclusion_polygons: u8,
}

enum LoadFail {
    Index,
    Origin,
    Alloc,
    Corrupt,
}

impl LoadFromStorageLeftover {
    fn from_state(fence: &PolyFence, already_attempted: bool, empty: bool) -> Self {
        Self {
            ok: fence.loaded || fence.load_time_ms != 0,
            load_attempted: fence.load_attempted,
            load_time_ms: fence.load_time_ms,
            origin_missing: false,
            empty,
            already_attempted,
            index_failed: false,
            alloc_failed: false,
            corrupt: false,
            return_point: fence.return_point,
            inclusion_circles: fence.inclusion_count,
            exclusion_circles: fence.exclusion_count,
            inclusion_polygons: fence.inclusion_poly_count,
            exclusion_polygons: fence.exclusion_poly_count,
        }
    }

    fn failed(fence: &PolyFence, why: LoadFail) -> Self {
        Self {
            ok: false,
            load_attempted: fence.load_attempted,
            load_time_ms: fence.load_time_ms,
            origin_missing: matches!(why, LoadFail::Origin),
            empty: false,
            already_attempted: false,
            index_failed: matches!(why, LoadFail::Index),
            alloc_failed: matches!(why, LoadFail::Alloc),
            corrupt: matches!(why, LoadFail::Corrupt),
            return_point: fence.return_point,
            inclusion_circles: fence.inclusion_count,
            exclusion_circles: fence.exclusion_count,
            inclusion_polygons: fence.inclusion_poly_count,
            exclusion_polygons: fence.exclusion_poly_count,
        }
    }
}

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
        self.vertices.get(..usize::from(self.count)).unwrap_or(&[])
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
/// polygons, plus the EEPROM load / SD-init flags.
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
    /// `_load_time_ms != 0` after a successful EEPROM load, or a seated zone.
    loaded: bool,
    /// `_load_attempted`. Blocks a second EEPROM walk until [`Self::void_index`].
    load_attempted: bool,
    /// `_load_time_ms`. Zero until a successful `load_from_storage`.
    load_time_ms: u32,
    /// `_index_attempted`. Leftover of `check_indexed`.
    index_attempted: bool,
    /// `_indexed`. Leftover of `check_indexed`.
    indexed: bool,
    /// `_loaded_return_point_lla`.
    return_point: Option<Vertex>,
    /// `_failed_sdcard_storage`.
    failed_sdcard_storage: bool,
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
            load_attempted: false,
            load_time_ms: 0,
            index_attempted: false,
            indexed: false,
            return_point: None,
            failed_sdcard_storage: false,
        }
    }

    /// `loaded()` — `_load_time_ms != 0` or a seated in-memory zone.
    #[must_use]
    pub const fn loaded(&self) -> bool {
        self.loaded || self.load_time_ms != 0
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
        self.unload();
        self.load_attempted = false;
        self.void_index();
    }

    /// `unload()`. Drops seated zones and `_load_time_ms`; keeps `_load_attempted`.
    pub fn unload(&mut self) {
        self.inclusion_count = 0;
        self.exclusion_count = 0;
        self.inclusion_poly_count = 0;
        self.exclusion_poly_count = 0;
        self.return_point = None;
        self.loaded = false;
        self.load_time_ms = 0;
    }

    /// `void_index()`. Allows `load_from_storage` to walk EEPROM again.
    pub fn void_index(&mut self) {
        self.index_attempted = false;
        self.indexed = false;
        self.load_attempted = false;
    }

    /// `_load_attempted`.
    #[must_use]
    pub const fn load_attempted(&self) -> bool {
        self.load_attempted
    }

    /// `_load_time_ms`.
    #[must_use]
    pub const fn load_time_ms(&self) -> u32 {
        self.load_time_ms
    }

    /// `_loaded_return_point_lla`.
    #[must_use]
    pub const fn return_point(&self) -> Option<Vertex> {
        self.return_point
    }

    /// `_failed_sdcard_storage`.
    #[must_use]
    pub const fn failed_sdcard_storage(&self) -> bool {
        self.failed_sdcard_storage
    }

    /// `AC_PolyFence_loader::init` SD leftover, then `check_indexed`.
    pub fn init(
        &mut self,
        ctx: SdcardFenceContext,
        total: u16,
        buf: &[u8],
        index: &mut [FenceIndex],
    ) -> SdcardInitLeftover {
        let leftover = init_sdcard_storage(ctx, total, buf, index);
        self.failed_sdcard_storage = leftover.failed_sdcard_storage;
        self.index_attempted = true;
        self.indexed = leftover.indexed;
        if leftover.indexed {
            self.load_attempted = false;
        }
        leftover
    }

    /// `check_indexed`. Indexes once; later calls reuse the flag.
    fn check_indexed(&mut self, buf: &[u8], index: &mut [FenceIndex]) -> bool {
        if !self.index_attempted {
            self.indexed = index_eeprom(buf, index).is_some();
            self.index_attempted = true;
            if self.indexed {
                self.load_attempted = false;
            }
        }
        self.indexed
    }

    /// `load_from_storage`. Walks the EEPROM index into the in-memory leftover.
    ///
    /// The poly-loader semaphore stays later. A missing origin does **not**
    /// set `_load_attempted`, so a later call can retry when AHRS is ready.
    pub fn load_from_storage(
        &mut self,
        buf: &[u8],
        index: &mut [FenceIndex],
        ctx: LoadFromStorageContext,
    ) -> LoadFromStorageLeftover {
        if !self.check_indexed(buf, index) {
            return LoadFromStorageLeftover::failed(self, LoadFail::Index);
        }
        if self.load_attempted {
            return LoadFromStorageLeftover::from_state(self, true, false);
        }
        let Some(origin) = ctx.origin else {
            return LoadFromStorageLeftover::failed(self, LoadFail::Origin);
        };
        let Some(indexed) = index_eeprom(buf, index) else {
            self.indexed = false;
            return LoadFromStorageLeftover::failed(self, LoadFail::Index);
        };

        self.load_attempted = true;
        self.unload();

        if indexed.counts.item_count == 0 {
            self.load_time_ms = ctx.now_ms;
            self.loaded = true;
            return LoadFromStorageLeftover::from_state(self, false, true);
        }

        if !capacity_ok(index, indexed.num_fences) {
            self.unload();
            return LoadFromStorageLeftover::failed(self, LoadFail::Alloc);
        }

        let mut storage_valid = true;
        let n = usize::from(indexed.num_fences).min(index.len());
        let Some(live) = index.get(..n) else {
            self.unload();
            return LoadFromStorageLeftover::failed(self, LoadFail::Corrupt);
        };
        for entry in live {
            if !storage_valid {
                break;
            }
            let mut storage_offset = entry.storage_offset.saturating_add(1);
            match entry.kind {
                PolyFenceType::EndOfStorage => {
                    storage_valid = false;
                }
                PolyFenceType::PolygonInclusion | PolyFenceType::PolygonExclusion => {
                    if entry.count < 3 {
                        storage_valid = false;
                        break;
                    }
                    storage_offset = storage_offset.saturating_add(1);
                    let mut poly = VertexPolygon::new();
                    if !read_polygon_from_storage(
                        buf,
                        &mut storage_offset,
                        entry.count,
                        origin,
                        &mut poly,
                    ) {
                        storage_valid = false;
                        break;
                    }
                    let seated = if entry.kind == PolyFenceType::PolygonInclusion {
                        self.push_inclusion_polygon(poly)
                    } else {
                        self.push_exclusion_polygon(poly)
                    };
                    if !seated {
                        storage_valid = false;
                    }
                }
                PolyFenceType::CircleInclusion
                | PolyFenceType::CircleExclusion
                | PolyFenceType::CircleInclusionInt
                | PolyFenceType::CircleExclusionInt => {
                    let Some((lat, lng)) = read_latlon_from_storage(buf, &mut storage_offset)
                    else {
                        storage_valid = false;
                        break;
                    };
                    let _pos_cm = scale_latlon_from_origin(origin, lat, lng);
                    let radius_m = if matches!(
                        entry.kind,
                        PolyFenceType::CircleInclusionInt | PolyFenceType::CircleExclusionInt
                    ) {
                        let Some(raw) = read_u32_from_storage(buf, &mut storage_offset) else {
                            storage_valid = false;
                            break;
                        };
                        raw as f32
                    } else {
                        let Some(raw) = read_f32_from_storage(buf, &mut storage_offset) else {
                            storage_valid = false;
                            break;
                        };
                        raw
                    };
                    if !is_positive(radius_m) {
                        storage_valid = false;
                        break;
                    }
                    let seated = if matches!(
                        entry.kind,
                        PolyFenceType::CircleInclusion | PolyFenceType::CircleInclusionInt
                    ) {
                        self.push_inclusion_circle(InclusionCircle::new(lat, lng, radius_m))
                    } else {
                        self.push_exclusion_circle(ExclusionCircle::new(lat, lng, radius_m))
                    };
                    if !seated {
                        storage_valid = false;
                    }
                }
                PolyFenceType::ReturnPoint => {
                    if self.return_point.is_some() {
                        storage_valid = false;
                        break;
                    }
                    let Some((lat, lng)) = read_latlon_from_storage(buf, &mut storage_offset)
                    else {
                        storage_valid = false;
                        break;
                    };
                    let _pos_cm = scale_latlon_from_origin(origin, lat, lng);
                    self.return_point = Some(Vertex::new(lat, lng));
                }
            }
        }

        if !storage_valid {
            self.unload();
            return LoadFromStorageLeftover::failed(self, LoadFail::Corrupt);
        }

        self.load_time_ms = ctx.now_ms;
        self.loaded = true;
        LoadFromStorageLeftover::from_state(self, false, false)
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

        let num_inclusion =
            u16::from(self.inclusion_count).saturating_add(u16::from(self.inclusion_poly_count));
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

fn capacity_ok(index: &[FenceIndex], num_fences: u16) -> bool {
    let inc_poly = index_fence_count(index, num_fences, PolyFenceType::PolygonInclusion);
    let exc_poly = index_fence_count(index, num_fences, PolyFenceType::PolygonExclusion);
    let inc_circ =
        index_fence_count(index, num_fences, PolyFenceType::CircleInclusion).saturating_add(
            index_fence_count(index, num_fences, PolyFenceType::CircleInclusionInt),
        );
    let exc_circ =
        index_fence_count(index, num_fences, PolyFenceType::CircleExclusion).saturating_add(
            index_fence_count(index, num_fences, PolyFenceType::CircleExclusionInt),
        );
    usize::from(inc_poly) <= MAX_INCLUSION_POLYGONS
        && usize::from(exc_poly) <= MAX_EXCLUSION_POLYGONS
        && usize::from(inc_circ) <= MAX_INCLUSION_CIRCLES
        && usize::from(exc_circ) <= MAX_EXCLUSION_CIRCLES
}

/// `read_polygon_from_storage`. Seats lat/lng vertices; scales as leftover.
fn read_polygon_from_storage(
    buf: &[u8],
    offset: &mut u16,
    vertex_count: u16,
    origin: Location,
    poly: &mut VertexPolygon,
) -> bool {
    for _ in 0..vertex_count {
        let Some((lat, lng)) = read_latlon_from_storage(buf, offset) else {
            return false;
        };
        let _pos_cm = scale_latlon_from_origin(origin, lat, lng);
        if !poly.push_vertex(Vertex::new(lat, lng)) {
            return false;
        }
    }
    true
}
