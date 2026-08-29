//! Dijkstra leftover. Upstream `AP_OADijkstra`.
//!
//! This slice is [`Dijkstra::update`]: inclusion / exclusion polygons with
//! margin, exclusion-circle hexagons, the fence visibility graph, and the
//! A*-style shortest path. [`DijkstraFenceContext`] is the leftover of
//! `AP::fence()->polyfence()` and `Location::get_vector_xy_from_origin_NE_cm`.
//!
//! The OA database, vertical BendyRuler, GCS error spam, and visgraph
//! logging stay later leftovers. ADR-0004 forbids the fence / AHRS / HAL
//! singletons.

use ap_math::location::Location;
use ap_math::polygon::{polygon_intersects, polygon_outside};
use ap_math::scalar::{is_equal, radians, wrap_pi, Real};
use ap_math::vector2::Vector2f;
use ap_math::Ftype;

use crate::fence_ne::{FenceCircle, FencePolygon};
use crate::oa_bendy_ruler::same_latlon;
use crate::oa_vis_graph::{OaItemId, VisGraph};

/// Default `_polyfence_margin`, metres. Overridden by `set_fence_margin`.
pub const POLYFENCE_MARGIN_M_DEFAULT: f32 = 10.0;
/// Expanding-array chunk. Upstream `OA_DIJKSTRA_EXPANDING_ARRAY_ELEMENTS_PER_CHUNK`.
pub const EXPANDING_CHUNK: usize = 32;
/// Sentinel "no previous node". Upstream `OA_DIJKSTRA_POLYGON_SHORTPATH_NOTSET_IDX`.
pub const SHORTPATH_NOTSET_IDX: u8 = 255;
/// Advance when this close to the current OA waypoint, metres.
pub const NEAR_OA_WP_M: f32 = 2.0;
/// Max leftover fence-margin vertices (all types combined).
pub const POINTS_MAX: usize = 24;
/// Max leftover shortest-path nodes (source + dest + [`POINTS_MAX`]).
pub const SHORTPATH_MAX: usize = 2 + POINTS_MAX;
/// Max leftover reconstructed path ids.
pub const PATH_MAX: usize = SHORTPATH_MAX;
/// Max injected inclusion / exclusion polygons.
pub const POLYGONS_MAX: usize = 2;
/// Max injected inclusion / exclusion circles.
pub const CIRCLES_MAX: usize = 2;
/// Hexagon vertices around one exclusion circle.
pub const CIRCLE_POINTS: usize = 6;

/// Dijkstra return state. Upstream `AP_OADijkstra_State`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DijkstraState {
    /// Avoidance not required. Upstream `DIJKSTRA_STATE_NOT_REQUIRED`.
    NotRequired = 0,
    /// Planner failed. Upstream `DIJKSTRA_STATE_ERROR`.
    Error = 1,
    /// Intermediate path is ready. Upstream `DIJKSTRA_STATE_SUCCESS`.
    Success = 2,
}

/// Planner error id. Upstream `AP_OADijkstra_Error`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DijkstraError {
    /// No error. Upstream `DIJKSTRA_ERROR_NONE`.
    None = 0,
    /// Fixed table full. Upstream `DIJKSTRA_ERROR_OUT_OF_MEMORY`.
    OutOfMemory = 1,
    /// Duplicate / zero-length polygon edge. Upstream `DIJKSTRA_ERROR_OVERLAPPING_POLYGON_POINTS`.
    OverlappingPolygonPoints = 2,
    /// Unused here. Upstream `DIJKSTRA_ERROR_FAILED_TO_BUILD_INNER_POLYGON`.
    FailedToBuildInnerPolygon = 3,
    /// Could not offset a vertex off the boundary. Upstream `DIJKSTRA_ERROR_OVERLAPPING_POLYGON_LINES`.
    OverlappingPolygonLines = 4,
    /// No fence context. Upstream `DIJKSTRA_ERROR_FENCE_DISABLED`.
    FenceDisabled = 5,
    /// More vertices than [`SHORTPATH_NOTSET_IDX`]. Upstream `DIJKSTRA_ERROR_TOO_MANY_FENCE_POINTS`.
    TooManyFencePoints = 6,
    /// Origin conversion failed. Upstream `DIJKSTRA_ERROR_NO_POSITION_ESTIMATE`.
    NoPositionEstimate = 7,
    /// Graph search failed. Upstream `DIJKSTRA_ERROR_COULD_NOT_FIND_PATH`.
    CouldNotFindPath = 8,
}

impl DijkstraError {
    /// Leftover of `AP_OADijkstra::get_error_msg`.
    #[must_use]
    pub const fn as_msg(self) -> &'static str {
        match self {
            Self::None => "no error",
            Self::OutOfMemory => "out of memory",
            Self::OverlappingPolygonPoints => "overlapping polygon points",
            Self::FailedToBuildInnerPolygon => "failed to build inner polygon",
            Self::OverlappingPolygonLines => "overlapping polygon lines",
            Self::FenceDisabled => "fence disabled",
            Self::TooManyFencePoints => "too many fence points",
            Self::NoPositionEstimate => "no position estimate",
            Self::CouldNotFindPath => "could not find path",
        }
    }
}

/// Injected leftover of `AP::fence()->polyfence()` + EKF origin.
///
/// Vertices / circle centres are earth-frame centimetres in the origin frame,
/// matching [`FencePolygon`] / [`FenceCircle`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DijkstraFenceContext {
    /// Leftover of `get_vector_xy_from_origin_NE_cm` succeeding.
    pub origin_valid: bool,
    /// Leftover of the EKF origin.
    pub origin: Location,
    /// Leftover of `get_enabled_fences() & AC_FENCE_TYPE_POLYGON`.
    pub polygon_fence_enabled: bool,
    /// Inclusion polygons. Unused slots are `None`.
    pub inclusion_polygons: [Option<FencePolygon>; POLYGONS_MAX],
    /// Exclusion polygons.
    pub exclusion_polygons: [Option<FencePolygon>; POLYGONS_MAX],
    /// Inclusion circles (intersection tests only).
    pub inclusion_circles: [Option<FenceCircle>; CIRCLES_MAX],
    /// Exclusion circles (hexagon nodes + intersection tests).
    pub exclusion_circles: [Option<FenceCircle>; CIRCLES_MAX],
    /// Leftover of `get_inclusion_polygon_update_ms`.
    pub inclusion_polygon_update_ms: u32,
    /// Leftover of `get_exclusion_polygon_update_ms`.
    pub exclusion_polygon_update_ms: u32,
    /// Leftover of `get_exclusion_circle_update_ms`.
    pub exclusion_circle_update_ms: u32,
}

impl Default for DijkstraFenceContext {
    fn default() -> Self {
        Self {
            origin_valid: true,
            origin: Location::new(0, 0),
            polygon_fence_enabled: false,
            inclusion_polygons: [None; POLYGONS_MAX],
            exclusion_polygons: [None; POLYGONS_MAX],
            inclusion_circles: [None; CIRCLES_MAX],
            exclusion_circles: [None; CIRCLES_MAX],
            inclusion_polygon_update_ms: 0,
            exclusion_polygon_update_ms: 0,
            exclusion_circle_update_ms: 0,
        }
    }
}

impl DijkstraFenceContext {
    /// One exclusion polygon, polygon fence enabled.
    #[must_use]
    pub fn one_exclusion_polygon(origin: Location, poly: FencePolygon) -> Self {
        let mut ctx = Self {
            origin_valid: true,
            origin,
            polygon_fence_enabled: true,
            inclusion_polygon_update_ms: 1,
            exclusion_polygon_update_ms: 1,
            exclusion_circle_update_ms: 1,
            ..Self::default()
        };
        if let Some(slot) = ctx.exclusion_polygons.get_mut(0) {
            *slot = Some(poly);
        }
        ctx
    }

    /// One exclusion circle, polygon fence enabled.
    #[must_use]
    pub fn one_exclusion_circle(origin: Location, circle: FenceCircle) -> Self {
        let mut ctx = Self {
            origin_valid: true,
            origin,
            polygon_fence_enabled: true,
            inclusion_polygon_update_ms: 1,
            exclusion_polygon_update_ms: 1,
            exclusion_circle_update_ms: 1,
            ..Self::default()
        };
        if let Some(slot) = ctx.exclusion_circles.get_mut(0) {
            *slot = Some(circle);
        }
        ctx
    }

    /// One inclusion polygon, polygon fence enabled.
    #[must_use]
    pub fn one_inclusion_polygon(origin: Location, poly: FencePolygon) -> Self {
        let mut ctx = Self {
            origin_valid: true,
            origin,
            polygon_fence_enabled: true,
            inclusion_polygon_update_ms: 1,
            exclusion_polygon_update_ms: 1,
            exclusion_circle_update_ms: 1,
            ..Self::default()
        };
        if let Some(slot) = ctx.inclusion_polygons.get_mut(0) {
            *slot = Some(poly);
        }
        ctx
    }
}

/// Leftover of one `AP_OADijkstra::update` call.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DijkstraUpdateLeftover {
    /// Return state.
    pub state: DijkstraState,
    /// Intermediate origin when [`Self::state`] is [`DijkstraState::Success`].
    pub origin_new: Location,
    /// Intermediate destination when success.
    pub destination_new: Location,
    /// Next destination (smooth cornering) when success.
    pub next_destination_new: Location,
    /// Path from input dest to input next dest does not cross a fence.
    pub dest_to_next_dest_clear: bool,
    /// Last error id (also set on [`DijkstraState::Error`]).
    pub error: DijkstraError,
    /// Index into the reconstructed path. Upstream `_path_idx_returned`.
    pub path_idx_returned: u8,
    /// Reconstructed path length. Upstream `_path_numpoints`.
    pub path_numpoints: u8,
}

impl Default for DijkstraUpdateLeftover {
    fn default() -> Self {
        Self {
            state: DijkstraState::NotRequired,
            origin_new: Location::new(0, 0),
            destination_new: Location::new(0, 0),
            next_destination_new: Location::new(0, 0),
            dest_to_next_dest_clear: false,
            error: DijkstraError::None,
            path_idx_returned: 0,
            path_numpoints: 0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct ShortPathNode {
    id: OaItemId,
    visited: bool,
    distance_from_idx: u8,
    distance_cm: f32,
}

impl Default for ShortPathNode {
    fn default() -> Self {
        Self {
            id: OaItemId::source(),
            visited: false,
            distance_from_idx: SHORTPATH_NOTSET_IDX,
            distance_cm: f32::MAX,
        }
    }
}

/// Dijkstra leftover. Upstream `AP_OADijkstra`.
#[derive(Debug, Clone)]
pub struct Dijkstra {
    options: u16,
    polyfence_margin_m: f32,
    inclusion_polygon_with_margin_ok: bool,
    exclusion_polygon_with_margin_ok: bool,
    exclusion_circle_with_margin_ok: bool,
    polyfence_visgraph_ok: bool,
    shortest_path_ok: bool,
    destination_prev: Location,
    next_destination_prev: Location,
    path_idx_returned: u8,
    dest_to_next_dest_clear: bool,
    inclusion_polygon_pts: [Vector2f; POINTS_MAX],
    inclusion_polygon_numpoints: u8,
    inclusion_polygon_update_ms: u32,
    exclusion_polygon_pts: [Vector2f; POINTS_MAX],
    exclusion_polygon_numpoints: u8,
    exclusion_polygon_update_ms: u32,
    exclusion_circle_pts: [Vector2f; POINTS_MAX],
    exclusion_circle_numpoints: u8,
    exclusion_circle_update_ms: u32,
    fence_visgraph: VisGraph,
    source_visgraph: VisGraph,
    destination_visgraph: VisGraph,
    short_path_data: [ShortPathNode; SHORTPATH_MAX],
    short_path_data_numpoints: u8,
    path: [OaItemId; PATH_MAX],
    path_numpoints: u8,
    path_source: Vector2f,
    path_destination: Vector2f,
    error_id: DijkstraError,
}

impl Default for Dijkstra {
    fn default() -> Self {
        Self::new(0)
    }
}

impl Dijkstra {
    /// Construct with path-planner `OA_OPTIONS`.
    #[must_use]
    pub fn new(options: u16) -> Self {
        Self {
            options,
            polyfence_margin_m: POLYFENCE_MARGIN_M_DEFAULT,
            inclusion_polygon_with_margin_ok: false,
            exclusion_polygon_with_margin_ok: false,
            exclusion_circle_with_margin_ok: false,
            polyfence_visgraph_ok: false,
            shortest_path_ok: false,
            destination_prev: Location::new(0, 0),
            next_destination_prev: Location::new(0, 0),
            path_idx_returned: 0,
            dest_to_next_dest_clear: false,
            inclusion_polygon_pts: [Vector2f::zero(); POINTS_MAX],
            inclusion_polygon_numpoints: 0,
            inclusion_polygon_update_ms: u32::MAX,
            exclusion_polygon_pts: [Vector2f::zero(); POINTS_MAX],
            exclusion_polygon_numpoints: 0,
            exclusion_polygon_update_ms: u32::MAX,
            exclusion_circle_pts: [Vector2f::zero(); POINTS_MAX],
            exclusion_circle_numpoints: 0,
            exclusion_circle_update_ms: u32::MAX,
            fence_visgraph: VisGraph::new(),
            source_visgraph: VisGraph::new(),
            destination_visgraph: VisGraph::new(),
            short_path_data: [ShortPathNode::default(); SHORTPATH_MAX],
            short_path_data_numpoints: 0,
            path: [OaItemId::source(); PATH_MAX],
            path_numpoints: 0,
            path_source: Vector2f::zero(),
            path_destination: Vector2f::zero(),
            error_id: DijkstraError::None,
        }
    }

    /// Leftover of `AP_OADijkstra::set_fence_margin`.
    pub fn set_fence_margin(&mut self, margin_m: f32) {
        self.polyfence_margin_m = margin_m.max(0.0);
    }

    /// Current fence margin, metres.
    #[must_use]
    pub fn fence_margin_m(&self) -> f32 {
        self.polyfence_margin_m
    }

    /// `OA_OPTIONS` setter (path-planner param load).
    pub fn set_options(&mut self, options: u16) {
        self.options = options;
    }

    /// `OA_OPTIONS`.
    #[must_use]
    pub fn options(&self) -> u16 {
        self.options
    }

    /// Leftover of `AP_OADijkstra::recalculate_path`.
    pub fn recalculate_path(&mut self) {
        self.shortest_path_ok = false;
    }

    /// Last error id.
    #[must_use]
    pub fn error_id(&self) -> DijkstraError {
        self.error_id
    }

    /// Reconstructed path length.
    #[must_use]
    pub fn path_numpoints(&self) -> u8 {
        self.path_numpoints
    }

    /// Leftover of `AP_OADijkstra::update`.
    #[must_use]
    pub fn update(
        &mut self,
        current_loc: Location,
        destination: Location,
        next_destination: Location,
        fence: &DijkstraFenceContext,
    ) -> DijkstraUpdateLeftover {
        let mut leftover = DijkstraUpdateLeftover {
            origin_new: current_loc,
            destination_new: destination,
            next_destination_new: next_destination,
            ..DijkstraUpdateLeftover::default()
        };

        if !self.some_fences_enabled(fence) {
            leftover.dest_to_next_dest_clear = true;
            self.dest_to_next_dest_clear = true;
            leftover.state = DijkstraState::NotRequired;
            return leftover;
        }

        if same_latlon(current_loc, destination) {
            leftover.dest_to_next_dest_clear = false;
            self.dest_to_next_dest_clear = false;
            leftover.state = DijkstraState::NotRequired;
            return leftover;
        }

        if self.check_inclusion_polygon_updated(fence) {
            self.inclusion_polygon_with_margin_ok = false;
            self.polyfence_visgraph_ok = false;
            self.shortest_path_ok = false;
        }
        if self.check_exclusion_polygon_updated(fence) {
            self.exclusion_polygon_with_margin_ok = false;
            self.polyfence_visgraph_ok = false;
            self.shortest_path_ok = false;
        }
        if self.check_exclusion_circle_updated(fence) {
            self.exclusion_circle_with_margin_ok = false;
            self.polyfence_visgraph_ok = false;
            self.shortest_path_ok = false;
        }

        let margin_cm = self.polyfence_margin_m * 100.0;
        if !self.inclusion_polygon_with_margin_ok {
            self.inclusion_polygon_with_margin_ok =
                self.create_inclusion_polygon_with_margin(margin_cm, fence);
            if !self.inclusion_polygon_with_margin_ok {
                return self.fail(&mut leftover, destination);
            }
        }
        if !self.exclusion_polygon_with_margin_ok {
            self.exclusion_polygon_with_margin_ok =
                self.create_exclusion_polygon_with_margin(margin_cm, fence);
            if !self.exclusion_polygon_with_margin_ok {
                return self.fail(&mut leftover, destination);
            }
        }
        if !self.exclusion_circle_with_margin_ok {
            self.exclusion_circle_with_margin_ok =
                self.create_exclusion_circle_with_margin(margin_cm, fence);
            if !self.exclusion_circle_with_margin_ok {
                return self.fail(&mut leftover, destination);
            }
        }

        if !self.polyfence_visgraph_ok {
            self.polyfence_visgraph_ok = self.create_fence_visgraph(fence);
            if !self.polyfence_visgraph_ok {
                self.shortest_path_ok = false;
                return self.fail(&mut leftover, destination);
            }
        }

        if !same_latlon(destination, self.destination_prev)
            || !same_latlon(next_destination, self.next_destination_prev)
        {
            self.destination_prev = destination;
            self.next_destination_prev = next_destination;
            self.shortest_path_ok = false;
        }

        if !self.shortest_path_ok {
            self.shortest_path_ok = self.calc_shortest_path(current_loc, destination, fence);
            if !self.shortest_path_ok {
                return self.fail(&mut leftover, destination);
            }
            self.path_idx_returned = 1;
            self.dest_to_next_dest_clear = false;
            if !loc_is_zero(next_destination) {
                let seg_start = loc_ne_cm(fence.origin, destination);
                let seg_end = loc_ne_cm(fence.origin, next_destination);
                self.dest_to_next_dest_clear = !self.intersects_fence(seg_start, seg_end, fence);
            }
        }

        let path_length = self.path_numpoints.saturating_sub(1);
        if self.path_idx_returned < path_length {
            if let Some(dest_pos) = self.get_shortest_path_point(self.path_idx_returned) {
                leftover.origin_new = if self.path_idx_returned > 0 {
                    self.get_shortest_path_point(self.path_idx_returned - 1)
                        .map(|origin_pos| loc_from_ne_cm(fence.origin, origin_pos))
                        .unwrap_or(current_loc)
                } else {
                    current_loc
                };

                let temp_loc = loc_from_ne_cm(fence.origin, dest_pos);
                leftover.destination_new = destination;
                leftover.destination_new.lat = temp_loc.lat;
                leftover.destination_new.lng = temp_loc.lng;

                leftover.next_destination_new = Location::new(0, 0);
                if self.path_idx_returned + 1 < path_length {
                    if let Some(next_dest_pos) =
                        self.get_shortest_path_point(self.path_idx_returned + 1)
                    {
                        let next_loc = loc_from_ne_cm(fence.origin, next_dest_pos);
                        leftover.next_destination_new = destination;
                        leftover.next_destination_new.lat = next_loc.lat;
                        leftover.next_destination_new.lng = next_loc.lng;
                    }
                } else {
                    leftover.next_destination_new = destination;
                }

                leftover.dest_to_next_dest_clear = self.dest_to_next_dest_clear;
                leftover.path_idx_returned = self.path_idx_returned;
                leftover.path_numpoints = self.path_numpoints;
                leftover.state = DijkstraState::Success;

                let near_oa_wp =
                    metres_f32(current_loc.get_distance(leftover.destination_new)) <= NEAR_OA_WP_M;
                let past_oa_wp = current_loc
                    .past_interval_finish_line(leftover.origin_new, leftover.destination_new);
                if near_oa_wp || past_oa_wp {
                    self.path_idx_returned = self.path_idx_returned.saturating_add(1);
                }
                leftover.path_idx_returned = self.path_idx_returned;
                return leftover;
            }
        }

        leftover.dest_to_next_dest_clear = self.dest_to_next_dest_clear;
        leftover.path_idx_returned = self.path_idx_returned;
        leftover.path_numpoints = self.path_numpoints;
        leftover.state = DijkstraState::NotRequired;
        leftover
    }

    fn fail(
        &mut self,
        leftover: &mut DijkstraUpdateLeftover,
        destination: Location,
    ) -> DijkstraUpdateLeftover {
        leftover.dest_to_next_dest_clear = false;
        self.dest_to_next_dest_clear = false;
        leftover.error = self.error_id;
        leftover.destination_new = destination;
        leftover.state = DijkstraState::Error;
        *leftover
    }

    /// Leftover of `AP_OADijkstra::some_fences_enabled`.
    #[must_use]
    pub fn some_fences_enabled(&self, fence: &DijkstraFenceContext) -> bool {
        let any_poly = filled_polygons(&fence.inclusion_polygons) > 0
            || filled_polygons(&fence.exclusion_polygons) > 0
            || filled_circles(&fence.exclusion_circles) > 0;
        any_poly && fence.polygon_fence_enabled
    }

    fn check_inclusion_polygon_updated(&self, fence: &DijkstraFenceContext) -> bool {
        self.inclusion_polygon_update_ms != fence.inclusion_polygon_update_ms
    }

    fn check_exclusion_polygon_updated(&self, fence: &DijkstraFenceContext) -> bool {
        self.exclusion_polygon_update_ms != fence.exclusion_polygon_update_ms
    }

    fn check_exclusion_circle_updated(&self, fence: &DijkstraFenceContext) -> bool {
        self.exclusion_circle_update_ms != fence.exclusion_circle_update_ms
    }

    fn create_inclusion_polygon_with_margin(
        &mut self,
        margin_cm: f32,
        fence: &DijkstraFenceContext,
    ) -> bool {
        if self.inclusion_polygon_update_ms == fence.inclusion_polygon_update_ms {
            self.error_id = DijkstraError::FenceDisabled;
            return false;
        }
        self.inclusion_polygon_update_ms = fence.inclusion_polygon_update_ms;
        self.inclusion_polygon_numpoints = 0;

        for slot in &fence.inclusion_polygons {
            let Some(poly) = slot else {
                continue;
            };
            let boundary = poly.as_slice();
            let num_points = boundary.len();
            if num_points == 0 {
                continue;
            }
            let mut new_points = 0_u16;
            for j in 0..num_points {
                match offset_vertex(boundary, j, margin_cm, true) {
                    OffsetVertex::Skip => {}
                    OffsetVertex::Fail(err) => {
                        self.error_id = err;
                        return false;
                    }
                    OffsetVertex::Point(temp_point) => {
                        let idx =
                            usize::from(self.inclusion_polygon_numpoints) + usize::from(new_points);
                        let Some(slot) = self.inclusion_polygon_pts.get_mut(idx) else {
                            self.error_id = DijkstraError::OutOfMemory;
                            return false;
                        };
                        *slot = temp_point;
                        new_points = new_points.saturating_add(1);
                    }
                }
            }
            self.inclusion_polygon_numpoints = self
                .inclusion_polygon_numpoints
                .saturating_add(new_points.min(u16::from(u8::MAX)) as u8);
        }
        true
    }

    fn create_exclusion_polygon_with_margin(
        &mut self,
        margin_cm: f32,
        fence: &DijkstraFenceContext,
    ) -> bool {
        if self.exclusion_polygon_update_ms == fence.exclusion_polygon_update_ms {
            self.error_id = DijkstraError::FenceDisabled;
            return false;
        }
        self.exclusion_polygon_update_ms = fence.exclusion_polygon_update_ms;
        self.exclusion_polygon_numpoints = 0;

        for slot in &fence.exclusion_polygons {
            let Some(poly) = slot else {
                continue;
            };
            let boundary = poly.as_slice();
            let num_points = boundary.len();
            if num_points == 0 {
                continue;
            }
            let mut new_points = 0_u16;
            for j in 0..num_points {
                match offset_vertex(boundary, j, margin_cm, false) {
                    OffsetVertex::Skip => {}
                    OffsetVertex::Fail(err) => {
                        self.error_id = err;
                        return false;
                    }
                    OffsetVertex::Point(temp_point) => {
                        let idx =
                            usize::from(self.exclusion_polygon_numpoints) + usize::from(new_points);
                        let Some(slot) = self.exclusion_polygon_pts.get_mut(idx) else {
                            self.error_id = DijkstraError::OutOfMemory;
                            return false;
                        };
                        *slot = temp_point;
                        new_points = new_points.saturating_add(1);
                    }
                }
            }
            self.exclusion_polygon_numpoints = self
                .exclusion_polygon_numpoints
                .saturating_add(new_points.min(u16::from(u8::MAX)) as u8);
        }
        true
    }

    fn create_exclusion_circle_with_margin(
        &mut self,
        margin_cm: f32,
        fence: &DijkstraFenceContext,
    ) -> bool {
        self.exclusion_circle_numpoints = 0;
        let offsets = exclusion_circle_unit_offsets();
        let num_circles = filled_circles(&fence.exclusion_circles);
        let needed = num_circles.saturating_mul(CIRCLE_POINTS);
        if needed > POINTS_MAX {
            self.error_id = DijkstraError::OutOfMemory;
            return false;
        }

        for slot in &fence.exclusion_circles {
            let Some(circle) = slot else {
                continue;
            };
            let scaler = (1.0 / radians(180.0 / CIRCLE_POINTS as f32).cos())
                * ((circle.radius_m * 100.0) + margin_cm);
            for offset in offsets {
                let idx = usize::from(self.exclusion_circle_numpoints);
                let Some(slot) = self.exclusion_circle_pts.get_mut(idx) else {
                    self.error_id = DijkstraError::OutOfMemory;
                    return false;
                };
                *slot = circle.center_ne_cm + (offset * scaler);
                self.exclusion_circle_numpoints = self.exclusion_circle_numpoints.saturating_add(1);
            }
        }
        self.exclusion_circle_update_ms = fence.exclusion_circle_update_ms;
        true
    }

    /// Total margin vertices. Upstream `total_numpoints`.
    #[must_use]
    pub fn total_numpoints(&self) -> u16 {
        u16::from(self.inclusion_polygon_numpoints)
            + u16::from(self.exclusion_polygon_numpoints)
            + u16::from(self.exclusion_circle_numpoints)
    }

    /// One margin vertex. Upstream `get_point`.
    #[must_use]
    pub fn get_point(&self, index: u16) -> Option<Vector2f> {
        let mut index = index;
        if index < u16::from(self.inclusion_polygon_numpoints) {
            return self.inclusion_polygon_pts.get(usize::from(index)).copied();
        }
        index = index.saturating_sub(u16::from(self.inclusion_polygon_numpoints));
        if index < u16::from(self.exclusion_polygon_numpoints) {
            return self.exclusion_polygon_pts.get(usize::from(index)).copied();
        }
        index = index.saturating_sub(u16::from(self.exclusion_polygon_numpoints));
        if index < u16::from(self.exclusion_circle_numpoints) {
            return self.exclusion_circle_pts.get(usize::from(index)).copied();
        }
        None
    }

    /// Leftover of `AP_OADijkstra::intersects_fence`.
    #[must_use]
    pub fn intersects_fence(
        &self,
        seg_start: Vector2f,
        seg_end: Vector2f,
        fence: &DijkstraFenceContext,
    ) -> bool {
        for slot in &fence.inclusion_polygons {
            if let Some(poly) = slot {
                if polygon_intersects(poly.as_slice(), seg_start, seg_end).is_some() {
                    return true;
                }
            }
        }
        for slot in &fence.exclusion_polygons {
            if let Some(poly) = slot {
                if polygon_intersects(poly.as_slice(), seg_start, seg_end).is_some() {
                    return true;
                }
            }
        }
        for slot in &fence.inclusion_circles {
            if let Some(circle) = slot {
                let radius_cm_sq = (circle.radius_m * 100.0) * (circle.radius_m * 100.0);
                if (seg_start - circle.center_ne_cm).length_squared() > radius_cm_sq {
                    return true;
                }
                if (seg_end - circle.center_ne_cm).length_squared() > radius_cm_sq {
                    return true;
                }
            }
        }
        for slot in &fence.exclusion_circles {
            if let Some(circle) = slot {
                let dist_cm = Vector2f::closest_distance_between_line_and_point(
                    seg_start,
                    seg_end,
                    circle.center_ne_cm,
                );
                if dist_cm <= circle.radius_m * 100.0 {
                    return true;
                }
            }
        }
        false
    }

    fn create_fence_visgraph(&mut self, fence: &DijkstraFenceContext) -> bool {
        let total = self.total_numpoints();
        if total >= u16::from(SHORTPATH_NOTSET_IDX) {
            self.error_id = DijkstraError::TooManyFencePoints;
            return false;
        }
        self.fence_visgraph.clear();
        if total < 2 {
            return true;
        }
        for i in 0..total.saturating_sub(1) {
            let Some(start_seg) = self.get_point(i) else {
                continue;
            };
            for j in i.saturating_add(1)..total {
                let Some(end_seg) = self.get_point(j) else {
                    continue;
                };
                if self.intersects_fence(start_seg, end_seg, fence) {
                    continue;
                }
                #[allow(
                    clippy::cast_possible_truncation,
                    reason = "upstream stores visgraph ids as uint8_t"
                )]
                let i8 = i as u8;
                #[allow(
                    clippy::cast_possible_truncation,
                    reason = "upstream stores visgraph ids as uint8_t"
                )]
                let j8 = j as u8;
                if !self.fence_visgraph.add_item(
                    OaItemId::intermediate(i8),
                    OaItemId::intermediate(j8),
                    (start_seg - end_seg).length(),
                ) {
                    self.error_id = DijkstraError::OutOfMemory;
                    return false;
                }
            }
        }
        true
    }

    fn update_visgraph(
        &mut self,
        which: VisWhich,
        oaid: OaItemId,
        position: Vector2f,
        extra_position: Option<Vector2f>,
        fence: &DijkstraFenceContext,
    ) -> bool {
        {
            let visgraph = match which {
                VisWhich::Source => &mut self.source_visgraph,
                VisWhich::Destination => &mut self.destination_visgraph,
                VisWhich::Fence => return false,
            };
            visgraph.clear();
        }
        for i in 0..self.total_numpoints() {
            let Some(seg_end) = self.get_point(i) else {
                continue;
            };
            if self.intersects_fence(position, seg_end, fence) {
                continue;
            }
            #[allow(
                clippy::cast_possible_truncation,
                reason = "upstream stores visgraph ids as uint8_t"
            )]
            let i8 = i as u8;
            let visgraph = match which {
                VisWhich::Source => &mut self.source_visgraph,
                VisWhich::Destination => &mut self.destination_visgraph,
                VisWhich::Fence => return false,
            };
            if !visgraph.add_item(
                oaid,
                OaItemId::intermediate(i8),
                (position - seg_end).length(),
            ) {
                return false;
            }
        }
        if let Some(extra) = extra_position {
            if !self.intersects_fence(position, extra, fence) {
                let visgraph = match which {
                    VisWhich::Source => &mut self.source_visgraph,
                    VisWhich::Destination => &mut self.destination_visgraph,
                    VisWhich::Fence => return false,
                };
                if !visgraph.add_item(oaid, OaItemId::destination(), (position - extra).length()) {
                    return false;
                }
            }
        }
        true
    }

    fn update_visible_node_distances(&mut self, curr_node_idx: u8) {
        if usize::from(curr_node_idx) >= usize::from(self.short_path_data_numpoints) {
            return;
        }
        let Some(curr_node) = self
            .short_path_data
            .get(usize::from(curr_node_idx))
            .copied()
        else {
            return;
        };
        for which in [VisWhich::Fence, VisWhich::Destination] {
            let count = match which {
                VisWhich::Fence => self.fence_visgraph.num_items(),
                VisWhich::Destination => self.destination_visgraph.num_items(),
                VisWhich::Source => 0,
            };
            if count == 0 {
                continue;
            }
            for i in 0..count {
                let Some(item) = (match which {
                    VisWhich::Fence => self.fence_visgraph.item(i),
                    VisWhich::Destination => self.destination_visgraph.item(i),
                    VisWhich::Source => None,
                }) else {
                    continue;
                };
                if curr_node.id != item.id1 && curr_node.id != item.id2 {
                    continue;
                }
                let matching_id = if curr_node.id == item.id1 {
                    item.id2
                } else {
                    item.id1
                };
                let Some(item_node_idx) = self.find_node_from_id(matching_id) else {
                    continue;
                };
                let Some(curr_dist) = self
                    .short_path_data
                    .get(usize::from(curr_node_idx))
                    .map(|n| n.distance_cm)
                else {
                    continue;
                };
                let via = curr_dist + item.distance_cm;
                let Some(node) = self.short_path_data.get_mut(usize::from(item_node_idx)) else {
                    continue;
                };
                if via < node.distance_cm {
                    node.distance_cm = via;
                    node.distance_from_idx = curr_node_idx;
                }
            }
        }
    }

    fn find_node_from_id(&self, id: OaItemId) -> Option<u8> {
        match id.id_type {
            crate::oa_vis_graph::OaType::Source => {
                if self.short_path_data_numpoints > 0 {
                    Some(0)
                } else {
                    None
                }
            }
            crate::oa_vis_graph::OaType::Destination => {
                if self.short_path_data_numpoints > 1 {
                    Some(1)
                } else {
                    None
                }
            }
            crate::oa_vis_graph::OaType::IntermediatePoint => {
                let idx = id.id_num.saturating_add(2);
                if self.short_path_data_numpoints > idx {
                    Some(idx)
                } else {
                    None
                }
            }
        }
    }

    fn find_closest_node_idx(&self) -> Option<u8> {
        let mut lowest_idx = 0_u8;
        let mut lowest_dist = f32::MAX;
        for i in 0..self.short_path_data_numpoints {
            let Some(node) = self.short_path_data.get(usize::from(i)) else {
                continue;
            };
            if node.visited || is_equal(node.distance_cm, f32::MAX) {
                continue;
            }
            let Some(node_pos) = self.convert_node_to_point(node.id) else {
                return None;
            };
            let heuristics = (node_pos - self.path_destination).length();
            let dist_with_heuristics = node.distance_cm + heuristics;
            if dist_with_heuristics < lowest_dist {
                lowest_idx = i;
                lowest_dist = dist_with_heuristics;
            }
        }
        if lowest_dist < f32::MAX {
            Some(lowest_idx)
        } else {
            None
        }
    }

    fn calc_shortest_path(
        &mut self,
        origin: Location,
        destination: Location,
        fence: &DijkstraFenceContext,
    ) -> bool {
        if !fence.origin_valid {
            self.error_id = DijkstraError::NoPositionEstimate;
            return false;
        }
        self.path_source = loc_ne_cm(fence.origin, origin);
        self.path_destination = loc_ne_cm(fence.origin, destination);

        if !self.update_visgraph(
            VisWhich::Source,
            OaItemId::source(),
            self.path_source,
            Some(self.path_destination),
            fence,
        ) {
            self.error_id = DijkstraError::OutOfMemory;
            return false;
        }
        if !self.update_visgraph(
            VisWhich::Destination,
            OaItemId::destination(),
            self.path_destination,
            None,
            fence,
        ) {
            self.error_id = DijkstraError::OutOfMemory;
            return false;
        }

        let needed = 2_u16.saturating_add(self.total_numpoints());
        if usize::from(needed) > SHORTPATH_MAX {
            self.error_id = DijkstraError::OutOfMemory;
            return false;
        }

        if let Some(slot) = self.short_path_data.get_mut(0) {
            *slot = ShortPathNode {
                id: OaItemId::source(),
                visited: false,
                distance_from_idx: 0,
                distance_cm: 0.0,
            };
        }
        if let Some(slot) = self.short_path_data.get_mut(1) {
            *slot = ShortPathNode {
                id: OaItemId::destination(),
                visited: false,
                distance_from_idx: SHORTPATH_NOTSET_IDX,
                distance_cm: f32::MAX,
            };
        }
        self.short_path_data_numpoints = 2;
        for i in 0..self.total_numpoints() {
            #[allow(
                clippy::cast_possible_truncation,
                reason = "upstream stores node ids as uint8_t"
            )]
            let i8 = i as u8;
            let idx = usize::from(self.short_path_data_numpoints);
            let Some(slot) = self.short_path_data.get_mut(idx) else {
                self.error_id = DijkstraError::OutOfMemory;
                return false;
            };
            *slot = ShortPathNode {
                id: OaItemId::intermediate(i8),
                visited: false,
                distance_from_idx: SHORTPATH_NOTSET_IDX,
                distance_cm: f32::MAX,
            };
            self.short_path_data_numpoints = self.short_path_data_numpoints.saturating_add(1);
        }

        let mut current_node_idx = 0_u8;
        for i in 0..self.source_visgraph.num_items() {
            let Some(item) = self.source_visgraph.item(i) else {
                continue;
            };
            let Some(node_idx) = self.find_node_from_id(item.id2) else {
                self.error_id = DijkstraError::CouldNotFindPath;
                return false;
            };
            if let Some(node) = self.short_path_data.get_mut(usize::from(node_idx)) {
                node.distance_cm = item.distance_cm;
                node.distance_from_idx = current_node_idx;
            }
        }
        if let Some(node) = self.short_path_data.get_mut(usize::from(current_node_idx)) {
            node.visited = true;
        }

        while let Some(next_idx) = self.find_closest_node_idx() {
            current_node_idx = next_idx;
            if self.find_node_from_id(OaItemId::destination()) == Some(current_node_idx) {
                break;
            }
            self.update_visible_node_distances(current_node_idx);
            if let Some(node) = self.short_path_data.get_mut(usize::from(current_node_idx)) {
                node.visited = true;
            }
        }

        let Some(mut nidx) = self.find_node_from_id(OaItemId::destination()) else {
            self.error_id = DijkstraError::CouldNotFindPath;
            return false;
        };
        self.path_numpoints = 0;
        let mut success = false;
        loop {
            let idx = usize::from(self.path_numpoints);
            if idx >= PATH_MAX {
                self.error_id = DijkstraError::OutOfMemory;
                return false;
            }
            let Some(node) = self.short_path_data.get(usize::from(nidx)).copied() else {
                break;
            };
            if node.distance_from_idx == SHORTPATH_NOTSET_IDX || node.distance_cm >= f32::MAX {
                break;
            }
            if let Some(slot) = self.path.get_mut(idx) {
                *slot = node.id;
            }
            self.path_numpoints = self.path_numpoints.saturating_add(1);
            if node.id.id_type == crate::oa_vis_graph::OaType::Source {
                success = true;
                break;
            }
            nidx = node.distance_from_idx;
        }
        if !success {
            self.error_id = DijkstraError::CouldNotFindPath;
        }
        success
    }

    /// Leftover of `get_shortest_path_point`.
    #[must_use]
    pub fn get_shortest_path_point(&self, point_num: u8) -> Option<Vector2f> {
        if self.path_numpoints == 0 || point_num >= self.path_numpoints {
            return None;
        }
        let idx = usize::from(self.path_numpoints - point_num - 1);
        let id = self.path.get(idx).copied()?;
        self.convert_node_to_point(id)
    }

    fn convert_node_to_point(&self, id: OaItemId) -> Option<Vector2f> {
        match id.id_type {
            crate::oa_vis_graph::OaType::Source => Some(self.path_source),
            crate::oa_vis_graph::OaType::Destination => Some(self.path_destination),
            crate::oa_vis_graph::OaType::IntermediatePoint => self.get_point(u16::from(id.id_num)),
        }
    }
}

#[derive(Clone, Copy)]
enum VisWhich {
    Source,
    Destination,
    Fence,
}

enum OffsetVertex {
    Skip,
    Fail(DijkstraError),
    Point(Vector2f),
}

/// Offset one polygon vertex by `margin_cm`.
///
/// `want_inside` is inclusion (point must not be outside) vs exclusion
/// (point must be outside).
fn offset_vertex(
    boundary: &[Vector2f],
    j: usize,
    margin_cm: f32,
    want_inside: bool,
) -> OffsetVertex {
    let num_points = boundary.len();
    let Some(&here) = boundary.get(j) else {
        return OffsetVertex::Fail(DijkstraError::OverlappingPolygonPoints);
    };
    let before_idx = if j == 0 { num_points - 1 } else { j - 1 };
    let after_idx = if j + 1 == num_points { 0 } else { j + 1 };
    let Some(&before_raw) = boundary.get(before_idx) else {
        return OffsetVertex::Fail(DijkstraError::OverlappingPolygonPoints);
    };
    let Some(&after_raw) = boundary.get(after_idx) else {
        return OffsetVertex::Fail(DijkstraError::OverlappingPolygonPoints);
    };
    let mut before_pt = before_raw - here;
    let mut after_pt = after_raw - here;
    if before_pt.is_zero() || after_pt.is_zero() || before_pt == after_pt {
        return OffsetVertex::Fail(DijkstraError::OverlappingPolygonPoints);
    }
    if !before_pt.normalize() || !after_pt.normalize() {
        return OffsetVertex::Fail(DijkstraError::OverlappingPolygonPoints);
    }
    let mut intermediate_pt = after_pt + before_pt;
    if !intermediate_pt.normalize() {
        return OffsetVertex::Fail(DijkstraError::OverlappingPolygonLines);
    }
    intermediate_pt *= margin_cm;

    let mut temp_point = here + intermediate_pt;
    let first_wrong = if want_inside {
        polygon_outside(temp_point, boundary)
    } else {
        !polygon_outside(temp_point, boundary)
    };
    if first_wrong {
        intermediate_pt *= -1.0;
        temp_point = here + intermediate_pt;
        let still_wrong = if want_inside {
            polygon_outside(temp_point, boundary)
        } else {
            !polygon_outside(temp_point, boundary)
        };
        if still_wrong {
            return OffsetVertex::Fail(DijkstraError::OverlappingPolygonLines);
        }
    }

    if wrap_pi(intermediate_pt.angle() - before_pt.angle()).abs() < core::f32::consts::FRAC_PI_2 {
        return OffsetVertex::Skip;
    }
    OffsetVertex::Point(temp_point)
}

fn exclusion_circle_unit_offsets() -> [Vector2f; CIRCLE_POINTS] {
    let deg = [30.0_f32, 90.0, 150.0, 210.0, 270.0, 330.0];
    let mut out = [Vector2f::zero(); CIRCLE_POINTS];
    for (i, &d) in deg.iter().enumerate() {
        if let Some(slot) = out.get_mut(i) {
            *slot = Vector2f::new(radians(d).cos(), radians(d - 90.0).cos());
        }
    }
    out
}

fn filled_polygons(slots: &[Option<FencePolygon>; POLYGONS_MAX]) -> usize {
    slots
        .iter()
        .filter(|s| s.as_ref().is_some_and(|p| p.num_points > 0))
        .count()
}

fn filled_circles(slots: &[Option<FenceCircle>; CIRCLES_MAX]) -> usize {
    slots.iter().filter(|s| s.is_some()).count()
}

fn loc_is_zero(loc: Location) -> bool {
    loc.lat == 0 && loc.lng == 0
}

fn loc_ne_cm(origin: Location, loc: Location) -> Vector2f {
    let ne = origin.get_distance_ne(loc);
    Vector2f::new(ne.x * 100.0, ne.y * 100.0)
}

fn loc_from_ne_cm(origin: Location, pos_cm: Vector2f) -> Location {
    let mut loc = origin;
    loc.offset(Ftype::from(pos_cm.x / 100.0), Ftype::from(pos_cm.y / 100.0));
    loc
}

fn metres_f32(v: Ftype) -> f32 {
    #[allow(
        clippy::cast_possible_truncation,
        reason = "upstream OA waypoint proximity is a float metres compare"
    )]
    {
        v.to_f64() as f32
    }
}
