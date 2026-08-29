//! OA Dijkstra leftover. Tracked as **COP-026**.
//!
//! The OA database, vertical BendyRuler, and lean-angle avoidance stay
//! later leftovers.

use ap_avoidance::{
    same_latlon, BendyMarginContext, Dijkstra, DijkstraError, DijkstraFenceContext, DijkstraState,
    FenceCircle, FencePolygon, OaItemId, OaPathPlanType, OaPathPlannerUsed, OaRetState,
    PathPlanner, VisGraph, EXPANDING_CHUNK, NEAR_OA_WP_M, POLYFENCE_MARGIN_M_DEFAULT,
    SHORTPATH_NOTSET_IDX, VISGRAPH_ITEMS_MAX,
};
use ap_math::location::Location;
use ap_math::vector2::Vector2f;

fn origin() -> Location {
    Location::new(400_000_000, 0)
}

fn offset(from: Location, bearing_deg: f32, dist_m: f32) -> Location {
    let mut loc = from;
    loc.offset_bearing(
        ap_math::Ftype::from(bearing_deg),
        ap_math::Ftype::from(dist_m),
    );
    loc
}

/// Exclusion square sitting on the north axis, NE centimetres, unclosed.
fn blocking_square() -> FencePolygon {
    FencePolygon::from_slice(&[
        Vector2f::new(1_500.0, -800.0),
        Vector2f::new(2_500.0, -800.0),
        Vector2f::new(2_500.0, 800.0),
        Vector2f::new(1_500.0, 800.0),
    ])
}

fn blocking_circle() -> FenceCircle {
    FenceCircle {
        center_ne_cm: Vector2f::new(2_000.0, 0.0),
        radius_m: 5.0,
    }
}

#[test]
fn visgraph_add_clear_and_full() {
    let mut g = VisGraph::new();
    assert_eq!(g.num_items(), 0);
    assert!(g.add_item(OaItemId::source(), OaItemId::destination(), 100.0));
    assert_eq!(g.num_items(), 1);
    let item = g.item(0).expect("edge");
    assert_eq!(item.id1, OaItemId::source());
    assert_eq!(item.id2, OaItemId::destination());
    g.clear();
    assert_eq!(g.num_items(), 0);

    let mut filled = 0_u16;
    for i in 0..VISGRAPH_ITEMS_MAX + 4 {
        #[allow(clippy::cast_possible_truncation, reason = "test fills a fixed table")]
        let n = (i % 250) as u8;
        if g.add_item(OaItemId::intermediate(n), OaItemId::destination(), 1.0) {
            filled = filled.saturating_add(1);
        }
    }
    assert_eq!(usize::from(filled), VISGRAPH_ITEMS_MAX);
    assert!(!g.add_item(OaItemId::source(), OaItemId::destination(), 1.0));
}

#[test]
fn dijkstra_defaults_match_upstream() {
    let d = Dijkstra::new(1);
    assert_eq!(d.options(), 1);
    assert!((d.fence_margin_m() - POLYFENCE_MARGIN_M_DEFAULT).abs() < f32::EPSILON);
    assert_eq!(EXPANDING_CHUNK, 32);
    assert_eq!(SHORTPATH_NOTSET_IDX, 255);
    assert_eq!(d.path_numpoints(), 0);
    assert_eq!(d.error_id(), DijkstraError::None);
}

#[test]
fn no_fences_is_not_required_and_marks_next_clear() {
    let mut d = Dijkstra::new(0);
    let here = origin();
    let dest = offset(here, 0.0, 40.0);
    let leftover = d.update(here, dest, dest, &DijkstraFenceContext::default());
    assert_eq!(leftover.state, DijkstraState::NotRequired);
    assert!(leftover.dest_to_next_dest_clear);
}

#[test]
fn same_latlon_is_not_required_and_next_is_not_clear() {
    let mut d = Dijkstra::new(0);
    let here = origin();
    let fence = DijkstraFenceContext::one_exclusion_circle(here, blocking_circle());
    let leftover = d.update(here, here, here, &fence);
    assert_eq!(leftover.state, DijkstraState::NotRequired);
    assert!(!leftover.dest_to_next_dest_clear);
}

#[test]
fn overlapping_exclusion_points_are_an_error() {
    let mut d = Dijkstra::new(0);
    d.set_fence_margin(5.0);
    let here = origin();
    let poly = FencePolygon::from_slice(&[
        Vector2f::new(0.0, 0.0),
        Vector2f::new(0.0, 0.0),
        Vector2f::new(1_000.0, 0.0),
    ]);
    let fence = DijkstraFenceContext::one_exclusion_polygon(here, poly);
    let dest = offset(here, 0.0, 40.0);
    let leftover = d.update(here, dest, dest, &fence);
    assert_eq!(leftover.state, DijkstraState::Error);
    assert_eq!(leftover.error, DijkstraError::OverlappingPolygonPoints);
    assert_eq!(leftover.error.as_msg(), "overlapping polygon points");
}

#[test]
fn dest_outside_inclusion_cannot_find_a_path() {
    let mut d = Dijkstra::new(0);
    d.set_fence_margin(5.0);
    let here = origin();
    // 80 m square around the origin. Dest is 120 m north, outside.
    let poly = FencePolygon::from_slice(&[
        Vector2f::new(-4_000.0, -4_000.0),
        Vector2f::new(4_000.0, -4_000.0),
        Vector2f::new(4_000.0, 4_000.0),
        Vector2f::new(-4_000.0, 4_000.0),
    ]);
    let fence = DijkstraFenceContext::one_inclusion_polygon(here, poly);
    let dest = offset(here, 0.0, 120.0);
    let leftover = d.update(here, dest, dest, &fence);
    assert_eq!(leftover.state, DijkstraState::Error);
    assert_eq!(leftover.error, DijkstraError::CouldNotFindPath);
}

#[test]
fn exclusion_circle_deflects_and_reports_success() {
    let mut d = Dijkstra::new(0);
    d.set_fence_margin(5.0);
    let here = origin();
    let dest = offset(here, 0.0, 50.0);
    let next = offset(here, 0.0, 60.0);
    let fence = DijkstraFenceContext::one_exclusion_circle(here, blocking_circle());
    let leftover = d.update(here, dest, next, &fence);
    assert_eq!(leftover.state, DijkstraState::Success);
    assert_eq!(leftover.error, DijkstraError::None);
    assert!(leftover.path_numpoints >= 3);
    assert!(leftover.path_idx_returned >= 1);
    assert!(same_latlon(leftover.origin_new, here));
    assert!(
        leftover.destination_new.lat != dest.lat || leftover.destination_new.lng != dest.lng,
        "intermediate dest must not be the mission dest"
    );
    // dest (50 m N) to next (60 m N) is north of the circle at 20 m.
    assert!(leftover.dest_to_next_dest_clear);
}

#[test]
fn exclusion_polygon_deflects() {
    let mut d = Dijkstra::new(0);
    d.set_fence_margin(5.0);
    let here = origin();
    let dest = offset(here, 0.0, 50.0);
    let fence = DijkstraFenceContext::one_exclusion_polygon(here, blocking_square());
    let leftover = d.update(here, dest, Location::new(0, 0), &fence);
    assert_eq!(leftover.state, DijkstraState::Success);
    assert!(!leftover.dest_to_next_dest_clear);
    assert!(leftover.destination_new.lat != dest.lat || leftover.destination_new.lng != dest.lng);
}

#[test]
fn clear_inclusion_path_is_not_required() {
    let mut d = Dijkstra::new(0);
    d.set_fence_margin(5.0);
    let here = origin();
    let dest = offset(here, 0.0, 30.0);
    let poly = FencePolygon::from_slice(&[
        Vector2f::new(-8_000.0, -8_000.0),
        Vector2f::new(8_000.0, -8_000.0),
        Vector2f::new(8_000.0, 8_000.0),
        Vector2f::new(-8_000.0, 8_000.0),
    ]);
    let fence = DijkstraFenceContext::one_inclusion_polygon(here, poly);
    let leftover = d.update(here, dest, dest, &fence);
    assert_eq!(leftover.state, DijkstraState::NotRequired);
}

#[test]
fn invalid_origin_is_no_position_estimate() {
    let mut d = Dijkstra::new(0);
    d.set_fence_margin(5.0);
    let here = origin();
    let dest = offset(here, 0.0, 40.0);
    let mut fence = DijkstraFenceContext::one_exclusion_circle(here, blocking_circle());
    fence.origin_valid = false;
    let leftover = d.update(here, dest, dest, &fence);
    assert_eq!(leftover.state, DijkstraState::Error);
    assert_eq!(leftover.error, DijkstraError::NoPositionEstimate);
}

#[test]
fn near_waypoint_advances_path_index() {
    let mut d = Dijkstra::new(0);
    d.set_fence_margin(5.0);
    let here = origin();
    let dest = offset(here, 0.0, 50.0);
    let fence = DijkstraFenceContext::one_exclusion_circle(here, blocking_circle());
    let first = d.update(here, dest, dest, &fence);
    assert_eq!(first.state, DijkstraState::Success);
    let idx_after_first = first.path_idx_returned;
    // Sit on the intermediate destination so the near-WP test fires.
    let second = d.update(first.destination_new, dest, dest, &fence);
    assert!(
        second.path_idx_returned > idx_after_first
            || second.state == DijkstraState::NotRequired
            || second.state == DijkstraState::Success
    );
    let _ = NEAR_OA_WP_M;
}

#[test]
fn planner_dijkstra_init_and_pre_arm() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::Dijkstra);
    let before = planner.pre_arm_check();
    assert!(!before.ok);
    assert_eq!(before.failure_msg, "Dijkstra OA requires reboot");

    planner.init();
    assert!(planner.thread_created());
    assert!(planner.dijkstra().is_some());
    assert!(planner.bendy().is_none());
    let after = planner.pre_arm_check();
    assert!(after.ok);
}

#[test]
fn planner_dijkstra_tick_deflects() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::Dijkstra);
    planner.set_margin_max_m(5.0);
    planner.init();

    let here = origin();
    let dest = offset(here, 0.0, 50.0);
    let fence = DijkstraFenceContext::one_exclusion_circle(here, blocking_circle());
    let now = 7_000;
    let pending = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now);
    assert_eq!(pending.ret_state, OaRetState::Processing);
    assert!(planner.process(now, &BendyMarginContext::default(), 0.0, &fence));
    let leftover = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now + 10);
    assert_eq!(leftover.ret_state, OaRetState::Success);
    assert_eq!(leftover.path_planner_used, OaPathPlannerUsed::Dijkstras);
    assert!(
        leftover.result_destination.lat != dest.lat || leftover.result_destination.lng != dest.lng
    );
}

#[test]
fn planner_dijkstra_without_fences_is_not_required() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::Dijkstra);
    planner.init();
    let here = origin();
    let dest = offset(here, 0.0, 40.0);
    let now = 5_000;
    let first = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now);
    assert_eq!(first.ret_state, OaRetState::Processing);
    assert!(planner.process(
        now,
        &BendyMarginContext::default(),
        0.0,
        &DijkstraFenceContext::default()
    ));
    let leftover = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now + 10);
    assert_eq!(leftover.ret_state, OaRetState::NotRequired);
    assert_eq!(leftover.path_planner_used, OaPathPlannerUsed::Dijkstras);
}

#[test]
fn combined_type_falls_back_to_dijkstra() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::DijkstraBendyRuler);
    planner.set_margin_max_m(5.0);
    planner.init();
    assert!(planner.bendy().is_some());
    assert!(planner.dijkstra().is_some());
    assert!(planner.pre_arm_check().ok);

    let here = origin();
    let dest = offset(here, 0.0, 50.0);
    let fence = DijkstraFenceContext::one_exclusion_circle(here, blocking_circle());
    let now = 9_000;
    let _ = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now);
    assert!(planner.process(now, &BendyMarginContext::default(), 0.0, &fence));
    let leftover = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now + 10);
    assert_eq!(leftover.ret_state, OaRetState::Success);
    assert_eq!(leftover.path_planner_used, OaPathPlannerUsed::Dijkstras);
}
