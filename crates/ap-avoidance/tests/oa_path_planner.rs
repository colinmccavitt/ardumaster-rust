//! OA path-planner + BendyRuler leftover. Tracked as **COP-026**.
//!
//! Dijkstra coverage lives in `oa_dijkstra.rs`. The OA database, vertical
//! BendyRuler, and lean-angle avoidance stay later leftovers.

use ap_avoidance::{
    BendyMarginContext, BendyRuler, DijkstraFenceContext, OaBendyType, OaDbItem, OaPathPlanType,
    OaPathPlannerUsed, OaRetState, PathPlanner, LOOKAHEAD_M_DEFAULT, MARGIN_MAX_M_DEFAULT,
    OPTIONS_DEFAULT, OPTION_WP_RESET, TIMEOUT_MS, UPDATE_MS,
};
use ap_math::location::Location;
use ap_math::scalar::is_equal;
use ap_math::vector2::Vector2f;
use ap_math::vector3::Vector3f;

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

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn empty_fence() -> DijkstraFenceContext {
    DijkstraFenceContext::default()
}

#[test]
fn planner_defaults_match_upstream() {
    let planner = PathPlanner::new();
    assert_eq!(planner.plan_type(), OaPathPlanType::Disabled);
    almost(planner.margin_max_m(), MARGIN_MAX_M_DEFAULT);
    assert_eq!(planner.options(), OPTIONS_DEFAULT);
    assert_eq!(planner.options() & OPTION_WP_RESET, OPTION_WP_RESET);
    assert!(!planner.thread_created());
    assert!(planner.bendy().is_none());
    assert!(planner.dijkstra().is_none());
}

#[test]
fn disabled_mission_avoidance_is_not_required() {
    let mut planner = PathPlanner::new();
    planner.init();
    let here = origin();
    let dest = offset(here, 0.0, 50.0);
    let leftover = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), 1_000);
    assert_eq!(leftover.ret_state, OaRetState::NotRequired);
    assert_eq!(leftover.path_planner_used, OaPathPlannerUsed::None);
}

#[test]
fn bendy_pre_arm_requires_init() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::BendyRuler);
    let before = planner.pre_arm_check();
    assert!(!before.ok);
    assert_eq!(before.failure_msg, "BendyRuler OA requires reboot");

    planner.init();
    assert!(planner.thread_created());
    assert!(planner.bendy().is_some());
    almost(planner.bendy().unwrap().lookahead_m(), LOOKAHEAD_M_DEFAULT);
    let after = planner.pre_arm_check();
    assert!(after.ok);
    assert_eq!(after.failure_msg, "");
}

#[test]
fn mission_avoidance_is_processing_until_tick() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::BendyRuler);
    planner.init();
    let here = origin();
    let dest = offset(here, 0.0, 60.0);
    let leftover = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), 2_000);
    assert_eq!(leftover.ret_state, OaRetState::Processing);
}

#[test]
fn mission_avoidance_times_out_without_a_tick() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::BendyRuler);
    planner.init();
    let here = origin();
    let dest = offset(here, 90.0, 40.0);
    let first = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), 1_000);
    assert_eq!(first.ret_state, OaRetState::Processing);
    // Caller keeps polling inside 200 ms so activation is not reset.
    let mut now = 1_000;
    let mut last = first;
    while now < 1_000 + TIMEOUT_MS + 50 {
        now += 50;
        last = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now);
    }
    assert_eq!(last.ret_state, OaRetState::Error);
}

#[test]
fn clear_path_returns_not_required_after_bendy_tick() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::BendyRuler);
    planner.init();
    let here = origin();
    let dest = offset(here, 0.0, 80.0);
    let now = 4_000;
    let pending = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now);
    assert_eq!(pending.ret_state, OaRetState::Processing);

    let ctx = BendyMarginContext {
        origin: here,
        ..BendyMarginContext::default()
    };
    assert!(planner.process(now, &ctx, 0.0, &empty_fence()));
    let leftover = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now + 10);
    assert_eq!(leftover.ret_state, OaRetState::NotRequired);
    assert_eq!(
        leftover.path_planner_used,
        OaPathPlannerUsed::BendyRulerHorizontal
    );
}

#[test]
fn obstacle_ahead_returns_success_and_deflects() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::BendyRuler);
    planner.set_margin_max_m(5.0);
    planner.init();

    let here = origin();
    let dest = offset(here, 0.0, 80.0);
    let item = OaDbItem {
        pos_neu_m: Vector3f::new(10.0, 0.0, 0.0),
        radius_m: 3.0,
    };
    let ctx = BendyMarginContext::one_item(here, item);

    let now = 6_000;
    let pending = planner.mission_avoidance(here, here, dest, dest, Vector2f::new(5.0, 0.0), now);
    assert_eq!(pending.ret_state, OaRetState::Processing);
    assert!(planner.process(now, &ctx, 0.0, &empty_fence()));
    let leftover =
        planner.mission_avoidance(here, here, dest, dest, Vector2f::new(5.0, 0.0), now + 10);
    assert_eq!(leftover.ret_state, OaRetState::Success);
    assert_eq!(
        leftover.path_planner_used,
        OaPathPlannerUsed::BendyRulerHorizontal
    );
    assert_eq!(leftover.result_origin.lat, here.lat);
    assert_eq!(leftover.result_origin.lng, here.lng);
    // Intermediate dest is not the original mission dest.
    assert!(
        leftover.result_destination.lat != dest.lat || leftover.result_destination.lng != dest.lng
    );
}

#[test]
fn process_respects_update_period() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::BendyRuler);
    planner.init();
    let here = origin();
    let dest = offset(here, 0.0, 30.0);
    let ctx = BendyMarginContext {
        origin: here,
        ..BendyMarginContext::default()
    };
    let now = 8_000;
    let _ = planner.mission_avoidance(here, here, dest, dest, Vector2f::zero(), now);
    assert!(planner.process(now, &ctx, 0.0, &empty_fence()));
    assert!(!planner.process(now + UPDATE_MS - 1, &ctx, 0.0, &empty_fence()));
    assert!(planner.process(now + UPDATE_MS, &ctx, 0.0, &empty_fence()));
}

#[test]
fn bendy_update_clear_path_is_not_required() {
    let mut bendy = BendyRuler::new();
    bendy.set_config(5.0);
    let here = origin();
    let dest = offset(here, 45.0, 40.0);
    let ctx = BendyMarginContext {
        origin: here,
        ..BendyMarginContext::default()
    };
    let leftover = bendy.update(here, dest, Vector2f::zero(), 45.0, &ctx);
    assert!(!leftover.required);
    assert_eq!(leftover.bendy_type, OaBendyType::Horizontal);
    assert_eq!(leftover.origin_new.lat, here.lat);
    assert_eq!(leftover.origin_new.lng, here.lng);
}

#[test]
fn bendy_vertical_type_stays_later_leftover() {
    let mut bendy = BendyRuler::new();
    bendy.set_bendy_type_param(2);
    assert_eq!(bendy.get_type(), OaBendyType::Vertical);
    let here = origin();
    let dest = offset(here, 0.0, 20.0);
    let leftover = bendy.update(
        here,
        dest,
        Vector2f::zero(),
        0.0,
        &BendyMarginContext::default(),
    );
    assert_eq!(leftover.bendy_type, OaBendyType::Vertical);
    assert!(!leftover.required);
}

#[test]
fn resist_bearing_change_holds_previous_when_margin_is_not_better() {
    let bendy = BendyRuler::new();
    let here = origin();
    let dest = offset(here, 0.0, 50.0);
    let ctx = BendyMarginContext {
        origin: here,
        ..BendyMarginContext::default()
    };
    let (resisted, _, bearing_out, final_bearing, _) = bendy.resist_bearing_change(
        dest, here, true, 90.0, 10.0, 6.0, dest, 0.0, 90.0, 6.0, &ctx,
    );
    assert!(resisted);
    assert!(is_equal(final_bearing, 0.0));
    assert!(is_equal(bearing_out, 0.0));
}

#[test]
fn resist_bearing_change_resets_when_destination_moves() {
    let bendy = BendyRuler::new();
    let here = origin();
    let dest = offset(here, 0.0, 50.0);
    let dest2 = offset(here, 10.0, 50.0);
    let ctx = BendyMarginContext::default();
    let (resisted, prev_dest, bearing_out, final_bearing, _) = bendy.resist_bearing_change(
        dest2, here, true, 90.0, 10.0, 6.0, dest, 0.0, 90.0, 6.0, &ctx,
    );
    assert!(!resisted);
    assert_eq!(prev_dest.lat, dest2.lat);
    assert!(is_equal(bearing_out, 90.0));
    assert!(is_equal(final_bearing, 90.0));
}

#[test]
fn combined_type_uses_bendy_arm() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::DijkstraBendyRuler);
    planner.init();
    assert!(planner.bendy().is_some());
    assert!(planner.dijkstra().is_some());
    let check = planner.pre_arm_check();
    assert!(check.ok);

    let here = origin();
    let dest = offset(here, 0.0, 80.0);
    let item = OaDbItem {
        pos_neu_m: Vector3f::new(10.0, 0.0, 0.0),
        radius_m: 3.0,
    };
    let ctx = BendyMarginContext::one_item(here, item);
    let now = 9_000;
    let _ = planner.mission_avoidance(here, here, dest, dest, Vector2f::new(4.0, 0.0), now);
    assert!(planner.process(now, &ctx, 0.0, &empty_fence()));
    let leftover =
        planner.mission_avoidance(here, here, dest, dest, Vector2f::new(4.0, 0.0), now + 10);
    assert_eq!(leftover.ret_state, OaRetState::Success);
    assert_eq!(
        leftover.path_planner_used,
        OaPathPlannerUsed::BendyRulerHorizontal
    );
}
