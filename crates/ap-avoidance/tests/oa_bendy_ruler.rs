//! Vertical BendyRuler leftover. Tracked as **COP-026**.
//!
//! Horizontal BendyRuler coverage lives in `oa_path_planner.rs`. Lean-angle
//! avoidance in non-GPS modes stays a later leftover.

use ap_avoidance::{
    BendyMarginContext, BendyRuler, DijkstraFenceContext, OaBendyType, OaDbItem, OaPathPlanType,
    OaPathPlannerUsed, OaRetState, PathPlanner, BEARING_INC_VERTICAL_DEG,
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

fn empty_fence() -> DijkstraFenceContext {
    DijkstraFenceContext::default()
}

fn vertical() -> BendyRuler {
    let mut bendy = BendyRuler::new();
    bendy.set_bendy_type_param(2);
    bendy.set_config(5.0);
    bendy
}

fn vertical_with_margin(margin_max_m: f32) -> BendyRuler {
    let mut bendy = BendyRuler::new();
    bendy.set_bendy_type_param(2);
    bendy.set_config(margin_max_m);
    bendy
}

#[test]
fn vertical_probe_increment_matches_upstream() {
    assert!(is_equal(BEARING_INC_VERTICAL_DEG, 90.0));
}

#[test]
fn vertical_type_param_selects_vertical_search() {
    let bendy = vertical();
    assert_eq!(bendy.get_type(), OaBendyType::Vertical);
    assert_eq!(bendy.bendy_type_param(), 2);
}

#[test]
fn vertical_clear_path_is_not_required() {
    let mut bendy = vertical();
    let here = origin();
    let dest = offset(here, 0.0, 40.0);
    let leftover = bendy.update(
        here,
        dest,
        Vector2f::zero(),
        0.0,
        &BendyMarginContext::default(),
    );
    assert_eq!(leftover.bendy_type, OaBendyType::Vertical);
    assert!(!leftover.required);
    assert_eq!(leftover.origin_new.lat, here.lat);
    assert_eq!(leftover.origin_new.lng, here.lng);
    assert!((leftover.destination_new.lat - dest.lat).abs() <= 2);
    assert!((leftover.destination_new.lng - dest.lng).abs() <= 2);
    assert_eq!(leftover.destination_new.alt, dest.alt);
}

#[test]
fn vertical_obstacle_ahead_changes_altitude() {
    let mut bendy = vertical();
    let here = origin();
    let dest = offset(here, 0.0, 80.0);
    let item = OaDbItem {
        pos_neu_m: Vector3f::new(10.0, 0.0, 0.0),
        radius_m: 3.0,
    };
    let leftover = bendy.update(
        here,
        dest,
        Vector2f::new(5.0, 0.0),
        0.0,
        &BendyMarginContext::one_item(here, item),
    );
    assert_eq!(leftover.bendy_type, OaBendyType::Vertical);
    assert!(leftover.required);
    assert_eq!(leftover.origin_new.lat, here.lat);
    assert_eq!(leftover.origin_new.lng, here.lng);
    // Pitch ±90 projects the destination straight up or down.
    assert_ne!(leftover.destination_new.alt, dest.alt);
    assert!((leftover.destination_new.alt - dest.alt).abs() > 1_000);
}

#[test]
fn vertical_obstacle_above_stays_level_or_descends() {
    let mut bendy = vertical();
    let here = origin();
    let dest = offset(here, 0.0, 80.0);
    let item = OaDbItem {
        pos_neu_m: Vector3f::new(10.0, 0.0, 8.0),
        radius_m: 3.0,
    };
    let leftover = bendy.update(
        here,
        dest,
        Vector2f::new(5.0, 0.0),
        0.0,
        &BendyMarginContext::one_item(here, item),
    );
    assert_eq!(leftover.bendy_type, OaBendyType::Vertical);
    // Level pitch 0 is probed first; an overhead blob should not force a climb.
    assert!(leftover.destination_new.alt <= dest.alt);
}

#[test]
fn vertical_side_proximity_keeps_oa_active_on_level_path() {
    let mut bendy = vertical_with_margin(3.0);
    let here = origin();
    let dest = offset(here, 0.0, 40.0);
    // Horizontal step-1/2 stay clear (blobs sit 6 m above/below the path).
    // Sub-tests at pitch ±90 by margin_max (3 m) clip them, so i==0,j==0
    // stays active instead of turning OA off.
    let mut ctx = BendyMarginContext {
        origin: here,
        ..BendyMarginContext::default()
    };
    if let Some(slot) = ctx.items.first_mut() {
        *slot = Some(OaDbItem {
            pos_neu_m: Vector3f::new(7.5, 0.0, 6.0),
            radius_m: 2.0,
        });
    }
    if let Some(slot) = ctx.items.get_mut(1) {
        *slot = Some(OaDbItem {
            pos_neu_m: Vector3f::new(7.5, 0.0, -6.0),
            radius_m: 2.0,
        });
    }
    let leftover = bendy.update(here, dest, Vector2f::new(5.0, 0.0), 0.0, &ctx);
    assert_eq!(leftover.bendy_type, OaBendyType::Vertical);
    assert!(leftover.required);
    assert!((leftover.destination_new.lat - dest.lat).abs() <= 2);
    assert!((leftover.destination_new.lng - dest.lng).abs() <= 2);
    assert_eq!(leftover.destination_new.alt, dest.alt);
}

#[test]
fn planner_vertical_type_reports_bendy_ruler_vertical() {
    let mut planner = PathPlanner::new();
    planner.set_plan_type(OaPathPlanType::BendyRuler);
    planner.set_margin_max_m(5.0);
    planner.init();
    planner
        .bendy_mut()
        .expect("init creates BendyRuler")
        .set_bendy_type_param(2);

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
        OaPathPlannerUsed::BendyRulerVertical
    );
    assert_ne!(leftover.result_destination.alt, dest.alt);
}
