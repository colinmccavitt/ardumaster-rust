//! First `AC_PolyFence_loader` leftover: inclusion-circle `breached()`
//! and `check_inclusion_circle_margin`, plus `AC_Fence::check_fence_polygon`.
//!
//! Tracked as **COP-025**. EEPROM / SD storage is not in this slice.

use ap_fence::{
    CheckContext, CheckPolygonContext, Fence, InclusionCircle, PolyFence, MAX_INCLUSION_CIRCLES,
    OPTION_INCLUSION_UNION, TYPE_ALT_MAX, TYPE_POLYGON,
};
use ap_math::location::Location;
use ap_math::scalar::is_equal;

fn almost(a: f32, b: f32) {
    assert!(is_equal(a, b), "{a} != {b}");
}

fn seat_home_circle(radius_m: f32) -> PolyFence {
    let mut loader = PolyFence::new();
    assert!(loader.push_inclusion_circle(InclusionCircle::new(0, 0, radius_m)));
    loader
}

#[test]
fn inclusion_union_bit_matches_upstream() {
    assert_eq!(OPTION_INCLUSION_UNION, 1 << 1);
    assert_eq!(MAX_INCLUSION_CIRCLES, 8);
}

#[test]
fn unloaded_or_empty_loader_is_not_breached() {
    let loader = PolyFence::new();
    assert!(!loader.loaded());
    assert_eq!(loader.total_fence_count(), 0);
    let leftover = loader.breached_at(Location::new(0, 0));
    assert!(leftover.skipped);
    assert!(!leftover.breached);
    almost(leftover.distance_outside_m, 0.0);

    let mut marked = PolyFence::new();
    marked.set_loaded(true);
    let empty = marked.breached_at(Location::new(0, 0));
    assert!(empty.skipped);
    assert!(!empty.breached);
}

#[test]
fn inside_inclusion_circle_is_not_breached() {
    let loader = seat_home_circle(300.0);
    assert!(loader.loaded());
    assert_eq!(loader.inclusion_circle_count(), 1);
    let leftover = loader.breached_at(Location::new(0, 0));
    assert!(!leftover.skipped);
    assert!(!leftover.breached);
    assert_eq!(leftover.num_inclusion, 1);
    assert_eq!(leftover.num_inclusion_outside, 0);
    assert!(leftover.distance_outside_m < 0.0);
    almost(leftover.distance_outside_m, -300.0);
    assert!(!loader.breached(Location::new(0, 0)));
}

#[test]
fn outside_inclusion_circle_is_breached() {
    let loader = seat_home_circle(300.0);
    // ~1.1 km north of the origin — well outside 300 m.
    let loc = Location::new(100_000, 0);
    let leftover = loader.breached_at(loc);
    assert!(leftover.breached);
    assert_eq!(leftover.num_inclusion_outside, 1);
    assert!(leftover.distance_outside_m > 0.0);
    assert!(loader.breached(loc));
}

#[test]
fn intersection_breaches_if_outside_any_inclusion_circle() {
    let mut loader = seat_home_circle(300.0);
    assert!(loader.push_inclusion_circle(InclusionCircle::new(100_000, 0, 300.0)));
    assert!(!loader.inclusion_union());

    let at_home = loader.breached_at(Location::new(0, 0));
    assert!(at_home.breached);
    assert_eq!(at_home.num_inclusion, 2);
    assert_eq!(at_home.num_inclusion_outside, 1);

    let far = loader.breached_at(Location::new(200_000, 0));
    assert!(far.breached);
    assert_eq!(far.num_inclusion_outside, 2);
}

#[test]
fn union_breaches_only_when_outside_every_inclusion_circle() {
    let mut loader = seat_home_circle(300.0);
    assert!(loader.push_inclusion_circle(InclusionCircle::new(100_000, 0, 300.0)));
    loader.set_options(OPTION_INCLUSION_UNION);
    assert!(loader.inclusion_union());

    let at_home = loader.breached_at(Location::new(0, 0));
    assert!(!at_home.breached);
    assert_eq!(at_home.num_inclusion_outside, 1);

    let far = loader.breached_at(Location::new(200_000, 0));
    assert!(far.breached);
    assert_eq!(far.num_inclusion_outside, 2);
}

#[test]
fn inclusion_circle_margin_refuses_a_tight_radius() {
    let loader = seat_home_circle(5.0);
    assert!(loader.check_inclusion_circle_margin(2.0));
    assert!(!loader.check_inclusion_circle_margin(10.0));
    assert!(PolyFence::new().check_inclusion_circle_margin(10.0));
}

#[test]
fn push_stops_at_the_in_memory_cap() {
    let mut loader = PolyFence::new();
    for i in 0..MAX_INCLUSION_CIRCLES {
        assert!(
            loader.push_inclusion_circle(InclusionCircle::new(i as i32, 0, 10.0)),
            "seat {i}"
        );
    }
    assert!(!loader.push_inclusion_circle(InclusionCircle::new(99, 0, 10.0)));
    assert_eq!(
        loader.inclusion_circle_count() as usize,
        MAX_INCLUSION_CIRCLES
    );
}

fn enable_polygon(fence: &mut Fence) {
    fence.set_configured_fences(TYPE_POLYGON);
    fence.set_poly_fence_count(1);
    let leftover = fence.enable(true, TYPE_POLYGON, false);
    assert_eq!(leftover.changed_mask, TYPE_POLYGON);
    assert_eq!(fence.get_enabled_fences() & TYPE_POLYGON, TYPE_POLYGON);
}

#[test]
fn check_fence_polygon_disabled_clears_a_stale_breach() {
    let mut fence = Fence::new();
    fence.set_configured_fences(TYPE_POLYGON);
    fence.set_poly_fence_count(1);
    fence.enable(true, TYPE_POLYGON, false);
    assert!(
        fence
            .check_fence_polygon(CheckPolygonContext {
                loc_valid: true,
                poly_breached: true,
                distance_outside_m: 12.0,
                now_ms: 1_001,
            })
            .newly_breached
    );
    assert_eq!(fence.get_breaches() & TYPE_POLYGON, TYPE_POLYGON);

    fence.enable(false, TYPE_POLYGON, false);
    let leftover = fence.check_fence_polygon(CheckPolygonContext::default());
    assert!(!leftover.enabled);
    assert!(!leftover.newly_breached);
    assert_eq!(fence.get_breaches() & TYPE_POLYGON, 0);
}

#[test]
fn check_fence_polygon_records_a_fresh_breach() {
    let mut fence = Fence::new();
    enable_polygon(&mut fence);
    let leftover = fence.check_fence_polygon(CheckPolygonContext {
        loc_valid: true,
        poly_breached: true,
        distance_outside_m: 8.0,
        now_ms: 1_001,
    });
    assert!(leftover.enabled);
    assert!(leftover.newly_breached);
    assert!(leftover.recorded_breach);
    assert!(leftover.need_location);
    almost(leftover.breach_distance_m, 8.0);
    almost(fence.polygon_breach_distance_m(), 8.0);
    assert_eq!(fence.get_breaches() & TYPE_POLYGON, TYPE_POLYGON);
    assert_eq!(fence.get_breach_count(), 1);
}

#[test]
fn check_fence_polygon_already_breached_does_not_re_fire() {
    let mut fence = Fence::new();
    enable_polygon(&mut fence);
    let first = fence.check_fence_polygon(CheckPolygonContext {
        loc_valid: true,
        poly_breached: true,
        distance_outside_m: 8.0,
        now_ms: 1_001,
    });
    assert!(first.newly_breached);
    let still = fence.check_fence_polygon(CheckPolygonContext {
        loc_valid: true,
        poly_breached: true,
        distance_outside_m: 9.0,
        now_ms: 1_101,
    });
    assert!(!still.newly_breached);
    assert!(!still.recorded_breach);
    assert_eq!(fence.get_breach_count(), 1);
    almost(fence.polygon_breach_distance_m(), 9.0);
}

#[test]
fn check_fence_polygon_clears_when_back_inside() {
    let mut fence = Fence::new();
    enable_polygon(&mut fence);
    assert!(
        fence
            .check_fence_polygon(CheckPolygonContext {
                loc_valid: true,
                poly_breached: true,
                distance_outside_m: 8.0,
                now_ms: 1_001,
            })
            .newly_breached
    );
    let back = fence.check_fence_polygon(CheckPolygonContext {
        loc_valid: true,
        poly_breached: false,
        distance_outside_m: -40.0,
        now_ms: 1_101,
    });
    assert!(!back.newly_breached);
    assert!(back.cleared_breach);
    assert!(!back.margin_breached);
    assert_eq!(fence.get_breaches() & TYPE_POLYGON, 0);
}

#[test]
fn check_fence_polygon_records_margin_when_close_inside() {
    let mut fence = Fence::new();
    enable_polygon(&mut fence);
    fence.set_margin_ne_m(5.0);
    let leftover = fence.check_fence_polygon(CheckPolygonContext {
        loc_valid: true,
        poly_breached: false,
        distance_outside_m: -2.0,
        now_ms: 1_001,
    });
    assert!(!leftover.newly_breached);
    assert!(leftover.recorded_margin);
    assert!(leftover.margin_breached);
    assert_eq!(fence.get_margin_breaches() & TYPE_POLYGON, TYPE_POLYGON);

    let far = fence.check_fence_polygon(CheckPolygonContext {
        loc_valid: true,
        poly_breached: false,
        distance_outside_m: -40.0,
        now_ms: 1_101,
    });
    assert!(!far.recorded_margin);
    assert!(!far.margin_breached);
    assert_eq!(fence.get_margin_breaches() & TYPE_POLYGON, 0);
}

#[test]
fn check_fence_polygon_keeps_stale_distance_when_loc_is_invalid() {
    let mut fence = Fence::new();
    enable_polygon(&mut fence);
    assert!(
        fence
            .check_fence_polygon(CheckPolygonContext {
                loc_valid: true,
                poly_breached: true,
                distance_outside_m: 8.0,
                now_ms: 1_001,
            })
            .newly_breached
    );
    let stale = fence.check_fence_polygon(CheckPolygonContext {
        loc_valid: false,
        poly_breached: true,
        distance_outside_m: 99.0,
        now_ms: 1_101,
    });
    assert!(!stale.newly_breached);
    assert!(stale.cleared_breach);
    almost(fence.polygon_breach_distance_m(), 8.0);
}

#[test]
fn check_ors_a_fresh_polygon_breach() {
    let mut fence = Fence::new();
    fence.set_configured_fences(TYPE_POLYGON | TYPE_ALT_MAX);
    fence.set_poly_fence_count(1);
    fence.enable(true, TYPE_POLYGON | TYPE_ALT_MAX, false);

    let leftover = fence.check(CheckContext {
        disable_auto_fences: false,
        now_ms: 1_001,
        location_valid: true,
        ne_home_m: Some((0.0, 0.0)),
        alt_max_u_m: Some(50.0),
        alt_min_u_m: Some(50.0),
        home_alt_amsl_m: 0.0,
        poly_breached: true,
        poly_distance_outside_m: 6.0,
    });
    assert!(leftover.polygon_checked);
    assert!(leftover.polygon.newly_breached);
    assert_eq!(leftover.new_breaches & TYPE_POLYGON, TYPE_POLYGON);
    assert_eq!(fence.get_breaches() & TYPE_POLYGON, TYPE_POLYGON);
}

#[test]
fn loader_breach_feeds_the_polygon_checker() {
    let loader = seat_home_circle(300.0);
    let hit = loader.breached_at(Location::new(100_000, 0));
    assert!(hit.breached);

    let mut fence = Fence::new();
    enable_polygon(&mut fence);
    let leftover = fence.check_fence_polygon(CheckPolygonContext {
        loc_valid: true,
        poly_breached: hit.breached,
        distance_outside_m: hit.distance_outside_m,
        now_ms: 1_001,
    });
    assert!(leftover.newly_breached);
    almost(leftover.breach_distance_m, hit.distance_outside_m);
}
