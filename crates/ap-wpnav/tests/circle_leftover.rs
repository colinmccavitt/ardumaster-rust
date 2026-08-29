//! AC_Circle set_center and closest-point leftovers.

use ap_math::location::{AltContext, AltFrame, Location};
use ap_math::vector3::Vector3f;
use ap_wpnav::{Circle, GetVectorNedContext, CIRCLE_RADIUS_MAX_M, CIRCLE_RADIUS_M_DEFAULT};

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-5, "expected {b}, got {a} (delta {d})");
}

fn almost_vec(got: Vector3f, expected: Vector3f) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
    almost(got.z, expected.z);
}

fn origin_loc() -> Location {
    Location::new(35_0000_000, -1_100_000_000)
}

fn vec_ctx_origin_alt() -> GetVectorNedContext {
    GetVectorNedContext {
        origin: origin_loc(),
        alt: AltContext {
            origin_alt_cm: Some(0),
            ..AltContext::default()
        },
    }
}

#[test]
fn set_center_origin_frame_seats_ned() {
    let mut circle = Circle::new();
    let origin = origin_loc();
    let dest = Location::new_with_alt(origin.lat + 1_000, origin.lng, 500, AltFrame::AboveOrigin);
    let leftover = circle.set_center(dest, vec_ctx_origin_alt(), Vector3f::new(9.0, 9.0, -1.0));
    assert!(!leftover.need_nav_error_log);
    assert!(!leftover.used_pos_estimate_fallback);
    assert!(!circle.center_is_terrain_alt());
    almost(circle.center_ned_m().y, 0.0);
    almost(circle.center_ned_m().z, -5.0);
    assert!(circle.center_ned_m().x > 10.0);
}

#[test]
fn set_center_terrain_frame_marks_terrain_alt() {
    let mut circle = Circle::new();
    let origin = origin_loc();
    let terr = Location::new_with_alt(origin.lat, origin.lng, 800, AltFrame::AboveTerrain);
    let leftover = circle.set_center(terr, vec_ctx_origin_alt(), Vector3f::zero());
    assert!(!leftover.need_nav_error_log);
    assert!(circle.center_is_terrain_alt());
    almost_vec(circle.center_ned_m(), Vector3f::new(0.0, 0.0, -8.0));
}

#[test]
fn set_center_unset_origin_falls_back_to_estimate() {
    let mut circle = Circle::new();
    let origin = origin_loc();
    let dest = Location::new_with_alt(origin.lat + 1_000, origin.lng, 500, AltFrame::AboveOrigin);
    let estimate = Vector3f::new(1.5, -2.25, -3.0);
    let leftover = circle.set_center(dest, GetVectorNedContext::default(), estimate);
    assert!(leftover.need_nav_error_log);
    assert!(leftover.used_pos_estimate_fallback);
    assert!(!circle.center_is_terrain_alt());
    almost_vec(circle.center_ned_m(), estimate);
}

#[test]
fn closest_point_zero_radius_returns_center() {
    let mut circle = Circle::new();
    circle.set_center_ned_m(Vector3f::new(4.0, -1.0, -6.0), false);
    circle.set_radius_m(0.0);
    let closest =
        circle.get_closest_point_on_circle_ned_m(Vector3f::new(10.0, 2.0, -6.0), 1.0, 0.0);
    almost_vec(closest.point_ned_m, Vector3f::new(4.0, -1.0, -6.0));
    almost(closest.dist_to_edge_m, 0.0);
}

#[test]
fn closest_point_at_center_sits_behind_yaw() {
    let mut circle = Circle::new();
    let center = Vector3f::new(2.0, 3.0, -4.0);
    circle.set_center_ned_m(center, false);
    circle.set_radius_m(10.0);
    let closest = circle.get_closest_point_on_circle_ned_m(center, 1.0, 0.0);
    almost_vec(closest.point_ned_m, Vector3f::new(2.0 - 10.0, 3.0, -4.0));
    almost(closest.dist_to_edge_m, 10.0);
}

#[test]
fn closest_point_projects_from_center() {
    let mut circle = Circle::new();
    circle.set_center_ned_m(Vector3f::zero(), false);
    circle.set_radius_m(8.0);
    let stop = Vector3f::new(16.0, 0.0, 0.0);
    let closest = circle.get_closest_point_on_circle_ned_m(stop, 0.0, 1.0);
    almost_vec(closest.point_ned_m, Vector3f::new(8.0, 0.0, 0.0));
    almost(closest.dist_to_edge_m, 8.0);
}

#[test]
fn closest_point_cm_wrapper_round_trips() {
    let mut circle = Circle::new();
    circle.set_center_ned_m(Vector3f::zero(), false);
    circle.set_radius_m(5.0);
    let (point_cm, dist_cm) =
        circle.get_closest_point_on_circle_neu_cm(Vector3f::new(1000.0, 0.0, 0.0), 1.0, 0.0);
    almost_vec(point_cm, Vector3f::new(500.0, 0.0, 0.0));
    almost(dist_cm, 500.0);
}

#[test]
fn radius_cm_wrappers_and_center_neu() {
    let mut circle = Circle::new();
    almost(circle.get_radius_cm(), CIRCLE_RADIUS_M_DEFAULT * 100.0);
    circle.set_radius_cm(250.0);
    almost(circle.radius_m(), 2.5);
    circle.set_radius_cm(500_000.0);
    almost(circle.radius_m(), CIRCLE_RADIUS_MAX_M);
    circle.set_center_ned_m(Vector3f::new(1.0, 2.0, -3.0), true);
    almost_vec(
        circle.get_center_neu_cm(),
        Vector3f::new(100.0, 200.0, 300.0),
    );
}
