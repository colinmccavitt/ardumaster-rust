//! `set_wp_destination_NED_m` / `set_wp_destination_NEU_cm` — destination set.

use ap_math::vector3::Vector3f;
use ap_wpnav::{AttitudeJerkLimits, SetWpDestinationContext, WpNav, WPNAV_ACTIVE_TIMEOUT_MS};

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-5, "expected {b}, got {a} (delta {d})");
}

fn ctx_at(now_ms: u32, stopping_point_ned_m: Vector3f) -> SetWpDestinationContext {
    SetWpDestinationContext {
        now_ms,
        attitude: AttitudeJerkLimits::default(),
        stopping_point_ned_m,
        terrain_d_m: None,
    }
}

#[test]
fn set_destination_uses_previous_dest_as_origin() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(1.0, 2.0, 3.0);
    nav.wp_and_spline_init_m(4.0, stop, 1_000, AttitudeJerkLimits::default());

    let dest = Vector3f::new(10.0, -4.0, 1.5);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.25, ctx_at(1_000, stop)));

    assert_eq!(nav.wp_origin_ned_m(), stop);
    assert_eq!(nav.wp_destination_ned_m(), dest);
    assert!(!nav.reached_wp_destination());
    assert!(!nav.flags().fast_waypoint);
    assert!(!nav.this_leg_is_spline());
    assert!(!nav.next_leg_is_spline());
    assert_eq!(nav.next_destination_ned_m(), Vector3f::zero());
    assert!(nav.scurve_this_leg_calculated());
    almost(nav.last_arc_rad(), 0.25);
    assert!(!nav.origin_and_destination_are_terrain_alt());
}

#[test]
fn neu_cm_wrapper_converts_and_getters_round_trip() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -2.0);
    nav.wp_and_spline_init_m(0.0, stop, 500, AttitudeJerkLimits::default());

    // NEU cm: N=1200, E=-300, U=400 → NED m: (12, -3, -4)
    let neu_cm = Vector3f::new(1200.0, -300.0, 400.0);
    assert!(nav.set_wp_destination_neu_cm(neu_cm, false, ctx_at(500, stop)));

    let dest = nav.wp_destination_ned_m();
    almost(dest.x, 12.0);
    almost(dest.y, -3.0);
    almost(dest.z, -4.0);

    let got = nav.wp_destination_neu_cm();
    almost(got.x, 1200.0);
    almost(got.y, -300.0);
    almost(got.z, 400.0);

    let origin_cm = nav.wp_origin_neu_cm();
    almost(origin_cm.x, 0.0);
    almost(origin_cm.y, 0.0);
    almost(origin_cm.z, 200.0);
}

#[test]
fn interrupted_leg_reinitialises_from_stopping_point() {
    let mut nav = WpNav::new();
    let first_stop = Vector3f::new(5.0, 5.0, 0.0);
    nav.wp_and_spline_init_m(3.0, first_stop, 1_000, AttitudeJerkLimits::default());
    let mid = Vector3f::new(20.0, 0.0, 0.0);
    assert!(nav.set_wp_destination_ned_m(mid, false, 0.0, ctx_at(1_000, first_stop)));
    assert!(!nav.reached_wp_destination());

    // Not reached, so the next set re-inits from the new stopping point.
    let new_stop = Vector3f::new(8.0, 1.0, -0.5);
    let dest = Vector3f::new(30.0, 4.0, -1.0);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, ctx_at(1_050, new_stop)));

    assert_eq!(nav.wp_origin_ned_m(), new_stop);
    assert_eq!(nav.wp_destination_ned_m(), dest);
    almost(nav.desired_speed_ne_ms(), 3.0);
}

#[test]
fn inactive_navigator_also_reinitialises() {
    let mut nav = WpNav::new();
    let first_stop = Vector3f::new(1.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(2.0, first_stop, 100, AttitudeJerkLimits::default());
    assert!(nav.reached_wp_destination());

    let later = 100 + WPNAV_ACTIVE_TIMEOUT_MS + 1;
    let new_stop = Vector3f::new(2.0, 2.0, 0.0);
    let dest = Vector3f::new(9.0, 0.0, 0.0);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, ctx_at(later, new_stop)));
    assert_eq!(nav.wp_origin_ned_m(), new_stop);
    assert_eq!(nav.wp_destination_ned_m(), dest);
}

#[test]
fn terrain_frame_flip_without_offset_fails() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -10.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());
    let dest = Vector3f::new(4.0, 0.0, -5.0);
    assert!(!nav.set_wp_destination_ned_m(dest, true, 0.0, ctx_at(0, stop)));
    assert_eq!(nav.wp_destination_ned_m(), stop);
    assert!(nav.reached_wp_destination());
}

#[test]
fn terrain_frame_flip_shifts_origin_z() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -10.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());

    let dest = Vector3f::new(4.0, 1.0, -3.0);
    let mut ctx = ctx_at(0, stop);
    ctx.terrain_d_m = Some(7.0);
    assert!(nav.set_wp_destination_ned_m(dest, true, 0.0, ctx));

    almost(nav.wp_origin_ned_m().z, -10.0 - 7.0);
    assert_eq!(nav.wp_destination_ned_m(), dest);
    assert!(nav.origin_and_destination_are_terrain_alt());
    almost(nav.pos_terrain_d_m(), 7.0);
}

#[test]
fn terrain_frame_flip_back_to_origin_clears_pos_terrain() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -4.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());

    let mut ctx = ctx_at(0, stop);
    ctx.terrain_d_m = Some(2.0);
    assert!(nav.set_wp_destination_ned_m(Vector3f::new(1.0, 0.0, -1.0), true, 0.0, ctx));

    // Second set: still active and not reached → re-init first (terrain
    // becomes origin-relative again), then flip to origin-relative dest
    // matches and does not need terrain.
    ctx.now_ms = 10;
    ctx.stopping_point_ned_m = Vector3f::new(0.5, 0.0, -3.0);
    ctx.terrain_d_m = None;
    let dest = Vector3f::new(8.0, 0.0, -2.0);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, ctx));
    assert!(!nav.origin_and_destination_are_terrain_alt());
    assert_eq!(nav.wp_destination_ned_m(), dest);
}
