//! `set_spline_destination_NED_m` — current spline dest leftover.

use ap_math::vector3::Vector3f;
use ap_wpnav::{AdvanceWpTargetContext, AttitudeJerkLimits, SetWpDestinationContext, WpNav};

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-5, "expected {b}, got {a} (delta {d})");
}

fn almost_vec(got: Vector3f, expected: Vector3f) {
    almost(got.x, expected.x);
    almost(got.y, expected.y);
    almost(got.z, expected.z);
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
fn set_spline_destination_uses_previous_dest_as_origin() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(1.0, 2.0, 3.0);
    nav.wp_and_spline_init_m(4.0, stop, 1_000, AttitudeJerkLimits::default());

    let dest = Vector3f::new(10.0, -4.0, 1.5);
    let next = Vector3f::new(20.0, 0.0, 0.0);
    assert!(nav.set_spline_destination_ned_m(dest, false, next, false, false, ctx_at(1_000, stop)));

    assert_eq!(nav.wp_origin_ned_m(), stop);
    assert_eq!(nav.wp_destination_ned_m(), dest);
    assert!(!nav.reached_wp_destination());
    assert!(nav.this_leg_is_spline());
    assert!(!nav.next_leg_is_spline());
    assert!(nav.spline_this_leg_set());
    assert!(!nav.scurve_this_leg_calculated());
    assert_eq!(nav.next_destination_ned_m(), next);
    assert!(nav.flags().fast_waypoint);
    almost_vec(nav.spline_origin_vel_ned_ms(), Vector3f::zero());
    // next_is_spline=false → leave toward the following straight leg
    almost_vec(nav.spline_destination_vel_ned_ms(), next - dest);
    assert!(!nav.origin_and_destination_are_terrain_alt());
}

#[test]
fn next_is_spline_aims_origin_to_next() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(5.0, stop, 0, AttitudeJerkLimits::default());

    let dest = Vector3f::new(8.0, 0.0, 0.0);
    let next = Vector3f::new(12.0, 6.0, -1.0);
    assert!(nav.set_spline_destination_ned_m(dest, false, next, false, true, ctx_at(0, stop)));

    almost_vec(nav.spline_destination_vel_ned_ms(), next - stop);
    assert!(nav.flags().fast_waypoint);
    assert!(nav.this_leg_is_spline());
}

#[test]
fn mismatched_next_terrain_clears_fast_waypoint() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -2.0);
    nav.wp_and_spline_init_m(3.0, stop, 0, AttitudeJerkLimits::default());

    let dest = Vector3f::new(4.0, 0.0, -2.0);
    let next = Vector3f::new(9.0, 0.0, -1.0);
    assert!(nav.set_spline_destination_ned_m(dest, false, next, true, false, ctx_at(0, stop)));

    assert!(!nav.flags().fast_waypoint);
    almost_vec(nav.spline_destination_vel_ned_ms(), Vector3f::zero());
    assert!(nav.this_leg_is_spline());
    assert!(!nav.reached_wp_destination());
}

#[test]
fn interrupted_spline_reinitialises_from_stopping_point() {
    let mut nav = WpNav::new();
    let first_stop = Vector3f::new(5.0, 5.0, 0.0);
    nav.wp_and_spline_init_m(3.0, first_stop, 1_000, AttitudeJerkLimits::default());
    let mid = Vector3f::new(20.0, 0.0, 0.0);
    let next = Vector3f::new(30.0, 0.0, 0.0);
    assert!(nav.set_spline_destination_ned_m(
        mid,
        false,
        next,
        false,
        false,
        ctx_at(1_000, first_stop)
    ));
    assert!(!nav.reached_wp_destination());

    let new_stop = Vector3f::new(8.0, 1.0, -0.5);
    let dest = Vector3f::new(40.0, 4.0, -1.0);
    let next2 = Vector3f::new(50.0, 4.0, -1.0);
    assert!(nav.set_spline_destination_ned_m(
        dest,
        false,
        next2,
        false,
        false,
        ctx_at(1_050, new_stop)
    ));

    assert_eq!(nav.wp_origin_ned_m(), new_stop);
    assert_eq!(nav.wp_destination_ned_m(), dest);
    almost(nav.desired_speed_ne_ms(), 3.0);
    almost_vec(nav.spline_origin_vel_ned_ms(), Vector3f::zero());
}

#[test]
fn terrain_frame_flip_without_offset_fails() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -10.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());
    let dest = Vector3f::new(4.0, 0.0, -5.0);
    let next = Vector3f::new(8.0, 0.0, -5.0);
    assert!(!nav.set_spline_destination_ned_m(dest, true, next, true, false, ctx_at(0, stop)));
    assert_eq!(nav.wp_destination_ned_m(), stop);
    assert!(nav.reached_wp_destination());
    assert!(!nav.this_leg_is_spline());
    assert!(!nav.spline_this_leg_set());
}

#[test]
fn terrain_frame_flip_shifts_origin_z() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -10.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());

    let dest = Vector3f::new(4.0, 1.0, -3.0);
    let next = Vector3f::new(7.0, 1.0, -3.0);
    let mut ctx = ctx_at(0, stop);
    ctx.terrain_d_m = Some(7.0);
    assert!(nav.set_spline_destination_ned_m(dest, true, next, true, false, ctx));

    almost(nav.wp_origin_ned_m().z, -10.0 - 7.0);
    assert_eq!(nav.wp_destination_ned_m(), dest);
    assert!(nav.origin_and_destination_are_terrain_alt());
    almost(nav.pos_terrain_d_m(), 7.0);
    assert!(nav.this_leg_is_spline());
}

#[test]
fn reached_fast_straight_leg_seeds_origin_velocity() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 1_000, AttitudeJerkLimits::default());

    let mid = Vector3f::new(10.0, 0.0, 0.0);
    assert!(nav.set_wp_destination_ned_m(mid, false, 0.0, ctx_at(1_000, stop)));
    nav.set_fast_waypoint(true);

    let leftover = nav.advance_wp_target_along_track(AdvanceWpTargetContext {
        path_finished: true,
        pos_estimate_ned_m: mid,
        ..AdvanceWpTargetContext::default()
    });
    assert!(leftover.ok);
    assert!(nav.reached_wp_destination());
    assert!(!nav.this_leg_is_spline());

    let dest = Vector3f::new(20.0, 5.0, 0.0);
    let next = Vector3f::new(30.0, 5.0, 0.0);
    assert!(nav.set_spline_destination_ned_m(dest, false, next, false, false, ctx_at(1_010, stop)));

    assert_eq!(nav.wp_origin_ned_m(), mid);
    almost_vec(nav.spline_origin_vel_ned_ms(), mid - stop);
    almost_vec(nav.spline_destination_vel_ned_ms(), next - dest);
    assert!(nav.this_leg_is_spline());
}

#[test]
fn chained_spline_reuses_previous_destination_vel() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 1_000, AttitudeJerkLimits::default());

    let first = Vector3f::new(8.0, 0.0, 0.0);
    let second = Vector3f::new(16.0, 4.0, 0.0);
    assert!(nav.set_spline_destination_ned_m(
        first,
        false,
        second,
        false,
        true,
        ctx_at(1_000, stop)
    ));
    let first_dest_vel = nav.spline_destination_vel_ned_ms();
    almost_vec(first_dest_vel, second - stop);
    assert!(nav.flags().fast_waypoint);

    nav.set_fast_waypoint(true);
    let leftover = nav.advance_wp_target_along_track(AdvanceWpTargetContext {
        path_finished: true,
        pos_estimate_ned_m: first,
        ..AdvanceWpTargetContext::default()
    });
    assert!(leftover.ok);
    assert!(nav.reached_wp_destination());
    assert!(nav.this_leg_is_spline());

    let third = Vector3f::new(24.0, 0.0, 0.0);
    assert!(nav.set_spline_destination_ned_m(
        second,
        false,
        third,
        false,
        false,
        ctx_at(1_010, stop)
    ));

    almost_vec(nav.spline_origin_vel_ned_ms(), first_dest_vel);
    assert_eq!(nav.wp_origin_ned_m(), first);
    assert_eq!(nav.wp_destination_ned_m(), second);
}
