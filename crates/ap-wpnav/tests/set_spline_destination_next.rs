//! `set_spline_destination_next_NED_m` — next-leg spline leftover.

use ap_math::vector3::Vector3f;
use ap_wpnav::{AttitudeJerkLimits, SetWpDestinationContext, WpNav};

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
fn next_spline_after_spline_leg_reuses_this_dest_vel() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 1_000, AttitudeJerkLimits::default());

    let dest = Vector3f::new(10.0, 0.0, 0.0);
    let next = Vector3f::new(18.0, 6.0, 0.0);
    assert!(nav.set_spline_destination_ned_m(dest, false, next, false, true, ctx_at(1_000, stop)));
    let this_dest_vel = nav.spline_destination_vel_ned_ms();
    almost_vec(this_dest_vel, next - stop);
    assert!(!nav.next_leg_is_spline());
    assert!(!nav.spline_next_leg_set());

    let next_next = Vector3f::new(28.0, 6.0, -1.0);
    assert!(nav.set_spline_destination_next_ned_m(next, false, next_next, false, false));

    assert!(nav.next_leg_is_spline());
    assert!(nav.spline_next_leg_set());
    assert!(nav.flags().fast_waypoint);
    assert!(nav.need_this_leg_dest_speed_max());
    assert_eq!(nav.spline_next_destination_ned_m(), next);
    // this-leg dest / origin stay put
    assert_eq!(nav.wp_destination_ned_m(), dest);
    assert_eq!(nav.wp_origin_ned_m(), stop);
    assert!(nav.this_leg_is_spline());
    // leftover of `_spline_this_leg.get_destination_vel`
    almost_vec(nav.spline_next_origin_vel_ned_ms(), this_dest_vel);
    // next_next_is_spline=false → leave toward the upcoming straight leg
    almost_vec(nav.spline_next_destination_vel_ned_ms(), next_next - next);
}

#[test]
fn next_spline_after_straight_leg_uses_track_vector() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(1.0, 2.0, 0.0);
    nav.wp_and_spline_init_m(5.0, stop, 0, AttitudeJerkLimits::default());

    let dest = Vector3f::new(11.0, 2.0, 0.0);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, ctx_at(0, stop)));
    assert!(!nav.this_leg_is_spline());
    assert!(!nav.flags().fast_waypoint);

    let next = Vector3f::new(20.0, 8.0, 1.0);
    let next_next = Vector3f::new(30.0, 8.0, 1.0);
    assert!(nav.set_spline_destination_next_ned_m(next, false, next_next, false, true));

    assert!(nav.next_leg_is_spline());
    assert!(nav.spline_next_leg_set());
    assert!(nav.flags().fast_waypoint);
    assert!(nav.need_this_leg_dest_speed_max());
    almost_vec(nav.spline_next_origin_vel_ned_ms(), dest - stop);
    // next_next_is_spline=true → leave toward the arc from current dest
    almost_vec(nav.spline_next_destination_vel_ned_ms(), next_next - dest);
    assert_eq!(nav.spline_next_destination_ned_m(), next);
}

#[test]
fn mismatched_next_terrain_skips_without_changing_state() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -2.0);
    nav.wp_and_spline_init_m(3.0, stop, 0, AttitudeJerkLimits::default());

    let dest = Vector3f::new(6.0, 0.0, -2.0);
    let next = Vector3f::new(12.0, 0.0, -2.0);
    assert!(nav.set_spline_destination_ned_m(dest, false, next, false, false, ctx_at(0, stop)));
    assert!(!nav.next_leg_is_spline());
    assert!(!nav.spline_next_leg_set());
    let fast_before = nav.flags().fast_waypoint;

    let next_next = Vector3f::new(18.0, 0.0, -1.0);
    assert!(nav.set_spline_destination_next_ned_m(next, true, next_next, true, false));

    assert!(!nav.next_leg_is_spline());
    assert!(!nav.spline_next_leg_set());
    assert!(!nav.need_this_leg_dest_speed_max());
    assert_eq!(nav.flags().fast_waypoint, fast_before);
    almost_vec(nav.spline_next_origin_vel_ned_ms(), Vector3f::zero());
    almost_vec(nav.spline_next_destination_vel_ned_ms(), Vector3f::zero());
    assert_eq!(nav.wp_destination_ned_m(), dest);
}

#[test]
fn mismatched_next_next_terrain_clears_next_dest_vel() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 0, AttitudeJerkLimits::default());

    let dest = Vector3f::new(8.0, 0.0, 0.0);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, ctx_at(0, stop)));

    let next = Vector3f::new(16.0, 4.0, 0.0);
    let next_next = Vector3f::new(24.0, 4.0, -2.0);
    assert!(nav.set_spline_destination_next_ned_m(next, false, next_next, true, false));

    // next dest frame matches current, so the next point is added
    assert!(nav.next_leg_is_spline());
    assert!(nav.spline_next_leg_set());
    assert!(nav.flags().fast_waypoint);
    almost_vec(nav.spline_next_origin_vel_ned_ms(), dest - stop);
    almost_vec(nav.spline_next_destination_vel_ned_ms(), Vector3f::zero());
}

#[test]
fn this_leg_spline_setter_clears_next_leg_leftover() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, 0.0);
    nav.wp_and_spline_init_m(4.0, stop, 1_000, AttitudeJerkLimits::default());

    let dest = Vector3f::new(10.0, 0.0, 0.0);
    let next = Vector3f::new(20.0, 0.0, 0.0);
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, ctx_at(1_000, stop)));
    assert!(nav.set_spline_destination_next_ned_m(
        next,
        false,
        Vector3f::new(30.0, 0.0, 0.0),
        false,
        false
    ));
    assert!(nav.spline_next_leg_set());
    assert!(nav.next_leg_is_spline());

    let dest2 = Vector3f::new(40.0, 2.0, 0.0);
    let next2 = Vector3f::new(50.0, 2.0, 0.0);
    assert!(nav.set_spline_destination_ned_m(
        dest2,
        false,
        next2,
        false,
        false,
        ctx_at(1_010, stop)
    ));

    assert!(!nav.next_leg_is_spline());
    assert!(!nav.spline_next_leg_set());
    assert!(!nav.need_this_leg_dest_speed_max());
    almost_vec(nav.spline_next_origin_vel_ned_ms(), Vector3f::zero());
}
