//! `advance_wp_target_along_track` leftover, `get_wp_bearing_to_destination`,
//! and `reached_wp_destination_NE`.

use ap_math::vector3::Vector3f;
use ap_wpnav::{
    AdvanceWpTargetContext, AttitudeJerkLimits, SetWpDestinationContext, WpNav,
    WP_RADIUS_M_DEFAULT, WP_SPD_DEFAULT,
};

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-4, "expected {b}, got {a} (delta {d})");
}

fn dest_ctx(now_ms: u32, stopping_point_ned_m: Vector3f) -> SetWpDestinationContext {
    SetWpDestinationContext {
        now_ms,
        attitude: AttitudeJerkLimits::default(),
        stopping_point_ned_m,
        terrain_d_m: None,
    }
}

fn seat_dest(nav: &mut WpNav, dest: Vector3f) {
    let stop = Vector3f::zero();
    nav.wp_and_spline_init_m(WP_SPD_DEFAULT, stop, 1_000, AttitudeJerkLimits::default());
    assert!(nav.set_wp_destination_ned_m(dest, false, 0.0, dest_ctx(1_000, stop)));
    assert!(!nav.reached_wp_destination());
}

fn advance_ctx(pos: Vector3f, path_finished: bool) -> AdvanceWpTargetContext {
    AdvanceWpTargetContext {
        pos_estimate_ned_m: pos,
        path_finished,
        ..AdvanceWpTargetContext::default()
    }
}

#[test]
fn terrain_alt_without_offset_fails_and_does_not_reach() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -4.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());
    let mut ctx = dest_ctx(0, stop);
    ctx.terrain_d_m = Some(2.0);
    assert!(nav.set_wp_destination_ned_m(Vector3f::new(8.0, 0.0, -1.0), true, 0.0, ctx));
    assert!(nav.origin_and_destination_are_terrain_alt());

    let leftover = nav.advance_wp_target_along_track(AdvanceWpTargetContext {
        terrain_d_m: None,
        path_finished: true,
        ..AdvanceWpTargetContext::default()
    });
    assert!(!leftover.ok);
    assert!(!leftover.need_set_pos_terrain_target);
    assert!(!leftover.need_scurve_advance);
    assert!(!leftover.need_spline_advance);
    assert!(!leftover.need_set_pos_vel_accel);
    assert!(!nav.reached_wp_destination());
}

#[test]
fn path_unfinished_does_not_set_reached() {
    let mut nav = WpNav::new();
    seat_dest(&mut nav, Vector3f::new(10.0, 0.0, 0.0));

    let leftover =
        nav.advance_wp_target_along_track(advance_ctx(Vector3f::new(10.0, 0.0, 0.0), false));
    assert!(leftover.ok);
    assert!(leftover.need_scurve_advance);
    assert!(!leftover.need_spline_advance);
    assert!(leftover.need_set_pos_vel_accel);
    assert!(leftover.need_set_pos_terrain_target);
    assert!(!nav.reached_wp_destination());
}

#[test]
fn regular_waypoint_reaches_only_inside_3d_radius() {
    let mut nav = WpNav::new();
    let dest = Vector3f::new(10.0, 0.0, 0.0);
    seat_dest(&mut nav, dest);
    assert!(!nav.flags().fast_waypoint);

    // Outside the 2 m radius: finished path is not enough.
    let leftover =
        nav.advance_wp_target_along_track(advance_ctx(Vector3f::new(6.0, 0.0, 0.0), true));
    assert!(leftover.ok);
    assert!(!nav.reached_wp_destination());
    assert!(!nav.reached_wp_destination_ne(Vector3f::new(6.0, 0.0, 0.0)));

    // Horizontal 1 m away, Z 3 m away: 3D length > radius, NE would pass.
    let leftover =
        nav.advance_wp_target_along_track(advance_ctx(Vector3f::new(11.0, 0.0, 3.0), true));
    assert!(leftover.ok);
    assert!(!nav.reached_wp_destination());
    assert!(nav.reached_wp_destination_ne(Vector3f::new(11.0, 0.0, 3.0)));

    // Inside the 3D radius.
    let leftover =
        nav.advance_wp_target_along_track(advance_ctx(Vector3f::new(11.0, 0.0, 0.5), true));
    assert!(leftover.ok);
    assert!(nav.reached_wp_destination());
}

#[test]
fn fast_waypoint_reaches_as_soon_as_path_finishes() {
    let mut nav = WpNav::new();
    seat_dest(&mut nav, Vector3f::new(30.0, 0.0, 0.0));
    nav.set_fast_waypoint(true);

    let leftover = nav.advance_wp_target_along_track(advance_ctx(Vector3f::zero(), true));
    assert!(leftover.ok);
    assert!(nav.reached_wp_destination());
    // Already reached: a later unfinished tick must not clear the flag.
    let leftover = nav.advance_wp_target_along_track(advance_ctx(Vector3f::zero(), false));
    assert!(leftover.ok);
    assert!(nav.reached_wp_destination());
}

#[test]
fn pause_shapes_offset_vel_toward_zero() {
    let mut nav = WpNav::new();
    seat_dest(&mut nav, Vector3f::new(20.0, 0.0, 0.0));
    almost(nav.offset_vel_ms(), WP_SPD_DEFAULT);
    almost(nav.offset_accel_mss(), 0.0);

    nav.set_pause();
    assert!(nav.paused());
    let leftover = nav.advance_wp_target_along_track(AdvanceWpTargetContext {
        dt_s: 0.05,
        shaping_jerk_ne_msss: 5.0,
        ..AdvanceWpTargetContext::default()
    });
    assert!(leftover.ok);
    // First tick only shapes accel (update_vel_accel saw accel=0).
    almost(leftover.vel_dt_scalar, 1.0);
    almost(nav.offset_vel_ms(), WP_SPD_DEFAULT);
    assert!(nav.offset_accel_mss() < 0.0);

    let leftover = nav.advance_wp_target_along_track(AdvanceWpTargetContext {
        dt_s: 0.05,
        shaping_jerk_ne_msss: 5.0,
        ..AdvanceWpTargetContext::default()
    });
    assert!(leftover.ok);
    assert!(leftover.vel_dt_scalar < 1.0);
    assert!(nav.offset_vel_ms() < WP_SPD_DEFAULT);

    nav.set_resume();
    assert!(!nav.paused());
}

#[test]
fn track_dt_scalar_filters_toward_speed_alignment() {
    let mut nav = WpNav::new();
    seat_dest(&mut nav, Vector3f::new(20.0, 0.0, 0.0));
    almost(nav.track_dt_scalar(), 1.0);

    // Desired 10 m/s North, vehicle at 5 m/s with 2 m of along-track error.
    // raw = constrain(0.05 + (5 - 1*2)/10, 0, 1) = 0.35
    let leftover = nav.advance_wp_target_along_track(AdvanceWpTargetContext {
        dt_s: 0.01,
        vel_desired_ned_ms: Vector3f::new(10.0, 0.0, 0.0),
        vel_estimate_ned_ms: Vector3f::new(5.0, 0.0, 0.0),
        pos_error_ned_m: Vector3f::new(2.0, 0.0, 0.0),
        pos_p_kp: 1.0,
        ..AdvanceWpTargetContext::default()
    });
    assert!(leftover.ok);
    almost(leftover.raw_track_dt_scalar, 0.35);
    // Filter: tc = accel/jerk = 2.5/1.0 = 2.5; 1 + (0.35-1)*(0.01/2.5)
    almost(nav.track_dt_scalar(), 1.0 + (0.35 - 1.0) * (0.01 / 2.5));
    almost(
        leftover.dt_along_track_s,
        nav.track_dt_scalar() * leftover.vel_dt_scalar * 0.01,
    );
}

#[test]
fn bearing_is_clockwise_from_north() {
    let mut nav = WpNav::new();
    seat_dest(&mut nav, Vector3f::new(10.0, 0.0, -3.0));

    // Already at dest: bearing is undefined-but-stable (0).
    almost(
        nav.get_wp_bearing_to_destination_rad(Vector3f::new(10.0, 0.0, 99.0)),
        0.0,
    );

    // East of dest looking west? Origin at (10, -5): dest is +Y (east).
    let from_south_of_east = Vector3f::new(10.0, -5.0, 0.0);
    almost(
        nav.get_wp_bearing_to_destination_rad(from_south_of_east),
        core::f32::consts::FRAC_PI_2,
    );

    // South of dest: dest is +X (north).
    almost(
        nav.get_wp_bearing_to_destination_rad(Vector3f::new(0.0, 0.0, 0.0)),
        0.0,
    );
    assert_eq!(
        nav.get_wp_bearing_to_destination_cd(Vector3f::new(0.0, 0.0, 0.0)),
        0
    );

    // West of dest: dest is +Y from (-something)? dest=(10,0), pos=(10,5) → dest is -Y (west).
    let west = nav.get_wp_bearing_to_destination_rad(Vector3f::new(10.0, 5.0, 0.0));
    almost(west, 3.0 * core::f32::consts::FRAC_PI_2);
}

#[test]
fn reached_ne_uses_horizontal_radius_only() {
    let mut nav = WpNav::new();
    seat_dest(&mut nav, Vector3f::new(0.0, 0.0, 0.0));
    almost(nav.wp_radius_m(), WP_RADIUS_M_DEFAULT);

    assert!(nav.reached_wp_destination_ne(Vector3f::new(1.0, 0.0, 50.0)));
    assert!(!nav.reached_wp_destination_ne(Vector3f::new(3.0, 0.0, 0.0)));
}
