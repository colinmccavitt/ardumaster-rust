//! `update_wpnav` leftover and `get_wp_distance_to_destination`.

use ap_math::vector3::Vector3f;
use ap_wpnav::{
    AttitudeJerkLimits, SetWpDestinationContext, UpdateWpNavContext, WpNav, WPNAV_ACCELERATION_MS,
    WPNAV_ACTIVE_TIMEOUT_MS, WP_SPD_DEFAULT, WP_SPD_DOWN_DEFAULT, WP_SPD_MIN, WP_SPD_UP_DEFAULT,
};

fn almost(a: f32, b: f32) {
    let d = (a - b).abs();
    assert!(d <= 1e-5, "expected {b}, got {a} (delta {d})");
}

fn dest_ctx(now_ms: u32, stopping_point_ned_m: Vector3f) -> SetWpDestinationContext {
    SetWpDestinationContext {
        now_ms,
        attitude: AttitudeJerkLimits::default(),
        stopping_point_ned_m,
        terrain_d_m: None,
    }
}

fn tick(now_ms: u32) -> UpdateWpNavContext {
    UpdateWpNavContext {
        now_ms,
        dt_s: 0.01,
        terrain_d_m: None,
    }
}

#[test]
fn update_stamps_last_update_and_stays_active() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(0.0, Vector3f::zero(), 1_000, AttitudeJerkLimits::default());
    assert!(!nav.is_active(1_000 + WPNAV_ACTIVE_TIMEOUT_MS));

    let leftover = nav.update_wpnav(tick(1_250));
    assert!(leftover.advance_ok);
    assert!(leftover.need_advance_track);
    assert!(leftover.need_ne_update_controller);
    assert!(!leftover.applied_speed_ne);
    assert!(!leftover.applied_speed_up);
    assert!(!leftover.applied_speed_down);
    assert!(!leftover.need_update_track_limits);
    almost(leftover.dt_s, 0.01);
    assert!(nav.is_active(1_250));
    assert!(nav.is_active(1_250 + WPNAV_ACTIVE_TIMEOUT_MS - 1));
    assert!(!nav.is_active(1_250 + WPNAV_ACTIVE_TIMEOUT_MS));
}

#[test]
fn wp_spd_change_applies_set_speed_ne_when_watched() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(0.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    assert!(nav.check_wp_speed_change());
    almost(nav.desired_speed_ne_ms(), WP_SPD_DEFAULT);
    almost(nav.offset_vel_ms(), WP_SPD_DEFAULT);

    nav.set_wp_speed_ms(7.0);
    let leftover = nav.update_wpnav(tick(10));
    assert!(leftover.applied_speed_ne);
    assert!(leftover.need_update_track_limits);
    almost(nav.desired_speed_ne_ms(), 7.0);
    almost(nav.offset_vel_ms(), 7.0);
    almost(nav.last_wp_speed_ms(), 7.0);
    almost(nav.pos_speed_accel().ne_speed_ms, 7.0);
    almost(nav.pos_speed_accel().ne_accel_mss, WPNAV_ACCELERATION_MS);

    // Second tick with the same param is a no-op.
    let leftover = nav.update_wpnav(tick(20));
    assert!(!leftover.applied_speed_ne);
    assert!(!leftover.need_update_track_limits);
    almost(nav.desired_speed_ne_ms(), 7.0);
}

#[test]
fn explicit_init_speed_does_not_watch_wp_spd() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(4.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    assert!(!nav.check_wp_speed_change());

    nav.set_wp_speed_ms(8.0);
    let leftover = nav.update_wpnav(tick(10));
    assert!(!leftover.applied_speed_ne);
    almost(nav.desired_speed_ne_ms(), 4.0);
    almost(nav.offset_vel_ms(), 4.0);
}

#[test]
fn climb_and_descent_param_changes_always_apply() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(3.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    almost(nav.pos_speed_accel().speed_up_ms, WP_SPD_UP_DEFAULT);
    almost(nav.pos_speed_accel().speed_down_ms, WP_SPD_DOWN_DEFAULT);

    nav.set_wp_speed_up_ms(1.25);
    nav.set_wp_speed_down_ms(0.75);
    let leftover = nav.update_wpnav(tick(10));
    assert!(leftover.applied_speed_up);
    assert!(leftover.applied_speed_down);
    assert!(leftover.need_update_track_limits);
    almost(nav.pos_speed_accel().speed_up_ms, 1.25);
    almost(nav.pos_speed_accel().speed_down_ms, 0.75);
    almost(nav.last_wp_speed_up_ms(), 1.25);
    almost(nav.last_wp_speed_down_ms(), 0.75);
}

#[test]
fn set_speed_ne_rejects_below_floor_and_zero_desired() {
    let mut nav = WpNav::new();
    assert!(!nav.set_speed_ne_ms(5.0));
    almost(nav.desired_speed_ne_ms(), 0.0);

    nav.wp_and_spline_init_m(4.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    assert!(!nav.set_speed_ne_ms(WP_SPD_MIN - 0.001));
    almost(nav.desired_speed_ne_ms(), 4.0);
    assert!(nav.set_speed_ne_ms(WP_SPD_MIN));
    almost(nav.desired_speed_ne_ms(), WP_SPD_MIN);
}

#[test]
fn set_speed_ne_scales_offset_vel_ratio() {
    let mut nav = WpNav::new();
    nav.wp_and_spline_init_m(10.0, Vector3f::zero(), 0, AttitudeJerkLimits::default());
    // Pretend terrain shaping has slowed the offset to 40% of desired.
    assert!(nav.set_speed_ne_ms(10.0));
    // After init offset == desired; apply 5 m/s → offset becomes 5.
    assert!(nav.set_speed_ne_ms(5.0));
    almost(nav.offset_vel_ms(), 5.0);
    almost(nav.desired_speed_ne_ms(), 5.0);
}

#[test]
fn terrain_alt_without_offset_fails_advance_but_still_stamps() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -4.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());
    let mut ctx = dest_ctx(0, stop);
    ctx.terrain_d_m = Some(2.0);
    assert!(nav.set_wp_destination_ned_m(Vector3f::new(8.0, 0.0, -1.0), true, 0.0, ctx));
    assert!(nav.origin_and_destination_are_terrain_alt());

    let leftover = nav.update_wpnav(tick(50));
    assert!(!leftover.advance_ok);
    assert!(leftover.need_advance_track);
    assert!(leftover.need_ne_update_controller);
    assert!(nav.is_active(50));
}

#[test]
fn terrain_alt_with_offset_advance_ok() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -4.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());
    let mut ctx = dest_ctx(0, stop);
    ctx.terrain_d_m = Some(2.0);
    assert!(nav.set_wp_destination_ned_m(Vector3f::new(8.0, 0.0, -1.0), true, 0.0, ctx));

    let leftover = nav.update_wpnav(UpdateWpNavContext {
        now_ms: 50,
        dt_s: 0.02,
        terrain_d_m: Some(2.0),
    });
    assert!(leftover.advance_ok);
    almost(leftover.dt_s, 0.02);
}

#[test]
fn get_wp_distance_is_horizontal_and_ignores_z() {
    let mut nav = WpNav::new();
    let stop = Vector3f::new(0.0, 0.0, -10.0);
    nav.wp_and_spline_init_m(0.0, stop, 0, AttitudeJerkLimits::default());
    assert!(nav.set_wp_destination_ned_m(
        Vector3f::new(3.0, 4.0, -1.0),
        false,
        0.0,
        dest_ctx(0, stop)
    ));

    let pos = Vector3f::new(0.0, 0.0, 99.0);
    almost(nav.get_wp_distance_to_destination_m(pos), 5.0);
    almost(nav.get_wp_distance_to_destination_cm(pos), 500.0);

    let at_dest = Vector3f::new(3.0, 4.0, 0.0);
    almost(nav.get_wp_distance_to_destination_m(at_dest), 0.0);
}
